#pragma once

#include "environment_scene_graph.hpp"
#include "finite_state_automaton.hpp"
#include "scene_graph.hpp"

#include "erl_common/block_timer.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>

#include <bitset>

namespace Eigen::internal {

    template<>
    struct cast_impl<std::bitset<32>, uint32_t> {
        EIGEN_DEVICE_FUNC
        static uint32_t
        run(const std::bitset<32> &x) {
            return static_cast<uint32_t>(x.to_ulong());
        }
    };

    template<>
    struct cast_impl<uint32_t, std::bitset<32>> {
        EIGEN_DEVICE_FUNC
        static std::bitset<32>
        run(const uint32_t &x) {
            return {x};
        }
    };
}  // namespace Eigen::internal

namespace erl::env {

    template<typename Dtype>
    class EnvironmentLTLSceneGraph : public EnvironmentSceneGraph<Dtype, 4> {
    public:
        struct AtomicProposition : public common::Yamlable<AtomicProposition> {
            std::string type = "NA";
            int uuid = -1;
            Dtype reach_distance = -1.0f;

            ERL_REFLECT_SCHEMA(
                AtomicProposition,
                ERL_REFLECT_MEMBER(AtomicProposition, type),
                ERL_REFLECT_MEMBER(AtomicProposition, uuid),
                ERL_REFLECT_MEMBER(AtomicProposition, reach_distance));

            AtomicProposition() = default;

            AtomicProposition(std::string type, const int uuid, const Dtype reach_distance)
                : type(std::move(type)),
                  uuid(uuid),
                  reach_distance(reach_distance) {}
        };

        using Super = EnvironmentSceneGraph<Dtype, 4>;
        template<typename Key, typename Value>
        using HashMap = absl::flat_hash_map<Key, Value>;

        struct Setting : public common::Yamlable<Setting, typename Super::Setting> {
            HashMap<std::string, AtomicProposition> atomic_propositions;
            std::shared_ptr<FiniteStateAutomaton::Setting> fsa;  // finite state automaton

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, atomic_propositions),
                ERL_REFLECT_MEMBER(Setting, fsa));

            void
            LoadAtomicPropositions(const std::string &yaml_file) {
                YAML::Node node = YAML::LoadFile(yaml_file);
                YAML::convert<HashMap<std::string, AtomicProposition>>::decode(
                    node,
                    atomic_propositions);
            }
        };

        using State = typename Super::State;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using Successor_t = typename Super::Successor_t;

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_ = nullptr;
        HashMap<int, Eigen::MatrixX<uint32_t>> m_label_maps_;
        mutable HashMap<int, Eigen::MatrixX<std::vector<int>>> m_up_stairs_path_q_maps_;
        mutable HashMap<int, Eigen::MatrixX<std::vector<int>>> m_down_stairs_path_q_maps_;
        mutable HashMap<int, Eigen::MatrixX<std::vector<int>>> m_object_path_q_maps_;
        mutable HashMap<int, HashMap<int, Eigen::MatrixX<std::vector<int>>>> m_room_path_q_maps_;

    public:
        EnvironmentLTLSceneGraph(
            std::shared_ptr<scene_graph::Building> building,
            const std::shared_ptr<EnvironmentLTLSceneGraph::Setting> &setting)
            : Super(std::move(building), setting),
              m_setting_(setting) {

            m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
            GenerateLabelMaps();

            // initialize path_q maps
            for (auto &[floor_id, cost_map]: this->m_up_stairs_cost_maps_) {
                m_up_stairs_path_q_maps_[floor_id].resize(cost_map.rows(), cost_map.cols());
            }
            for (auto &[floor_id, cost_map]: this->m_down_stairs_cost_maps_) {
                m_down_stairs_path_q_maps_[floor_id].resize(cost_map.rows(), cost_map.cols());
            }
            for (auto &[obj_id, cost_map]: this->m_object_cost_maps_) {
                m_object_path_q_maps_[obj_id].resize(
                    cost_map.cost_map.rows(),
                    cost_map.cost_map.cols());
            }
            for (auto &[room1_id, cost_maps]: this->m_room_cost_maps_) {
                for (auto &[room2_id, cost_map]: cost_maps) {
                    m_room_path_q_maps_[room1_id][room2_id].resize(
                        cost_map.cost_map.rows(),
                        cost_map.cost_map.cols());
                }
            }
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<FiniteStateAutomaton>
        GetFiniteStateAutomaton() const {
            return m_fsa_;
        }

        [[nodiscard]] const HashMap<int, Eigen::MatrixX<uint32_t>> &
        GetLabelMaps() const {
            return m_label_maps_;
        }

        /**
         * @brief Get the trajectory starting from the given state and following the given action.
         * @param env_state
         * @param level index of the environment (resolution level) to use, 1-based
         * @param action_idx index of the action to apply
         * @return
         */
        [[nodiscard]] std::vector<State>
        ForwardActionAtLevel(const State &env_state, long level, long action_idx) const override {

            const int &cur_x = env_state.grid[0];
            const int &cur_y = env_state.grid[1];
            const int &cur_z = env_state.grid[2];
            const int &cur_q = env_state.grid[3];

            switch (static_cast<scene_graph::NodeType>(level - 1)) {
                case scene_graph::NodeType::kOcc: {  // atomic action
                    ERL_DEBUG("kOcc action id: {}", action_idx);
                    ERL_DEBUG_ASSERT(
                        action_idx >= 0 &&
                            static_cast<std::size_t>(action_idx) < this->m_atomic_actions_.size(),
                        "atomic_action_id is out of range.");

                    if (action_idx < this->m_floor_up_action_id_) {
                        // grid movement
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        nx = cur_x + this->m_atomic_actions_[action_idx].state_diff[0];  // next x
                        ny = cur_y + this->m_atomic_actions_[action_idx].state_diff[1];  // next y
                        nz = cur_z;                                                      // next z
                        nq = static_cast<int>(m_fsa_->GetNextState(
                            cur_q,
                            m_label_maps_.at(cur_z)(nx, ny)));  // next LTL state
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        return {next_env_state};
                    }
                    if (action_idx == this->m_floor_up_action_id_) {
                        // floor up
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        nz = cur_z + 1;  // go upstairs
                        auto &floor = this->m_scene_graph_->floors.at(nz);
                        nx = floor->down_stairs_portal.value()[0];
                        ny = floor->down_stairs_portal.value()[1];
                        nq = static_cast<int>(  //
                            m_fsa_->GetNextState(cur_q, m_label_maps_.at(nz)(nx, ny)));
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        return {next_env_state};
                    }
                    // floor down
                    State next_env_state;
                    int &nx = next_env_state.grid[0];
                    int &ny = next_env_state.grid[1];
                    int &nz = next_env_state.grid[2];
                    int &nq = next_env_state.grid[3];
                    nz = cur_z - 1;  // go downstairs
                    auto &floor = this->m_scene_graph_->floors.at(nz);
                    nx = floor->up_stairs_portal.value()[0];
                    ny = floor->up_stairs_portal.value()[1];
                    nq =
                        static_cast<int>(m_fsa_->GetNextState(cur_q, m_label_maps_.at(nz)(nx, ny)));
                    next_env_state.metric = GridToMetric(next_env_state.grid);
                    return {next_env_state};
                }
                case scene_graph::NodeType::kObject: {  // reach object
                    const auto &goal_object_id = action_idx;

                    ERL_DEBUG("kObject action id: {}", goal_object_id);
                    const auto &local_cost_map = this->m_object_cost_maps_.at(goal_object_id);
                    auto &path = local_cost_map.path_map(
                        cur_x - local_cost_map.grid_min_x,
                        cur_y - local_cost_map.grid_min_y);
                    return ConvertPath(path, cur_z, cur_q);
                }
                case scene_graph::NodeType::kRoom: {  // reach room
                    const auto &goal_room_id = action_idx;

                    const int &at_room_id =
                        this->m_room_maps_[cur_z].template at<int>(cur_x, cur_y);
                    ERL_DEBUG("kRoom action id: {}, at_room_id: {}", goal_room_id, at_room_id);
                    auto &local_cost_map = this->m_room_cost_maps_.at(at_room_id).at(goal_room_id);
                    auto &path = local_cost_map.path_map(
                        cur_x - local_cost_map.grid_min_x,
                        cur_y - local_cost_map.grid_min_y);
                    return ConvertPath(path, cur_z, cur_q);
                }
                case scene_graph::NodeType::kFloor: {  // floor up or down
                    const auto &goal_floor_num = action_idx;
                    ERL_DEBUG("kFloor action id: {}", goal_floor_num);
                    return GetPathToFloor(cur_x, cur_y, cur_z, cur_q, goal_floor_num);
                }
                case scene_graph::NodeType::kBuilding:
                    throw std::runtime_error("No action for building.");
                default:
                    throw std::runtime_error("Invalid action: unknown level.");
            }
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessorsAtLevel(const State &env_state, long level) const override {
            if (!this->InStateSpace(env_state)) { return {}; }
            if (level == 0) { return Super::GetSuccessors(env_state); }
            std::vector<Successor_t> successors;
            const int &cur_x = env_state.grid[0];
            const int &cur_y = env_state.grid[1];
            const int &cur_z = env_state.grid[2];
            const int &cur_q = env_state.grid[3];
            switch (static_cast<scene_graph::NodeType>(level - 1)) {
                case scene_graph::NodeType::kOcc: {
                    int num_actions = static_cast<int>(this->m_atomic_actions_.size()) - 2;
                    for (int atomic_action_id = 0; atomic_action_id < num_actions;
                         ++atomic_action_id) {
                        // grid movement
                        auto &atomic_action = this->m_atomic_actions_[atomic_action_id];
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        nx = cur_x + atomic_action.state_diff[0];  // next x
                        if (nx < 0 || nx >= this->m_grid_map_info_->Shape(0)) { continue; }
                        ny = cur_y + atomic_action.state_diff[1];  // next y
                        if (ny < 0 || ny >= this->m_grid_map_info_->Shape(1)) { continue; }
                        if (this->m_obstacle_maps_[cur_z].template at<uint8_t>(nx, ny) > 0) {
                            continue;
                        }
                        // next LTL state
                        nq = static_cast<int>(
                            m_fsa_->GetNextState(cur_q, m_label_maps_.at(cur_z)(nx, ny)));
                        if (m_fsa_->IsSinkState(nq)) { continue; }  // sink state
                        nz = cur_z;  // write nz only if nq is not a sink state
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        // room id missing, skip it.
                        if (this->m_room_maps_[cur_z].template at<int>(nx, ny) <= 0) { continue; }
                        successors.emplace_back(
                            next_env_state,
                            atomic_action.cost,
                            atomic_action_id,
                            static_cast<long>(scene_graph::NodeType::kOcc) + 1);
                    }
                    // going up/down-stairs only happens when the robot arrives at the stairs portal
                    auto &floor = this->m_scene_graph_->floors.at(cur_z);
                    int floor_num_up = cur_z + 1;
                    if (floor_num_up < this->m_scene_graph_->num_floors &&
                        floor->up_stairs_portal.has_value() &&
                        cur_x == floor->up_stairs_portal.value()[0] &&
                        cur_y == floor->up_stairs_portal.value()[1]) {  // go upstairs
                        auto &floor_up = this->m_scene_graph_->floors.at(floor_num_up);
                        ERL_ASSERTM(
                            floor_up->down_stairs_portal.has_value(),
                            "floor_up->down_stairs_portal should have value.");
                        const Dtype &cost = floor->up_stairs_cost;
                        if (!std::isinf(cost)) {
                            State next_env_state;
                            int &nq = next_env_state.grid[3];
                            std::vector<int> &dst_q =
                                m_up_stairs_path_q_maps_.at(cur_z)(cur_x, cur_y);
                            // initialize
                            if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                            nq = dst_q[cur_q];  // read from the cache
                            // path to the upstairs portal
                            auto &path = this->m_up_stairs_path_maps_.at(cur_z)(cur_x, cur_y);
                            if (nq < 0) {    // not computed yet
                                nq = cur_q;  // initialize
                                bool encounter_sink_state = false;
                                for (auto &point: path) {  // compute the next LTL state
                                    nq = static_cast<int>(m_fsa_->GetNextState(
                                        nq,
                                        m_label_maps_.at(cur_z)(point[0], point[1])));
                                    if (m_fsa_->IsSinkState(nq)) {
                                        encounter_sink_state = true;
                                        break;
                                    }
                                }
                                if (!encounter_sink_state) {  // one more step to go upstairs
                                    nq = static_cast<int>(m_fsa_->GetNextState(
                                        nq,
                                        m_label_maps_.at(floor_num_up)(
                                            floor_up->down_stairs_portal.value()[0],
                                            floor_up->down_stairs_portal.value()[1])));
                                }
                                dst_q[cur_q] = nq;  // write to the cache
                            }
                            if (!m_fsa_->IsSinkState(nq)) {
                                int &nx = next_env_state.grid[0];
                                int &ny = next_env_state.grid[1];
                                int &nz = next_env_state.grid[2];
                                nx = floor_up->down_stairs_portal.value()[0];
                                ny = floor_up->down_stairs_portal.value()[1];
                                nz = floor_num_up;
                                // nq is already computed or read from the cache
                                next_env_state.metric = GridToMetric(next_env_state.grid);
                                successors.emplace_back(
                                    next_env_state,
                                    cost,
                                    this->m_floor_up_action_id_,
                                    static_cast<long>(scene_graph::NodeType::kOcc) + 1);
                            }
                        }
                    }
                    if (int floor_num_down = cur_z - 1;
                        floor_num_down >= 0 && floor->down_stairs_portal.has_value() &&
                        cur_x == floor->down_stairs_portal.value()[0] &&
                        cur_y == floor->down_stairs_portal.value()[1]) {  // go downstairs
                        auto &floor_down = this->m_scene_graph_->floors.at(floor_num_down);
                        ERL_ASSERTM(
                            floor_down->up_stairs_portal.has_value(),
                            "floor_down->up_stairs_portal should have value.");
                        const Dtype &cost = floor->down_stairs_cost;
                        // no path to go downstairs (should not happen, but just in case)
                        if (std::isinf(cost)) { return successors; }
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        std::vector<int> &dst_q =
                            m_down_stairs_path_q_maps_.at(cur_z)(cur_x, cur_y);
                        if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                        nq = dst_q[cur_q];  // read from the cache
                        // path to the downstairs portal
                        auto &path = this->m_down_stairs_path_maps_.at(cur_z)(cur_x, cur_y);
                        if (nq < 0) {  // not computed yet
                            nq = cur_q;
                            bool encounter_sink_state = false;
                            for (auto &point: path) {
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(cur_z)(point[0], point[1])));
                                if (m_fsa_->IsSinkState(nq)) {
                                    encounter_sink_state = true;
                                    break;
                                }
                            }
                            if (!encounter_sink_state) {  // one more step to go downstairs
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(floor_num_down)(
                                        floor_down->up_stairs_portal.value()[0],
                                        floor_down->up_stairs_portal.value()[1])));
                            }
                            dst_q[cur_q] = nq;  // write to the cache
                        }
                        if (!m_fsa_->IsSinkState(nq)) {
                            nx = floor_down->up_stairs_portal.value()[0];
                            ny = floor_down->up_stairs_portal.value()[1];
                            nz = floor_num_down;
                            // nq is already computed or read from the cache
                            next_env_state.metric = GridToMetric(next_env_state.grid);
                            successors.emplace_back(
                                next_env_state,
                                cost,
                                this->m_floor_down_action_id_,
                                static_cast<long>(scene_graph::NodeType::kOcc) + 1);
                        }
                    }
                    return successors;
                }
                case scene_graph::NodeType::kObject: {
                    int at_room_id = this->m_room_maps_.at(cur_z).template at<int>(cur_x, cur_y);
                    auto &room = this->m_scene_graph_->id_to_room.at(at_room_id);
                    auto &reached_object_ids = this->m_object_reached_maps_.at(cur_z)(cur_x, cur_y);
                    // no object reached at this grid
                    if (reached_object_ids.empty()) { return successors; }
                    successors.reserve(room->objects.size() - reached_object_ids.size());
                    for (auto &itr: room->objects) {
                        const int &object_id = itr.first;  // try to reach this object
                        if (reached_object_ids.count(object_id) > 0) {
                            continue;
                        }  // already reached
                        auto &local_cost_map = this->m_object_cost_maps_.at(object_id);
                        ERL_DEBUG_ASSERT(
                            local_cost_map.grid_min_x <= cur_x &&
                                cur_x <= local_cost_map.grid_max_x,
                            "x is out of range.");
                        ERL_DEBUG_ASSERT(
                            local_cost_map.grid_min_y <= cur_y &&
                                cur_y <= local_cost_map.grid_max_y,
                            "y is out of range.");
                        int r = cur_x - local_cost_map.grid_min_x;
                        int c = cur_y - local_cost_map.grid_min_y;
                        auto &path = local_cost_map.path_map(r, c);  // path to reach the object
                        if (path.empty()) { continue; }              // no path to reach the object
                        const Dtype &cost =
                            local_cost_map.cost_map(r, c);  // cost to reach the object
                        // no path to reach the object (should not happen, but just in case)
                        if (std::isinf(cost)) { continue; }
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        std::vector<int> &dst_q = m_object_path_q_maps_.at(object_id)(r, c);
                        if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                        nq = dst_q[cur_q];             // read from the cache
                        if (nq < 0) {                  // not computed yet
                            nq = cur_q;                // initialize
                            for (auto &point: path) {  // compute the next LTL state
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(cur_z)(point[0], point[1])));
                                if (m_fsa_->IsSinkState(nq)) { break; }  // sink state
                            }
                            dst_q[cur_q] = nq;  // write to the cache
                        }
                        if (m_fsa_->IsSinkState(nq)) { continue; }  // sink state
                        nx = path.back()[0];
                        ny = path.back()[1];
                        nz = cur_z;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            object_id,
                            static_cast<long>(scene_graph::NodeType::kObject) + 1);
                    }
                    return successors;
                }
                case scene_graph::NodeType::kRoom: {
                    int at_room_id = this->m_room_maps_.at(cur_z).template at<int>(cur_x, cur_y);
                    auto connected_room_cost_maps_itr = this->m_room_cost_maps_.find(at_room_id);
                    if (connected_room_cost_maps_itr == this->m_room_cost_maps_.end()) {
                        return successors;  // no action for this room
                    }
                    auto &connected_room_cost_maps = connected_room_cost_maps_itr->second;
                    auto &room = this->m_scene_graph_->id_to_room.at(at_room_id);
                    ERL_DEBUG_ASSERT(
                        room->parent_id == cur_z,
                        "On {} floor but action is to reach room on {} floor.",
                        cur_z,
                        room->parent_id);
                    ERL_DEBUG_ASSERT(
                        room->id == at_room_id,
                        "In room {} but action is to reach room {}.",
                        at_room_id,
                        room->id);
                    successors.reserve(room->connected_room_ids.size());
                    for (int &connected_room_id: room->connected_room_ids) {
                        ERL_DEBUG_ASSERT(
                            this->m_room_maps_.at(cur_z).template at<int>(cur_x, cur_y) !=
                                connected_room_id,
                            "The current state is in the connected room.");
                        ERL_DEBUG_ASSERT(  // self-connected (loop)
                        connected_room_id != room->id,
                        "Room {} is connected to itself.",
                        room->id);
                        auto local_cost_map_itr = connected_room_cost_maps.find(connected_room_id);
                        // no action to go to this room
                        if (local_cost_map_itr == connected_room_cost_maps.end()) { continue; }
                        auto &local_cost_map = local_cost_map_itr->second;
                        int r = cur_x - local_cost_map.grid_min_x;
                        int c = cur_y - local_cost_map.grid_min_y;
                        auto &path = local_cost_map.path_map(r, c);
                        const Dtype &cost = local_cost_map.cost_map(r, c);
                        // no path to reach the room (should not happen, but just in case)
                        if (std::isinf(cost)) { continue; }
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        std::vector<int> &dst_q =
                            m_room_path_q_maps_.at(at_room_id).at(connected_room_id)(r, c);
                        if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                        nq = dst_q[cur_q];             // read from the cache
                        if (nq < 0) {                  // not computed yet
                            nq = cur_q;                // initialize
                            for (auto &point: path) {  // compute the next LTL state
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(cur_z)(point[0], point[1])));
                                if (m_fsa_->IsSinkState(nq)) { break; }  // sink state
                            }
                            dst_q[cur_q] = nq;  // write to the cache
                        }
                        if (m_fsa_->IsSinkState(nq)) { continue; }  // sink state
                        nx = path.back()[0];
                        ny = path.back()[1];
                        nz = cur_z;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            connected_room_id,
                            static_cast<long>(scene_graph::NodeType::kRoom) + 1);
                        ERL_DEBUG_ASSERT(
                            this->m_room_maps_.at(cur_z).template at<int>(nx, ny) ==
                                connected_room_id,
                            "The next state is not in the connected room.");
                    }
                    return successors;
                }
                case scene_graph::NodeType::kFloor: {
                    int floor_num_up = cur_z + 1;
                    int floor_num_down = cur_z - 1;
                    successors.reserve(2);
                    if (floor_num_up < this->m_scene_graph_->num_floors) {  // go upstairs
                        auto &floor_up = this->m_scene_graph_->floors.at(floor_num_up);
                        if (const Dtype cost = this->m_up_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                            !std::isinf(cost)) {
                            State next_env_state;
                            int &nx = next_env_state.grid[0];
                            int &ny = next_env_state.grid[1];
                            int &nz = next_env_state.grid[2];
                            int &nq = next_env_state.grid[3];
                            std::vector<int> &dst_q =
                                m_up_stairs_path_q_maps_.at(cur_z)(cur_x, cur_y);
                            if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                            nq = dst_q[cur_q];  // read from the cache
                            auto &path = this->m_up_stairs_path_maps_.at(
                                cur_z)(cur_x, cur_y);  // path to the upstairs portal
                            if (nq < 0) {              // not computed yet
                                nq = cur_q;            // initialize
                                bool encounter_sink_state = false;
                                for (auto &point: path) {  // compute the next LTL state
                                    nq = static_cast<int>(m_fsa_->GetNextState(
                                        nq,
                                        m_label_maps_.at(cur_z)(point[0], point[1])));
                                    if (m_fsa_->IsSinkState(nq)) {
                                        encounter_sink_state = true;
                                        break;
                                    }
                                }
                                if (!encounter_sink_state) {  // one more step to go upstairs
                                    nq = static_cast<int>(m_fsa_->GetNextState(
                                        nq,
                                        m_label_maps_.at(floor_num_up)(
                                            floor_up->down_stairs_portal.value()[0],
                                            floor_up->down_stairs_portal.value()[1])));
                                }
                                dst_q[cur_q] = nq;  // write to the cache
                            }
                            if (!m_fsa_->IsSinkState(nq)) {
                                nx = floor_up->down_stairs_portal.value()[0];
                                ny = floor_up->down_stairs_portal.value()[1];
                                nz = floor_num_up;
                                // nq is already computed or read from the cache
                                next_env_state.metric = GridToMetric(next_env_state.grid);
                                successors.emplace_back(
                                    next_env_state,
                                    cost,
                                    this->m_floor_up_action_id_,
                                    static_cast<long>(scene_graph::NodeType::kOcc) + 1);
                            }
                        }
                    }
                    if (floor_num_down >= 0) {  // go downstairs
                        auto &floor_down = this->m_scene_graph_->floors.at(floor_num_down);
                        const Dtype &cost = this->m_down_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                        if (std::isinf(cost)) { return successors; }
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        int &nq = next_env_state.grid[3];
                        std::vector<int> &dst_q =
                            m_down_stairs_path_q_maps_.at(cur_z)(cur_x, cur_y);
                        if (dst_q.empty()) { dst_q.resize(m_setting_->fsa->num_states, -1); }
                        nq = dst_q[cur_q];  // read from the cache
                        auto &path = this->m_down_stairs_path_maps_.at(cur_z)(cur_x, cur_y);
                        if (nq < 0) {  // not computed yet
                            nq = cur_q;
                            bool encounter_sink_state = false;
                            for (auto &point: path) {
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(cur_z)(point[0], point[1])));
                                if (m_fsa_->IsSinkState(nq)) {
                                    encounter_sink_state = true;
                                    break;
                                }
                            }
                            if (!encounter_sink_state) {  // one more step to go downstairs
                                nq = static_cast<int>(m_fsa_->GetNextState(
                                    nq,
                                    m_label_maps_.at(floor_num_down)(
                                        floor_down->up_stairs_portal.value()[0],
                                        floor_down->up_stairs_portal.value()[1])));
                            }
                            dst_q[cur_q] = nq;  // write to the cache
                        }
                        if (!m_fsa_->IsSinkState(nq)) {
                            nx = floor_down->up_stairs_portal.value()[0];
                            ny = floor_down->up_stairs_portal.value()[1];
                            nz = floor_num_down;
                            // nq is already computed or read from the cache
                            next_env_state.metric = GridToMetric(next_env_state.grid);
                            successors.emplace_back(
                                next_env_state,
                                cost,
                                this->m_floor_down_action_id_,
                                static_cast<long>(scene_graph::NodeType::kOcc) + 1);
                        }
                    }
                    return successors;
                }
                case scene_graph::NodeType::kBuilding:
                    throw std::runtime_error("No action for building.");
                default:
                    throw std::runtime_error("Invalid action: unknown level.");
            }
        }

        [[nodiscard]] uint32_t
        StateHashing(const State &env_state) const override {
            uint32_t hashing =
                this->m_grid_map_info_->GridToIndex(env_state.grid.template head<3>(), true);
            hashing = hashing * m_setting_->fsa->num_states + env_state.grid[3];
            return hashing;
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            GridState grid_state;
            grid_state[0] = this->m_grid_map_info_->MeterToGridAtDim(metric_state[0], 0);
            grid_state[1] = this->m_grid_map_info_->MeterToGridAtDim(metric_state[1], 1);
            grid_state[2] = static_cast<int>(metric_state[2]);
            grid_state[3] = static_cast<int>(metric_state[3]);
            return grid_state;
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            MetricState metric_state;
            metric_state[0] = this->m_grid_map_info_->GridToMeterAtDim(grid_state[0], 0);
            metric_state[1] = this->m_grid_map_info_->GridToMeterAtDim(grid_state[1], 1);
            metric_state[2] = static_cast<Dtype>(grid_state[2]);
            metric_state[3] = static_cast<Dtype>(grid_state[3]);
            return metric_state;
        }

    protected:
        void
        GenerateLabelMaps() {
            // initialize the label maps
            std::unordered_map<int, Eigen::MatrixX<std::bitset<32>>> label_maps = {};
            for (int i = 0; i < this->m_scene_graph_->num_floors; ++i) {
                label_maps[i].resize(
                    this->m_grid_map_info_->Shape(0),
                    this->m_grid_map_info_->Shape(1));
            }

            for (int i = 0; i < this->m_scene_graph_->num_floors; ++i) {  // each floor
                int rows = this->m_grid_map_info_->Shape(0);
                int cols = this->m_grid_map_info_->Shape(1);
                Eigen::MatrixX<std::bitset<32>> &label_map = label_maps[i];
#pragma omp parallel for collapse(2) default(none) shared(rows, cols, label_map, i)
                for (int r = 0; r < rows; ++r) {      // each row of the map
                    for (int c = 0; c < cols; ++c) {  // each column of the map
                        const auto &aps = m_setting_->atomic_propositions;
                        const auto &fsa_aps = m_setting_->fsa->atomic_propositions;
                        const std::size_t num_aps = fsa_aps.size();
                        auto &bitset = label_map(r, c);
                        for (std::size_t j = 0; j < num_aps; ++j) {
                            const AtomicProposition &proposition = aps.at(fsa_aps[j]);
                            bitset.set(j, EvaluateAtomicProposition(r, c, i, proposition));
                        }
                    }
                }
                m_label_maps_[i] = label_map.cast<uint32_t>();
            }
        }

        bool
        EvaluateAtomicProposition(
            int x,
            int y,
            int floor_num,
            const AtomicProposition &proposition) {
            if (proposition.type == "NA") { return false; }
            if (proposition.type == "EnterRoom") {
                return EvaluateEnterRoom(x, y, floor_num, proposition.uuid);
            }
            if (proposition.type == "ReachObject") {
                double reach_distance = proposition.reach_distance > 0
                                            ? proposition.reach_distance
                                            : m_setting_->object_reach_distance;
                return EvaluateReachObject(x, y, floor_num, proposition.uuid, reach_distance);
            }
            ERL_ERROR("Unknown atomic proposition type: {}", proposition.type);
            return false;
        }

        bool
        EvaluateEnterRoom(int x, int y, int floor_num, int uuid) {
            auto room = this->m_scene_graph_->template GetNode<scene_graph::Room>(uuid);
            return this->m_room_maps_[floor_num].template at<int>(x, y) == room->id;
        }

        bool
        EvaluateReachObject(int x, int y, int floor_num, int uuid, Dtype reach_distance) {
            auto half_rows =
                static_cast<int>(reach_distance / this->m_grid_map_info_->Resolution(0));
            if (half_rows == 0) { half_rows = 1; }
            auto half_cols =
                static_cast<int>(reach_distance / this->m_grid_map_info_->Resolution(1));
            if (half_cols == 0) { half_cols = 1; }

            int object_id = this->m_scene_graph_->template GetNode<scene_graph::Object>(uuid)->id;
            auto &cat_map = this->m_cat_maps_[floor_num];
            int roi_min_x = x - half_rows;
            int roi_min_y = y - half_cols;
            int roi_max_x = x + half_rows;
            int roi_max_y = y + half_cols;
            if (roi_min_x < 0) { roi_min_x = 0; }
            if (roi_min_y < 0) { roi_min_y = 0; }
            if (roi_max_x >= cat_map.rows) { roi_max_x = cat_map.rows - 1; }
            if (roi_max_y >= cat_map.cols) { roi_max_y = cat_map.cols - 1; }
            for (int r = roi_min_x; r <= roi_max_x; ++r) {
                for (int c = roi_min_y; c <= roi_max_y; ++c) {
                    if (cat_map.template at<int>(r, c) == object_id) { return true; }
                }
            }
            return false;
        }

    private:
        [[nodiscard]] std::vector<State>
        ConvertPath(const std::vector<Eigen::Vector2i> &path, int floor_num, int cur_q) const {
            std::vector<State> next_env_states;
            next_env_states.reserve(path.size() + 1);
            for (auto &point: path) {
                State next_env_state;
                int &nx = next_env_state.grid[0];
                int &ny = next_env_state.grid[1];
                int &nz = next_env_state.grid[2];
                int &nq = next_env_state.grid[3];
                nx = point[0];
                ny = point[1];
                nz = floor_num;
                nq = static_cast<int>(m_fsa_->GetNextState(cur_q, m_label_maps_.at(nz)(nx, ny)));
                cur_q = nq;  // update cur_q
                next_env_state.metric = GridToMetric(next_env_state.grid);
                next_env_states.push_back(next_env_state);
            }
            return next_env_states;
        }

        [[nodiscard]] std::vector<State>
        GetPathToFloor(int xg, int yg, int floor_num, int cur_q, int next_floor_num) const {
            ERL_DEBUG_ASSERT(
                std::abs(floor_num - next_floor_num) == 1,
                "floor_num and next_floor_num should differ by 1.");
            std::vector<State> next_env_states;
            if (floor_num < next_floor_num) {  // go upstairs
                auto &path = this->m_up_stairs_path_maps_.at(floor_num)(xg, yg);
                next_env_states = ConvertPath(path, floor_num, cur_q);
            } else {  // go downstairs
                auto &path = this->m_down_stairs_path_maps_.at(floor_num)(xg, yg);
                next_env_states = ConvertPath(path, floor_num, cur_q);
            }
            State next_env_state;
            auto &floor = this->m_scene_graph_->floors.at(next_floor_num);
            next_env_state.grid.resize(4);
            int &nx = next_env_state.grid[0];
            int &ny = next_env_state.grid[1];
            int &nz = next_env_state.grid[2];
            int &nq = next_env_state.grid[3];
            if (floor_num < next_floor_num) {  // go upstairs
                nx = floor->down_stairs_portal.value()[0];
                ny = floor->down_stairs_portal.value()[1];
                nz = next_floor_num;
            } else {  // go downstairs
                nx = floor->up_stairs_portal.value()[0];
                ny = floor->up_stairs_portal.value()[1];
                nz = next_floor_num;
            }
            if (!next_env_states.empty()) { cur_q = next_env_states.back().grid[3]; }
            nq = static_cast<int>(  //
                m_fsa_->GetNextState(cur_q, m_label_maps_.at(next_floor_num)(nx, ny)));
            next_env_state.metric = GridToMetric(next_env_state.grid);
            next_env_states.push_back(next_env_state);
            return next_env_states;
        }
    };

    extern template class EnvironmentLTLSceneGraph<float>;
    extern template class EnvironmentLTLSceneGraph<double>;
}  // namespace erl::env

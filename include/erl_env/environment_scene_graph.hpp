#pragma once

#include "environment_multi_resolution.hpp"
#include "scene_graph.hpp"

#include "erl_common/block_timer.hpp"
#include "erl_common/exception.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>
#include <boost/heap/d_ary_heap.hpp>

namespace erl::env {

    /**
     * Scene graph environment. The scene graph is a representation of the environment that
     * captures the hierarchical and semantic structure of the environment. The scene graph is
     * represented as a graph where nodes represent different levels of the environment (e.g., rooms
     * and objects) and edges represent the connectivity between these nodes (e.g., doors and
     * pathways). This environment supports multi-resolution planning, where the planner can operate
     * at different levels of abstraction in the scene graph.
     * @tparam Dtype data type of the metric space, float or double.
     * @tparam Dim dimension of the metric space, default is 3 for (x, y, floor_num). Dim > 3 is
     * allowed but the extra dimensions are ignored.
     */
    template<typename Dtype, int Dim = 3>
    class EnvironmentSceneGraph : public EnvironmentMultiResolution<Dtype, Dim> {

    public:
        using Super = EnvironmentMultiResolution<Dtype, Dim>;
        using State = typename Super::State;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using Successor_t = typename Super::Successor_t;

        struct Setting : public common::Yamlable<Setting> {
            std::string data_dir = {};   // folder of scene graph data, actions, cost maps, etc.
            long num_threads = 64;       // number of threads to use
            bool allow_diagonal = true;  // whether allow diagonal movement
            Dtype object_reach_distance = 0.6f;  // distance (meter) to reach an object
            Eigen::Matrix2X<Dtype> robot_metric_contour = {};  // the shape center is at the origin
            scene_graph::Node::Type max_level = scene_graph::Node::Type::kFloor;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting) {
                    YAML::Node node;
                    ERL_YAML_SAVE_ATTR(node, setting, data_dir);
                    ERL_YAML_SAVE_ATTR(node, setting, num_threads);
                    ERL_YAML_SAVE_ATTR(node, setting, allow_diagonal);
                    ERL_YAML_SAVE_ATTR(node, setting, object_reach_distance);
                    ERL_YAML_SAVE_ATTR(node, setting, robot_metric_contour);
                    ERL_YAML_SAVE_ATTR(node, setting, max_level);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, Setting &setting) {
                    if (!node.IsMap()) { return false; }
                    ERL_YAML_LOAD_ATTR(node, setting, data_dir);
                    ERL_YAML_LOAD_ATTR(node, setting, num_threads);
                    ERL_YAML_LOAD_ATTR(node, setting, allow_diagonal);
                    ERL_YAML_LOAD_ATTR(node, setting, object_reach_distance);
                    ERL_YAML_LOAD_ATTR(node, setting, robot_metric_contour);
                    ERL_YAML_LOAD_ATTR(node, setting, max_level);
                    return true;
                }
            };
        };

        struct AtomicAction {
            Dtype cost = 0.0;
            GridState state_diff = GridState::Zero();

            AtomicAction(Dtype cost_in, int dx, int dy, int dz)
                : cost(cost_in) {
                state_diff[0] = dx;
                state_diff[1] = dy;
                state_diff[2] = dz;
            }
        };

    protected:
        using PathMatrix = Eigen::MatrixX<std::vector<Eigen::Vector2i>>;
        using GridMapInfo = common::GridMapInfo3D<Dtype>;
        using CostMap = Eigen::MatrixX<Dtype>;
        template<typename Key, typename Value>
        using HashMap = absl::flat_hash_map<Key, Value>;

        struct LocalCostMap {
            int grid_min_x = 0;        // global x coordinate of the local cost map origin
            int grid_min_y = 0;        // global y coordinate of the local cost map origin
            int grid_max_x = 0;        // width of the local cost map
            int grid_max_y = 0;        // height of the local cost map
            CostMap cost_map = {};     // local cost map
            PathMatrix path_map = {};  // path map
        };

        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<scene_graph::Building> m_scene_graph_ = nullptr;
        std::shared_ptr<GridMapInfo> m_grid_map_info_ = nullptr;  // (x, y, floor_num), for hashing
        std::vector<cv::Mat> m_room_maps_ = {};                   // room maps for each floor
        std::vector<cv::Mat> m_cat_maps_ = {};                    // category maps for each floor
        std::vector<cv::Mat> m_ground_masks_ = {};   // ground masks for each floor, 0: is ground
        std::vector<cv::Mat> m_obstacle_maps_ = {};  // obstacle space maps, 0: free, >=1: obstacle
        HashMap<int, CostMap> m_up_stairs_cost_maps_ = {};       // floor -> upstairs cost
        HashMap<int, PathMatrix> m_up_stairs_path_maps_ = {};    // floor -> upstairs path
        HashMap<int, CostMap> m_down_stairs_cost_maps_ = {};     // floor -> downstairs cost
        HashMap<int, PathMatrix> m_down_stairs_path_maps_ = {};  // floor -> downstairs path
        // room -> cost to go from this room to each other room
        HashMap<int, HashMap<int, LocalCostMap>> m_room_cost_maps_ = {};
        // object reached maps for each floor: reached objects at each grid position
        HashMap<int, Eigen::MatrixX<std::unordered_set<int>>> m_object_reached_maps_ = {};
        // cost maps to reach each object, key: object id
        HashMap<int, LocalCostMap> m_object_cost_maps_ = {};
        std::vector<AtomicAction> m_atomic_actions_ = {};  // atomic actions
        long m_floor_up_action_id_ = 0;                    // atomic action id to go upstairs
        long m_floor_down_action_id_ = 0;                  // atomic action id to go downstairs

    public:
        explicit EnvironmentSceneGraph(
            std::shared_ptr<scene_graph::Building> scene_graph,
            std::shared_ptr<Setting> setting = nullptr)
            : Super(0),
              m_setting_(std::move(setting)),
              m_scene_graph_(std::move(scene_graph)) {
            ERL_ASSERTM(m_scene_graph_ != nullptr, "scene_graph should not be nullptr.");
            if (m_setting_ == nullptr) { m_setting_ = std::make_shared<Setting>(); }
            GenerateAtomicActions();
            LoadMaps();
        }

        template<typename T = Setting>
        [[nodiscard]] std::shared_ptr<T>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<scene_graph::Building>
        GetSceneGraph() const {
            return m_scene_graph_;
        }

        [[nodiscard]] std::shared_ptr<GridMapInfo>
        GetGridMapInfo() const {
            return m_grid_map_info_;
        }

        [[nodiscard]] const std::vector<cv::Mat> &
        GetRoomMaps() const {
            return m_room_maps_;
        }

        [[nodiscard]] const std::vector<cv::Mat> &
        GetCatMaps() const {
            return m_cat_maps_;
        }

        [[nodiscard]] const std::vector<cv::Mat> &
        GetGroundMasks() const {
            return m_ground_masks_;
        }

        [[nodiscard]] const std::vector<cv::Mat> &
        GetObstacleMaps() const {
            return m_obstacle_maps_;
        }

        [[nodiscard]] std::size_t
        GetNumResolutionLevels() const override {
            // num of non-anchor levels (max_level + 1) + 1 anchor level -> max_level + 2
            return static_cast<int>(m_setting_->max_level) + 2;
        }

        [[nodiscard]] std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] std::size_t
        GetActionSpaceSize() const override {
            /**
             * (level, goal_id)
             * level  | num of goal_id
             * floor  | 2
             * room   | m_scene_graph_->room_ids.size()
             * object | m_scene_graph_->object_ids.size()
             * atomic | m_atomic_actions_.size()
             */
            return 2 + m_scene_graph_->room_ids.size() + m_scene_graph_->object_ids.size() +
                   m_atomic_actions_.size();
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
            ERL_DEBUG(
                "forward action, x: {}, y: {}, floor_num: {}, cur_room_id: {}",
                cur_x,
                cur_y,
                cur_z,
                m_room_maps_[cur_z].at<int>(cur_x, cur_y));
            ERL_DEBUG_ASSERT(
                level >= 1 && level <= static_cast<long>(m_setting_->max_level) + 1,
                "level should be in [1, {}], but got {}.",
                static_cast<long>(m_setting_->max_level) + 1,
                level);

            switch (static_cast<scene_graph::Node::Type>(level - 1)) {
                case scene_graph::Node::Type::kOcc: {  // occupancy
                    ERL_DEBUG("kOcc action id: {}", action_idx);
                    ERL_DEBUG_ASSERT(
                        action_idx >= 0 && action_idx < static_cast<long>(m_atomic_actions_.size()),
                        "atomic_action_id is out of range.");

                    if (action_idx < m_floor_up_action_id_) {
                        State next_env_state;
                        next_env_state.grid =
                            env_state.grid + m_atomic_actions_[action_idx].state_diff;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        return {next_env_state};
                    }
                    if (action_idx == m_floor_up_action_id_) {
                        // floor up
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        int &nz = next_env_state.grid[2];
                        nz = cur_z + 1;  // go upstairs
                        const auto &floor = m_scene_graph_->floors.at(nz);
                        nx = floor->down_stairs_portal.value()[0];
                        ny = floor->down_stairs_portal.value()[1];
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        return {next_env_state};
                    }
                    // floor down
                    State next_env_state;
                    int &nx = next_env_state.grid[0];
                    int &ny = next_env_state.grid[1];
                    int &nz = next_env_state.grid[2];
                    nz = cur_z - 1;  // go downstairs
                    const auto &floor = m_scene_graph_->floors.at(nz);
                    nx = floor->up_stairs_portal.value()[0];
                    ny = floor->up_stairs_portal.value()[1];
                    next_env_state.metric = GridToMetric(next_env_state.grid);
                    return {next_env_state};
                }
                case scene_graph::Node::Type::kObject: {  // reach object
                    const auto &goal_object_id = action_idx;
                    ERL_DEBUG("kObject action id: {}", goal_object_id);
                    const LocalCostMap &local_cost_map = m_object_cost_maps_.at(goal_object_id);
                    auto &path = local_cost_map.path_map(
                        cur_x - local_cost_map.grid_min_x,
                        cur_y - local_cost_map.grid_min_y);
                    return ConvertPath(path, cur_z);
                }
                case scene_graph::Node::Type::kRoom: {  // reach room
                    const auto &goal_room_id = action_idx;
                    const int &room_id = m_room_maps_[cur_z].at<int>(cur_x, cur_y);
                    ERL_DEBUG("kRoom action id: {}, at_room_id: {}", goal_room_id, room_id);
                    const LocalCostMap &local_cost_map =
                        m_room_cost_maps_.at(room_id).at(goal_room_id);
                    auto &path = local_cost_map.path_map(
                        cur_x - local_cost_map.grid_min_x,
                        cur_y - local_cost_map.grid_min_y);
                    return ConvertPath(path, cur_z);
                }
                case scene_graph::Node::Type::kFloor: {  // floor up or down
                    const auto &goal_floor_num = action_idx;
                    ERL_DEBUG("kFloor action id: {}", goal_floor_num);
                    return GetPathToFloor(cur_x, cur_y, cur_z, goal_floor_num);
                }
                case scene_graph::Node::Type::kBuilding:
                    throw std::runtime_error("No action for building.");
                default:
                    throw std::runtime_error("Invalid action: unknown level.");
            }
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessors(const State &env_state) const override {
            std::vector<Successor_t> successors;
            successors.reserve(m_scene_graph_->object_ids.size() + m_scene_graph_->room_ids.size());
            for (auto level: {
                     scene_graph::Node::Type::kOcc,
                     scene_graph::Node::Type::kObject,
                     scene_graph::Node::Type::kRoom,
                     scene_graph::Node::Type::kFloor,
                 }) {
                if (level > m_setting_->max_level) { break; }
                std::vector<Successor_t> level_successors =
                    GetSuccessorsAtLevel(env_state, static_cast<std::size_t>(level) + 1);
                successors.insert(
                    successors.end(),
                    level_successors.begin(),
                    level_successors.end());
            }
            return successors;
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessorsAtLevel(const State &env_state, long level) const override {
            if (!InStateSpace(env_state)) { return {}; }
            if (level == 0) { return GetSuccessors(env_state); }
            const int &cur_x = env_state.grid[0];
            const int &cur_y = env_state.grid[1];
            const int &cur_z = env_state.grid[2];
            std::vector<Successor_t> successors;
            switch (static_cast<env::scene_graph::Node::Type>(level - 1)) {
                case scene_graph::Node::Type::kOcc: {
                    for (int atomic_action_id = 0; atomic_action_id < m_floor_up_action_id_;
                         ++atomic_action_id) {  // grid movement
                        auto &atomic_action = m_atomic_actions_[atomic_action_id];
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        nx = cur_x + atomic_action.state_diff[0];
                        if (nx < 0 || nx >= m_grid_map_info_->Shape(0)) { continue; }
                        ny = cur_y + atomic_action.state_diff[1];
                        if (ny < 0 || ny >= m_grid_map_info_->Shape(1)) { continue; }
                        if (m_obstacle_maps_[cur_z].at<uint8_t>(nx, ny) > 0) {
                            continue;
                        }  // obstacle
                        next_env_state.grid[2] = cur_z;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        // the room maps may have some small regions marked N/A due to the original
                        // mesh processing, we should skip it.
                        if (m_room_maps_[cur_z].at<int>(nx, ny) <= 0) { continue; }
                        successors.emplace_back(
                            next_env_state,
                            atomic_action.cost,
                            atomic_action_id,
                            static_cast<long>(scene_graph::Node::Type::kOcc) + 1);
                    }
                    // going up/down-stairs only happens when the robot arrives at the stairs portal
                    auto &floor = m_scene_graph_->floors.at(cur_z);
                    int floor_num_up = cur_z + 1;
                    if (floor_num_up < m_scene_graph_->num_floors &&
                        floor->up_stairs_portal.has_value() &&
                        cur_x == floor->up_stairs_portal.value()[0] &&
                        cur_y == floor->up_stairs_portal.value()[1]) {  // go upstairs
                        auto &floor_up = m_scene_graph_->floors.at(floor_num_up);
                        ERL_ASSERTM(
                            floor_up->down_stairs_portal.has_value(),
                            "floor_up->down_stairs_portal should have value.");
                        const Dtype cost = floor->up_stairs_cost;
                        if (!std::isinf(cost)) {
                            // no path to go upstairs (should not happen, but just in case)
                            State next_env_state;
                            next_env_state.grid[0] = floor_up->down_stairs_portal.value()[0];
                            next_env_state.grid[1] = floor_up->down_stairs_portal.value()[1];
                            next_env_state.grid[2] = floor_num_up;
                            next_env_state.metric = GridToMetric(next_env_state.grid);
                            successors.emplace_back(
                                next_env_state,
                                cost,
                                m_floor_up_action_id_,
                                static_cast<long>(scene_graph::Node::Type::kOcc) + 1);
                        }
                    }
                    int floor_num_down = cur_z - 1;
                    if (floor_num_down >= 0 && floor->down_stairs_portal.has_value() &&
                        cur_x == floor->down_stairs_portal.value()[0] &&
                        cur_y == floor->down_stairs_portal.value()[1]) {  // go downstairs
                        auto &floor_down = m_scene_graph_->floors.at(floor_num_down);
                        ERL_ASSERTM(
                            floor_down->up_stairs_portal.has_value(),
                            "floor_down->up_stairs_portal should have value.");
                        const Dtype cost = floor->down_stairs_cost;
                        // no path to go downstairs (should not happen, but just in case)
                        if (std::isinf(cost)) { return successors; }
                        State next_env_state;
                        next_env_state.grid[0] = floor_down->up_stairs_portal.value()[0];
                        next_env_state.grid[1] = floor_down->up_stairs_portal.value()[1];
                        next_env_state.grid[2] = floor_num_down;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            m_floor_down_action_id_,
                            static_cast<long>(scene_graph::Node::Type::kOcc) + 1);
                    }
                    return successors;
                }
                case scene_graph::Node::Type::kObject: {
                    int at_room_id = m_room_maps_.at(cur_z).at<int>(cur_x, cur_y);
                    auto &room = m_scene_graph_->id_to_room.at(at_room_id);
                    auto &reached_object_ids = m_object_reached_maps_.at(cur_z)(cur_x, cur_y);
                    if (reached_object_ids.empty()) {
                        return successors;
                    }  // no object reached at this grid
                    successors.reserve(room->objects.size() - reached_object_ids.size());
                    for (auto &itr: room->objects) {
                        const int &object_id = itr.first;  // reach this object
                        if (reached_object_ids.count(object_id) > 0) {
                            continue;
                        }  // already reached
                        auto &local_cost_map = m_object_cost_maps_.at(object_id);
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
                        if (path.empty()) { continue; }              // cannot reach the object
                        const Dtype cost =
                            local_cost_map.cost_map(r, c);   // cost to reach the object
                        if (std::isinf(cost)) { continue; }  // cannot reach the object
                        State next_env_state;
                        next_env_state.grid[0] = path.back()[0];
                        next_env_state.grid[1] = path.back()[1];
                        next_env_state.grid[2] = cur_z;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            object_id,
                            static_cast<long>(scene_graph::Node::Type::kObject) + 1);
                    }
                    return successors;
                }
                case scene_graph::Node::Type::kRoom: {
                    int at_room_id = m_room_maps_.at(cur_z).at<int>(cur_x, cur_y);
                    auto connected_room_cost_maps_itr = m_room_cost_maps_.find(at_room_id);
                    if (connected_room_cost_maps_itr == m_room_cost_maps_.end()) {
                        return successors;
                    }
                    auto &connected_room_cost_maps = connected_room_cost_maps_itr->second;
                    auto &room = m_scene_graph_->id_to_room.at(at_room_id);
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
                            m_room_maps_.at(cur_z).at<int>(cur_x, cur_y) != connected_room_id,
                            "The current state is in the connected room.");
                        ERL_DEBUG_ASSERT(  // self-connected (loop)
                        connected_room_id != room->id,
                        "Room {} is connected to itself.",
                        room->id);
                        auto local_cost_map_itr = connected_room_cost_maps.find(connected_room_id);
                        if (local_cost_map_itr == connected_room_cost_maps.end()) { continue; }
                        auto &local_cost_map = local_cost_map_itr->second;
                        int r = cur_x - local_cost_map.grid_min_x;
                        int c = cur_y - local_cost_map.grid_min_y;
                        auto &path = local_cost_map.path_map(r, c);
                        const Dtype &cost = local_cost_map.cost_map(r, c);
                        if (path.empty()) { continue; }  // cannot reach the room
                        State next_env_state;
                        int &nx = next_env_state.grid[0];
                        int &ny = next_env_state.grid[1];
                        nx = path.back()[0];
                        ny = path.back()[1];
                        ERL_DEBUG_ASSERT(
                            m_room_maps_.at(cur_z).at<int>(nx, ny) == connected_room_id,
                            "The next state is not in the connected room: {}, but in room: {}.",
                            connected_room_id,
                            m_room_maps_.at(cur_z).at<int>(nx, ny));
                        next_env_state.grid[2] = cur_z;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            connected_room_id,
                            static_cast<long>(scene_graph::Node::Type::kRoom) + 1);
                    }
                    return successors;
                }
                case scene_graph::Node::Type::kFloor: {
                    int floor_num_up = cur_z + 1;
                    int floor_num_down = cur_z - 1;
                    successors.reserve(2);
                    if (floor_num_up < m_scene_graph_->num_floors) {  // go upstairs
                        auto &floor = m_scene_graph_->floors.at(floor_num_up);
                        const Dtype &cost = m_up_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                        if (!std::isinf(cost)) {
                            State next_env_state;
                            next_env_state.grid[0] = floor->down_stairs_portal.value()[0];
                            next_env_state.grid[1] = floor->down_stairs_portal.value()[1];
                            next_env_state.grid[2] = floor_num_up;
                            next_env_state.metric = GridToMetric(next_env_state.grid);
                            successors.emplace_back(
                                next_env_state,
                                cost,
                                floor_num_up,
                                static_cast<long>(scene_graph::Node::Type::kFloor) + 1);
                        }
                    }
                    if (floor_num_down >= 0) {  // go downstairs
                        auto &floor = m_scene_graph_->floors.at(floor_num_down);
                        const Dtype &cost = m_down_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                        if (std::isinf(cost)) { return successors; }
                        State next_env_state;
                        next_env_state.grid[0] = floor->up_stairs_portal.value()[0];
                        next_env_state.grid[1] = floor->up_stairs_portal.value()[1];
                        next_env_state.grid[2] = floor_num_down;
                        next_env_state.metric = GridToMetric(next_env_state.grid);
                        successors.emplace_back(
                            next_env_state,
                            cost,
                            floor_num_down,
                            static_cast<long>(scene_graph::Node::Type::kFloor) + 1);
                    }
                    return successors;
                }
                case scene_graph::Node::Type::kBuilding:
                    throw std::runtime_error("No action for building.");
                default:
                    throw std::runtime_error("Invalid action: unknown level.");
            }
        }

        [[nodiscard]] bool
        InStateSpace(const State &env_state) const override {
            return m_grid_map_info_->InGrids(env_state.grid.template head<3>());
        }

        [[nodiscard]] bool
        InStateSpaceAtLevel(const State &env_state, long level) const override {
            if (!InStateSpace(env_state)) { return false; }
            if (level == 0) { return true; }

            switch (static_cast<scene_graph::Node::Type>(level - 1)) {
                case scene_graph::Node::Type::kObject:
                    return !m_object_reached_maps_
                                .at(env_state.grid[2])(env_state.grid[0], env_state.grid[1])
                                .empty();
                case scene_graph::Node::Type::kRoom:
                    return m_room_maps_[env_state.grid[2]].template at<int>(
                               env_state.grid[0],
                               env_state.grid[1]) > 0;
                case scene_graph::Node::Type::kOcc:
                case scene_graph::Node::Type::kFloor:
                case scene_graph::Node::Type::kBuilding:
                    return true;
                default:
                    throw std::runtime_error("Unknown level.");
            }
        }

        [[nodiscard]] uint32_t
        StateHashing(const State &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state.grid.template head<3>(), true);
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            GridState grid_state;
            grid_state[0] = m_grid_map_info_->MeterToGridAtDim(metric_state[0], 0);
            grid_state[1] = m_grid_map_info_->MeterToGridAtDim(metric_state[1], 1);
            grid_state[2] = static_cast<int>(metric_state[2]);
            return grid_state;
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            MetricState metric_state;
            metric_state[0] = m_grid_map_info_->GridToMeterAtDim(grid_state[0], 0);
            metric_state[1] = m_grid_map_info_->GridToMeterAtDim(grid_state[1], 1);
            metric_state[2] = static_cast<Dtype>(grid_state[2]);
            return metric_state;
        }

        // [[nodiscard]] cv::Mat
        // ShowPaths(const std::map<int, Eigen::MatrixXd> &, bool) const override {
        //     throw NotImplemented(__PRETTY_FUNCTION__);
        // }

        [[nodiscard]] std::vector<State>
        SampleValidStates(int /*num_samples*/) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

    protected:
        // virtual bool
        // LoadFromCache(const std::filesystem::path &cache_dir);
        //
        // virtual bool
        // SaveToCache(const std::filesystem::path &cache_dir);

        void
        LoadMaps() {
            const auto &floor = m_scene_graph_->floors[0];
            auto floor_grid_map_info = std::make_shared<common::GridMapInfo2D<Dtype>>(
                floor->grid_map_origin.cast<Dtype>(),
                floor->grid_map_resolution.cast<Dtype>(),
                floor->grid_map_size);

            // load room maps, category maps and obstacle maps, initialize object reach maps
            m_room_maps_.reserve(m_scene_graph_->num_floors);
            m_cat_maps_.reserve(m_scene_graph_->num_floors);
            for (int i = 0; i < m_scene_graph_->num_floors; ++i) {
                // room map, CV_32SC1
                m_room_maps_.push_back(m_scene_graph_->LoadRoomMap(m_setting_->data_dir, i));
                // category map, CV_32SC1
                m_cat_maps_.push_back(m_scene_graph_->LoadCatMap(m_setting_->data_dir, i));
                cv::Mat &cat_map = m_cat_maps_.back();
                // CV_8UC1
                m_ground_masks_.push_back(
                    cat_map != static_cast<int>(scene_graph::Object::SOC::kGround));
                // CV_8UC1, 0: free, >=1: obstacle
                m_obstacle_maps_.push_back(m_ground_masks_.back().clone());
                if (m_setting_->robot_metric_contour.cols() > 0) {
                    // inflate the obstacle map if the robot shape is given
                    cv::Mat mask_stairs_up =
                        cat_map != static_cast<int>(scene_graph::Object::SOC::kStairsUp);
                    cv::Mat mask_stairs_down =
                        cat_map != static_cast<int>(scene_graph::Object::SOC::kStairsDown);
                    cv::Mat mask_stairs = mask_stairs_up & mask_stairs_down;
                    cv::Mat &obstacle_map = m_obstacle_maps_.back();
                    obstacle_map &= mask_stairs;  // when inflate the map, do not inflate the stairs
                    // inflate the obstacle map
                    common::InflateWithShape(
                        obstacle_map,
                        floor_grid_map_info,
                        m_setting_->robot_metric_contour,
                        obstacle_map);
                    obstacle_map |= ~mask_stairs;  // remove the stairs
                }
            }

            GenerateFloorCostMaps();  // needed by kOcc and kFloor actions
            if (m_setting_->max_level >= scene_graph::Node::Type::kRoom) { GenerateRoomCostMaps(); }
            if (m_setting_->max_level >= scene_graph::Node::Type::kObject) {
                GenerateObjectCostMaps();
            }
        }

        void
        GenerateAtomicActions() {
            const auto &floor = m_scene_graph_->floors[0];
            Dtype floor_height = std::numeric_limits<Dtype>::infinity();
            if (m_scene_graph_->num_floors == 1) {
                // default height for one-floor building, which is not used in the heuristic
                floor_height = 2.0;
            } else {
                for (int i = 1; i < m_scene_graph_->num_floors; ++i) {
                    // get minimum height between two floors to make sure the heuristics is
                    // admissible
                    Dtype height = m_scene_graph_->floors[i]->ground_z -
                                   m_scene_graph_->floors[i - 1]->ground_z;
                    if (height < floor_height) { floor_height = height; }
                }
            }
            Eigen::Vector3<Dtype> grid_map_origin(
                floor->grid_map_origin.x(),
                floor->grid_map_origin.y(),
                floor->ground_z - 0.5 * floor_height);
            Eigen::Vector3<Dtype> grid_map_resolution(
                floor->grid_map_resolution.x(),
                floor->grid_map_resolution.y(),
                floor_height);
            Eigen::Vector3i grid_map_size(
                floor->grid_map_size.x(),
                floor->grid_map_size.y(),
                m_scene_graph_->num_floors);
            m_grid_map_info_ =
                std::make_shared<GridMapInfo>(grid_map_origin, grid_map_resolution, grid_map_size);

            Dtype x_res = m_grid_map_info_->Resolution(0);
            Dtype y_res = m_grid_map_info_->Resolution(1);
            if (m_setting_->allow_diagonal) {
                m_atomic_actions_.reserve(10);
                m_floor_up_action_id_ = 8;
                m_floor_down_action_id_ = 9;
                for (int i = -1; i <= 1; ++i) {
                    for (int j = -1; j <= 1; ++j) {
                        if (i == j && i == 0) { continue; }
                        Dtype dx = i * x_res;
                        Dtype dy = j * y_res;
                        m_atomic_actions_.emplace_back(std::sqrt(dx * dx + dy * dy), i, j, 0);
                    }
                }
            } else {
                m_atomic_actions_.reserve(6);
                m_floor_up_action_id_ = 4;
                m_floor_down_action_id_ = 5;
                for (int i: {-1, 1}) {
                    m_atomic_actions_.emplace_back(std::abs(x_res), i, 0, 0);
                    m_atomic_actions_.emplace_back(std::abs(y_res), 0, i, 0);
                }
            }
            m_atomic_actions_.emplace_back(-1.0f, 0, 0, 1);   // floor up
            m_atomic_actions_.emplace_back(-1.0f, 0, 0, -1);  // floor down
        }

        void
        GenerateFloorCostMaps() {
            ERL_BLOCK_TIMER();

            if (m_scene_graph_->num_floors <= 1) { return; }

            int grid_map_rows = m_grid_map_info_->Shape(0);
            int grid_map_cols = m_grid_map_info_->Shape(1);
            for (int floor_num = 1; floor_num < m_scene_graph_->num_floors; ++floor_num) {
                // initialize cost maps
                int n1 = floor_num - 1;
                int n2 = floor_num;
                ERL_ASSERTM(
                    m_scene_graph_->floors[n1]->up_stairs_portal.has_value(),
                    "Floor {} does not have up stairs portal.",
                    n1);
                ERL_ASSERTM(
                    m_scene_graph_->floors[n2]->down_stairs_portal.has_value(),
                    "Floor {} does not have down stairs portal.",
                    n2);
                m_up_stairs_cost_maps_[floor_num - 1].resize(grid_map_rows, grid_map_cols);
                m_up_stairs_path_maps_[floor_num - 1].resize(grid_map_rows, grid_map_cols);
                m_down_stairs_cost_maps_[floor_num].resize(grid_map_rows, grid_map_cols);
                m_down_stairs_path_maps_[floor_num].resize(grid_map_rows, grid_map_cols);
            }

#pragma omp parallel for default(none)   \
    shared(m_scene_graph_,               \
               m_obstacle_maps_,         \
               m_up_stairs_cost_maps_,   \
               m_up_stairs_path_maps_,   \
               m_down_stairs_cost_maps_, \
               m_down_stairs_path_maps_)
            for (int floor_num = 1; floor_num < m_scene_graph_->num_floors; ++floor_num) {
                // initialize cost maps
                int n1 = floor_num - 1;
                auto &floor1 = m_scene_graph_->floors[n1];
                Eigen::Vector2i goal(
                    floor1->up_stairs_portal.value()[0],
                    floor1->up_stairs_portal.value()[1]);
                cv::Mat &obstacle_map1 = m_obstacle_maps_[n1];
                obstacle_map1.at<uint8_t>(goal[0], goal[1]) = 0;  // set the goal to be free
                CostMap &cost_map1 = m_up_stairs_cost_maps_[n1];
                ReverseAStar(goal, obstacle_map1, cost_map1, m_up_stairs_path_maps_[n1]);
                for (int r = 0; r < obstacle_map1.rows; ++r) {
                    for (int c = 0; c < obstacle_map1.cols; ++c) {
                        Dtype &cost = cost_map1(r, c);
                        if (std::isinf(cost)) { continue; }
                        cost += floor1->up_stairs_cost;
                    }
                }

                int n2 = floor_num;
                auto &floor2 = m_scene_graph_->floors[n2];
                goal[0] = floor2->down_stairs_portal.value()[0];
                goal[1] = floor2->down_stairs_portal.value()[1];
                cv::Mat &obstacle_map2 = m_obstacle_maps_[n2];
                obstacle_map2.at<uint8_t>(goal[0], goal[1]) = 0;  // set the goal to be free
                CostMap &cost_map2 = m_down_stairs_cost_maps_[n2];
                ReverseAStar(goal, obstacle_map2, cost_map2, m_down_stairs_path_maps_[n2]);
                for (int r = 0; r < obstacle_map2.rows; ++r) {
                    for (int c = 0; c < obstacle_map2.cols; ++c) {
                        Dtype &cost = cost_map2(r, c);
                        if (std::isinf(cost)) { continue; }
                        cost += floor2->down_stairs_cost;
                    }
                }
            }
        }

        void
        GenerateRoomCostMaps() {
            ERL_BLOCK_TIMER();

            m_room_cost_maps_.reserve(m_scene_graph_->room_ids.size());
            for (int room_id: m_scene_graph_->room_ids) {  // initialize room cost maps
                auto &room = m_scene_graph_->id_to_room[room_id];
                if (room->name == "staircase") { continue; }
                m_room_cost_maps_[room_id].reserve(room->connected_room_ids.size());
                for (int connected_room_id: room->connected_room_ids) {
                    if (m_scene_graph_->id_to_room[connected_room_id]->name == "staircase") {
                        continue;
                    }
                    m_room_cost_maps_[room_id][connected_room_id] = {};
                }
            }

#pragma omp parallel for default(none) shared(m_scene_graph_, m_obstacle_maps_, m_room_cost_maps_)
            for (int room_id: m_scene_graph_->room_ids) {
                auto &room = m_scene_graph_->id_to_room.at(room_id);
                auto connected_room_cost_maps_itr = m_room_cost_maps_.find(room_id);
                if (connected_room_cost_maps_itr == m_room_cost_maps_.end()) { continue; }
                auto &connected_room_cost_maps = connected_room_cost_maps_itr->second;
                cv::Mat obstacle_map = m_obstacle_maps_[room->parent_id];
                for (int connected_room_id: room->connected_room_ids) {
                    auto local_cost_map_itr = connected_room_cost_maps.find(connected_room_id);
                    if (local_cost_map_itr == connected_room_cost_maps.end()) { continue; }
                    LocalCostMap &local_cost_map = local_cost_map_itr->second;
                    local_cost_map.grid_min_x = room->grid_map_min.x();
                    local_cost_map.grid_min_y = room->grid_map_min.y();
                    local_cost_map.grid_max_x = room->grid_map_max.x();
                    local_cost_map.grid_max_y = room->grid_map_max.y();
                    auto &door_grids = room->door_grids.at(connected_room_id);
                    long max_num_goals = door_grids.cols();
                    Eigen::Matrix2Xi goals(2, max_num_goals);
                    long num_goals = 0;
                    for (long i = 0; i < max_num_goals; ++i) {
                        int &r = door_grids(0, i);
                        int &c = door_grids(1, i);
                        if (obstacle_map.at<uint8_t>(r, c) > 0) { continue; }  // obstacle
                        goals(0, num_goals) = r;
                        goals(1, num_goals) = c;
                        ++num_goals;
                        // obstacle_map.at<int>(r, c) = 0;  // set the goal to be free
                        if (r < local_cost_map.grid_min_x) { local_cost_map.grid_min_x = r; }
                        if (r > local_cost_map.grid_max_x) { local_cost_map.grid_max_x = r; }
                        if (c < local_cost_map.grid_min_y) { local_cost_map.grid_min_y = c; }
                        if (c > local_cost_map.grid_max_y) { local_cost_map.grid_max_y = c; }
                    }
                    goals.conservativeResize(2, num_goals);
                    for (long i = 0; i < num_goals; ++i) {
                        goals(0, i) -= local_cost_map.grid_min_x;
                        goals(1, i) -= local_cost_map.grid_min_y;
                    }

                    cv::Rect2i room_roi(
                        cv::Point2i(local_cost_map.grid_min_y, local_cost_map.grid_min_x),
                        cv::Point2i(local_cost_map.grid_max_y + 1, local_cost_map.grid_max_x + 1));
                    cv::Mat obstacle_map_roi = obstacle_map(room_roi);
                    local_cost_map.cost_map.setConstant(
                        obstacle_map_roi.rows,
                        obstacle_map_roi.cols,
                        std::numeric_limits<Dtype>::infinity());
                    local_cost_map.path_map.resize(obstacle_map_roi.rows, obstacle_map_roi.cols);
                    ReverseAStar(
                        goals,
                        obstacle_map_roi,
                        local_cost_map.cost_map,
                        local_cost_map.path_map);

                    for (int r = 0; r < obstacle_map_roi.rows; ++r) {
                        for (int c = 0; c < obstacle_map_roi.cols; ++c) {
                            if (std::isinf(local_cost_map.cost_map(r, c))) { continue; }
                            auto &path = local_cost_map.path_map(r, c);
                            for (auto &p: path) {
                                p[0] += local_cost_map.grid_min_x;
                                p[1] += local_cost_map.grid_min_y;
                            }
                        }
                    }
                }
            }
        }

        void
        GenerateObjectCostMaps() {
            ERL_BLOCK_TIMER();

            m_object_cost_maps_.reserve(m_scene_graph_->object_ids.size());
            for (int floor_num = 0; floor_num < m_scene_graph_->num_floors; ++floor_num) {
                m_object_reached_maps_[floor_num].resize(
                    m_grid_map_info_->Shape(0),
                    m_grid_map_info_->Shape(1));
            }
            // initialize object cost maps
            for (int object_id: m_scene_graph_->object_ids) { m_object_cost_maps_[object_id] = {}; }
#pragma omp parallel for default(none) \
    shared(m_scene_graph_, m_room_maps_, m_obstacle_maps_, m_cat_maps_, m_object_cost_maps_)
            for (int object_id: m_scene_graph_->object_ids) {
                int row_padding =
                    static_cast<int>(
                        m_setting_->object_reach_distance / m_grid_map_info_->Resolution(0)) +
                    1;
                int col_padding =
                    static_cast<int>(
                        m_setting_->object_reach_distance / m_grid_map_info_->Resolution(1)) +
                    1;
                auto kernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(2 * row_padding + 1, 2 * col_padding + 1));
                auto &object = m_scene_graph_->id_to_object[object_id];
                auto &room = m_scene_graph_->id_to_room[object->parent_id];
                int floor_id = room->parent_id;
                cv::Mat obstacle_map = m_obstacle_maps_[floor_id];
                auto &local_cost_map = m_object_cost_maps_[object_id];
                local_cost_map.grid_min_x = room->grid_map_min.x();
                local_cost_map.grid_min_y = room->grid_map_min.y();
                local_cost_map.grid_max_x = room->grid_map_max.x();
                local_cost_map.grid_max_y = room->grid_map_max.y();
                int x, y;
                x = object->grid_map_min[0] - row_padding;
                if (x < local_cost_map.grid_min_x) { local_cost_map.grid_min_x = x; }
                y = object->grid_map_min[1] - col_padding;
                if (y < local_cost_map.grid_min_y) { local_cost_map.grid_min_y = y; }
                x = object->grid_map_max[0] + row_padding;
                if (x > local_cost_map.grid_max_x) { local_cost_map.grid_max_x = x; }
                y = object->grid_map_max[1] + col_padding;
                if (y > local_cost_map.grid_max_y) { local_cost_map.grid_max_y = y; }
                if (local_cost_map.grid_min_x < 0) { local_cost_map.grid_min_x = 0; }
                if (local_cost_map.grid_min_y < 0) { local_cost_map.grid_min_y = 0; }
                x = obstacle_map.rows - 1;
                if (local_cost_map.grid_max_x > x) { local_cost_map.grid_max_x = x; }
                y = obstacle_map.cols - 1;
                if (local_cost_map.grid_max_y > y) { local_cost_map.grid_max_y = y; }

                // obstacle map
                cv::Rect2i room_roi(
                    cv::Point2i(local_cost_map.grid_min_y, local_cost_map.grid_min_x),
                    cv::Point2i(local_cost_map.grid_max_y + 1, local_cost_map.grid_max_x + 1));
                cv::Mat obstacle_map_roi = obstacle_map(room_roi);

                // get grids that reach the object
                cv::Mat object_seg_mask =
                    m_cat_maps_[floor_id](room_roi) ==
                    object_id;  // CV_8UC1, 0: object not reached, 1: object reached
                cv::dilate(
                    object_seg_mask,
                    object_seg_mask,
                    kernel);  // dilate the object segmentation mask
                int max_num_goals = object_seg_mask.rows * object_seg_mask.cols;
                Eigen::Matrix2Xi goals(2, max_num_goals);
                cv::Mat room_map_roi = m_room_maps_[floor_id](room_roi);
                Eigen::MatrixX<std::unordered_set<int>> &object_reached_map =
                    m_object_reached_maps_[floor_id];
                int num_goals = 0;
                for (int row = 0; row < object_seg_mask.rows; ++row) {
                    for (int col = 0; col < object_seg_mask.cols; ++col) {
                        if (room_map_roi.at<int>(row, col) != room->id) {
                            object_seg_mask.at<uint8_t>(row, col) = 0;  // not in the same room
                            continue;
                        }
                        if (obstacle_map_roi.at<uint8_t>(row, col) > 0) { continue; }  // obstacle
                        uint8_t v0 = object_seg_mask.at<uint8_t>(row, col);
                        if (v0 == 0) { continue; }  // object not reached

                        {
#pragma omp critical
                            object_reached_map(
                                row + local_cost_map.grid_min_x,
                                col + local_cost_map.grid_min_y)
                                .insert(object_id);
                        }

                        bool on_boundary = false;
                        if (row < object_seg_mask.rows - 1) {
                            uint8_t v1 = object_seg_mask.at<uint8_t>(row + 1, col);
                            if (v1 != v0) { on_boundary = true; }
                        }
                        if (row > 0) {
                            uint8_t v1 = object_seg_mask.at<uint8_t>(row - 1, col);
                            if (v1 != v0) { on_boundary = true; }
                        }
                        if (col < object_seg_mask.cols - 1) {
                            uint8_t v1 = object_seg_mask.at<uint8_t>(row, col + 1);
                            if (v1 != v0) { on_boundary = true; }
                        }
                        if (col > 0) {
                            uint8_t v1 = object_seg_mask.at<uint8_t>(row, col - 1);
                            if (v1 != v0) { on_boundary = true; }
                        }
                        if (!on_boundary) { continue; }

                        goals(0, num_goals) = row;
                        goals(1, num_goals++) = col;
                    }
                }
                goals.conservativeResize(2, num_goals);

                // compute cost maps
                local_cost_map.cost_map.setConstant(
                    obstacle_map_roi.rows,
                    obstacle_map_roi.cols,
                    std::numeric_limits<Dtype>::infinity());
                local_cost_map.path_map.resize(obstacle_map_roi.rows, obstacle_map_roi.cols);
                ReverseAStar(
                    goals,
                    obstacle_map_roi,
                    local_cost_map.cost_map,
                    local_cost_map.path_map);

                for (int r = 0; r < obstacle_map_roi.rows; ++r) {
                    for (int c = 0; c < obstacle_map_roi.cols; ++c) {
                        if (std::isinf(local_cost_map.cost_map(r, c))) { continue; }
                        auto &path = local_cost_map.path_map(r, c);
                        for (auto &p: path) {
                            p[0] += local_cost_map.grid_min_x;
                            p[1] += local_cost_map.grid_min_y;
                        }
                    }
                }
            }
        }

        void
        ReverseAStar(
            const Eigen::Ref<Eigen::Matrix2Xi> &goals,
            const cv::Mat &obstacle_map,
            CostMap &cost_map,
            PathMatrix &path_map) const {
            Eigen::MatrixX<std::vector<uint32_t>> action_map;  // empty
            Eigen::MatrixXi goal_index_map;                    // empty
            ReverseAStar(goals, obstacle_map, cost_map, path_map, action_map, goal_index_map);
        }

        /**
         * @brief reverse A* search to compute the cost of a composite action
         * @param goals
         * @param obstacle_map
         * @param cost_map
         * @param path_map
         * @param action_map
         * @param goal_index_map
         * @return
         */
        void
        ReverseAStar(
            const Eigen::Ref<Eigen::Matrix2Xi> &goals,
            const cv::Mat &obstacle_map,
            CostMap &cost_map,
            PathMatrix &path_map,
            Eigen::MatrixX<std::vector<uint32_t>> &action_map,
            Eigen::MatrixXi &goal_index_map) const {

            ERL_ASSERTM(
                cost_map.rows() == obstacle_map.rows,
                "cost_map.rows() != obstacle_map.rows");
            ERL_ASSERTM(
                cost_map.cols() == obstacle_map.cols,
                "cost_map.cols() != obstacle_map.cols");
            cost_map.setConstant(std::numeric_limits<Dtype>::infinity());
            long num_goals = goals.cols();
            long goal_index = 0;
            Eigen::Matrix2Xi reachable_goals(2, num_goals);
            for (long i = 0; i < num_goals; ++i) {
                const int &r = goals(0, i);
                const int &c = goals(1, i);
                if (r < 0 || r >= obstacle_map.rows) { continue; }     // out of bound
                if (c < 0 || c >= obstacle_map.cols) { continue; }     // out of bound
                if (obstacle_map.at<uint8_t>(r, c) > 0) { continue; }  // obstacle
                reachable_goals(0, goal_index) = r;
                reachable_goals(1, goal_index++) = c;
            }
            if (goal_index == 0) { return; }  // no reachable goal
            reachable_goals.conservativeResize(2, goal_index);
            num_goals = goal_index;

            struct Node;

            struct QueueItem {
                Dtype f_value = std::numeric_limits<Dtype>::infinity();
                std::shared_ptr<Node> node = nullptr;
                QueueItem() = default;

                QueueItem(Dtype f, std::shared_ptr<Node> n)
                    : f_value(f),
                      node(std::move(n)) {}
            };

            struct Node {
                int x_grid = 0;
                int y_grid = 0;
                std::shared_ptr<Node> parent = nullptr;
                uint32_t action_id = -1;  // action id from parent to this node, also used as goal
                                          // index if this node is a goal
                Dtype g_value = std::numeric_limits<Dtype>::infinity();
                Dtype h_value = std::numeric_limits<Dtype>::infinity();
            };

            struct Greater {
                bool
                operator()(
                    const std::shared_ptr<QueueItem> &s1,
                    const std::shared_ptr<QueueItem> &s2) const {
                    if (std::abs(s1->f_value - s2->f_value) < 1.e-6) {
                        // f value is too close, compare g value
                        return s1->node->g_value > s2->node->g_value;
                    }
                    return s1->f_value > s2->f_value;
                }
            };

            using PriorityQueue = boost::heap::d_ary_heap<
                std::shared_ptr<QueueItem>,
                boost::heap::mutable_<true>,
                boost::heap::arity<8>,
                boost::heap::compare<Greater>>;

            Eigen::MatrixXb closed =
                Eigen::MatrixXb::Constant(obstacle_map.rows, obstacle_map.cols, false);
            Eigen::MatrixXb opened =
                Eigen::MatrixXb::Constant(obstacle_map.rows, obstacle_map.cols, false);
            Eigen::MatrixX<std::shared_ptr<Node>> nodes(obstacle_map.rows, obstacle_map.cols);
            Eigen::MatrixX<typename PriorityQueue::handle_type> heap_keys(
                obstacle_map.rows,
                obstacle_map.cols);
            PriorityQueue queue;

            auto heuristic_func = [&reachable_goals, &num_goals](int x, int y) -> Dtype {
                Dtype h = std::numeric_limits<Dtype>::infinity();
                for (long i = 0; i < num_goals; ++i) {
                    Dtype dx = x - reachable_goals(0, i);
                    Dtype dy = y - reachable_goals(1, i);
                    Dtype d = dx * dx + dy * dy;
                    if (d < h) { h = d; }
                }
                return std::sqrt(h);
            };

            // initialize start nodes
            for (long i = 0; i < num_goals; ++i) {
                auto node = std::make_shared<Node>();
                node->x_grid = reachable_goals(0, i);
                node->y_grid = reachable_goals(1, i);
                node->action_id = i;
                node->g_value = 0;
                node->h_value = 0;
                nodes(node->x_grid, node->y_grid) = node;
                heap_keys(node->x_grid, node->y_grid) =
                    queue.push(std::make_shared<QueueItem>(node->g_value + node->h_value, node));
                cost_map(node->x_grid, node->y_grid) = 0;
            }

            // start search
            // we do search within the obstacle map of a specific floor, so we don't use the floor
            // up/down atomic actions, which are used when generating composite actions
            while (!queue.empty()) {
                auto node = queue.top()->node;
                queue.pop();
                opened(node->x_grid, node->y_grid) = false;
                closed(node->x_grid, node->y_grid) = true;
                uint32_t n = m_setting_->allow_diagonal ? 8 : 4;
                for (uint32_t i = 0; i < n; ++i) {
                    auto &action = m_atomic_actions_[i];
                    int x_grid = node->x_grid + action.state_diff[0];
                    if (x_grid < 0 || x_grid >= obstacle_map.rows) { continue; }  // out of boundary
                    int y_grid = node->y_grid + action.state_diff[1];
                    if (y_grid < 0 || y_grid >= obstacle_map.cols) { continue; }  // out of boundary
                    if (obstacle_map.at<uint8_t>(x_grid, y_grid) > 0) { continue; }  // obstacle
                    if (closed(x_grid, y_grid)) { continue; }
                    Dtype tentative_g = node->g_value + action.cost;
                    Dtype &g_value = cost_map(x_grid, y_grid);
                    if (tentative_g >= g_value) { continue; }
                    g_value = tentative_g;
                    if (opened(x_grid, y_grid)) {
                        auto &heap_key = heap_keys(x_grid, y_grid);
                        (*heap_key)->node->parent = node;
                        (*heap_key)->node->action_id = i;
                        (*heap_key)->f_value = tentative_g + heuristic_func(x_grid, y_grid);
                        queue.increase(heap_key);
                    } else {
                        auto child = std::make_shared<Node>();
                        child->x_grid = x_grid;
                        child->y_grid = y_grid;
                        child->parent = node;
                        child->action_id = i;
                        child->g_value = tentative_g;
                        child->h_value = heuristic_func(x_grid, y_grid);
                        nodes(x_grid, y_grid) = child;
                        heap_keys(x_grid, y_grid) = queue.push(
                            std::make_shared<QueueItem>(child->g_value + child->h_value, child));
                        opened(x_grid, y_grid) = true;
                    }
                }
            }

            bool get_path = path_map.size() > 0;
            bool get_action = action_map.size() > 0;
            bool get_goal_index = goal_index_map.size() > 0;
            if (!get_path && !get_action && !get_goal_index) { return; }

            // compute path map / action map / goal index map
            int n_reserve = std::max(obstacle_map.rows, obstacle_map.cols);
            for (int r = 0; r < obstacle_map.rows; ++r) {
                for (int c = 0; c < obstacle_map.cols; ++c) {
                    auto &node = nodes(r, c);
                    if (node == nullptr) { continue; }
                    std::vector<Eigen::Vector2i> path;
                    if (get_path) {
                        path.clear();
                        path.reserve(n_reserve);
                    }
                    std::vector<uint32_t> actions;
                    if (get_action) {
                        actions.clear();
                        actions.reserve(n_reserve);
                    }
                    while (node->parent != nullptr) {
                        if (get_path) {
                            path.emplace_back(Eigen::Vector2i{node->x_grid, node->y_grid});
                        }
                        if (get_action) { actions.emplace_back(node->action_id); }
                        node = node->parent;
                    }
                    if (get_path) {
                        path.emplace_back(Eigen::Vector2i{node->x_grid, node->y_grid});
                        path.resize(path.size());
                        path_map(r, c) = std::move(path);
                    }
                    if (get_action) {
                        actions.resize(actions.size());
                        action_map(r, c) = std::move(actions);
                    }
                    if (get_goal_index) {
                        goal_index_map(r, c) = static_cast<int>(node->action_id);
                    }
                }
            }
        }

    private:
        [[nodiscard]] std::vector<State>
        ConvertPath(const std::vector<Eigen::Vector2i> &path, const int floor_num) const {
            std::vector<State> next_env_states;
            next_env_states.reserve(path.size() + 1);
            for (auto &point: path) {
                State next_env_state;
                next_env_state.grid[0] = point[0];
                next_env_state.grid[1] = point[1];
                next_env_state.grid[2] = floor_num;
                next_env_state.metric = GridToMetric(next_env_state.grid);
                next_env_states.push_back(next_env_state);
            }
            return next_env_states;
        }

        [[nodiscard]] std::vector<State>
        GetPathToFloor(const int xg, const int yg, const int floor_num, const int next_floor_num)
            const {
            ERL_DEBUG_ASSERT(
                std::abs(floor_num - next_floor_num) == 1,
                "floor_num and next_floor_num should differ by 1.");
            std::vector<State> next_env_states;
            if (floor_num < next_floor_num) {  // go upstairs
                auto &path = m_up_stairs_path_maps_.at(floor_num)(xg, yg);
                next_env_states = ConvertPath(path, floor_num);
            } else {  // go downstairs
                auto &path = m_down_stairs_path_maps_.at(floor_num)(xg, yg);
                next_env_states = ConvertPath(path, floor_num);
            }
            State next_env_state;
            auto &floor = m_scene_graph_->floors.at(next_floor_num);
            if (floor_num < next_floor_num) {  // go upstairs
                next_env_state.grid[0] = floor->down_stairs_portal.value()[0];
                next_env_state.grid[1] = floor->down_stairs_portal.value()[1];
            } else {  // go downstairs
                next_env_state.grid[0] = floor->up_stairs_portal.value()[0];
                next_env_state.grid[1] = floor->up_stairs_portal.value()[1];
            }
            next_env_state.grid[2] = floor->id;
            next_env_state.metric = GridToMetric(next_env_state.grid);
            next_env_states.push_back(next_env_state);
            return next_env_states;
        }
    };

    extern template class EnvironmentSceneGraph<float, 3>;
    extern template class EnvironmentSceneGraph<double, 3>;
    extern template class EnvironmentSceneGraph<float, 4>;
    extern template class EnvironmentSceneGraph<double, 4>;
}  // namespace erl::env

template<>
struct YAML::convert<erl::env::EnvironmentSceneGraph<float>::Setting>
    : public erl::env::EnvironmentSceneGraph<float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSceneGraph<double>::Setting>
    : public erl::env::EnvironmentSceneGraph<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSceneGraph<float, 4>::Setting>
    : public erl::env::EnvironmentSceneGraph<float, 4>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSceneGraph<double, 4>::Setting>
    : public erl::env::EnvironmentSceneGraph<double, 4>::Setting::YamlConvertImpl {};

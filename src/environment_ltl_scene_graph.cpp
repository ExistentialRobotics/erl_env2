#include "erl_env/environment_ltl_scene_graph.hpp"

namespace Eigen::internal {

    template<>
    struct cast_impl<std::bitset<32>, uint32_t> {
        EIGEN_DEVICE_FUNC
        static inline uint32_t
        run(const std::bitset<32> &x) {
            return static_cast<uint32_t>(x.to_ulong());
        }
    };

    template<>
    struct cast_impl<uint32_t, std::bitset<32>> {
        EIGEN_DEVICE_FUNC
        static inline std::bitset<32>
        run(const uint32_t &x) {
            return {x};
        }
    };
}  // namespace Eigen::internal

namespace erl::env {

    std::vector<std::shared_ptr<EnvironmentState>>
    EnvironmentLTLSceneGraph::ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
        // TODO: implement this
        auto level = env::scene_graph::Node::Type(action_coords[0]);
        const int &cur_x = env_state->grid[0];
        const int &cur_y = env_state->grid[1];
        const int &cur_z = env_state->grid[2];
        const int &cur_q = env_state->grid[3];
        switch (level) {
            case scene_graph::Node::Type::kNA: {  // atomic action
                const int &atomic_action_id = action_coords[1];
                ERL_DEBUG("kNA action id: %d", atomic_action_id);
                ERL_DEBUG_ASSERT(atomic_action_id >= 0 && std::size_t(atomic_action_id) < m_atomic_actions_.size(), "atomic_action_id is out of range.");
                if (std::size_t(atomic_action_id) < m_atomic_actions_.size() - 2) {  // grid movement
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);  // x, y, z, q
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = cur_x + m_atomic_actions_[atomic_action_id].state_diff[0];          // next x
                    ny = cur_y + m_atomic_actions_[atomic_action_id].state_diff[1];          // next y
                    nz = cur_z;                                                              // next z
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(cur_z)(nx, ny)));  // next LTL state
                    next_env_state->metric = GridToMetric(next_env_state->grid);
                    return {next_env_state};
                } else if (std::size_t(atomic_action_id) == m_atomic_actions_.size() - 2) {  // floor up
                    return GetPathToFloor(cur_x, cur_y, cur_z, cur_q, cur_z + 1);
                } else if (std::size_t(atomic_action_id) == m_atomic_actions_.size() - 1) {  // floor down
                    return GetPathToFloor(cur_x, cur_y, cur_z, cur_q, cur_z - 1);
                } else {
                    throw std::runtime_error("Invalid atomic action.");
                }
            }
            case scene_graph::Node::Type::kObject: {  // reach object
                const int &goal_object_id = action_coords[1];
                ERL_DEBUG("kObject action id: %d", goal_object_id);
                const LocalCostMap &local_cost_map = m_object_cost_maps_.at(goal_object_id);
#ifndef NDEBUG
                auto &object = m_scene_graph_->id_to_object[goal_object_id];
                const int &room_id = object->parent_id;
                auto &room = m_scene_graph_->id_to_room[room_id];
                ERL_ASSERTM(room->parent_id == cur_z, "On %d floor but action is to reach object on %d floor.", cur_z, room->parent_id);
                const int &at_room_id = m_room_maps_[cur_z].at<int>(cur_x, cur_y);
                ERL_ASSERTM(at_room_id == room_id, "In room %d but action is to reach object in room %d.", at_room_id, room_id);
                ERL_ASSERTM(
                    (cur_x >= local_cost_map.grid_min_x && cur_x < local_cost_map.grid_max_x) &&
                        (cur_y >= local_cost_map.grid_min_y && cur_y < local_cost_map.grid_max_y),
                    "Not in the local cost map of object (id: %d) to reach.",
                    goal_object_id);
#endif
                auto &path = local_cost_map.path_map(cur_x - local_cost_map.grid_min_x, cur_y - local_cost_map.grid_min_y);
                return ConvertPath(path, cur_z, cur_q);
            }
            case scene_graph::Node::Type::kRoom: {  // reach room
                const int &goal_room_id = action_coords[1];
                const int &at_room_id = m_room_maps_[cur_z].at<int>(cur_x, cur_y);
                ERL_DEBUG("kRoom action id: %d, at_room_id: %d", goal_room_id, at_room_id);
                const LocalCostMap &local_cost_map = m_room_cost_maps_.at(at_room_id).at(goal_room_id);
#ifndef NDEBUG
                auto &room = m_scene_graph_->id_to_room[goal_room_id];
                ERL_ASSERTM(room->parent_id == cur_z, "On %d floor but action is to reach room on %d floor.", cur_z, room->parent_id);
                ERL_ASSERTM(
                    (cur_x >= local_cost_map.grid_min_x && cur_x < local_cost_map.grid_max_x) &&
                        (cur_y >= local_cost_map.grid_min_y && cur_y < local_cost_map.grid_max_y),
                    "Not in the local cost map of room (id: %d) to take the action.",
                    goal_room_id);
#endif
                auto &path = local_cost_map.path_map(cur_x - local_cost_map.grid_min_x, cur_y - local_cost_map.grid_min_y);
                return ConvertPath(path, cur_z, cur_q);
            }
            case scene_graph::Node::Type::kFloor: {  // floor up or down
                const int &goal_floor_num = action_coords[1];
                ERL_DEBUG("kFloor action id: %d", goal_floor_num);
                return GetPathToFloor(cur_x, cur_y, cur_z, cur_q, goal_floor_num);
            }
            case scene_graph::Node::Type::kBuilding:
                throw std::runtime_error("No action for building.");
            default:
                throw std::runtime_error("Invalid action: unknown level.");
        }
    }

    std::vector<Successor>
    EnvironmentLTLSceneGraph::GetSuccessorsAtLevel(const std::shared_ptr<EnvironmentState> &env_state, std::size_t resolution_level) const {
        if (!InStateSpace(env_state)) { return {}; }
        if (resolution_level == 0) { return EnvironmentSceneGraph::GetSuccessors(env_state); }
        auto level = env::scene_graph::Node::Type(resolution_level - 1);
        std::vector<Successor> successors;
        int &cur_x = env_state->grid[0];
        int &cur_y = env_state->grid[1];
        int &cur_z = env_state->grid[2];
        int &cur_q = env_state->grid[3];
        switch (level) {
            case scene_graph::Node::Type::kNA: {
                int num_actions = int(m_atomic_actions_.size()) - 2;
                for (int atomic_action_id = 0; atomic_action_id < num_actions; ++atomic_action_id) {  // grid movement
                    auto &atomic_action = m_atomic_actions_[atomic_action_id];
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);  // x, y, z, q
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = cur_x + atomic_action.state_diff[0];                                // next x
                    if (nx < 0 || nx >= m_grid_map_info_->Shape(0)) { continue; }            // out of map boundary
                    ny = cur_y + atomic_action.state_diff[1];                                // next y
                    if (ny < 0 || ny >= m_grid_map_info_->Shape(1)) { continue; }            // out of map boundary
                    if (m_obstacle_maps_[cur_z].at<uint8_t>(nx, ny) > 0) { continue; }       // obstacle
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(cur_z)(nx, ny)));  // next LTL state
                    if (m_fsa_->IsSinkState(nq)) { continue; }                               // sink state
                    nz = cur_z;                                                              // write nz only if nq is not a sink state
                    next_env_state->metric = GridToMetric(next_env_state->grid);
                    // the room maps may have some small regions marked N/A due to the original mesh processing, we fix it here.
                    int &next_room_id = const_cast<int &>(m_room_maps_[cur_z].at<int>(nx, ny));
                    if (next_room_id <= 0) { next_room_id = m_room_maps_[cur_z].at<int>(cur_x, cur_y); }  // room id missing, fix it. This should rarely happen.
                    successors.emplace_back(next_env_state, atomic_action.cost, std::vector<int>{int(scene_graph::Node::Type::kNA), atomic_action_id});
                }
                auto &floor = m_scene_graph_->floors.at(cur_z);
                int floor_num_up = cur_z + 1;
                if (floor_num_up < m_scene_graph_->num_floors) {  // go upstairs
                    auto &floor_up = m_scene_graph_->floors.at(floor_num_up);
                    ERL_ASSERTM(floor->up_stairs_portal.has_value(), "floor->up_stairs_portal should have value.");
                    ERL_ASSERTM(floor_up->down_stairs_portal.has_value(), "floor_up->down_stairs_portal should have value.");
                    const double &cost = m_up_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = floor_up->down_stairs_portal.value()[0];
                    ny = floor_up->down_stairs_portal.value()[1];
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(floor_num_up)(nx, ny)));
                    if (!m_fsa_->IsSinkState(nq)) {
                        nz = floor_num_up;  // write nz only if nq is not a sink state
                        next_env_state->metric = GridToMetric(next_env_state->grid);
                        successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kNA), m_floor_up_action_id_});
                    }
                }
                int floor_num_down = cur_z - 1;
                if (floor_num_down >= 0) {  // go downstairs
                    auto &floor_down = m_scene_graph_->floors.at(floor_num_down);
                    ERL_ASSERTM(floor->down_stairs_portal.has_value(), "floor->down_stairs_portal should have value.");
                    ERL_ASSERTM(floor_down->up_stairs_portal.has_value(), "floor_down->up_stairs_portal should have value.");
                    const double &cost = m_down_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);  // cost includes the cost of going downstairs
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = floor_down->up_stairs_portal.value()[0];
                    ny = floor_down->up_stairs_portal.value()[1];
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(floor_num_down)(nx, ny)));
                    if (!m_fsa_->IsSinkState(nq)) {
                        nz = floor_num_down;  // write nz only if nq is not a sink state
                        next_env_state->metric = GridToMetric(next_env_state->grid);
                        successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kNA), m_floor_down_action_id_});
                    }
                }
                return successors;
            }
            case scene_graph::Node::Type::kObject: {
                int at_room_id = m_room_maps_.at(cur_z).at<int>(cur_x, cur_y);
                auto &room = m_scene_graph_->id_to_room.at(at_room_id);
                auto &reached_object_ids = m_object_reached_maps_.at(cur_z)(cur_x, cur_y);
                if (reached_object_ids.empty()) { return successors; }  // no object reached at this grid
                successors.reserve(room->objects.size() - reached_object_ids.size());
                for (auto &itr: room->objects) {
                    const int &object_id = itr.first;                           // try to reach this object
                    if (reached_object_ids.count(object_id) > 0) { continue; }  // already reached
                    auto &local_cost_map = m_object_cost_maps_.at(object_id);
                    ERL_DEBUG_ASSERT(local_cost_map.grid_min_x <= cur_x && cur_x <= local_cost_map.grid_max_x, "x is out of range.");
                    ERL_DEBUG_ASSERT(local_cost_map.grid_min_y <= cur_y && cur_y <= local_cost_map.grid_max_y, "y is out of range.");
                    int r = cur_x - local_cost_map.grid_min_x;
                    int c = cur_y - local_cost_map.grid_min_y;
                    auto &path = local_cost_map.path_map(r, c);          // path to reach the object
                    if (path.empty()) { continue; }                      // no path to reach the object
                    const double &cost = local_cost_map.cost_map(r, c);  // cost to reach the object
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = path.back()[0];
                    ny = path.back()[1];
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(cur_z)(nx, ny)));
                    if (m_fsa_->IsSinkState(nq)) { continue; }  // sink state
                    nz = cur_z;                                 // write nz only if nq is not a sink state
                    next_env_state->metric = GridToMetric(next_env_state->grid);
                    successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kObject), object_id});
                }
                return successors;
            }
            case scene_graph::Node::Type::kRoom: {
                int at_room_id = m_room_maps_.at(cur_z).at<int>(cur_x, cur_y);
                auto connected_room_cost_maps_itr = m_room_cost_maps_.find(at_room_id);
                if (connected_room_cost_maps_itr == m_room_cost_maps_.end()) { return successors; }  // no action for this room
                auto &connected_room_cost_maps = connected_room_cost_maps_itr->second;
                auto &room = m_scene_graph_->id_to_room.at(at_room_id);
                ERL_DEBUG_ASSERT(room->parent_id == cur_z, "On %d floor but action is to reach room on %d floor.", cur_z, room->parent_id);
                ERL_DEBUG_ASSERT(room->id == at_room_id, "In room %d but action is to reach room %d.", at_room_id, room->id);
                successors.reserve(room->connected_room_ids.size());
                for (int &connected_room_id: room->connected_room_ids) {
                    ERL_DEBUG_ASSERT(m_room_maps_.at(cur_z).at<int>(cur_x, cur_y) != connected_room_id, "The current state is in the connected room.");
                    ERL_DEBUG_ASSERT(connected_room_id != room->id, "Room %d is connected to itself.", room->id);  // self-connected (loop)
                    auto local_cost_map_itr = connected_room_cost_maps.find(connected_room_id);
                    if (local_cost_map_itr == connected_room_cost_maps.end()) { continue; }  // no action to go to this room
                    auto &local_cost_map = local_cost_map_itr->second;
                    int r = cur_x - local_cost_map.grid_min_x;
                    int c = cur_y - local_cost_map.grid_min_y;
                    auto &path = local_cost_map.path_map(r, c);
                    const double &cost = local_cost_map.cost_map(r, c);
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = path.back()[0];
                    ny = path.back()[1];
                    ERL_DEBUG_ASSERT(m_room_maps_.at(cur_z).at<int>(nx, ny) == connected_room_id, "The next state is not in the connected room.");
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(cur_z)(nx, ny)));
                    if (m_fsa_->IsSinkState(nq)) { continue; }  // sink state
                    nz = cur_z;                                 // write nz only if nq is not a sink state
                    next_env_state->metric = GridToMetric(next_env_state->grid);
                    successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kRoom), connected_room_id});
                }
                return successors;
            }
            case scene_graph::Node::Type::kFloor: {
                int floor_num_up = cur_z + 1;
                int floor_num_down = cur_z - 1;
                successors.reserve(2);
                if (floor_num_up < m_scene_graph_->num_floors) {  // go upstairs
                    auto &floor = m_scene_graph_->floors.at(floor_num_up);
                    const double &cost = m_up_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = floor->down_stairs_portal.value()[0];
                    ny = floor->down_stairs_portal.value()[1];
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(floor_num_up)(nx, ny)));
                    if (!m_fsa_->IsSinkState(nq)) {
                        nz = floor_num_up;  // write nz only if nq is not a sink state
                        next_env_state->metric = GridToMetric(next_env_state->grid);
                        successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kFloor), floor_num_up});
                    }
                }
                if (floor_num_down >= 0) {  // go downstairs
                    auto &floor = m_scene_graph_->floors.at(floor_num_down);
                    const double &cost = m_down_stairs_cost_maps_.at(cur_z)(cur_x, cur_y);
                    auto next_env_state = std::make_shared<EnvironmentState>();
                    next_env_state->grid.resize(4);
                    int &nx = next_env_state->grid[0];
                    int &ny = next_env_state->grid[1];
                    int &nz = next_env_state->grid[2];
                    int &nq = next_env_state->grid[3];
                    nx = floor->up_stairs_portal.value()[0];
                    ny = floor->up_stairs_portal.value()[1];
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(floor_num_down)(nx, ny)));
                    if (!m_fsa_->IsSinkState(nq)) {
                        nz = floor_num_down;  // write nz only if nq is not a sink state
                        next_env_state->metric = GridToMetric(next_env_state->grid);
                        successors.emplace_back(next_env_state, cost, std::vector<int>{int(scene_graph::Node::Type::kFloor), floor_num_down});
                    }
                }
                return successors;
            }
            case scene_graph::Node::Type::kBuilding:
                throw std::runtime_error("No action for building.");
            default:
                throw std::runtime_error("Invalid action: unknown level.");
        }
    }

    void
    EnvironmentLTLSceneGraph::GenerateLabelMaps() {
        auto t0 = std::chrono::high_resolution_clock::now();

        // initialize the label maps
        std::unordered_map<int, Eigen::MatrixX<std::bitset<32>>> label_maps = {};
        for (int i = 0; i < m_scene_graph_->num_floors; ++i) { label_maps[i].resize(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)); }

        for (int i = 0; i < m_scene_graph_->num_floors; ++i) {  // each floor
            int rows = m_grid_map_info_->Shape(0);
#pragma omp parallel for default(none) shared(rows, label_maps, i)
            for (int r = 0; r < rows; ++r) {  // each row of the map
                int cols = m_grid_map_info_->Shape(1);
                for (int c = 0; c < cols; ++c) {  // each column of the map
                    std::size_t num_propositions = m_setting_->fsa->atomic_propositions.size();
                    auto &bitset = label_maps[i](r, c);
                    for (std::size_t j = 0; j < num_propositions; ++j) {  // each atomic proposition
                        auto &proposition = m_setting_->atomic_propositions[m_setting_->fsa->atomic_propositions[j]];
                        bitset.set(j, EvaluateAtomicProposition(r, c, i, proposition));
                    }
                }
            }
            m_label_maps_[i] = label_maps[i].cast<uint32_t>();
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        ERL_INFO("GenerateLabelMaps: %f ms", std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

}  // namespace erl::env

#pragma once

#include "cost.hpp"
#include "ddc_motion_primitive.hpp"
#include "environment_base.hpp"

#include "erl_common/exception.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"

#include <map>

namespace erl::env {

    template<typename Dtype, typename MapDtype = uint8_t>
    class EnvironmentSe2 : public EnvironmentBase<Dtype, 3> {

    public:
        struct Setting : public common::Yamlable<Setting> {
            Dtype time_step = 0.05;
            std::vector<DdcMotionPrimitive<Dtype>> motion_primitives;
            int num_orientations = 16;
            Dtype cost_theta_weight = 0.0;
            MapDtype obstacle_threshold = 1;  // minimum map value to be considered as obstacle
            bool add_map_cost = false;    // indicate whether to add map cost to the successor cost
            Dtype map_cost_factor = 1.0;  // map cost = map_cost_factor * map_cost
            Eigen::Matrix2X<Dtype> robot_metric_contour;  // robot shape in metric space

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting) {
                    YAML::Node node;
                    ERL_YAML_SAVE_ATTR(node, setting, time_step);
                    ERL_YAML_SAVE_ATTR(node, setting, motion_primitives);
                    ERL_YAML_SAVE_ATTR(node, setting, num_orientations);
                    ERL_YAML_SAVE_ATTR(node, setting, cost_theta_weight);
                    ERL_YAML_SAVE_ATTR(node, setting, obstacle_threshold);
                    ERL_YAML_SAVE_ATTR(node, setting, add_map_cost);
                    ERL_YAML_SAVE_ATTR(node, setting, map_cost_factor);
                    ERL_YAML_SAVE_ATTR(node, setting, robot_metric_contour);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, Setting &setting) {
                    if (!node.IsMap()) { return false; }
                    ERL_YAML_LOAD_ATTR(node, setting, time_step);
                    ERL_YAML_LOAD_ATTR(node, setting, motion_primitives);
                    ERL_YAML_LOAD_ATTR(node, setting, num_orientations);
                    ERL_YAML_LOAD_ATTR(node, setting, cost_theta_weight);
                    ERL_YAML_LOAD_ATTR(node, setting, obstacle_threshold);
                    ERL_YAML_LOAD_ATTR(node, setting, add_map_cost);
                    ERL_YAML_LOAD_ATTR(node, setting, map_cost_factor);
                    ERL_YAML_LOAD_ATTR(node, setting, robot_metric_contour);
                    return true;
                }
            };
        };

        using Cost = CostBase<Dtype, 3>;
        using Se2Cost_t = Se2Cost<Dtype>;
        using GridMap2D = common::GridMap<MapDtype, Dtype, 2>;
        using GridMapInfo = common::GridMapInfo3D<Dtype>;
        using State = EnvironmentState<Dtype, 3>;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using MetricTrajectory = Eigen::Matrix3X<Dtype>;
        using Successor_t = Successor<Dtype, 3>;

    protected:
        std::shared_ptr<Setting> m_setting_;
        std::shared_ptr<GridMapInfo> m_grid_map_info_;

        std::shared_ptr<GridMap2D> m_grid_map_ext_;
        cv::Mat m_original_grid_map_;
        std::vector<cv::Mat> m_inflated_grid_maps_;

        // [theta][motion_primitive][control][trajectory_waypoint_index]
        std::vector<std::vector<std::vector<std::vector<State>>>> m_rel_trajectories_;
        Se2Cost_t m_distance_cost_func_;

        union action_index_t {
            struct {
                int motion_idx;
                int control_idx;
            };

            long index;

            explicit action_index_t(const int motion_idx_in = -1, const int control_idx_in = -1)
                : motion_idx(motion_idx_in), control_idx(control_idx_in) {}
        };

        /*
         * for each orientation
         *     for each unique relative successor (distinguished by grid state hashing)
         *         controls that lead to the successor, sorted by cost (ascending)
         */
        struct RelSuccessorInfo {
            State rel_state;
            std::vector<std::size_t> orders;
            std::vector<int> motion_indices;
            std::vector<int> control_indices;
            std::vector<action_index_t> action_indices;
            std::vector<Dtype> costs;
        };

        std::vector<std::vector<RelSuccessorInfo>> m_rel_successors_;

    public:
        EnvironmentSe2(
            const std::shared_ptr<GridMap2D> &grid_map,
            std::shared_ptr<Setting> setting): EnvironmentBase<Dtype, 3>(),
              m_setting_(NotNull(std::move(setting), true,"setting is nullptr.")),
              m_grid_map_info_(
                  std::make_shared<GridMapInfo>(
                      grid_map->info->Extend(m_setting_->num_orientations, -M_PI, M_PI, 2))),
                m_grid_map_ext_(NotNull(grid_map, true, "grid_map is nullptr.")),
            m_original_grid_map_(  // external storage from grid_map
                m_grid_map_info_->Shape(0),
                m_grid_map_info_->Shape(1),
                common::CvMatType<MapDtype, 1>(),
                m_grid_map_ext_->data.Data().data()){

            m_distance_cost_func_.w_theta = m_setting_->cost_theta_weight;

            /*
             * Part 1
             * 1. calculate max travel distance
             * 2. calculate metric relative trajectories
             * 3. calculate trajectory costs
             */

            // The motion primitives may generate trajectories that go outside the map. We need to
            // determine the furthest distance and create a new grid_map_info that is large enough
            // to contain all the trajectories.
            Dtype max_distance = 0;
            // process motion primitives and prepare metric relative trajectories
            // starting from [0, 0, 0]
            // [motion_primitive][control][3, num_states]
            std::vector<std::vector<MetricTrajectory>> metric_rel_trajs;
            std::vector<std::vector<Dtype>> trajectory_costs;  // [motion_idx][control_idx]
            trajectory_costs.reserve(m_setting_->motion_primitives.size());
            metric_rel_trajs.reserve(m_setting_->motion_primitives.size());
            for (auto &motion: m_setting_->motion_primitives) {
                // convert incremental cost to cumulative cost
                std::partial_sum(motion.costs.begin(), motion.costs.end(), motion.costs.begin());
                // compute metric relative trajectory segments
                metric_rel_trajs.push_back(motion.ComputeTrajectorySegments(
                    MetricState::Zero(),
                    m_setting_->time_step,
                    MotionModel));
                // compute the maximum distance of the trajectory segments
                std::vector<Dtype> segment_costs;
                segment_costs.reserve(metric_rel_trajs.back().size());
                for (MetricTrajectory &segment: metric_rel_trajs.back()) {
                    Dtype segment_cost = 0;
                    for (long i = 0; i < segment.cols(); ++i) {
                        Dtype distance = (segment.col(i).template head<2>()).norm();
                        if (distance > max_distance) { max_distance = distance; }
                        if (i == 0) {
                            segment_cost +=
                                m_distance_cost_func_.Calc(MetricState::Zero(), segment.col(i));
                            continue;
                        }
                        segment_cost +=
                            m_distance_cost_func_.Calc(segment.col(i - 1), segment.col(i));
                    }
                    segment_costs.push_back(segment_cost);
                }
                trajectory_costs.push_back(std::move(segment_costs));
            }

            /*
             * Part 2: create a new grid map info if necessary
             */

            // We should use the grid map center as the reference point and make sure the
            // grid_map_info contains the longest metric relative trajectory, so that the grid state
            // hashing can work properly. The hashing result will be wrong if there is negative
            // value in the grid state.
            Dtype x_center = m_grid_map_info_->Center().x();
            Dtype y_center = m_grid_map_info_->Center().y();
            // We don't need to replace m_grid_map_info_ if hashing_grid_map_info is different,
            // because hashing_grid_map_info is used to find unique relative successors only.
            // m_grid_map_info_ is used to convert between metric and grid states, and to check if
            // a state is in the map.
            auto hashing_grid_map_info = m_grid_map_info_;
            if ((x_center - max_distance < m_grid_map_info_->Min(0)) ||
                (y_center - max_distance < m_grid_map_info_->Min(1)) ||
                (x_center + max_distance > m_grid_map_info_->Max(0)) ||
                (y_center + max_distance > m_grid_map_info_->Max(1))) {
                // create a new grid map info
                max_distance *= 1.5;  // add 15% considering sqrt(2) * max_distance for diagonal
                auto num_cells_x =
                    static_cast<int>(max_distance / m_grid_map_info_->Resolution(0)) + 1;
                auto num_cells_y =
                    static_cast<int>(max_distance / m_grid_map_info_->Resolution(1)) + 1;
                const Dtype &kXRes = m_grid_map_info_->Resolution(0);
                const Dtype &kYRes = m_grid_map_info_->Resolution(1);
                const Dtype &kThetaRes = m_grid_map_info_->Resolution(2);

                // m_grid_map_info_ and hashing_grid_map_info_ share the same map center.
                hashing_grid_map_info = std::make_shared<GridMapInfo>(
                    MetricState(
                        x_center - static_cast<Dtype>(num_cells_x) * kXRes,
                        y_center - static_cast<Dtype>(num_cells_y) * kYRes,
                        -M_PI),  // min
                    MetricState(
                        x_center + static_cast<Dtype>(num_cells_x) * kXRes,
                        y_center + static_cast<Dtype>(num_cells_y) * kYRes,
                        M_PI),                             // max
                    MetricState(kXRes, kYRes, kThetaRes),  // resolution
                    Eigen::Vector3i(0, 0, 0));             // padding
                ERL_DEBUG_ASSERT(
                    hashing_grid_map_info->Center().x() == x_center,
                    "Grid map center changed! x_center: {:f}, grid_map_info->Center().x(): {:f}",
                    x_center,
                    hashing_grid_map_info->Center().x());
                ERL_DEBUG_ASSERT(
                    hashing_grid_map_info->Center().y() == y_center,
                    "Grid map center changed! y_center: {:f}, grid_map_info->Center().y(): {:f}",
                    y_center,
                    hashing_grid_map_info->Center().y());
            }

            /*
             * Part 3: compute discrete relative trajectories and unique relative successors
             */

            // init discrete relative trajectories
            auto num_motions = static_cast<int>(m_setting_->motion_primitives.size());
            int x_center_g = hashing_grid_map_info->CenterGrid().x();
            int y_center_g = hashing_grid_map_info->CenterGrid().y();
            m_rel_trajectories_.clear();
            m_rel_trajectories_.resize(m_setting_->num_orientations);
            m_rel_successors_.clear();
            m_rel_successors_.resize(m_setting_->num_orientations);

            for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
                // each orientation

                Dtype theta = hashing_grid_map_info->GridToMeterAtDim(theta_g, 2);
                Dtype sin_theta = std::sin(theta);
                Dtype cos_theta = std::cos(theta);

                State ref_state(
                    MetricState(x_center, y_center, theta),
                    GridState(x_center_g, y_center_g, theta_g));

                // std::vector<std::vector<std::vector<State>>>
                auto &rel_trajectories = m_rel_trajectories_[theta_g];
                rel_trajectories.resize(num_motions);

                std::map<std::size_t, RelSuccessorInfo> unique_rel_successors;
                for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                    // each motion primitive

                    DdcMotionPrimitive<Dtype> &motion = m_setting_->motion_primitives[motion_idx];
                    // a metric trajectory starts from [0, 0, 0]
                    // grid trajectories starts from [0, 0, theta_g]
                    // std::vector<MetricTrajectory>
                    auto &motion_rel_trajs = metric_rel_trajs[motion_idx];
                    // std::vector<std::vector<State>>
                    auto &motion_rel_trajectories = rel_trajectories[motion_idx];
                    auto num_controls = static_cast<int>(motion.controls.size());
                    motion_rel_trajectories.reserve(num_controls);

                    long max_num_trajectory_states = 0;
                    for (MetricTrajectory &segment: motion_rel_trajs) {
                        max_num_trajectory_states += segment.cols();
                    }

                    // a trajectory starts from [x_center_g, y_center_g, theta_g]
                    std::vector<State> motion_states;
                    motion_states.reserve(max_num_trajectory_states);

                    // a motion primitive can have multiple controls, and the intermediate states
                    // are considered as successors.
                    for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
                        // each control of the motion primitive

                        MetricTrajectory &metric_segment = motion_rel_trajs[control_idx];
                        long num_metric_states = metric_segment.cols();
                        GridState grid_state = {};
                        MetricState metric_state = {};
                        for (long i = 0; i < num_metric_states; ++i) {
                            // to hash the grid state correctly, use grid map center as the
                            // reference point, [x_center_g, y_center_g, theta_g]
                            auto wp = metric_segment.col(i);
                            grid_state[0] = hashing_grid_map_info->MeterToGridAtDim(
                                cos_theta * wp[0] - sin_theta * wp[1] + x_center,
                                0);
                            grid_state[1] = hashing_grid_map_info->MeterToGridAtDim(
                                sin_theta * wp[0] + cos_theta * wp[1] + y_center,
                                1);
                            grid_state[2] = hashing_grid_map_info->MeterToGridAtDim(
                                common::WrapAnglePi(wp[2] + theta),
                                2);
                            metric_state = hashing_grid_map_info->GridToMeterForPoints(grid_state);
                            if (motion_states.empty()) {
                                // skip the start state [x_center_g, y_center_g, theta_g]
                                if (grid_state == ref_state.grid) { continue; }
                            } else if (grid_state == motion_states.back().grid) {
                                continue;  // skip duplicate states
                            }
                            // the reference state is still [x_center, y_center, theta].
                            // it should be changed to [0, 0, 0].
                            motion_states.emplace_back(// new state
                                MetricState(
                                    metric_state[0] - x_center,
                                    metric_state[1] - y_center,
                                    metric_state[2] - theta),
                                GridState(
                                    grid_state[0] - x_center_g,
                                    grid_state[1] - y_center_g,
                                    grid_state[2] - theta_g));
                        }
                        // empty trajectory is possible if the control is smaller than resolution
                        if (motion_states.empty()) {
                            auto &control = motion.controls[control_idx];
                            ERL_WARN_ONCE(
                                "Empty trajectory for control {} of motion {}: v={:f}, w={:f}.",
                                control_idx,
                                motion_idx,
                                control.linear_v,
                                control.angular_v);
                            motion_rel_trajectories.emplace_back();  // empty trajectory
                            continue;  // skip the control if the trajectory is empty
                        }

                        // hashing will be wrong if there are negative values!
                        ERL_DEBUG_ASSERT(
                            (grid_state.array() >= 0).all(),
                            "Grid state has negative value: [{}, {}, {}].",
                            grid_state[0],
                            grid_state[1],
                            grid_state[2]);
                        auto hashing = hashing_grid_map_info->GridToIndex(grid_state, true);
                        // get the info of the successor
                        RelSuccessorInfo &successor_info = unique_rel_successors[hashing];
                        // reached by the control
                        // do we need to make the ref state [0, 0, 0] here?
                        // No, because the ref state is only used to distinguish unique successors.
                        successor_info.rel_state.metric = metric_state;
                        successor_info.rel_state.grid = grid_state;
                        successor_info.motion_indices.push_back(motion_idx);
                        successor_info.control_indices.push_back(control_idx);
                        successor_info.action_indices.emplace_back(motion_idx, control_idx);
                        successor_info.costs.push_back(
                            motion.costs[control_idx] +
                            trajectory_costs[motion_idx][control_idx]);  // add trajectory length
                        motion_rel_trajectories.emplace_back(motion_states);
                    }
                }

                // sort controls by cost for each unique successor
                std::vector<RelSuccessorInfo> &rel_successors = m_rel_successors_[theta_g];
                for (auto &[hashing, successor_info]: unique_rel_successors) {
                    successor_info.orders.resize(successor_info.costs.size());
                    std::iota(successor_info.orders.begin(), successor_info.orders.end(), 0);
                    if (successor_info.orders.size() <= 1) {
                        rel_successors.push_back(std::move(successor_info));
                        continue;
                    }

                    std::vector<int> &motion_indices = successor_info.motion_indices;
                    std::vector<int> &control_indices = successor_info.control_indices;
                    std::vector<Dtype> &costs = successor_info.costs;
                    std::sort(
                        successor_info.orders.begin(),
                        successor_info.orders.end(),
                        [&motion_indices, &control_indices, &costs, &trajectory_costs](
                            std::size_t i,
                            std::size_t j) {
                            if (costs[i] == costs[j]) {
                                // identical control_cost + trajectory_length
                                const int &motion_idx_i = motion_indices[i];
                                const int &motion_idx_j = motion_indices[j];
                                const int &control_idx_i = control_indices[i];
                                const int &control_idx_j = control_indices[j];
                                // prefer shorter trajectory
                                return trajectory_costs[motion_idx_i][control_idx_i] <
                                       trajectory_costs[motion_idx_j][control_idx_j];
                            }
                            return costs[i] < costs[j];
                        });
                    rel_successors.push_back(std::move(successor_info));
                }
            }

            /*
             * Part 4: inflate grid map according to robot shape
             */
            m_inflated_grid_maps_.resize(m_setting_->num_orientations);
            for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
                m_original_grid_map_.copyTo(m_inflated_grid_maps_[theta_g]);
            }
            if (m_setting_->robot_metric_contour.cols() == 0) { return; }

            // inflate grid map for different orientations
            for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
                Dtype theta = m_grid_map_info_->GridToMeterAtDim(theta_g, 2);
                Eigen::Matrix2<Dtype> rotation = Eigen::Rotation2D<Dtype>(theta).toRotationMatrix();
                Eigen::Matrix2X<Dtype> vertices = rotation * m_setting_->robot_metric_contour;
                common::InflateWithShape(
                    m_original_grid_map_,
                    grid_map->info,
                    vertices,
                    m_inflated_grid_maps_[theta_g]);
            }
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] std::size_t
        GetActionSpaceSize() const override {
            std::size_t size = 0;
            for (auto &motion_primitive: m_setting_->motion_primitives) {
                size += motion_primitive.controls.size();
            }
            return size;
        }

        static MetricState
        MotionModel(
            const MetricState &metric_state,
            const DifferentialDriveControl<Dtype> &control,
            Dtype t) {
            MetricState next_state;

            DifferentialDriveKinematic<Dtype>(
                metric_state[0],
                metric_state[1],
                metric_state[2],
                control.linear_v,
                control.angular_v,
                t,
                next_state[0],
                next_state[1],
                next_state[2]);
            return next_state;
        }

        [[nodiscard]] std::vector<State>
        ForwardAction(const State &env_state, long action_idx) const override {
            action_index_t action;
            action.index = action_idx;

            const std::vector<State> &kRelTraj =
                m_rel_trajectories_[env_state.grid[2]][action.motion_idx][action.control_idx];
            std::vector<State> trajectory;
            trajectory.reserve(kRelTraj.size());
            for (const State &kRelState: kRelTraj) {
                trajectory.emplace_back(
                    kRelState.metric + env_state.metric,
                    kRelState.grid + env_state.grid);
            }
            return trajectory;
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessors(const State &env_state) const override {
            if (!InStateSpace(env_state)) { return {}; }
            int cur_theta_g = env_state.grid[2];

            const auto &kRelTrajectories = m_rel_trajectories_[cur_theta_g];
            const std::vector<RelSuccessorInfo> &kRelSuccessors = m_rel_successors_[cur_theta_g];
            std::size_t max_num_successors = kRelSuccessors.size();
            std::vector<Successor_t> successors;
            successors.reserve(max_num_successors);

            const int x_len = m_grid_map_info_->Shape(0);
            const int y_len = m_grid_map_info_->Shape(1);

            for (const RelSuccessorInfo &kRelSuccessor: kRelSuccessors) {
                // for each successor, we pick the one with the lowest cost
                for (const std::size_t index: kRelSuccessor.orders) {
                    // iterate over controls that lead to the successor

                    const int &kMotionIdx = kRelSuccessor.motion_indices[index];
                    const int &kControlIdx = kRelSuccessor.control_indices[index];
                    const auto &kRelTrajectory = kRelTrajectories[kMotionIdx][kControlIdx];

                    // check collision
                    bool collided = false;
                    const std::size_t num_steps = kRelTrajectory.size();

                    State state;
                    int &x_g = state.grid[0];
                    int &y_g = state.grid[1];
                    int &theta_g = state.grid[2];

                    for (std::size_t i = 0; i < num_steps; ++i) {
                        const State &kRel = kRelTrajectory[i];

                        x_g = kRel.grid[0] + env_state.grid[0];
                        y_g = kRel.grid[1] + env_state.grid[1];
                        theta_g = kRel.grid[2] + env_state.grid[2];

                        ERL_DEBUG_ASSERT(
                            theta_g >= 0 && theta_g < m_setting_->num_orientations,
                            "theta_g out of range: {} (env_state: [{}, {}, {}], rel: [{}, {}, {}])",
                            theta_g,
                            env_state.grid[0],
                            env_state.grid[1],
                            env_state.grid[2],
                            kRel.grid[0],
                            kRel.grid[1],
                            kRel.grid[2]);

                        if (x_g < 0 || x_g >= x_len || y_g < 0 || y_g >= y_len) {
                            collided = true;
                            break;  // out of grid map
                        }

                        if (m_inflated_grid_maps_[theta_g].at<MapDtype>(x_g, y_g) >=
                            m_setting_->obstacle_threshold) {
                            collided = true;
                            break;  // in collision
                        }
                    }
                    if (collided) { continue; }  // this control is invalid, try next one

                    state.metric = kRelTrajectory.back().metric + env_state.metric;
                    Dtype cost = kRelSuccessor.costs[index];
                    if (m_setting_->add_map_cost) {
                        cost += static_cast<Dtype>(
                                    m_inflated_grid_maps_[theta_g].at<MapDtype>(x_g, y_g)) *
                                m_setting_->map_cost_factor;
                    }
                    const long action_idx = kRelSuccessor.action_indices[index].index;
                    successors.emplace_back(state, cost, action_idx, this->m_env_id_);
                }
            }
            return successors;
        }

        [[nodiscard]] bool
        InStateSpace(const State &env_state) const override {
            return m_grid_map_info_->InGrids(env_state.grid);
        }

        [[nodiscard]] uint32_t
        StateHashing(const State &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state.grid, true);
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            ERL_DEBUG_ASSERT(
                m_grid_map_info_,
                "Not supported when not initialized with grid_map_info.");
            return {
                m_grid_map_info_->MeterToGridAtDim(metric_state[0], 0),
                m_grid_map_info_->MeterToGridAtDim(metric_state[1], 1),
                m_grid_map_info_->MeterToGridAtDim(metric_state[2], 2)};
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            ERL_DEBUG_ASSERT(
                m_grid_map_info_,
                "Not supported when not initialized with grid_map_info.");
            return {
                m_grid_map_info_->GridToMeterAtDim(grid_state[0], 0),
                m_grid_map_info_->GridToMeterAtDim(grid_state[1], 1),
                m_grid_map_info_->GridToMeterAtDim(grid_state[2], 2)};
        }

        // [[nodiscard]] cv::Mat
        // ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const override;

        [[nodiscard]] std::vector<State>
        SampleValidStates(int /*num_samples*/) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };

    extern template class EnvironmentSe2<float, uint8_t>;
    extern template class EnvironmentSe2<double, uint8_t>;
    extern template class EnvironmentSe2<float, float>;
    extern template class EnvironmentSe2<double, float>;
    extern template class EnvironmentSe2<float, double>;
    extern template class EnvironmentSe2<double, double>;
}  // namespace erl::env

template<>
struct YAML::convert<erl::env::EnvironmentSe2<float>::Setting>
    : public erl::env::EnvironmentSe2<float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSe2<double>::Setting>
    : public erl::env::EnvironmentSe2<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSe2<float, float>::Setting>
    : public erl::env::EnvironmentSe2<float, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSe2<double /*Dtype*/, float /*MapDtype*/>::Setting>
    : public erl::env::EnvironmentSe2<double, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSe2<float, double>::Setting>
    // WARNING: map cost will be cast from double to float.
    : public erl::env::EnvironmentSe2<float, double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentSe2<double, double>::Setting>
    : public erl::env::EnvironmentSe2<double, double>::Setting::YamlConvertImpl {};

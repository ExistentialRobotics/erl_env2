#include <opencv2/highgui.hpp>
#include <utility>

#include "erl_env/environment_se2.hpp"
#include "erl_env/differential_drive_model.hpp"


namespace erl::env {
    EnvironmentSe2::EnvironmentSe2(
        double collision_check_dt,
        std::vector<DdcMotionPrimitive> motion_primitives,
        int num_orientations,
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        uint8_t obstacle_threshold,
        bool add_map_cost,
        double map_cost_factor)
        : EnvironmentBase(std::make_shared<EuclideanDistanceCost>(), collision_check_dt),
          m_motion_primitives_(std::move(motion_primitives)),
          m_grid_map_info_(std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(num_orientations, -M_PI, M_PI, 2))),
          m_obstacle_threshold_(obstacle_threshold),
          m_add_map_cost_(add_map_cost),
          m_map_cost_factor_(map_cost_factor) {

        // init grid maps
        m_original_grid_map_ = erl::env::EnvironmentSe2::InitializeGridMap2D(grid_map);
        m_inflated_grid_maps_.resize(num_orientations);
        for (int i = 0; i < num_orientations; ++i) { m_original_grid_map_.copyTo(m_inflated_grid_maps_[i]); }

        // The motion primitives may generate trajectories that go outside the map. We need to determine the furthest distance and create a new grid_map_info
        // that is large enough to contain all the trajectories.
        double max_distance = 0;
        // process motion primitives and prepare metric relative trajectories starting from [0, 0, 0]
        m_metric_rel_trajectories_.reserve(m_motion_primitives_.size());
        for (auto &motion: m_motion_primitives_) {
            std::size_t num_controls = motion.controls.size();
            if (num_controls > m_max_num_controls_) { m_max_num_controls_ = num_controls; }
            // convert incremental cost to cumulative cost
            std::partial_sum(motion.costs.begin(), motion.costs.end(), motion.costs.begin());
            // compute metric relative trajectory segments
            m_metric_rel_trajectories_.push_back(motion.ComputeTrajectorySegments(Eigen::Vector3d::Zero(), m_dt_, MotionModel));
            // compute the maximum distance of the trajectory segments
            for (auto &segment: m_metric_rel_trajectories_.back()) {
                for (long i = 0; i < segment.cols(); ++i) {
                    double distance = (segment.col(i).head<2>()).norm();
                    if (distance > max_distance) { max_distance = distance; }
                }
            }
        }

        // We should use the grid map center as the reference point and make sure the grid_map_info contains the longest metric relative trajectory, so that
        // the grid state hashing can work properly. The hashing result will be wrong if there is negative value in the grid state.
        double x_center = m_grid_map_info_->Center().x();
        double y_center = m_grid_map_info_->Center().y();
        // m_max_num_successors_ = m_max_num_controls_ * m_motion_primitives_.size();
        auto grid_map_info = m_grid_map_info_;
        if ((x_center - max_distance < m_grid_map_info_->Min(0)) || (y_center - max_distance < m_grid_map_info_->Min(1)) ||
            (x_center + max_distance > m_grid_map_info_->Max(0)) || (y_center + max_distance > m_grid_map_info_->Max(1))) {  // create a new grid map info

            max_distance *= 1.1;  // add 10% to the max distance to avoid edge cases
            auto num_cells_x = int(max_distance / m_grid_map_info_->Resolution(0)) + 1;
            auto num_cells_y = int(max_distance / m_grid_map_info_->Resolution(1)) + 1;
            const double &kXRes = m_grid_map_info_->Resolution(0);
            const double &kYRes = m_grid_map_info_->Resolution(1);
            const double &kThetaRes = m_grid_map_info_->Resolution(2);

            grid_map_info = std::make_shared<common::GridMapInfo3D>(
                Eigen::Vector3d(x_center - double(num_cells_x) * kXRes, y_center - double(num_cells_y) * kYRes, -M_PI),  // min
                Eigen::Vector3d(x_center + double(num_cells_x) * kXRes, y_center + double(num_cells_y) * kYRes, M_PI),   // max
                Eigen::Vector3d(kXRes, kYRes, kThetaRes),                                                                // resolution
                Eigen::Vector3i(0, 0, 0));                                                                               // padding
            ERL_DEBUG_ASSERT(
                grid_map_info->Center().x() == x_center,
                "Grid map center changed! x_center: %f, grid_map_info->Center().x(): %f",
                x_center,
                grid_map_info->Center().x());
            ERL_DEBUG_ASSERT(
                grid_map_info->Center().y() == y_center,
                "Grid map center changed! y_center: %f, grid_map_info->Center().y(): %f",
                y_center,
                grid_map_info->Center().y());
        }

        // init discrete relative trajectories
        auto num_motions = int(m_motion_primitives_.size());
        int x_center_g = grid_map_info->CenterGrid().x();
        int y_center_g = grid_map_info->CenterGrid().y();
        m_grid_rel_trajectories_.clear();
        m_grid_rel_trajectories_.resize(num_orientations);
        m_grid_rel_successors_.clear();
        m_grid_rel_successors_.resize(num_orientations);
        for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
            double theta = grid_map_info->GridToMeterForValue(theta_g, 2);
            double sin_theta = std::sin(theta);
            double cos_theta = std::cos(theta);
            auto ref_state = std::make_shared<EnvironmentState>(Eigen::Vector3d(x_center, y_center, theta), Eigen::Vector3i(x_center_g, y_center_g, theta_g));
            auto &grid_rel_trajectories = m_grid_rel_trajectories_[theta_g];  // std::vector<std::vector<Eigen::MatrixXi>>
            grid_rel_trajectories.resize(num_motions);
            std::map<std::size_t, GridRelSuccessorInfo> unique_grid_rel_successors;
            std::vector<std::vector<double>> grid_rel_trajectory_metric_lengths(num_motions);
            for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                auto &motion = m_motion_primitives_[motion_idx];

                // a metric trajectory starts from [0, 0, 0]
                auto &motion_metric_rel_trajectory_segments = m_metric_rel_trajectories_[motion_idx];
                // grid trajectories starts from [0, 0, theta_g]
                auto &motion_grid_rel_trajectories = grid_rel_trajectories[motion_idx];
                auto num_controls = int(motion.controls.size());
                motion_grid_rel_trajectories.reserve(num_controls);

                auto &motion_grid_rel_trajectory_metric_lengths = grid_rel_trajectory_metric_lengths[motion_idx];
                motion_grid_rel_trajectory_metric_lengths.reserve(num_controls);

                long max_num_grid_trajectory_states = 0;
                for (auto &segment: motion_metric_rel_trajectory_segments) { max_num_grid_trajectory_states += segment.cols(); }
                std::vector<std::shared_ptr<EnvironmentState>> grid_trajectory;  // a grid trajectory starts from [x_center_g, y_center_g, theta_g]
                grid_trajectory.reserve(max_num_grid_trajectory_states);

                double trajectory_length = 0;
                // a motion primitive can have multiple controls, and the mediate states are considered as successors
                for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
                    auto &metric_segment = motion_metric_rel_trajectory_segments[control_idx];

                    long num_metric_states = metric_segment.cols();
                    for (long i = 0; i < num_metric_states; ++i) {
                        // to hash the grid state correctly, use grid map center as the reference point, [x_center_g, y_center_g, theta_g]
                        Eigen::Vector3i grid_state(
                            grid_map_info->MeterToGridForValue(cos_theta * metric_segment(0, i) - sin_theta * metric_segment(1, i) + x_center, 0),
                            grid_map_info->MeterToGridForValue(sin_theta * metric_segment(0, i) + cos_theta * metric_segment(1, i) + y_center, 1),
                            grid_map_info->MeterToGridForValue(common::ClipAngle(metric_segment(2, i) + theta), 2));
                        Eigen::Vector3d metric_state(
                            grid_map_info->GridToMeterForValue(grid_state[0], 0),
                            grid_map_info->GridToMeterForValue(grid_state[1], 1),
                            grid_map_info->GridToMeterForValue(grid_state[2], 2));

                        if (grid_trajectory.empty()) {
                            if (grid_state == ref_state->grid) { continue; }                 // skip the start state [x_center_g, y_center_g, theta_g]
                            trajectory_length += (ref_state->metric - metric_state).norm();  // Euclidean distance
                        } else if (grid_state != grid_trajectory.back()->grid) {
                            trajectory_length += (grid_trajectory.back()->metric - metric_state).norm();
                        } else {
                            continue;  // skip duplicate states
                        }
                        grid_trajectory.push_back(std::make_shared<EnvironmentState>(metric_state, grid_state));
#ifndef NDEBUG
                        if (grid_trajectory.size() >= 2) {
                            auto &state1 = grid_trajectory[grid_trajectory.size() - 2]->grid;
                            auto &state2 = grid_trajectory.back()->grid;
                            Eigen::Vector3i diff = (state1 - state2).cwiseAbs();
                            ERL_WARN_ONCE_COND(
                                diff[0] > 1 || diff[1] > 1 || std::min(diff[2], num_orientations - diff[2]) > 1,
                                "grid trajectory segment [%d, %d] -> [%d, %d] has a step larger than 1. Collision checking may be compromised.",
                                state1[0],
                                state1[1],
                                state2[0],
                                state2[1]);
                        }
#endif
                    }
                    if (grid_trajectory.empty()) {
                        auto &control = motion.controls[control_idx];
                        ERL_WARN_ONCE(
                            "Empty trajectory for control %d of motion %d: v=%f, w=%f.",
                            control_idx,
                            motion_idx,
                            control.linear_v,
                            control.angular_v);
                        motion_grid_rel_trajectory_metric_lengths.push_back(0.0);
                        motion_grid_rel_trajectories.emplace_back();
                        continue;  // skip the control if the trajectory is empty
                    }

                    auto hashing = grid_map_info->GridToIndex(grid_trajectory.back()->grid, true);  // hashing will be wrong if there are negative values!
                    for (auto &state: grid_trajectory) {
                        state->metric -= ref_state->metric;
                        state->grid -= ref_state->grid;
                    }
                    auto &successor_info = unique_grid_rel_successors[hashing];  // get the info of the successor reached by the control
                    if (successor_info.rel_state == nullptr) { successor_info.rel_state = grid_trajectory.back(); }
                    successor_info.motion_indices.push_back(motion_idx);
                    successor_info.control_indices.push_back(control_idx);
                    successor_info.action_indices.push_back(motion_idx * int(m_max_num_controls_) + control_idx);
                    successor_info.costs.push_back(motion.costs[control_idx] + trajectory_length);  // add trajectory length
                    motion_grid_rel_trajectory_metric_lengths.push_back(trajectory_length);
                    motion_grid_rel_trajectories.emplace_back(grid_trajectory);
                }
                // all state's reference position is still [x_center, y_center]
                for (auto &state: grid_trajectory) {
                    state->metric[0] -= x_center;
                    state->metric[1] -= y_center;
                    if (grid_map_info != m_grid_map_info_) {  // generated by the temporary grid map
                        // might be negative, but it is fine.
                        state->grid[0] = m_grid_map_info_->MeterToGridForValue(state->metric[0], 0);
                        state->grid[1] = m_grid_map_info_->MeterToGridForValue(state->metric[1], 1);
                        state->grid[2] = m_grid_map_info_->MeterToGridForValue(state->metric[2], 2);
                        state->metric[0] = m_grid_map_info_->GridToMeterForValue(state->grid[0], 0);
                        state->metric[1] = m_grid_map_info_->GridToMeterForValue(state->grid[1], 1);
                        state->metric[2] = m_grid_map_info_->GridToMeterForValue(state->grid[2], 2);
                    }
                }
            }

            auto &grid_rel_successors = m_grid_rel_successors_[theta_g];  // std::vector<GridRelSuccessorInfo>
            for (auto &[hashing, successor_info]: unique_grid_rel_successors) {
                successor_info.orders.resize(successor_info.control_indices.size());
                std::iota(successor_info.orders.begin(), successor_info.orders.end(), 0);
                if (successor_info.orders.size() <= 1) {
                    grid_rel_successors.push_back(std::move(successor_info));
                    continue;
                }

                auto &motion_indices = successor_info.motion_indices;
                auto &control_indices = successor_info.control_indices;
                auto &costs = successor_info.costs;
                std::sort(
                    successor_info.orders.begin(),
                    successor_info.orders.end(),
                    [&motion_indices, &control_indices, &costs, &grid_rel_trajectory_metric_lengths](std::size_t i, std::size_t j) {
                        if (costs[i] == costs[j]) {  // identical control_cost + trajectory_length, prefer shorter trajectory
                            auto &motion_idx_i = motion_indices[i];
                            auto &motion_idx_j = motion_indices[j];
                            auto &control_idx_i = control_indices[i];
                            auto &control_idx_j = control_indices[j];
                            return grid_rel_trajectory_metric_lengths[motion_idx_i][control_idx_i] <
                                   grid_rel_trajectory_metric_lengths[motion_idx_j][control_idx_j];
                        }
                        return costs[i] < costs[j];
                    });
                grid_rel_successors.push_back(std::move(successor_info));
            }
        }
    }

    EnvironmentSe2::EnvironmentSe2(
        double collision_check_dt,
        std::vector<DdcMotionPrimitive> motion_primitives,
        int num_orientations,
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        double inflate_scale,
        const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices,
        uint8_t obstacle_threshold,
        bool add_map_cost,
        double map_cost_factor)
        : EnvironmentSe2(collision_check_dt, std::move(motion_primitives), num_orientations, grid_map, obstacle_threshold, add_map_cost, map_cost_factor) {

        if (shape_metric_vertices.cols() == 0) {
            ERL_WARN("shape_metric_vertices is empty, skip inflation.\n");
            return;
        }
        if (inflate_scale <= 0) {
            ERL_WARN("inflate_scale <= 0, skip inflation.\n");
            return;
        }
        m_shape_metric_vertices_ = shape_metric_vertices.array() * inflate_scale;

        // inflate grid map for different orientations
        m_inflated_grid_maps_.resize(num_orientations);
        for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
            Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_shape_metric_vertices_;
            InflateGridMap2D(m_original_grid_map_, m_inflated_grid_maps_[theta_g], grid_map->info, vertices);
        }
    }

    std::vector<std::shared_ptr<EnvironmentState>>
    EnvironmentSe2::ForwardAction(const std::shared_ptr<const EnvironmentState> &state, int action_index) const {
        auto action_coords = ActionIndexToActionCoords(action_index);
        auto motion_idx = action_coords[0];
        auto control_idx = action_coords[1];

        Eigen::MatrixXd states = m_motion_primitives_[motion_idx].ComputeTrajectorySegment(state->metric, control_idx, m_dt_, MotionModel);

        std::vector<std::shared_ptr<EnvironmentState>> trajectory(states.cols());
        long num_states = states.cols();
        for (long i = 0; i < num_states; ++i) {
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->metric = states.col(i);
            new_state->grid = MetricToGrid(new_state->metric);
            trajectory[i] = std::move(new_state);
        }
        return trajectory;
    }

    std::vector<Successor>
    EnvironmentSe2::GetSuccessors(const std::shared_ptr<EnvironmentState> &state) const {

        int current_theta_g = state->grid[2];

        auto &grid_rel_trajectories = m_grid_rel_trajectories_[current_theta_g];
        auto &grid_rel_successors = m_grid_rel_successors_[current_theta_g];
        std::size_t max_num_successors = grid_rel_successors.size();
        std::vector<Successor> successors;
        successors.reserve(max_num_successors);

        auto x_len = m_grid_map_info_->Shape(0);
        auto y_len = m_grid_map_info_->Shape(1);

        for (auto &rel_successor_info: grid_rel_successors) {
            // for each successor, we pick the one with the lowest cost
            for (auto index: rel_successor_info.orders) {
                auto &motion_idx = rel_successor_info.motion_indices[index];
                auto &control_idx = rel_successor_info.control_indices[index];
                auto &grid_rel_trajectory = grid_rel_trajectories[motion_idx][control_idx];

                // check collision
                bool collided = false;
                std::size_t num_steps = grid_rel_trajectory.size();

                Eigen::Vector3i grid_state;
                int &x_g = grid_state[0];
                int &y_g = grid_state[1];
                int &theta_g = grid_state[2];

                Eigen::Vector3d metric_state;
                double &x = metric_state[0];
                double &y = metric_state[1];
                double &theta = metric_state[2];

                for (std::size_t i = 0; i < num_steps; ++i) {
                    auto &rel_grid = grid_rel_trajectory[i]->grid;
                    x_g = rel_grid[0] + state->grid[0];
                    y_g = rel_grid[1] + state->grid[1];
                    theta_g = rel_grid[2];

                    if (x_g < 0 || x_g >= x_len || y_g < 0 || y_g >= y_len) {  // out of grid map
                        collided = true;
                        break;
                    }

                    if (m_inflated_grid_maps_[theta_g].at<uint8_t>(x_g, y_g) > 0) {  // in collision
                        collided = true;
                        break;
                    }

                    auto &rel_metric = grid_rel_trajectory[i]->metric;
                    x = rel_metric[0] + state->metric[0];
                    y = rel_metric[1] + state->metric[1];
                    theta = rel_metric[2];
                }
                if (collided) { continue; }  // this control is invalid, try next one
                if (m_add_map_cost_) {
                    double map_cost = double(m_inflated_grid_maps_[theta_g].at<uint8_t>(x_g, y_g)) * m_map_cost_factor_;
                    double cost = rel_successor_info.costs[index] + map_cost;
                    successors.emplace_back(metric_state, grid_state, cost, rel_successor_info.action_indices[index]);
                } else {
                    successors.emplace_back(metric_state, grid_state, rel_successor_info.costs[index], rel_successor_info.action_indices[index]);
                }

                ERL_DEBUG_ASSERT((grid_state.array() >= 0).all(), "Grid state is negative: [%d, %d, %d].\n", grid_state[0], grid_state[1], grid_state[2]);
            }
        }
        return successors;
    }

    void
    EnvironmentSe2::PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) {
        m_already_reset_ = false;

        if (m_shape_metric_vertices_.cols() == 0) {
            Eigen::VectorXi grid_state = MetricToGrid(metric_state);
            int num_orientations = m_grid_map_info_->Shape(2);
            int block_half_size = 1;
            for (int i = -block_half_size; i <= block_half_size; ++i) {
                for (int j = -block_half_size; j <= block_half_size; ++j) {
                    for (int k = 0; k < num_orientations; ++k) {
                        int x = grid_state[0] + i;
                        int y = grid_state[1] + j;
                        if (x < 0 || x >= m_grid_map_info_->Shape(0) || y < 0 || y >= m_grid_map_info_->Shape(1)) { continue; }
                        m_inflated_grid_maps_[k].at<uint8_t>(x, y) = 0;
                    }
                }
            }
            return;
        }

        Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(metric_state[2]).toRotationMatrix();
        Eigen::Vector2d translation_vector = metric_state.head<2>();
        Eigen::Matrix2Xd vertices = (rotation_matrix * m_shape_metric_vertices_).colwise() + translation_vector;
        auto &grid_map = m_inflated_grid_maps_[m_grid_map_info_->MeterToGridForValue(metric_state[2], 2)];
        std::vector<std::vector<cv::Point>> contour(1);
        auto &contour_points = contour[0];
        for (int i = 0; i < vertices.cols(); ++i) {
            contour_points.emplace_back(m_grid_map_info_->MeterToGridForValue(vertices(1, i), 1), m_grid_map_info_->MeterToGridForValue(vertices(0, i), 0));
        }
        cv::drawContours(grid_map, contour, 0, cv::Scalar(0), cv::FILLED);
    }

    void
    EnvironmentSe2::Reset() {
        if (m_already_reset_) { return; }
        m_already_reset_ = true;

        int num_orientations = m_grid_map_info_->Shape(2);
        m_inflated_grid_maps_.clear();
        m_inflated_grid_maps_.resize(num_orientations);

        if (m_shape_metric_vertices_.cols() == 0) {
            for (int i = 0; i < num_orientations; ++i) { m_original_grid_map_.copyTo(m_inflated_grid_maps_[i]); }
            return;
        }

        for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
            Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_shape_metric_vertices_;
            InflateGridMap2D(
                m_original_grid_map_,
                m_inflated_grid_maps_[theta_g],
                std::make_shared<common::GridMapInfo2D>(m_grid_map_info_->Squeeze(2)),
                vertices);
        }
    }

    void
    EnvironmentSe2::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
        cv::Mat img = m_original_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, 255);
        for (const auto &[kGoalIndex, kPath]: paths) {
            std::vector<cv::Point> points;
            auto num_points = kPath.cols();
            points.reserve(num_points);
            for (long i = 0; i < num_points; ++i) {
                auto grid_point = MetricToGrid(kPath.col(i));
                points.emplace_back(grid_point[1], grid_point[0]);
                // cv::Point2i arrow_point(
                //     m_grid_map_info_->MeterToGridForValue(kPath(1, i) + 0.1 * std::cos(kPath(2, i)), 1),
                //     m_grid_map_info_->MeterToGridForValue(kPath(0, i) + 0.1 * std::sin(kPath(2, i)), 0));
                // cv::arrowedLine(img, points.back(), arrow_point, cv::Scalar(0, 255, 255), 1, cv::LINE_AA, 0, 0.01);
            }
            cv::polylines(img, points, false, cv::Scalar(dist(rng), dist(rng), dist(rng)), 1);
        }
        cv::namedWindow("environment se2: paths", cv::WINDOW_NORMAL);
        cv::imshow("environment se2: paths", img);
        cv::waitKey(0);
    }
}  // namespace erl::env

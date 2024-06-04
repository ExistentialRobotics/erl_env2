#include <opencv2/highgui.hpp>
#include <utility>
#include <iomanip>

#include "erl_common/random.hpp"
#include "erl_env/environment_se2.hpp"
#include "erl_env/differential_drive_model.hpp"

// #define DEBUG_ENVIRONMENT_SE2_1
// #define DEBUG_ENVIRONMENT_SE2_2
// #define DEBUG_ENVIRONMENT_SE2_3

namespace erl::env {
    EnvironmentSe2::EnvironmentSe2(const std::shared_ptr<common::GridMapUnsigned2D> &grid_map, std::shared_ptr<Setting> setting)
        : EnvironmentBase(std::make_shared<Se2Cost>(), 0),
          m_setting_(setting == nullptr ? std::make_shared<Setting>() : std::move(setting)),
          m_grid_map_info_(std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(m_setting_->num_orientations, -M_PI, M_PI, 2))) {
        m_time_step_ = m_setting_->time_step;
        m_distance_cost_func_ = std::make_shared<Se2Cost>(m_setting_->cost_theta_weight);

        // init grid maps
        InitializeGridMap2D(grid_map, m_original_grid_map_);
#ifdef DEBUG_ENVIRONMENT_SE2_1
        cv::imshow("original grid map", m_original_grid_map_ * 255);
        cv::waitKey(0);
#endif

        // The motion primitives may generate trajectories that go outside the map. We need to determine the furthest
        // distance and create a new grid_map_info that is large enough to contain all the trajectories.
        double max_distance = 0;
        // process motion primitives and prepare metric relative trajectories starting from [0, 0, 0]
        std::vector<std::vector<Eigen::MatrixXd>> metric_rel_trajectories;  // [motion_primitive][control][3, num_states]
        std::vector<std::vector<double>> trajectory_costs;                  // [motion_idx][control_idx]
        trajectory_costs.reserve(m_setting_->motion_primitives.size());
        metric_rel_trajectories.reserve(m_setting_->motion_primitives.size());
        for (auto &motion: m_setting_->motion_primitives) {
            // convert incremental cost to cumulative cost
            std::partial_sum(motion.costs.begin(), motion.costs.end(), motion.costs.begin());
            // compute metric relative trajectory segments
            metric_rel_trajectories.push_back(motion.ComputeTrajectorySegments(Eigen::Vector3d::Zero(), m_time_step_, MotionModel));
            // compute the maximum distance of the trajectory segments
            std::vector<double> segment_costs;
            segment_costs.reserve(metric_rel_trajectories.back().size());
            for (auto &segment: metric_rel_trajectories.back()) {
                double segment_cost = 0;
                for (long i = 0; i < segment.cols(); ++i) {
                    double distance = (segment.col(i).head<2>()).norm();
                    if (distance > max_distance) { max_distance = distance; }
                    if (i == 0) {
                        segment_cost += (*m_distance_cost_func_)(
                            EnvironmentState(Eigen::VectorXd(Eigen::Vector3d::Zero())),  // previous state
                            EnvironmentState(Eigen::VectorXd(segment.col(i))));          // current state
                        continue;
                    }
                    segment_cost += (*m_distance_cost_func_)(
                        EnvironmentState(Eigen::VectorXd(segment.col(i - 1))),  // previous state
                        EnvironmentState(Eigen::VectorXd(segment.col(i))));     // current state
                }
                segment_costs.push_back(segment_cost);
            }
            trajectory_costs.push_back(std::move(segment_costs));
#ifdef DEBUG_ENVIRONMENT_SE2_2
            cv::Mat img = m_original_grid_map_ * 255;
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
            int x_offset = img.cols / 2;
            int y_offset = img.rows / 2;
            for (auto &segment: m_metric_rel_trajectories_.back()) {
                Eigen::Matrix3Xi points = m_grid_map_info_->MeterToGridForPoints(segment);
                std::vector<cv::Point> cv_points;
                cv_points.reserve(points.cols());
                for (long i = 0; i < points.cols(); ++i) { cv_points.emplace_back(points(1, i) + x_offset, points(0, i) + y_offset); }
                cv::polylines(img, cv_points, false, cv::Scalar(0, 0, 255), 1);
            }
            std::cout << "motion primitive:" << std::endl << YAML::convert<DdcMotionPrimitive>::encode(motion) << std::endl;
            cv::imshow("metric relative trajectory", img);
            cv::waitKey(0);
#endif
        }

        // We should use the grid map center as the reference point and make sure the grid_map_info contains the longest
        // metric relative trajectory, so that the grid state hashing can work properly. The hashing result will be
        // wrong if there is negative value in the grid state.
        double x_center = m_grid_map_info_->Center().x();
        double y_center = m_grid_map_info_->Center().y();
        // m_max_num_successors_ = m_max_num_controls_ * m_motion_primitives_.size();
        auto hashing_grid_map_info = m_grid_map_info_;
        if ((x_center - max_distance < m_grid_map_info_->Min(0)) || (y_center - max_distance < m_grid_map_info_->Min(1)) ||
            (x_center + max_distance > m_grid_map_info_->Max(0)) || (y_center + max_distance > m_grid_map_info_->Max(1))) {
            // create a new grid map info
            max_distance *= 1.1;  // add 10% to the max distance to avoid edge cases
            auto num_cells_x = int(max_distance / m_grid_map_info_->Resolution(0)) + 1;
            auto num_cells_y = int(max_distance / m_grid_map_info_->Resolution(1)) + 1;
            const double &kXRes = m_grid_map_info_->Resolution(0);
            const double &kYRes = m_grid_map_info_->Resolution(1);
            const double &kThetaRes = m_grid_map_info_->Resolution(2);

            hashing_grid_map_info = std::make_shared<common::GridMapInfo3D>(
                Eigen::Vector3d(x_center - double(num_cells_x) * kXRes, y_center - double(num_cells_y) * kYRes, -M_PI),  // min
                Eigen::Vector3d(x_center + double(num_cells_x) * kXRes, y_center + double(num_cells_y) * kYRes, M_PI),   // max
                Eigen::Vector3d(kXRes, kYRes, kThetaRes),                                                                // resolution
                Eigen::Vector3i(0, 0, 0));                                                                               // padding
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

        // init discrete relative trajectories
        auto num_motions = int(m_setting_->motion_primitives.size());
        int x_center_g = hashing_grid_map_info->CenterGrid().x();
        int y_center_g = hashing_grid_map_info->CenterGrid().y();
        m_rel_trajectories_.clear();
        m_rel_trajectories_.resize(m_setting_->num_orientations);
        m_rel_successors_.clear();
        m_rel_successors_.resize(m_setting_->num_orientations);
#ifdef DEBUG_ENVIRONMENT_SE2_3
        cv::Mat img = m_original_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
#endif
        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
            double theta = hashing_grid_map_info->GridToMeterForValue(theta_g, 2);
            double sin_theta = std::sin(theta);
            double cos_theta = std::cos(theta);

            auto ref_state = std::make_shared<EnvironmentState>(Eigen::Vector3d(x_center, y_center, theta), Eigen::Vector3i(x_center_g, y_center_g, theta_g));
            auto &rel_trajectories = m_rel_trajectories_[theta_g];  // std::vector<std::vector<Eigen::MatrixXi>>
            rel_trajectories.resize(num_motions);

            std::map<std::size_t, RelSuccessorInfo> unique_rel_successors;

            for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                auto &motion = m_setting_->motion_primitives[motion_idx];
                // a metric trajectory starts from [0, 0, 0]
                // grid trajectories starts from [0, 0, theta_g]
                auto &motion_metric_rel_trajectories = metric_rel_trajectories[motion_idx];
                auto &motion_rel_trajectories = rel_trajectories[motion_idx];
                auto num_controls = int(motion.controls.size());
                motion_rel_trajectories.reserve(num_controls);

                long max_num_trajectory_states = 0;
                for (Eigen::MatrixXd &segment: motion_metric_rel_trajectories) { max_num_trajectory_states += segment.cols(); }
                std::vector<std::shared_ptr<EnvironmentState>> grid_trajectory;  // a grid trajectory starts from [x_center_g, y_center_g, theta_g]
                grid_trajectory.reserve(max_num_trajectory_states);

                // a motion primitive can have multiple controls, and the intermediate states are considered as successors
                for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
                    Eigen::MatrixXd &metric_segment = motion_metric_rel_trajectories[control_idx];  // [3, num_states]
                    long num_metric_states = metric_segment.cols();
                    for (long i = 0; i < num_metric_states; ++i) {
                        // to hash the grid state correctly, use grid map center as the reference point, [x_center_g, y_center_g, theta_g]
                        Eigen::Vector3i grid_state(
                            hashing_grid_map_info->MeterToGridForValue(cos_theta * metric_segment(0, i) - sin_theta * metric_segment(1, i) + x_center, 0),
                            hashing_grid_map_info->MeterToGridForValue(sin_theta * metric_segment(0, i) + cos_theta * metric_segment(1, i) + y_center, 1),
                            hashing_grid_map_info->MeterToGridForValue(common::WrapAnglePi(metric_segment(2, i) + theta), 2));
                        Eigen::Vector3d metric_state = hashing_grid_map_info->GridToMeterForPoints(grid_state);
                        auto state = std::make_shared<EnvironmentState>(metric_state, grid_state);
                        if (grid_trajectory.empty()) {
                            if (grid_state == ref_state->grid) { continue; }  // skip the start state [x_center_g, y_center_g, theta_g]
                        } else if (grid_state == grid_trajectory.back()->grid) {
                            continue;  // skip duplicate states
                        }
                        grid_trajectory.push_back(state);  // new state
#ifndef NDEBUG
                        if (grid_trajectory.size() >= 2) {  // check consecutive states
                            Eigen::VectorXi &state1 = grid_trajectory[grid_trajectory.size() - 2]->grid;
                            Eigen::VectorXi &state2 = grid_trajectory.back()->grid;
                            Eigen::Vector3i diff = (state1 - state2).cwiseAbs();
                            ERL_WARN_ONCE_COND(
                                diff[0] > 1 || diff[1] > 1 || std::min(diff[2], m_setting_->num_orientations - diff[2]) > 1,
                                "grid trajectory segment [{}, {}, {}] -> [{}, {}, {}] has a step larger than 1. Collision checking may be compromised.",
                                state1[0],
                                state1[1],
                                state1[2],
                                state2[0],
                                state2[1],
                                state2[2]);
                        }
#endif
                    }
                    // empty trajectory is possible if the control is smaller than the resolution
                    if (grid_trajectory.empty()) {
                        auto &control = motion.controls[control_idx];
                        ERL_WARN_ONCE(
                            "Empty trajectory for control {} of motion {}: v={:f}, w={:f}.",
                            control_idx,
                            motion_idx,
                            control.linear_v,
                            control.angular_v);
                        motion_rel_trajectories.emplace_back();  // empty trajectory
                        continue;                                // skip the control if the trajectory is empty
                    }

                    // hashing will be wrong if there are negative values!
                    ERL_DEBUG_ASSERT(
                        (grid_trajectory.back()->grid.array() >= 0).all(),
                        "Grid state has negative value: [{}, {}, {}].",
                        grid_trajectory.back()->grid[0],
                        grid_trajectory.back()->grid[1],
                        grid_trajectory.back()->grid[2]);
                    auto hashing = hashing_grid_map_info->GridToIndex(grid_trajectory.back()->grid, true);
                    auto &successor_info = unique_rel_successors[hashing];  // get the info of the successor reached by the control
                    if (successor_info.rel_state == nullptr) { successor_info.rel_state = grid_trajectory.back(); }
                    successor_info.motion_indices.push_back(motion_idx);
                    successor_info.control_indices.push_back(control_idx);
                    successor_info.action_coords.push_back({motion_idx, control_idx});
                    successor_info.costs.push_back(motion.costs[control_idx] + trajectory_costs[motion_idx][control_idx]);  // add trajectory length
                    motion_rel_trajectories.emplace_back(grid_trajectory);
                }

#ifdef DEBUG_ENVIRONMENT_SE2_3
                // visualize grid relative trajectories
                std::cout << "theta: " << theta * 180 / M_PI << std::endl
                          << "motion primitive:" << std::endl
                          << YAML::convert<DdcMotionPrimitive>::encode(motion) << std::endl;
                std::vector<cv::Point> points;
                for (auto &state: grid_trajectory) { points.emplace_back(state->grid[1], state->grid[0]); }
                cv::circle(img, points.front(), 2, cv::Scalar(0, 255, 0), -1);
                cv::polylines(img, points, false, cv::Scalar(0, 0, 255), 1);
                cv::imshow("grid relative trajectory", img);
                cv::waitKey(0);
#endif

                // all state's reference state is still [x_center, y_center, theta]
                // m_grid_map_info_ and hashing_grid_map_info_ share the same map center.
                for (auto &state: grid_trajectory) {
                    state->grid[0] -= x_center_g;
                    state->grid[1] -= y_center_g;
                    state->grid[2] -= theta_g;
                    state->metric[0] -= x_center;
                    state->metric[1] -= y_center;
                    state->metric[2] -= theta;
                }
            }

            // sort controls by cost for each unique successor
            std::vector<RelSuccessorInfo> &rel_successors = m_rel_successors_[theta_g];
            for (auto &[hashing, successor_info]: unique_rel_successors) {
                successor_info.orders.resize(successor_info.control_indices.size());
                std::iota(successor_info.orders.begin(), successor_info.orders.end(), 0);
                if (successor_info.orders.size() <= 1) {
                    rel_successors.push_back(std::move(successor_info));
                    continue;
                }

                std::vector<int> &motion_indices = successor_info.motion_indices;
                std::vector<int> &control_indices = successor_info.control_indices;
                std::vector<double> &costs = successor_info.costs;
                std::sort(
                    successor_info.orders.begin(),
                    successor_info.orders.end(),
                    [&motion_indices, &control_indices, &costs, &trajectory_costs](std::size_t i, std::size_t j) {
                        if (costs[i] == costs[j]) {  // identical control_cost + trajectory_length, prefer shorter trajectory
                            auto &motion_idx_i = motion_indices[i];
                            auto &motion_idx_j = motion_indices[j];
                            auto &control_idx_i = control_indices[i];
                            auto &control_idx_j = control_indices[j];
                            return trajectory_costs[motion_idx_i][control_idx_i] < trajectory_costs[motion_idx_j][control_idx_j];
                        }
                        return costs[i] < costs[j];
                    });
                rel_successors.push_back(std::move(successor_info));
            }
        }

        int max_dx = 0, max_dy = 0;
        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
            for (auto &successor_info: m_rel_successors_[theta_g]) {
                int dx = std::abs(successor_info.rel_state->grid[0]);
                int dy = std::abs(successor_info.rel_state->grid[1]);
                if (dx > max_dx) { max_dx = dx; }
                if (dy > max_dy) { max_dy = dy; }
            }
        }
        Eigen::MatrixXi total_local_map(2 * max_dx + 1, 2 * max_dy + 1);
        total_local_map.setConstant(0);
        std::vector<Eigen::MatrixXi> local_maps;
        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
            Eigen::MatrixXi local_map(2 * max_dx + 1, 2 * max_dy + 1);
            local_map.setConstant(0);
            for (auto &successor_info: m_rel_successors_[theta_g]) {
                int dx = successor_info.rel_state->grid[0] + max_dx;
                int dy = successor_info.rel_state->grid[1] + max_dy;
                local_map(dx, dy) += 1;
                total_local_map(dx, dy) += 1;
            }
            local_maps.push_back(local_map);
        }
//        std::cout << std::endl
//                  << "num_orientations: " << m_setting_->num_orientations << std::endl
//                  << "max_dx: " << max_dx << ", max_dy: " << max_dy << std::endl
//                  << "total local map:" << std::endl
//                  << total_local_map << std::endl
//                  << "local map:" << std::endl;
//        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
//            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
//            std::cout << "theta_g: " << theta_g << ", theta: " << theta * 180 / M_PI << std::endl << local_maps[theta_g] << std::endl;
//        }

        m_inflated_grid_maps_.resize(m_setting_->num_orientations);
        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) { m_original_grid_map_.copyTo(m_inflated_grid_maps_[theta_g]); }
        if (m_setting_->shape.cols() == 0) { return; }

        // inflate grid map for different orientations
        for (int theta_g = 0; theta_g < m_setting_->num_orientations; ++theta_g) {
            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
            Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_setting_->shape;
            InflateGridMap2D(m_original_grid_map_, m_inflated_grid_maps_[theta_g], grid_map->info, vertices);
        }
    }

    std::vector<std::shared_ptr<EnvironmentState>>
    EnvironmentSe2::ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
        ERL_DEBUG_ASSERT(action_coords.size() == 2, "action_coords.size() == 2");
        int motion_idx = action_coords[0];
        int control_idx = action_coords[1];

        const std::vector<std::shared_ptr<EnvironmentState>> &kRelTraj = m_rel_trajectories_[env_state->grid[2]][motion_idx][control_idx];
        std::vector<std::shared_ptr<EnvironmentState>> trajectory;
        trajectory.reserve(kRelTraj.size());
        for (const std::shared_ptr<EnvironmentState> &kRelState: kRelTraj) {
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->metric = kRelState->metric + env_state->metric;
            new_state->grid = kRelState->grid + env_state->grid;
            trajectory.push_back(std::move(new_state));
        }

        // Eigen::MatrixXd states = m_setting_->motion_primitives[motion_idx].ComputeTrajectorySegment(env_state->metric, control_idx, m_time_step_,
        // MotionModel); std::vector<std::shared_ptr<EnvironmentState>> trajectory(states.cols()); long num_states = states.cols(); for (long i = 0; i <
        // num_states; ++i) {
        //     auto new_state = std::make_shared<EnvironmentState>();
        //     new_state->metric = states.col(i);
        //     new_state->grid = MetricToGrid(new_state->metric);
        //     trajectory[i] = std::move(new_state);
        // }
        return trajectory;
    }

    std::vector<Successor>
    EnvironmentSe2::GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const {
        if (!InStateSpace(env_state)) { return {}; }
        int current_theta_g = env_state->grid[2];

        const auto &kRelTrajectories = m_rel_trajectories_[current_theta_g];
        const std::vector<RelSuccessorInfo> &kRelSuccessors = m_rel_successors_[current_theta_g];
        std::size_t max_num_successors = kRelSuccessors.size();
        std::vector<Successor> successors;
        successors.reserve(max_num_successors);

        int x_len = m_grid_map_info_->Shape(0);
        int y_len = m_grid_map_info_->Shape(1);

        for (const RelSuccessorInfo &kRelSuccessor: kRelSuccessors) {
            // for each successor, we pick the one with the lowest cost
            for (std::size_t index: kRelSuccessor.orders) {  // iterate over controls that lead to the successor
                const int &kMotionIdx = kRelSuccessor.motion_indices[index];
                const int &kControlIdx = kRelSuccessor.control_indices[index];
                const std::vector<std::shared_ptr<EnvironmentState>> &kRelTrajectory = kRelTrajectories[kMotionIdx][kControlIdx];

                // check collision
                bool collided = false;
                std::size_t num_steps = kRelTrajectory.size();

                Eigen::Vector3i grid_state;
                int &x_g = grid_state[0];
                int &y_g = grid_state[1];
                int &theta_g = grid_state[2];

                Eigen::Vector3d metric_state;
                double &x = metric_state[0];
                double &y = metric_state[1];
                double &theta = metric_state[2];

                for (std::size_t i = 0; i < num_steps; ++i) {
                    const Eigen::VectorXi &kRelGrid = kRelTrajectory[i]->grid;
                    x_g = kRelGrid[0] + env_state->grid[0];
                    y_g = kRelGrid[1] + env_state->grid[1];
                    theta_g = kRelGrid[2] + env_state->grid[2];
                    if (theta_g >= m_setting_->num_orientations) { theta_g -= m_setting_->num_orientations; }

                    if (x_g < 0 || x_g >= x_len || y_g < 0 || y_g >= y_len) {  // out of grid map
                        collided = true;
                        break;
                    }

                    if (m_inflated_grid_maps_[theta_g].at<uint8_t>(x_g, y_g) >= m_setting_->obstacle_threshold) {  // in collision
                        collided = true;
                        break;
                    }

                    const Eigen::VectorXd &kRelMetric = kRelTrajectory[i]->metric;
                    x = kRelMetric[0] + env_state->metric[0];
                    y = kRelMetric[1] + env_state->metric[1];
                    theta = kRelMetric[2] + env_state->metric[2];
                }
                if (collided) { continue; }  // this control is invalid, try next one

                double cost = kRelSuccessor.costs[index];
                if (m_setting_->add_map_cost) { cost += double(m_inflated_grid_maps_[theta_g].at<uint8_t>(x_g, y_g)) * m_setting_->map_cost_factor; }
                successors.emplace_back(metric_state, grid_state, cost, kRelSuccessor.action_coords[index]);
                ERL_DEBUG_ASSERT((grid_state.array() >= 0).all(), "Grid state is negative: [{}, {}, {}].\n", grid_state[0], grid_state[1], grid_state[2]);
            }
        }
        return successors;
    }

    cv::Mat
    EnvironmentSe2::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const {
        cv::Mat img = m_original_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
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
            cv::Scalar color(dist(common::g_random_engine), dist(common::g_random_engine), dist(common::g_random_engine));
            cv::polylines(img, points, false, color, 1);
        }
        cv::namedWindow("environment se2: paths", cv::WINDOW_NORMAL | cv::WINDOW_AUTOSIZE);
        cv::imshow("environment se2: paths", img);
        if (block) {
            cv::waitKey(0);
        } else {
            cv::waitKey(100);
        }
        return img;
    }
}  // namespace erl::env

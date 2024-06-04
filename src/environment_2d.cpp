#include "erl_common/random.hpp"
#include "erl_env/environment_2d.hpp"

namespace erl::env {

    Environment2D::Environment2D(
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        std::shared_ptr<Setting> setting,
        std::shared_ptr<CostBase> distance_cost_func)
        : EnvironmentBase(std::move(distance_cost_func)),
          m_setting_(std::move(setting)),
          m_grid_map_info_((assert(grid_map != nullptr), grid_map->info)) {  // x to the bottom, y to the right, along y first

        if (m_setting_ == nullptr) { m_setting_ = std::make_shared<Setting>(); }

        // init grid map
        InitializeGridMap2D(grid_map, m_original_grid_map_);
        m_original_grid_map_.copyTo(m_grid_map_);
        m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());
        if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, m_grid_map_info_, m_setting_->shape); }

        // compute relative trajectories and costs of each control
        m_rel_trajectories_.reserve(m_setting_->motions.size());
        m_motion_costs_.reserve(m_setting_->motions.size());
        EnvironmentState ref_start;
        ref_start.metric = m_grid_map_info_->Center();
        ref_start.grid = m_grid_map_info_->CenterGrid();
        for (const Eigen::Vector2i &control: m_setting_->motions) {
            EnvironmentState end;
            end.grid = ref_start.grid + control;
            end.metric = m_grid_map_info_->GridToMeterForPoints(end.grid);
            Eigen::Matrix2Xi rel_trajectory = m_grid_map_info_->RayCasting(ref_start.metric, end.metric);
            rel_trajectory.colwise() -= ref_start.grid;
            double control_cost = (*m_distance_cost_func_)(ref_start, end);
            m_rel_trajectories_.push_back(rel_trajectory);
            m_motion_costs_.push_back(control_cost);
        }
    }

    std::vector<Successor>
    Environment2D::GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const {
        if (!InStateSpace(env_state)) { return {}; }

        int cur_xg = env_state->grid[0];
        int cur_yg = env_state->grid[1];
        auto num_motions = int(m_rel_trajectories_.size());
        auto &reachable_motions = const_cast<std::vector<int> &>(m_reachable_motions_(cur_xg, cur_yg));
        if (reachable_motions.empty()) {
            for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                auto &rel_trajectory = m_rel_trajectories_[motion_idx];
                bool is_reachable = true;
                for (long i = 0; i < rel_trajectory.cols(); ++i) {
                    int nx_grid = cur_xg + rel_trajectory(0, i);
                    if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
                        is_reachable = false;
                        break;
                    }
                    int ny_grid = cur_yg + rel_trajectory(1, i);
                    if (ny_grid < 0 || ny_grid >= m_grid_map_info_->Shape(1)) {
                        is_reachable = false;
                        break;
                    }
                    if (m_grid_map_.at<uint8_t>(nx_grid, ny_grid) >= m_setting_->obstacle_threshold) {
                        is_reachable = false;
                        break;
                    }
                }
                if (is_reachable) { reachable_motions.push_back(motion_idx); }
            }
            if (reachable_motions.empty()) { reachable_motions.push_back(-1); }
        }
        if (reachable_motions[0] == -1) { return {}; }  // no reachable control

        std::vector<Successor> successors;
        successors.clear();
        successors.reserve(reachable_motions.size());
        for (auto &motion_idx: reachable_motions) {
            ERL_DEBUG_ASSERT(motion_idx >= 0 && motion_idx < num_motions, "Invalid motion index: {}.", motion_idx);
            auto next_state = std::make_shared<EnvironmentState>();
            next_state->grid = env_state->grid + m_setting_->motions[motion_idx];
            next_state->metric = GridToMetric(next_state->grid);
            if (m_setting_->add_map_cost) {
                double map_cost = m_setting_->map_cost_factor * double(m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]));
                double cost = m_motion_costs_[motion_idx] + map_cost;
                successors.emplace_back(next_state, cost, std::vector<int>{});
            } else {
                successors.emplace_back(next_state, m_motion_costs_[motion_idx], std::vector<int>{});
            }
            successors.back().action_coords.reserve(2);  // reserve one more for multi resolution search
            successors.back().action_coords.push_back(motion_idx);
        }
        // for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
        //     auto &direction = m_grid_motion_primitive_.controls[control_idx];
        //     bool is_reachable = true;
        //     auto next_state = std::make_shared<EnvironmentState>();
        //     next_state->grid = env_state->grid;
        //     int &nx_grid = next_state->grid[0];
        //     int &ny_grid = next_state->grid[1];
        //     for (long i = 0; i < m_setting_->max_axis_step; ++i) {
        //         nx_grid += direction[0];
        //         if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
        //             is_reachable = false;
        //             break;
        //         }
        //         ny_grid += direction[1];
        //         if (ny_grid < 0 || ny_grid >= m_grid_map_info_->Shape(1)) {
        //             is_reachable = false;
        //             break;
        //         }
        //         if (m_grid_map_.at<uint8_t>(nx_grid, ny_grid) >= m_setting_->obstacle_threshold) {
        //             is_reachable = false;
        //             break;
        //         }
        //     }
        //     if (!is_reachable) { continue; }
        //     next_state->metric = GridToMetric(next_state->grid);
        //
        //     if (m_setting_->add_map_cost) {
        //         double map_cost = m_setting_->map_cost_factor * double(m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]));
        //         double cost = m_grid_motion_primitive_.costs[control_idx] + map_cost;
        //         successors.emplace_back(next_state, cost, std::vector<int>{});
        //     } else {
        //         successors.emplace_back(next_state, m_grid_motion_primitive_.costs[control_idx], std::vector<int>{});
        //     }
        //     successors.back().action_coords.reserve(2);  // reserve one more for multi resolution search
        //     successors.back().action_coords.push_back(control_idx);
        // }
        return successors;
    }

    cv::Mat
    Environment2D::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const {
        cv::Mat img = m_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        std::uniform_int_distribution dist(0, 255);
        for (const auto &[kGoalIndex, kPath]: paths) {
            std::vector<cv::Point> points;
            auto num_points = kPath.cols();
            points.reserve(num_points);
            for (long i = 0; i < num_points; ++i) {
                auto grid_point = MetricToGrid(kPath.col(i));
                points.emplace_back(grid_point[1], grid_point[0]);
            }
            cv::Scalar color(dist(common::g_random_engine), dist(common::g_random_engine), dist(common::g_random_engine));
            cv::polylines(img, points, false, color, 1);
        }
        cv::namedWindow("environment 2d: paths", cv::WINDOW_NORMAL);
        cv::imshow("environment 2d: paths", img);
        if (block) {
            cv::waitKey(0);
        } else {
            cv::waitKey(100);
        }
        return img;
    }
}  // namespace erl::env

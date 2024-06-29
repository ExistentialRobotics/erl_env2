#include "erl_env/environment_2d.hpp"

#include "erl_common/random.hpp"

namespace erl::env {

    Environment2D::Environment2D(
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        std::shared_ptr<Setting> setting,
        std::shared_ptr<CostBase> distance_cost_func)
        : EnvironmentBase(std::move(distance_cost_func)),
          m_setting_(std::move(setting)),
          m_grid_map_info_((assert(grid_map != nullptr), grid_map->info)) {  // x to the bottom, y to the right, along y first

        if (m_setting_ == nullptr) {
            m_setting_ = std::make_shared<Setting>();
            m_setting_->SetGridMotionPrimitive(1, true);
        }

        // init grid map
        InitializeGridMap2D(grid_map, m_original_grid_map_);
        m_original_grid_map_.copyTo(m_grid_map_);
        m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());
        if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, m_grid_map_info_, m_setting_->shape); }

        InitRelTrajectoriesAndCosts();
    }

    Environment2D::Environment2D(
        std::shared_ptr<common::GridMapInfo2D> grid_map_info,
        const cv::Mat &cost_map,
        std::shared_ptr<Setting> setting,
        std::shared_ptr<CostBase> distance_cost_func)
        : EnvironmentBase(std::move(distance_cost_func)),
          m_setting_(std::move(setting)),
          m_original_grid_map_(cost_map.clone()),
          m_grid_map_info_(std::move(grid_map_info)) {

        if (m_setting_ == nullptr) {
            m_setting_ = std::make_shared<Setting>();
            m_setting_->SetGridMotionPrimitive(1, true);
        }

        // init grid map
        m_original_grid_map_.copyTo(m_grid_map_);
        m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());
        if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, m_grid_map_info_, m_setting_->shape); }

        InitRelTrajectoriesAndCosts();
    }

    std::vector<Successor>
    Environment2D::GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const {
        if (!InStateSpace(env_state)) { return {}; }

        const int cur_xg = env_state->grid[0];
        const int cur_yg = env_state->grid[1];
        const auto num_motions = static_cast<int>(m_rel_trajectories_.size());
        auto &reachable_motions = const_cast<std::vector<int> &>(m_reachable_motions_(cur_xg, cur_yg));
        if (reachable_motions.empty()) {
            for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                auto &rel_trajectory = m_rel_trajectories_[motion_idx];
                bool is_reachable = true;
                for (long i = 0; i < rel_trajectory.cols(); ++i) {
                    const int nx_grid = cur_xg + rel_trajectory(0, i);
                    if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
                        is_reachable = false;
                        break;
                    }
                    const int ny_grid = cur_yg + rel_trajectory(1, i);
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
                const double map_cost = m_setting_->map_cost_factor * static_cast<double>(m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]));
                double cost = m_motion_costs_[motion_idx] + map_cost;
                successors.emplace_back(next_state, cost, std::vector<int>{});
            } else {
                successors.emplace_back(next_state, m_motion_costs_[motion_idx], std::vector<int>{});
            }
            successors.back().action_coords.reserve(2);  // reserve one more for multi resolution search
            successors.back().action_coords.push_back(motion_idx);
        }
        return successors;
    }

    cv::Mat
    Environment2D::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, const bool block) const {
        cv::Mat img = m_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        std::uniform_int_distribution dist(0, 255);
        for (const auto &[kGoalIndex, kPath]: paths) {
            std::vector<cv::Point> points;
            const long num_points = kPath.cols();
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

    void
    Environment2D::InitRelTrajectoriesAndCosts() {
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

    std::vector<std::shared_ptr<EnvironmentState>>
    Environment2D::SampleValidStates(const int num_samples) const {
        if (m_grid_map_.rows * m_grid_map_.cols == 0) { throw std::runtime_error("collision map is empty"); }
        auto x_rng = std::uniform_int_distribution(0, m_grid_map_.rows - 1);
        auto y_rng = std::uniform_int_distribution(0, m_grid_map_.cols - 1);
        std::vector<std::shared_ptr<EnvironmentState>> states;
        states.reserve(num_samples);
        while (states.size() < static_cast<std::size_t>(num_samples)) {
            if (const int x = x_rng(common::g_random_engine), y = y_rng(common::g_random_engine);
                m_grid_map_.at<uint8_t>(x, y) < m_setting_->obstacle_threshold) {
                auto state = std::make_shared<EnvironmentState>();
                state->grid = Eigen::Vector2i(x, y);
                state->metric = GridToMetric(state->grid);
                states.push_back(state);
            }
        }
        return states;
    }

}  // namespace erl::env

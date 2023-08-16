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
        m_original_grid_map_ = InitializeGridMap2D(grid_map);
        m_original_grid_map_.copyTo(m_grid_map_);

        if (m_setting_->allow_diagonal) {
            m_grid_motion_primitive_.controls = {
                {1, 0},    // kForward
                {-1, 0},   // kBack
                {0, 1},    // kRight
                {0, -1},   // kLeft
                {1, 1},    // kForwardRight
                {1, -1},   // kForwardLeft
                {-1, 1},   // kBackRight
                {-1, -1},  // kBackLeft
            };
        } else {
            m_grid_motion_primitive_.controls = {
                {1, 0},   // kForward
                {-1, 0},  // kBack
                {0, 1},   // kRight
                {0, -1},  // kLeft
            };
        }

        m_grid_motion_primitive_.durations.resize(m_grid_motion_primitive_.controls.size(), 1);
        m_grid_motion_primitive_.costs.reserve(m_grid_motion_primitive_.controls.size());
        EnvironmentState state0, state1;
        state0.grid = Eigen::VectorXi::Zero(2);
        state0.metric = m_grid_map_info_->GridToMeterForPoints(state0.grid);

        for (auto &control: m_grid_motion_primitive_.controls) {
            state1.grid = state0.grid + control * m_setting_->step_size;
            state1.metric = m_grid_map_info_->GridToMeterForPoints(state1.grid);
            m_grid_motion_primitive_.costs.push_back((*m_distance_cost_func_)(state0, state1));  // compute distance cost in metric space
        }

        if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, m_grid_map_info_, m_setting_->shape); }
    }

    std::vector<Successor>
    Environment2D::GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const {
        if (!InStateSpace(env_state)) { return {}; }

        std::vector<Successor> successors;
        auto num_controls = int(m_grid_motion_primitive_.controls.size());
        successors.clear();
        successors.reserve(num_controls);
        for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
            auto &direction = m_grid_motion_primitive_.controls[control_idx];
            bool is_reachable = true;
            auto next_state = std::make_shared<EnvironmentState>();
            next_state->grid = env_state->grid;
            int &nx_grid = next_state->grid[0];
            int &ny_grid = next_state->grid[1];
            for (long i = 0; i < m_setting_->step_size; ++i) {
                nx_grid += direction[0];
                if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
                    is_reachable = false;
                    break;
                }
                ny_grid += direction[1];
                if (ny_grid < 0 || ny_grid >= m_grid_map_info_->Shape(1)) {
                    is_reachable = false;
                    break;
                }
                if (m_grid_map_.at<uint8_t>(nx_grid, ny_grid) >= m_setting_->obstacle_threshold) {
                    is_reachable = false;
                    break;
                }
            }
            if (!is_reachable) { continue; }
            next_state->metric = GridToMetric(next_state->grid);

            if (m_setting_->add_map_cost) {
                double map_cost = m_setting_->map_cost_factor * double(m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]));
                double cost = m_grid_motion_primitive_.costs[control_idx] + map_cost;
                successors.emplace_back(next_state, cost, std::vector<int>{});
            } else {
                successors.emplace_back(next_state, m_grid_motion_primitive_.costs[control_idx], std::vector<int>{});
            }
            successors.back().action_coords.reserve(2);  // reserve one more for multi resolution search
            successors.back().action_coords.push_back(control_idx);
        }
        return successors;
    }

    cv::Mat
    Environment2D::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
        cv::Mat img = m_grid_map_ * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        std::uniform_int_distribution random_int(0, 255);
        for (const auto &[kGoalIndex, kPath]: paths) {
            std::vector<cv::Point> points;
            auto num_points = kPath.cols();
            points.reserve(num_points);
            for (long i = 0; i < num_points; ++i) {
                auto grid_point = MetricToGrid(kPath.col(i));
                points.emplace_back(grid_point[1], grid_point[0]);
            }
            cv::Scalar color(random_int(common::g_random_engine), random_int(common::g_random_engine), random_int(common::g_random_engine));
            cv::polylines(img, points, false, color, 1);
        }
        cv::namedWindow("environment 2d: paths", cv::WINDOW_NORMAL);
        cv::imshow("environment 2d: paths", img);
        cv::waitKey(100);
        return img;
    }
}  // namespace erl::env

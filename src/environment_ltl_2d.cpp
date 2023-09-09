#include "erl_env/environment_ltl_2d.hpp"

namespace erl::env {

    EnvironmentLTL2D::EnvironmentLTL2D(
        Eigen::MatrixX<uint64_t> label_map,
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        std::shared_ptr<Setting> setting,
        std::shared_ptr<CostBase> distance_cost_func)
        : EnvironmentBase(std::move(distance_cost_func)),
          m_setting_(std::move(setting)),
          m_label_map_(std::move(label_map)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        ERL_ASSERTM(grid_map != nullptr, "grid_map is nullptr.");
        ERL_ASSERTM(m_setting_->fsa->atomic_propositions.size() <= 64, "Does not support more than 64 atomic propositions.");

        auto num_states = int(m_setting_->fsa->num_states);
        if (num_states % 2 == 0) { num_states += 1; }
        m_grid_map_info_ = std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(num_states, -0.5, double(num_states) - 0.5, 2));

        // generate motions and compute their costs
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
        state0.metric = grid_map->info->GridToMeterForPoints(state0.grid);
        for (auto &control: m_grid_motion_primitive_.controls) {
            state1.grid = state0.grid + control * m_setting_->step_size;
            state1.metric = grid_map->info->GridToMeterForPoints(state1.grid);
            m_grid_motion_primitive_.costs.push_back((*m_distance_cost_func_)(state0, state1));  // compute distance cost in metric space
        }

        // generate 2D obstacle/cost map
        InitializeGridMap2D(grid_map, m_original_grid_map_);
        m_original_grid_map_.copyTo(m_grid_map_);
        if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, grid_map->info, m_setting_->shape); }

        // configure finite state automaton
        ERL_ASSERTM(m_setting_->fsa != nullptr, "setting->fsa is nullptr.");
        m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
        ERL_ASSERTM(m_grid_map_.rows == m_label_map_.rows(), "label_map and grid_map should have the same number of rows.");
        ERL_ASSERTM(m_grid_map_.cols == m_label_map_.cols(), "label_map and grid_map should have the same number of columns.");
    }

    std::vector<std::shared_ptr<EnvironmentState>>
    EnvironmentLTL2D::ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
        ERL_ASSERTM(action_coords.size() == 1, "Invalid action_coords size: %lu.", action_coords.size());
        auto new_state = std::make_shared<EnvironmentState>();
        new_state->grid.resize(3);
        int &nx_grid = new_state->grid[0];
        int &ny_grid = new_state->grid[1];
        int &nq = new_state->grid[2];
        auto &control = m_grid_motion_primitive_.controls[action_coords[0]];
        nx_grid = env_state->grid[0] + control[0];
        ny_grid = env_state->grid[1] + control[1];
        nq = int(m_fsa_->GetNextState(env_state->grid[2], m_label_map_(nx_grid, ny_grid)));

        new_state->metric.resize(3);
        new_state->metric[0] = m_grid_map_info_->GridToMeterForValue(nx_grid, 0);
        new_state->metric[1] = m_grid_map_info_->GridToMeterForValue(ny_grid, 1);
        new_state->metric[2] = new_state->grid[2];
        return {new_state};
    }

    std::vector<Successor>
    EnvironmentLTL2D::GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const {
        if (!InStateSpace(env_state)) { return {}; }

        std::vector<Successor> successors;
        auto num_controls = int(m_grid_motion_primitive_.controls.size());
        successors.reserve(num_controls);
        int &cur_q = env_state->grid[2];
        for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
            auto &direction = m_grid_motion_primitive_.controls[control_idx];
            bool is_reachable = true;
            auto next_state = std::make_shared<EnvironmentState>();
            next_state->grid = env_state->grid;
            int &nx_grid = next_state->grid[0];
            int &ny_grid = next_state->grid[1];
            int &nq = next_state->grid[2];

            for (long i = 0; i < m_setting_->step_size; ++i) {
                nx_grid += direction[0];
                if (nx_grid < 0 || nx_grid > m_grid_map_info_->Shape(0)) {
                    is_reachable = false;
                    break;
                }
                ny_grid += direction[1];
                if (ny_grid < 0 || ny_grid > m_grid_map_info_->Shape(1)) {
                    is_reachable = false;
                    break;
                }
                // check collision
                if (m_grid_map_.at<uint8_t>(nx_grid, ny_grid) >= m_setting_->obstacle_threshold) {
                    is_reachable = false;
                    break;
                }
                // check LTL
                nq = int(m_fsa_->GetNextState(cur_q, m_label_map_(nx_grid, ny_grid)));
                if (m_fsa_->IsSinkState(nq)) {
                    is_reachable = false;
                    break;
                }
            }
            if (!is_reachable) { continue; }
            next_state->metric.resize(3);
            next_state->metric[0] = m_grid_map_info_->GridToMeterForValue(nx_grid, 0);
            next_state->metric[1] = m_grid_map_info_->GridToMeterForValue(ny_grid, 1);
            next_state->metric[2] = nq;

            if (m_setting_->add_map_cost) {
                double map_cost = m_setting_->map_cost_factor * double(m_grid_map_.at<uint8_t>(nx_grid, ny_grid));
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
    EnvironmentLTL2D::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
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

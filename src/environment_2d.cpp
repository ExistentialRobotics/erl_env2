#include "erl_common/random.hpp"
#include "erl_env/environment_2d.hpp"

namespace erl::env {

    Environment2D::Environment2D(
        bool allow_diagonal,
        int step_size,
        bool down_sampled,
        std::shared_ptr<CostBase> cost_func,
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        uint8_t obstacle_threshold,
        bool add_map_cost,
        double map_cost_factor)
        : EnvironmentBase(std::move(cost_func)),
          m_step_size_(step_size),
          m_down_sampled_(down_sampled),
          m_obstacle_threshold_(obstacle_threshold),
          m_add_map_cost_(add_map_cost),
          m_map_cost_factor_(map_cost_factor),
          m_grid_map_info_(grid_map->info) {  // x to the bottom, y to the right, along y first

        // init grid map
        m_original_grid_map_ = InitializeGridMap2D(grid_map);
        m_original_grid_map_.copyTo(m_grid_map_);

        if (allow_diagonal) {
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
            state1.grid = state0.grid + control * m_step_size_;
            state1.metric = m_grid_map_info_->GridToMeterForPoints(state1.grid);
            m_grid_motion_primitive_.costs.push_back((*m_distance_cost_func_)(state0, state1));  // compute distance cost in metric space
        }
    }

    Environment2D::Environment2D(
        bool allow_diagonal,
        int step_size,
        bool down_sampled,
        const std::shared_ptr<CostBase> &cost_func,
        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
        uint8_t obstacle_threshold,
        double inflate_scale,
        const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices,
        bool add_map_cost,
        double map_cost_factor)
        : Environment2D(allow_diagonal, step_size, down_sampled, cost_func, grid_map, obstacle_threshold, add_map_cost, map_cost_factor) {

        if (shape_metric_vertices.cols() == 0) {
            ERL_WARN("shape_metric_vertices is empty, no inflation");
            return;
        }
        if (inflate_scale <= 0) {
            ERL_WARN("inflate_scale <= 0, no inflation");
            return;
        }
        m_shape_metric_vertices_ = shape_metric_vertices.array() * inflate_scale;
        InflateGridMap2D(m_original_grid_map_, m_grid_map_, grid_map->info, m_shape_metric_vertices_);
    }

    std::vector<Successor>
    Environment2D::GetSuccessors(const std::shared_ptr<EnvironmentState> &state) const {
        if (!InStateSpace(state)) { return {}; }

        std::vector<Successor> successors;
        auto num_controls = int(m_grid_motion_primitive_.controls.size());
        successors.clear();
        successors.reserve(num_controls);
        for (int control_idx = 0; control_idx < num_controls; control_idx++) {
            auto &direction = m_grid_motion_primitive_.controls[control_idx];
            bool is_reachable = true;
            auto next_state = std::make_shared<EnvironmentState>();
            next_state->grid = state->grid;
            for (long i = 0; i < m_step_size_; ++i) {
                next_state->grid += direction;
                if (!InStateSpace(next_state) || m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]) >= m_obstacle_threshold_) {
                    is_reachable = false;
                    break;
                }
            }
            if (!is_reachable) { continue; }
            next_state->metric = GridToMetric(next_state->grid);

            if (m_add_map_cost_) {
                double map_cost = m_map_cost_factor_ * double(m_grid_map_.at<uint8_t>(next_state->grid[0], next_state->grid[1]));
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

#pragma once

#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"
#include "environment_base.hpp"
#include "motion_primitive.hpp"
#include "cost.hpp"
#include <opencv2/highgui.hpp>

namespace erl::env {

    class Environment2D : virtual public EnvironmentBase {  // virtual inheritance is used to avoid diamond problem

    public:
        enum class Action {
            /*          Left
             *            |
             *  Back -----|------x  Forward
             *            |
             *            y
             *          Right
             */
            kBack = 0,
            kBackRight = 1,
            kRight = 2,
            kForwardRight = 3,
            kForward = 4,
            kForwardLeft = 5,
            kLeft = 6,
            kBackLeft = 7
        };

        static inline const char *
        GetActionName(const Action &action) {
            static const char *names[] = {
                ERL_AS_STRING(kForward),
                ERL_AS_STRING(kBack),
                ERL_AS_STRING(kRight),
                ERL_AS_STRING(kLeft),
                ERL_AS_STRING(kForwardRight),
                ERL_AS_STRING(kForwardLeft),
                ERL_AS_STRING(kBackRight),
                ERL_AS_STRING(kBackLeft)};

            return names[int(action)];
        }

        static inline Action
        GetActionFromName(const std::string &action_name) {
            if (action_name == ERL_AS_STRING(kForward)) { return Action::kForward; }
            if (action_name == ERL_AS_STRING(kBack)) { return Action::kBack; }
            if (action_name == ERL_AS_STRING(kRight)) { return Action::kRight; }
            if (action_name == ERL_AS_STRING(kLeft)) { return Action::kLeft; }
            if (action_name == ERL_AS_STRING(kForwardRight)) { return Action::kForwardRight; }
            if (action_name == ERL_AS_STRING(kForwardLeft)) { return Action::kForwardLeft; }
            if (action_name == ERL_AS_STRING(kBackRight)) { return Action::kBackRight; }
            if (action_name == ERL_AS_STRING(kBackLeft)) { return Action::kBackLeft; }

            throw std::runtime_error("Unknown Action: " + action_name);
        }

        typedef MotionPrimitive<Eigen::Vector2i> GridMotionPrimitive;  // control signal is 2D-grid movement

    protected:
        int m_step_size_ = 1;                                     // step size in grid space
        bool m_down_sampled_ = false;                             // indicate whether the state space is down sampled by m_step_size_
        GridMotionPrimitive m_grid_motion_primitive_;             // motion primitives in grid space
        std::vector<double> m_motion_cost_;                       // cost of each motion primitive
        uint8_t m_obstacle_threshold_ = 1;                        // minimum map value to be considered as obstacle
        bool m_add_map_cost_ = false;                             // indicate whether to add map cost to the successor cost
        double m_map_cost_factor_ = 1.0;                          // map cost = map_cost_factor * map_cost
        cv::Mat m_original_grid_map_;                             // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;                                      // inflated grid map
        std::shared_ptr<common::GridMapInfo2D> m_grid_map_info_;  // grid map description, x to the bottom, y to the right, along y first
        Eigen::Matrix2Xd m_shape_metric_vertices_;                // vertices of the shape in metric space, assume the shape center is at the origin

    public:
        Environment2D(
            bool allow_diagonal,
            int step_size,
            bool down_sampled,
            std::shared_ptr<CostBase> cost_func,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
            uint8_t obstacle_threshold = 1,
            bool add_map_cost = false,
            double map_cost_factor = 1.0);

        Environment2D(
            bool allow_diagonal,
            int step_size,
            bool down_sampled,
            const std::shared_ptr<CostBase> &cost_func,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,       // x to the bottom, y to the right, along y first
            uint8_t obstacle_threshold,                                       // obstacle threshold, if <=0, use the default value
            double inflate_scale,                                             // inflate the map by inflate_scale * shape_metric_vertices, if <=0, no inflation
            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices,  // assume the shape center is at the origin
            bool add_map_cost = false,
            double map_cost_factor = 1.0);

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            return m_grid_motion_primitive_.controls.size();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &state, const std::vector<int> &action_coords) const override {
            ERL_ASSERTM(action_coords.size() == 1, "Invalid action_coords size: %lu.", action_coords.size());
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->grid = state->grid + m_grid_motion_primitive_.controls[action_coords[0]];
            new_state->metric = GridToMetric(new_state->grid);
            return {new_state};
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &state) const override;

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &state) const override {
            return m_grid_map_info_->InGrids(state->grid) && (!m_down_sampled_ || (state->grid[0] % m_step_size_ == 0 && state->grid[1] % m_step_size_ == 0));
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const override {
            return m_grid_map_info_->GridToIndex(state->grid, true);  // row-major
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector2i grid_state(m_grid_map_info_->MeterToGridForValue(metric_state[0], 0), m_grid_map_info_->MeterToGridForValue(metric_state[1], 1));
            return grid_state;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector2d metric_state(m_grid_map_info_->GridToMeterForValue(grid_state[0], 0), m_grid_map_info_->GridToMeterForValue(grid_state[1], 1));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override;
    };

}  // namespace erl::env

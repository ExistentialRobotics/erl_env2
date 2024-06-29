// ReSharper disable CppInconsistentNaming
#pragma once

#include "cost.hpp"
#include "environment_base.hpp"

#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

namespace erl::env {

    class Environment2D : public EnvironmentBase {

    public:
        struct Setting : common::Yamlable<Setting> {
            std::vector<Eigen::Vector2i> motions;  // 2d grid motions
            int grid_stride = 1;                   // grid stride in grid space
            uint8_t obstacle_threshold = 1;        // minimum map value to be considered as obstacle
            bool add_map_cost = false;             // indicate whether to add map cost to the successor cost
            double map_cost_factor = 1.0;          // map cost = map_cost_factor * map_cost
            Eigen::Matrix2Xd shape = {};           // assume the shape center is at the origin

            void
            SetGridMotionPrimitive(const int max_axis_step, const bool allow_diagonal) {
                motions.clear();
                ERL_ASSERTM(max_axis_step > 0, "max_axis_step must be positive.");
                int num_controls = max_axis_step * 2 + 1;
                if (allow_diagonal) {
                    num_controls = num_controls * num_controls - 1;
                    motions.reserve(num_controls);
                    for (int i = -max_axis_step; i <= max_axis_step; ++i) {
                        for (int j = -max_axis_step; j <= max_axis_step; ++j) {
                            if (i == 0 && j == 0) { continue; }
                            motions.emplace_back(i, j);
                        }
                    }
                } else {
                    num_controls = num_controls * 2 - 2;
                    motions.reserve(num_controls);
                    for (int i = -max_axis_step; i <= max_axis_step; ++i) {
                        if (i == 0) { continue; }
                        motions.emplace_back(i, 0);
                        motions.emplace_back(0, i);
                    }
                }
            }
        };

    protected:
        std::shared_ptr<Setting> m_setting_;                      // environment setting
        std::vector<Eigen::Matrix2Xi> m_rel_trajectories_;        // relative trajectories of motion primitives
        std::vector<double> m_motion_costs_;                      // cost of each motion
        Eigen::MatrixX<std::vector<int>> m_reachable_motions_;    // reachable controls for each grid
        cv::Mat m_original_grid_map_;                             // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;                                      // inflated grid map
        std::shared_ptr<common::GridMapInfo2D> m_grid_map_info_;  // grid map description, x to the bottom, y to the right, along y first

    public:
        explicit Environment2D(
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,  // x to the bottom, y to the right, along y first
            std::shared_ptr<Setting> setting,
            std::shared_ptr<CostBase> distance_cost_func = nullptr);

        Environment2D(
            std::shared_ptr<common::GridMapInfo2D> grid_map_info,
            const cv::Mat &cost_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<CostBase> distance_cost_func = nullptr);

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
            return m_setting_->motions.size();
        }

        [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
            ERL_ASSERTM(action_coords.size() == 1, "Invalid action_coords size: %lu.", action_coords.size());
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->grid = env_state->grid + m_setting_->motions[action_coords[0]];
            new_state->metric = GridToMetric(new_state->grid);
            return {new_state};
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override;

        [[nodiscard]] bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid) &&
                   (m_setting_->grid_stride == 1 || (env_state->grid[0] % m_setting_->grid_stride == 0 && env_state->grid[1] % m_setting_->grid_stride == 0));
        }

        [[nodiscard]] uint32_t
        StateHashing(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state->grid, true);  // row-major
        }

        [[nodiscard]] Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector2i grid_state(m_grid_map_info_->MeterToGridForValue(metric_state[0], 0), m_grid_map_info_->MeterToGridForValue(metric_state[1], 1));
            return grid_state;
        }

        [[nodiscard]] Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector2d metric_state(m_grid_map_info_->GridToMeterForValue(grid_state[0], 0), m_grid_map_info_->GridToMeterForValue(grid_state[1], 1));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const override;

        std::vector<std::shared_ptr<EnvironmentState>> SampleValidStates(int num_samples) const override;

    private:
        void
        InitRelTrajectoriesAndCosts();
    };

}  // namespace erl::env

template<>
struct YAML::convert<erl::env::Environment2D::Setting> {
    static Node
    encode(const erl::env::Environment2D::Setting &rhs) {
        Node node;
        node["motions"] = rhs.motions;
        node["grid_stride"] = rhs.grid_stride;
        node["obstacle_threshold"] = rhs.obstacle_threshold;
        node["add_map_cost"] = rhs.add_map_cost;
        node["map_cost_factor"] = rhs.map_cost_factor;
        node["shape"] = rhs.shape;
        return node;
    }

    static bool
    decode(const Node &node, erl::env::Environment2D::Setting &rhs) {
        rhs.motions = node["motions"].as<std::vector<Eigen::Vector2i>>();
        rhs.grid_stride = node["grid_stride"].as<int>();
        rhs.obstacle_threshold = node["obstacle_threshold"].as<uint8_t>();
        rhs.add_map_cost = node["add_map_cost"].as<bool>();
        rhs.map_cost_factor = node["map_cost_factor"].as<double>();
        rhs.shape = node["shape"].as<Eigen::Matrix2Xd>();
        return true;
    }
};  // namespace YAML

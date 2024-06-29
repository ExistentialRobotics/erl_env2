#pragma once

// https://github.com/ExistentialRobotics/erl_astar/blob/master/test/old/test_astar_2d_ltl.cpp
// https://github.com/ExistentialRobotics/erl_env/blob/master/include/erl_env/ltl/fsa.h
// https://github.com/ExistentialRobotics/erl_astar/blob/master/data/maps/ltl/mapjie_2d.yaml

#include "cost.hpp"
#include "environment_base.hpp"
#include "finite_state_automaton.hpp"
#include "motion_primitive.hpp"

#include "erl_common/grid_map.hpp"
#include "erl_common/random.hpp"

#include <numeric>

namespace erl::env {

    /**
     * 2D Grid Environment with Linear Temporal Logic. The state is (x, y, ltl_state).
     */
    class EnvironmentLTL2D : public EnvironmentBase {
    public:
        struct Setting : public common::Yamlable<Setting> {
            std::vector<Eigen::Vector2i> motions;                // 2d grid motions
            int grid_stride = 1;                                 // grid stride in grid space
            uint8_t obstacle_threshold = 1;                      // minimum map value to be considered as obstacle
            bool add_map_cost = false;                           // indicate whether to add map cost to the successor cost
            double map_cost_factor = 1.0;                        // map cost = map_cost_factor * map_cost
            Eigen::Matrix2Xd shape = {};                         // assume the shape center is at the origin
            std::shared_ptr<FiniteStateAutomaton::Setting> fsa;  // finite state automaton

            inline void
            SetGridMotionPrimitive(int max_axis_step, bool allow_diagonal) {
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

    private:
        std::shared_ptr<Setting> m_setting_;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_;
        std::vector<Eigen::Matrix2Xi> m_rel_trajectories_;        // relative trajectories of motion primitives
        std::vector<double> m_motion_costs_;                      // cost of each control
        Eigen::MatrixX<std::vector<int>> m_reachable_motions_;    // reachable controls for each grid
        cv::Mat m_original_grid_map_;                             // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;                                      // inflated grid map
        std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_;  // grid map description (x, y, q), x to the bottom, y to the right, along y first
        Eigen::MatrixX<uint32_t> m_label_map_;                    // each element is a |AP|-bit word representing the result of atomic propositions

    public:
        EnvironmentLTL2D(
            Eigen::MatrixX<uint32_t> label_map,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<CostBase> distance_cost_func = nullptr);

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline std::shared_ptr<FiniteStateAutomaton>
        GetFiniteStateAutomaton() const {
            return m_fsa_;
        }

        [[nodiscard]] inline std::shared_ptr<common::GridMapInfo3D>
        GetGridMapInfo() const {
            return m_grid_map_info_;
        }

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size() * m_setting_->fsa->num_states;
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            return m_setting_->motions.size();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override;

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override;

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid) &&
                   (m_setting_->grid_stride == 1 || (env_state->grid[0] % m_setting_->grid_stride == 0 && env_state->grid[1] % m_setting_->grid_stride == 0));
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            uint32_t hashing = m_grid_map_info_->GridToIndex(env_state->grid, true);
            return hashing;
            // equivalent to
            // uint32_t hashing = m_grid_map_info_->GridToIndex(env_state->grid.head<2>(), true);
            // hashing = hashing * m_setting_->fsa->num_states + env_state->grid[2];
            // return hashing;
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector3i grid_state(
                m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
                m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
                int(metric_state[2]));
            return grid_state;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector3d metric_state(
                m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
                m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
                double(grid_state[2]));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const override;

        [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
        SampleValidStates(int num_samples) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };
}  // namespace erl::env

namespace YAML {
    template<>
    struct convert<erl::env::EnvironmentLTL2D::Setting> {
        inline static Node
        encode(const erl::env::EnvironmentLTL2D::Setting &rhs) {
            Node node;
            node["motions"] = rhs.motions;
            node["grid_stride"] = rhs.grid_stride;
            node["obstacle_threshold"] = rhs.obstacle_threshold;
            node["add_map_cost"] = rhs.add_map_cost;
            node["map_cost_factor"] = rhs.map_cost_factor;
            node["shape"] = rhs.shape;
            node["fsa"] = rhs.fsa;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::env::EnvironmentLTL2D::Setting &rhs) {
            rhs.motions = node["motions"].as<std::vector<Eigen::Vector2i>>();
            rhs.grid_stride = node["grid_stride"].as<int>();
            rhs.obstacle_threshold = node["obstacle_threshold"].as<uint8_t>();
            rhs.add_map_cost = node["add_map_cost"].as<bool>();
            rhs.map_cost_factor = node["map_cost_factor"].as<double>();
            rhs.shape = node["shape"].as<Eigen::Matrix2Xd>();
            rhs.fsa = node["fsa"].as<std::shared_ptr<erl::env::FiniteStateAutomaton::Setting>>();
            return true;
        }
    };
}  // namespace YAML

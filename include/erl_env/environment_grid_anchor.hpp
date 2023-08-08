#pragma once

#include "erl_common/grid_map_info.hpp"
#include "environment_anchor.hpp"

namespace erl::env {

    template<int Dim>
    class EnvironmentGridAnchor : public EnvironmentAnchor {
        std::shared_ptr<common::GridMapInfo<Dim>> m_grid_map_info_;

    public:
        EnvironmentGridAnchor(std::vector<std::shared_ptr<env::EnvironmentBase>> environments, std::shared_ptr<common::GridMapInfo<Dim>> grid_map_info)
            : EnvironmentAnchor(std::move(environments)),
              m_grid_map_info_(std::move(grid_map_info)) {
            ERL_ASSERTM(m_grid_map_info_ != nullptr, "grid_map_info should not be nullptr.");
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            ERL_DEBUG_ASSERT(env_state->metric.size() == Dim, "state dimension is not equal to grid map dimension.");
            return m_grid_map_info_->GridToIndex(env_state->grid, true);  // row major
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector<int, Dim> grid;
            for (int i = 0; i < Dim; ++i) { grid[i] = m_grid_map_info_->MeterToGridForValue(metric_state[i], i); }
            return grid;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector<double, Dim> metric;
            for (int i = 0; i < Dim; ++i) { metric[i] = m_grid_map_info_->GridToMeterForValue(grid_state[i], i); }
            return metric;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };

    using EnvironmentGridAnchor2D = EnvironmentGridAnchor<2>;
    using EnvironmentGridAnchor3D = EnvironmentGridAnchor<3>;
}  // namespace erl::env

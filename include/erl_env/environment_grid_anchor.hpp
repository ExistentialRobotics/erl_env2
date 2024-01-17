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
            ERL_WARN_ONCE_COND(env_state->grid.size() > Dim, "only the first %d dimensions of grid state are used for hashing.", Dim);
            return m_grid_map_info_->GridToIndex(env_state->grid, true);  // row major
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::VectorXi grid;
            grid.resize(metric_state.size());
            for (int i = 0; i < Dim; ++i) { grid[i] = m_grid_map_info_->MeterToGridForValue(metric_state[i], i); }
            if (metric_state.size() > Dim) {
                ERL_WARN_ONCE(
                    "metric_state has more dimensions than grid map dimension, "
                    "the extra dimensions are casted to int directly, which may cause information loss.");
                for (int i = Dim; i < metric_state.size(); ++i) { grid[i] = int(metric_state[i]); }
            }
            return grid;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::VectorXd metric;
            metric.resize(grid_state.size());
            for (int i = 0; i < Dim; ++i) { metric[i] = m_grid_map_info_->GridToMeterForValue(grid_state[i], i); }
            if (grid_state.size() > Dim) {
                ERL_WARN_ONCE("grid_state has more dimensions than grid map dimension, the extra dimensions are casted to double directly.");
                for (int i = Dim; i < grid_state.size(); ++i) { metric[i] = double(grid_state[i]); }
            }
            return metric;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &, bool) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };

    using EnvironmentGridAnchor2D = EnvironmentGridAnchor<2>;
    using EnvironmentGridAnchor3D = EnvironmentGridAnchor<3>;
}  // namespace erl::env

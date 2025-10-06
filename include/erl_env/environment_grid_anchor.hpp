#pragma once

#include "environment_anchor.hpp"

#include "erl_common/exception.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/template_helper.hpp"

#include <memory>

namespace erl::env {

    template<typename Dtype, int Dim>
    class EnvironmentGridAnchor : public EnvironmentAnchor<Dtype, Dim> {
    public:
        using EnvBase = EnvironmentBase<Dtype, Dim>;
        using State = EnvironmentState<Dtype, Dim>;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using Successor_t = Successor<Dtype, Dim>;
        using GridMapInfo = common::GridMapInfo<Dtype, Dim>;

    protected:
        std::shared_ptr<GridMapInfo> m_grid_map_info_;

    public:
        EnvironmentGridAnchor(
            std::vector<std::shared_ptr<EnvBase>> environments,
            std::shared_ptr<GridMapInfo> grid_map_info)
            : EnvironmentAnchor<Dtype, Dim>(std::move(environments)),
              m_grid_map_info_(
                  NotNull(std::move(grid_map_info), true, "grid_map_info should not be nullptr.")) {
        }

        [[nodiscard]] uint32_t
        StateHashing(const State &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state.grid, true);  // row major
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            GridState grid;
            for (int i = 0; i < Dim; ++i) {
                grid[i] = m_grid_map_info_->MeterToGridAtDim(metric_state[i], i);
            }
            return grid;
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            MetricState metric;
            for (int i = 0; i < Dim; ++i) {
                metric[i] = m_grid_map_info_->GridToMeterAtDim(grid_state[i], i);
            }
            return metric;
        }

        // [[nodiscard]] cv::Mat
        // ShowPaths(const std::map<int, Eigen::MatrixXd> &, bool) const override {
        //     throw NotImplemented(__PRETTY_FUNCTION__);
        // }

        [[nodiscard]] std::vector<State>
        SampleValidStates(int /*num_samples*/) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };
}  // namespace erl::env

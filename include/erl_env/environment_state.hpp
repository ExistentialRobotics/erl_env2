#pragma once

#include "erl_common/eigen.hpp"

namespace erl::env {
    enum VirtualStateValue {  // used for multi-start and multi-goal search
        kStart = -2,
        kGoal = -1
    };

    template<typename Dtype, int Dim>
    struct EnvironmentState {
        using MetricState = Eigen::Vector<Dtype, Dim>;
        using GridState = Eigen::Vector<int, Dim>;

        MetricState metric = {};
        GridState grid = {};  // use signed int to allow for virtual states

        EnvironmentState() = default;

        explicit EnvironmentState(MetricState metric_state)
            : metric(std::move(metric_state)) {}

        explicit EnvironmentState(GridState grid_state)
            : grid(std::move(grid_state)) {}

        EnvironmentState(MetricState metric_state, GridState grid_state)
            : metric(std::move(metric_state)),
              grid(std::move(grid_state)) {}
    };
}  // namespace erl::env

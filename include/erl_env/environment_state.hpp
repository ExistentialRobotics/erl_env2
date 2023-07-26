#pragma once

#include "erl_common/eigen.hpp"

namespace erl::env {
    enum VirtualStateValue { kStart = -1, kGoal = -2 };  // used for multi-start and multi-goal search

    struct EnvironmentState {
        Eigen::VectorXd metric = {};
        Eigen::VectorXi grid = {};

        EnvironmentState() = default;

        EnvironmentState(Eigen::VectorXd metric_state, Eigen::VectorXi grid_state)
            : metric(std::move(metric_state)),
              grid(std::move(grid_state)) {}
    };
}  // namespace erl::env

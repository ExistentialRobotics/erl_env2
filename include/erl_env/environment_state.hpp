#pragma once

#include "erl_common/eigen.hpp"

namespace erl::env {
    enum VirtualStateValue { kStart = -2, kGoal = -1 };  // used for multi-start and multi-goal search

    struct EnvironmentState {
        static_assert(sizeof(void *) == sizeof(uint64_t), "not a 64-bit system.");

        uint64_t id = u_int64_t(this);
        Eigen::VectorXd metric = {};
        Eigen::VectorXi grid = {};

        EnvironmentState() = default;

        EnvironmentState(Eigen::VectorXd metric_state, Eigen::VectorXi grid_state)
            : metric(std::move(metric_state)),
              grid(std::move(grid_state)) {}
    };
}  // namespace erl::env

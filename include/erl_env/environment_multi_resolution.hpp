#pragma once

#include "environment_base.hpp"

namespace erl::env {

    template<typename Dtype, int Dim>
    struct EnvironmentMultiResolution : public EnvironmentBase<Dtype, Dim> {
        using State = EnvironmentState<Dtype, Dim>;
        using Successor_t = Successor<Dtype, Dim>;

        explicit EnvironmentMultiResolution(const long env_id = 0)
            : EnvironmentBase<Dtype, Dim>(env_id) {}

        [[nodiscard]] virtual std::size_t
        GetNumResolutionLevels() const = 0;

        [[nodiscard]] std::vector<State>
        ForwardAction(const State &env_state, long action_idx) const override {
            (void) env_state;
            (void) action_idx;
            ERL_FATAL("Call ForwardActionAtLevel() instead for {}.", type_name(*this));
        }

        [[nodiscard]] virtual std::vector<State>
        ForwardActionAtLevel(const State &env_state, long level, long action_idx) const = 0;

        [[nodiscard]] virtual std::vector<Successor_t>
        GetSuccessorsAtLevel(const State &env_state, long level) const = 0;

        [[nodiscard]] virtual bool
        InStateSpaceAtLevel(const State &env_state, long level) const = 0;
    };

}  // namespace erl::env

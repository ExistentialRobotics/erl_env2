#pragma once

#include "environment_state.hpp"

#include "erl_common/grid_map.hpp"

#include <vector>

namespace erl::env {

    template<typename Dtype, int Dim>
    struct Successor {
        using EnvState = EnvironmentState<Dtype, Dim>;
        using MetricState = typename EnvState::MetricState;
        using GridState = typename EnvState::GridState;

        EnvState env_state = {};
        Dtype cost = 0.0;
        long action_idx = -1;  // index of the action in the action set
        long env_id = 0;       // id of the environment that generates this successor

        Successor() = default;

        Successor(EnvState state, const Dtype cost, const long action_idx, const long env_id)
            : env_state(std::move(state)),
              cost(cost),
              action_idx(action_idx),
              env_id(env_id) {}

        Successor(
            MetricState env_metric_state,
            GridState env_grid_state,
            const Dtype cost,
            const long action_idx,
            const long env_id)
            : env_state(std::move(env_metric_state), std::move(env_grid_state)),
              cost(cost),
              action_idx(action_idx),
              env_id(env_id) {}
    };

    /**
     * @brief EnvironmentBase is a virtual interface for search-based planning on a metric space.
     * Thus, the EnvironmentBase includes a set of map parameters such as grid cell resolution, and
     * a collision checker. The internal state representation is discrete grid coordinate.
     * @tparam Dtype data type of the metric space, float or double.
     * @tparam Dim dimension of the metric space, e.g. 2 for 2D space, 3 for SE(2) space.
     */
    template<typename Dtype, int Dim>
    struct EnvironmentBase {

        using State = EnvironmentState<Dtype, Dim>;
        using Successor_t = Successor<Dtype, Dim>;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;

    protected:
        long m_env_id_;  // unique id of this environment

    public:
        explicit EnvironmentBase(const long env_id = 0)
            : m_env_id_(env_id) {}

        virtual ~EnvironmentBase() = default;

        [[nodiscard]] long
        GetEnvId() const {
            return m_env_id_;
        }

        void
        SetEnvId(const long env_id) {
            m_env_id_ = env_id;
        }

        [[nodiscard]] virtual std::size_t
        GetStateSpaceSize() const = 0;

        [[nodiscard]] virtual std::size_t
        GetActionSpaceSize() const = 0;

        /**
         * Apply an action on the given environment state to get the next state. No collision check
         * guarantee!
         * @param env_state
         * @param action_idx
         * @return vector of next environment states after applying the action
         */
        [[nodiscard]] virtual std::vector<State>
        ForwardAction(const State &env_state, long action_idx) const = 0;

        /**
         * Get reachable next environment states with the current state. Collision check is applied.
         * @param env_state the current environment state
         * @return vector of reachable next environment states with their costs and action
         * coordinates.
         */
        [[nodiscard]] virtual std::vector<Successor_t>
        GetSuccessors(const State &env_state) const = 0;

        [[nodiscard]] virtual bool
        InStateSpace(const State &env_state) const = 0;

        [[nodiscard]] virtual uint32_t
        StateHashing(const State &env_state) const = 0;

        [[nodiscard]] virtual GridState
        MetricToGrid(const MetricState &metric_state) const = 0;

        [[nodiscard]] virtual MetricState
        GridToMetric(const GridState &grid_state) const = 0;

        [[nodiscard]] virtual std::vector<State>
        SampleValidStates(int num_samples) const = 0;
    };

    extern template class EnvironmentBase<float, 2>;
    extern template class EnvironmentBase<double, 2>;
    extern template class EnvironmentBase<float, 3>;
    extern template class EnvironmentBase<double, 3>;
    extern template class EnvironmentBase<float, 4>;
    extern template class EnvironmentBase<double, 4>;

}  // namespace erl::env

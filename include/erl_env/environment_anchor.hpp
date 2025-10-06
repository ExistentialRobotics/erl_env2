#pragma once

#include "environment_multi_resolution.hpp"

namespace erl::env {

    /**
     * @brief EnvironmentAnchor is used as the finest resolution level of a multi-resolution search.
     * Its state space is the union of all the state spaces of the other resolution levels. And its
     * action space is the union of all the action spaces of the other resolution levels. This class
     * is used mainly for hashing the state space and generating all possible neighboring states of
     * a given state with the finest resolution, i.e. all possible actions.
     */
    template<typename Dtype, int Dim>
    class EnvironmentAnchor : public EnvironmentMultiResolution<Dtype, Dim> {
    public:
        using EnvBase = EnvironmentBase<Dtype, Dim>;
        using State = EnvironmentState<Dtype, Dim>;
        using Successor_t = Successor<Dtype, Dim>;

    protected:
        std::vector<std::shared_ptr<EnvBase>> m_envs_ = {};
        std::size_t m_action_space_size_ = 0;

    public:
        explicit EnvironmentAnchor(std::vector<std::shared_ptr<EnvBase>> environments)
            : EnvironmentMultiResolution<Dtype, Dim>(0), m_envs_(std::move(environments)) {
            for (int i = 0; i < static_cast<int>(m_envs_.size()); ++i) {
                std::shared_ptr<EnvBase> &env = m_envs_[i];
                env->SetEnvId(i + 1);
                ERL_ASSERTM(env != nullptr, "env is nullptr");
                m_action_space_size_ += env->GetActionSpaceSize();
            }
        }

        [[nodiscard]] std::size_t
        GetNumResolutionLevels() const override {
            return m_envs_.size() + 1;  // +1 for the anchor level
        }

        [[nodiscard]] std::size_t
        GetStateSpaceSize() const override {
            ERL_WARN_ONCE(
                "Default implementation of GetStateSpaceSize() is used. The returned size is an "
                "upper bound.");
            std::size_t state_space_size = 0;
            for (auto &env: m_envs_) { state_space_size += env->GetStateSpaceSize(); }
            return state_space_size;
        }

        [[nodiscard]] std::size_t
        GetActionSpaceSize() const override {
            ERL_WARN(
                "Default implementation of GetActionSpaceSize() is used. The returned size is an "
                "upper bound.");
            return m_action_space_size_;
        }

        /**
         * @brief Get the trajectory starting from the given state and following the given action.
         * @param env_state
         * @param level index of the environment (resolution level) to use, 1-based
         * @param action_idx index of the action to apply
         * @return
         */
        [[nodiscard]] std::vector<State>
        ForwardActionAtLevel(const State &env_state, long level, long action_idx) const override {
            return m_envs_[level - 1]->ForwardAction(env_state, action_idx);
        }

        /**
         * @brief Get successors of the given state with the finest resolution, i.e. all possible
         * actions.
         * @param env_state
         * @return
         */
        [[nodiscard]] std::vector<Successor_t>
        GetSuccessors(const State &env_state) const override {
            if (!InStateSpace(env_state)) { return {}; }
            std::vector<Successor_t> successors;
            successors.reserve(m_action_space_size_);
            for (auto it = m_envs_.begin(); it < m_envs_.end(); ++it) {
                std::vector<Successor_t> env_successors = (*it)->GetSuccessors(env_state);
                if (env_successors.empty()) { continue; }
                successors.insert(successors.end(), env_successors.begin(), env_successors.end());
            }
            return successors;
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessorsAtLevel(const State &state, const long level) const override {
            if (level == 0) { return GetSuccessors(state); }
            ERL_ASSERTM(level >= 1 && level <= static_cast<long>(m_envs_.size()), "Invalid level.");
            return m_envs_[level - 1]->GetSuccessors(state);
        }

        [[nodiscard]] bool
        InStateSpace(const State &env_state) const override {
            return std::any_of(m_envs_.begin(), m_envs_.end(), [&env_state](const auto &env) {
                return env->InStateSpace(env_state);
            });
        }

        [[nodiscard]] bool
        InStateSpaceAtLevel(const State &env_state, const long level) const override {
            ERL_ASSERTM(level >= 0 && level <= static_cast<long>(m_envs_.size()), "Invalid level.");
            if (level == 0) { return InStateSpace(env_state); }
            return m_envs_[level - 1]->InStateSpace(env_state);
        }
    };

}  // namespace erl::env

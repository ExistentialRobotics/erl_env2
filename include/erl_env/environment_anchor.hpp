#pragma once

#include "environment_base.hpp"

namespace erl::env {

    /**
     * @brief EnvironmentAnchor is used as the finest resolution level of a multi-resolution search. Its state space is
     * the union of all the state spaces of the other resolution levels. And its action space is the union of all the
     * action spaces of the other resolution levels. This class is used mainly for hashing the state space and
     * generating all possible neighboring states of a given state with the finest resolution, i.e. all possible
     * actions.
     */
    class EnvironmentAnchor : public EnvironmentBase {
    protected:
        std::vector<std::shared_ptr<EnvironmentBase>> m_envs_ = {};
        std::size_t m_action_space_size_ = 0;

    public:
        explicit EnvironmentAnchor(std::vector<std::shared_ptr<EnvironmentBase>> environments)
            : EnvironmentBase(nullptr),  // just use the interface of EnvironmentBase, no need to use the distance cost function
              m_envs_(std::move(environments)) {
            for (auto &env: m_envs_) {
                ERL_ASSERTM(env != nullptr, "env is nullptr");
                m_action_space_size_ += env->GetActionSpaceSize();
            }
        }

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            ERL_WARN("Default implementation of GetStateSpaceSize() is used. The returned size is an upper bound.");
            std::size_t state_space_size = 0;
            for (auto &env: m_envs_) { state_space_size += env->GetStateSpaceSize(); }
            return state_space_size;
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            ERL_WARN("Default implementation of GetActionSpaceSize() is used. The returned size is an upper bound.");
            return m_action_space_size_;
        }

        /**
         * @brief Get the trajectory starting from the given state and following the given action.
         * @param env_state
         * @param action_coords (env_action_coords, resolution_level), where resolution_level = 1, 2, ..., n
         * @return
         */
        [[nodiscard]] inline std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
            return m_envs_[action_coords.back() - 1]->ForwardAction(env_state, {action_coords.begin(), action_coords.end() - 1});
        }

        /**
         * @brief Get successors of the given state with the finest resolution, i.e. all possible actions.
         * @param env_state
         * @return
         */
        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override {
            if (!InStateSpace(env_state)) { return {}; }
            std::vector<Successor> successors;
            successors.reserve(m_action_space_size_);
            for (auto it = m_envs_.begin(); it < m_envs_.end(); ++it) {
                std::vector<Successor> env_successors = (*it)->GetSuccessors(env_state);
                if (env_successors.empty()) { continue; }
                auto resolution_level = int(std::distance(m_envs_.begin(), it)) + 1;
                for (auto &successor: env_successors) {
                    successor.action_coords.push_back(resolution_level);
                    successors.push_back(successor);
                }
            }
            return successors;
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &state, std::size_t resolution_level) const {
            if (resolution_level == 0) { return GetSuccessors(state); }
            std::vector<Successor> successors = m_envs_[resolution_level - 1]->GetSuccessors(state);
            for (auto &successor: successors) { successor.action_coords.push_back(int(resolution_level)); }
            return successors;
        }

        [[nodiscard]] bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return std::any_of(m_envs_.begin(), m_envs_.end(), [&env_state](const auto &env) { return env->InStateSpace(env_state); });
        }
    };

}  // namespace erl::env

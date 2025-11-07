#pragma once

#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <spot/parseaut/public.hh>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/hoa.hh>

namespace erl::env {

    /**
     * An interface of Büchi automaton.
     */
    class FiniteStateAutomaton {

    public:
        typedef boost::property<
            boost::vertex_index_t,
            uint32_t,
            boost::property<
                boost::vertex_name_t,
                std::string,
                boost::property<boost::vertex_color_t, std::string>>>
            BoostVertexProp;
        typedef boost::property<boost::edge_color_t, uint32_t> BoostEdgeProp;
        typedef boost::property<boost::graph_name_t, std::string> BoostGraphProp;
        typedef boost::adjacency_list<
            boost::vecS,
            boost::vecS,
            boost::directedS,
            BoostVertexProp,
            BoostEdgeProp,
            BoostGraphProp>
            BoostGraph;
        using SpotGraph = spot::twa_graph_ptr;

        struct Setting : public common::Yamlable<Setting> {

            struct Transition : public common::Yamlable<Transition> {
                uint32_t from = 0;
                uint32_t to = 0;
                std::vector<uint32_t> labels = {};

                ERL_REFLECT_SCHEMA(
                    Transition,
                    ERL_REFLECT_MEMBER(Transition, from),
                    ERL_REFLECT_MEMBER(Transition, to),
                    ERL_REFLECT_MEMBER(Transition, labels));

                Transition() = default;

                Transition(const uint32_t from, const uint32_t to, std::vector<uint32_t> labels)
                    : from(from),
                      to(to),
                      labels(std::move(labels)) {}
            };

            uint32_t num_states = 0;                            // number of states
            uint32_t initial_state = 0;                         // initial state
            std::vector<uint32_t> accepting_states = {};        // accepting states
            std::vector<std::string> atomic_propositions = {};  // atomic propositions
            std::vector<Transition> transitions = {};           // transitions

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, num_states),
                ERL_REFLECT_MEMBER(Setting, initial_state),
                ERL_REFLECT_MEMBER(Setting, accepting_states),
                ERL_REFLECT_MEMBER(Setting, atomic_propositions),
                ERL_REFLECT_MEMBER(Setting, transitions));

            enum class FileType { kSpotHoa = 0, kBoostDot = 1, kYaml = 2 };

            Setting() = default;

            /**
             *
             * @param filepath path to the file
             * @param file_type type of the file
             * @param complete if true, complete the automaton first (adding a sink state if
             * necessary so that for each state, it has a transition for each letter in the
             * alphabet). This parameter is only used when file_type is kSpotHoa.
             */
            Setting(const std::string &filepath, FileType file_type, bool complete);

            bool
            PostDeserialization() override {
                std::sort(accepting_states.begin(), accepting_states.end(), std::greater<>());
                for (auto &transition: transitions) {
                    std::sort(transition.labels.begin(), transition.labels.end(), std::greater<>());
                }
                return true;
            }

            [[nodiscard]] BoostGraph
            AsBoostGraph() const;

            void
            AsBoostGraphDotFile(const std::string &filename) const;

            void
            FromBoostGraphDotFile(const std::string &filepath);

            [[nodiscard]] SpotGraph
            AsSpotGraph(bool complete) const;

            void
            AsSpotGraphHoaFile(const std::string &filename, const bool complete) const {
                std::ofstream ofs(filename);
                spot::print_hoa(ofs, AsSpotGraph(complete));
            }

            /**
             *
             * @param filepath path to the hoa file
             * @param complete if true, complete the automaton first (adding a sink state if
             * necessary so that for each state, it has a transition for each letter in the
             * alphabet).
             */
            void
            FromSpotGraphHoaFile(const std::string &filepath, bool complete);

            void
            FromSpotGraphHoaString(const std::string &hoa_str, bool complete);

            void
            FromSpotGraph(const spot::twa_graph_ptr &aut, bool complete);

            /**
             *
             * @param filename path to the dot file
             * @param complete if true, complete the automaton first (adding a sink state if
             * necessary so that for each state, it has a transition for each letter in the
             * alphabet).
             */
            void
            AsSpotGraphDotFile(const std::string &filename, const bool complete) const {
                std::ofstream ofs(filename);
                AsSpotGraph(complete)->dump_storage_as_dot(ofs);
            }
        };

    protected:
        std::shared_ptr<Setting> m_setting_;  // setting
        uint32_t m_alphabet_size_ = 0;        // alphabet size
        // given key of p->q, return the edge labels
        absl::flat_hash_map<uint32_t, std::vector<uint32_t>> m_transition_labels_;
        // given hashing of state p and label `a`, return the next state q
        absl::flat_hash_map<uint32_t, uint32_t> m_transition_next_state_;
        std::vector<std::vector<uint32_t>> m_levels_;   // states in each level
        std::vector<std::vector<uint8_t>> m_levels_b_;  // states in each level (boolean version)
        std::vector<uint8_t> m_sink_states_;            // sink states
        std::vector<uint8_t> m_accepting_states_;       // accepting states

    public:
        explicit FiniteStateAutomaton(std::shared_ptr<Setting> setting);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] uint32_t
        GetAlphabetSize() const {
            return m_alphabet_size_;
        }

        [[nodiscard]] const std::vector<std::vector<uint32_t>> &
        GetLevels() const {
            return m_levels_;
        }

        [[nodiscard]] const std::vector<std::vector<uint8_t>> &
        GetLevelsB() const {
            return m_levels_b_;
        }

        [[nodiscard]] const std::vector<uint8_t> &
        GetSinkStates() const {
            return m_sink_states_;
        }

        [[nodiscard]] std::vector<uint32_t>
        GetTransitionLabels(const uint32_t from, const uint32_t to) const {
            const auto result = m_transition_labels_.find(HashingTransition(from, to));
            if (result == m_transition_labels_.end()) { return {}; }
            return result->second;
        }

        [[nodiscard]] uint32_t
        GetNextState(const uint32_t state, const uint32_t label) const {
            const auto result = m_transition_next_state_.find(HashingStateLabelPair(state, label));
            if (result == m_transition_next_state_.end()) { return state; }
            return result->second;
        }

        /**
         * @brief Get the lowest level index of the given state.
         * @param state
         * @return
         */
        [[nodiscard]] int
        GetLevelIndex(const uint32_t state) const {
            if (m_sink_states_[state]) { return -1; }
            for (std::size_t i = 0; i < m_levels_.size(); ++i) {
                if (m_levels_b_[i][state]) { return static_cast<int>(i); }
            }
            throw std::runtime_error("state is not in any level and is not a sink state!");
        }

        [[nodiscard]] bool
        IsSinkState(const uint32_t state) const {
            return m_sink_states_[state];
        }

        [[nodiscard]] bool
        IsAcceptingState(const uint32_t state) const {
            return m_accepting_states_[state];
        }

    private:
        void
        Init();

        [[nodiscard]] uint32_t
        HashingTransition(const uint32_t from, const uint32_t to) const {
            return from + to * m_setting_->num_states;  // column-major
        }

        [[nodiscard]] uint32_t
        HashingStateLabelPair(const uint32_t state, const uint32_t label) const {
            return state + label * m_setting_->num_states;  // column-major
        }
    };

}  // namespace erl::env

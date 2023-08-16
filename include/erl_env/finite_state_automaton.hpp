#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <numeric>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <spot/twaalgos/hoa.hh>
#include <spot/twa/twagraph.hh>
#include "erl_common/assert.hpp"
#include "erl_common/yaml.hpp"

namespace erl::env {

    /**
     * An interface of Büchi automaton.
     */
    class FiniteStateAutomaton {

    public:
        struct Setting : public common::Yamlable<Setting> {

            struct Transition : public common::Yamlable<Transition> {
                uint32_t from = 0;
                uint32_t to = 0;
                std::vector<uint32_t> labels = {};
            };

            uint32_t num_states = 0;                            // number of states
            uint32_t initial_state = 0;                         // initial state
            std::vector<uint32_t> accepting_states = {};        // accepting states
            std::vector<std::string> atomic_propositions = {};  // atomic propositions
            std::vector<Transition> transitions = {};           // transitions
        };

    protected:
        std::shared_ptr<Setting> m_setting_;                                       // setting
        uint32_t m_alphabet_size_ = 0;                                             // alphabet size
        std::unordered_map<uint32_t, std::vector<uint32_t>> m_transition_labels_;  // given key of p->q, return the edge labels
        std::unordered_map<uint32_t, uint32_t> m_transition_next_state_;           // given key of state p and label a, return the next state q
        std::vector<std::vector<uint32_t>> m_levels_;                              // states in each level
        std::vector<std::vector<bool>> m_levels_b_;                                // states in each level (boolean version)
        std::vector<bool> m_sink_states_;                                          // sink states
        std::vector<bool> m_accepting_states_;                                     // accepting states

    public:
        explicit FiniteStateAutomaton(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr");

            m_alphabet_size_ = 1 << m_setting_->atomic_propositions.size();

            // construct transition labels and next state
            ERL_ASSERTM(!m_setting_->transitions.empty(), "transitions is empty");
            for (auto &transition: m_setting_->transitions) {
                {
                    uint32_t key = HashingTransition(transition.from, transition.to);
                    auto result = m_transition_labels_.emplace(key, transition.labels);
                    ERL_ASSERTM(result.second, "duplicated transition");
                }

                if (transition.from == transition.to) { continue; }

                for (auto &label: transition.labels) {
                    uint32_t key = HashingStateLabelPair(transition.from, label);
                    auto result = m_transition_next_state_.emplace(key, transition.to);
                    ERL_ASSERTM(result.second, "duplicated transition");
                }
            }

            // set accepting states
            m_accepting_states_.resize(m_setting_->num_states, false);
            for (auto &state: m_setting_->accepting_states) { m_accepting_states_[state] = true; }

            // compute levels
            m_levels_.emplace_back(m_setting_->accepting_states.begin(), m_setting_->accepting_states.end());  // level 0
            m_levels_b_.emplace_back(m_setting_->num_states, false);                                           // level 0
            m_sink_states_.resize(m_setting_->num_states, true);
            for (auto &state: m_setting_->accepting_states) {
                m_sink_states_[state] = false;
                m_levels_b_[0][state] = true;
            }
            uint32_t level = 0;
            while (true) {
                bool done = true;
                auto &level_states = m_levels_[level];
                for (auto &state: level_states) {
                    for (uint32_t prev_state = 0; prev_state < m_setting_->num_states; ++prev_state) {
                        if (!m_sink_states_[prev_state]) { continue; }
                        if (m_transition_labels_.find(HashingTransition(prev_state, state)) == m_transition_labels_.end()) { continue; }
                        // exist a transition from prev_state to state but prev_state is marked as a sink state
                        m_sink_states_[prev_state] = false;
                        if (done) {  // add a new level
                            done = false;
                            m_levels_.emplace_back(1, prev_state);
                            m_levels_b_.emplace_back(m_setting_->num_states, false);
                            m_levels_b_[level + 1][prev_state] = true;
                        } else {
                            m_levels_[level + 1].emplace_back(prev_state);
                            m_levels_b_[level + 1][prev_state] = true;
                        }
                    }
                }
                if (done) { break; }
                level++;
            }
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline uint32_t
        GetAlphabetSize() const {
            return m_alphabet_size_;
        }

        [[nodiscard]] inline std::vector<std::vector<uint32_t>>
        GetLevels() const {
            return m_levels_;
        }

        [[nodiscard]] inline std::vector<std::vector<bool>>
        GetLevelsB() const {
            return m_levels_b_;
        }

        [[nodiscard]] inline std::vector<bool>
        GetSinkStates() const {
            return m_sink_states_;
        }

        [[nodiscard]] inline std::vector<uint32_t>
        GetTransitionLabels(uint32_t from, uint32_t to) const {
            auto result = m_transition_labels_.find(HashingTransition(from, to));
            if (result == m_transition_labels_.end()) { return {}; }
            return result->second;
        }

        [[nodiscard]] inline uint32_t
        GetNextState(uint32_t state, uint32_t label) const {
            auto result = m_transition_next_state_.find(HashingStateLabelPair(state, label));
            if (result == m_transition_next_state_.end()) { return state; }
            return result->second;
        }

        [[nodiscard]] inline int
        GetLevelIndex(uint32_t state) const {
            if (m_sink_states_[state]) { return -1; }
            for (std::size_t i = 0; i < m_levels_.size(); ++i) {
                if (m_levels_b_[i][state]) { return int(i); }
            }
            throw std::runtime_error("state is not in any level and is not a sink state!");
        }

        [[nodiscard]] inline bool
        IsSinkState(uint32_t state) const {
            return m_sink_states_[state];
        }

        [[nodiscard]] inline bool
        IsAcceptingState(uint32_t state) const {
            return m_accepting_states_[state];
        }

        using BoostEdgeLabelProperty = boost::edge_color_t;
        typedef boost::property<BoostEdgeLabelProperty, uint32_t> BoostEdgeLabel;
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, boost::no_property, BoostEdgeLabel> BoostGraph;

        [[nodiscard]] BoostGraph
        AsBoostGraph() const {

            BoostGraph graph(m_setting_->num_states);

            for (auto &transition: m_setting_->transitions) {
                for (auto &label: transition.labels) { boost::add_edge(transition.from, transition.to, BoostEdgeLabel(label), graph); }
            }
            return graph;
        }

        void
        SaveAsBoostGraphDotFile(
            const std::string &filename,
            const std::optional<std::map<std::string, std::string>> &graph_attr_opt = {},
            const std::optional<std::map<std::string, std::string>> &vertex_attr_opt = {},
            const std::optional<std::map<std::string, std::string>> &edge_attr_opt = {}) const {
            std::ofstream ofs(filename);

            std::map<std::string, std::string> graph_attr, vertex_attr, edge_attr;
            if (graph_attr_opt.has_value()) {
                graph_attr = graph_attr_opt.value();
            } else {
                graph_attr["center"] = "true";
                graph_attr["rankdir"] = "TB";
            }
            if (vertex_attr_opt.has_value()) {
                vertex_attr = vertex_attr_opt.value();
            } else {
                vertex_attr["shape"] = "circle";
            }
            if (edge_attr_opt.has_value()) {
                edge_attr = edge_attr_opt.value();
            } else {
                edge_attr["arrowhead"] = "vee";
            }

            auto graph = AsBoostGraph();
            auto edge_label_map = boost::get(BoostEdgeLabelProperty::edge_color, graph);
            boost::write_graphviz(
                ofs,
                graph,
                boost::default_writer(),
                boost::make_label_writer(edge_label_map),
                boost::make_graph_attributes_writer(graph_attr, vertex_attr, edge_attr));
        }

        using SpotGraph = spot::twa_graph_ptr;

        [[nodiscard]] SpotGraph
        AsSpotGraph() const {
            // https://spot.lre.epita.fr/tut22.html
            spot::bdd_dict_ptr dict = spot::make_bdd_dict();
            SpotGraph graph = spot::make_twa_graph(dict);  // Buchi automaton

            std::vector<bdd> atomic_propositions;
            atomic_propositions.reserve(m_setting_->atomic_propositions.size());
            for (auto &ap: m_setting_->atomic_propositions) { atomic_propositions.emplace_back(bdd_ithvar(graph->register_ap(ap))); }

            // we do not support multiple accepting sets yet: https://spot.lre.epita.fr/concepts.html#acceptance-set
            graph->set_generalized_buchi(1);                   // set the number of acceptance sets to use
            graph->new_states(m_setting_->num_states);         // set the number of states
            graph->set_init_state(m_setting_->initial_state);  // set the initial state

            auto label_to_bdd = [&atomic_propositions](uint32_t label) -> bdd {
                bdd result = bddtrue;
                for (std::size_t i = 0; i < atomic_propositions.size(); ++i) {
                    if ((label >> i) & 1) {
                        result &= atomic_propositions[i];
                    } else {
                        result &= !atomic_propositions[i];
                    }
                }
                return result;
            };
            for (auto &transition: m_setting_->transitions) {
                for (auto &label: transition.labels) {
                    if (IsAcceptingState(transition.to)) {
                        graph->new_edge(transition.from, transition.to, label_to_bdd(label), {0});
                    } else {
                        graph->new_edge(transition.from, transition.to, label_to_bdd(label));
                    }
                }
            }

            return graph;
        }

    private:
        [[nodiscard]] inline uint32_t
        HashingTransition(uint32_t from, uint32_t to) const {
            return from + to * m_setting_->num_states;  // column-major
        }

        [[nodiscard]] inline uint32_t
        HashingStateLabelPair(uint32_t state, uint32_t label) const {
            return state + label * m_setting_->num_states;  // column-major
        }
    };

}  // namespace erl::env

namespace YAML {

    template<>
    struct convert<erl::env::FiniteStateAutomaton::Setting::Transition> {
        static Node
        encode(const erl::env::FiniteStateAutomaton::Setting::Transition &rhs) {
            Node node(NodeType::Sequence);
            node.push_back(Node(std::vector<uint32_t>{rhs.from, rhs.to}));
            node.push_back(rhs.labels);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::FiniteStateAutomaton::Setting::Transition &rhs) {
            ERL_ASSERTM(node.IsSequence(), "node is not a sequence");
            rhs.from = node[0][0].as<uint32_t>();
            rhs.to = node[0][1].as<uint32_t>();
            rhs.labels = node[1].as<std::vector<uint32_t>>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::env::FiniteStateAutomaton::Setting::Transition &rhs) {
        out << Flow << BeginSeq << Flow;
        out << BeginSeq << rhs.from << rhs.to << EndSeq;
        out << Flow << rhs.labels;
        out << EndSeq;
        return out;
    }

    template<>
    struct convert<erl::env::FiniteStateAutomaton::Setting> {
        static Node
        encode(const erl::env::FiniteStateAutomaton::Setting &rhs) {
            Node node;
            node["num_states"] = rhs.num_states;
            node["initial_state"] = rhs.initial_state;
            node["accepting_states"] = rhs.accepting_states;
            node["atomic_propositions"] = rhs.atomic_propositions;
            node["transitions"] = rhs.transitions;
            return node;
        }

        static bool
        decode(const Node &node, erl::env::FiniteStateAutomaton::Setting &rhs) {
            rhs.num_states = node["num_states"].as<uint32_t>();
            rhs.initial_state = node["initial_state"].as<uint32_t>();
            rhs.accepting_states = node["accepting_states"].as<std::vector<uint32_t>>();
            rhs.atomic_propositions = node["atomic_propositions"].as<std::vector<std::string>>();
            rhs.transitions = node["transitions"].as<std::vector<erl::env::FiniteStateAutomaton::Setting::Transition>>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::env::FiniteStateAutomaton::Setting &rhs) {
        out << BeginMap;
        out << Key << "num_states" << Value << rhs.num_states;
        out << Key << "initial_state" << Value << rhs.initial_state;
        out << Key << "accepting_states" << Value << rhs.accepting_states;
        out << Key << "atomic_propositions" << Value << rhs.atomic_propositions;
        out << Key << "transitions" << Value << rhs.transitions;
        out << EndMap;
        return out;
    }
}  // namespace YAML

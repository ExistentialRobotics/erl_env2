#include "erl_env/finite_state_automaton.hpp"
#include "spot/twa/bddprint.hh"
#include "spot/twa/formula2bdd.hh"
#include "spot/twaalgos/hoa.hh"
#include "spot/misc/minato.hh"
#include <bvecx.h>

namespace erl::env {
    FiniteStateAutomaton::FiniteStateAutomaton(std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)) {
        ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr");
        Init();
    }

    FiniteStateAutomaton::FiniteStateAutomaton(const std::string &filepath, FileType file_type) {
        switch (file_type) {
            case FileType::kYaml:
                m_setting_ = std::make_shared<Setting>();
                m_setting_->FromYamlFile(filepath);
                break;
            case FileType::kSpotHoa:
                ReadSpotHoa(filepath);
                break;
            case FileType::kBoostDot:
                ReadBoostDot(filepath);
                break;
        }
        Init();
    }

    FiniteStateAutomaton::BoostGraph
    FiniteStateAutomaton::AsBoostGraph() const {
        std::stringstream ss;
        ss << m_setting_->atomic_propositions[0];
        std::size_t num_aps = m_setting_->atomic_propositions.size();
        for (std::size_t i = 1; i < num_aps; ++i) { ss << " " << m_setting_->atomic_propositions[i]; }
        BoostGraph graph(0, BoostGraphProp(ss.str()));
        // add vertex with label
        std::vector<boost::graph_traits<BoostGraph>::vertex_descriptor> vertices;
        vertices.reserve(m_setting_->num_states);
        for (uint32_t state = 0; state < m_setting_->num_states; ++state) {
            std::string label = std::to_string(state);
            if (state == m_setting_->initial_state) {
                label += ":initial";
            } else if (IsAcceptingState(state)) {
                label += ":accepting";
            }
            vertices.emplace_back(boost::add_vertex(BoostVertexProp(state, {std::to_string(state), label}), graph));
        }
        // add edge with label
        for (auto &transition: m_setting_->transitions) {
            for (auto &label: transition.labels) { boost::add_edge(vertices[transition.from], vertices[transition.to], BoostEdgeProp(label), graph); }
        }
        return graph;
    }

    void
    FiniteStateAutomaton::SaveAsBoostGraphDotFile(const std::string &filename) const  {
        std::ofstream ofs(filename);
        std::map<std::string, std::string> graph_attr, vertex_attr, edge_attr;
        auto graph = AsBoostGraph();
        graph_attr["name"] = boost::get_property(graph, boost::graph_name);
        auto vertex_label_map = boost::get(boost::vertex_color, graph);
        auto edge_label_map = boost::get(boost::edge_color, graph);
        boost::write_graphviz(
            ofs,
            graph,
            boost::make_label_writer(vertex_label_map),
            boost::make_label_writer(edge_label_map),
            boost::make_graph_attributes_writer(graph_attr, vertex_attr, edge_attr));
    }

    FiniteStateAutomaton::SpotGraph
    FiniteStateAutomaton::AsSpotGraph() const {
        // https://spot.lre.epita.fr/tut22.html
        spot::bdd_dict_ptr dict = spot::make_bdd_dict();
        SpotGraph graph = spot::make_twa_graph(dict);  // Buchi automaton

        std::vector<bdd> atomic_propositions;
        atomic_propositions.reserve(m_setting_->atomic_propositions.size());
        for (auto &ap: m_setting_->atomic_propositions) {
            int ap_index = graph->register_ap(ap);
            ERL_ASSERTM(ap_index >= 0, "spot error: ap_index < 0");
            bdd ap_bdd = bdd_ithvar(ap_index);
            atomic_propositions.emplace_back(ap_bdd);
        }

        // we do not support multiple accepting sets yet: https://spot.lre.epita.fr/concepts.html#acceptance-set
        graph->set_generalized_buchi(1);                   // set the number of acceptance sets to use
        graph->new_states(m_setting_->num_states);         // set the number of states
        graph->set_init_state(m_setting_->initial_state);  // set the initial state
        for (auto &transition: m_setting_->transitions) {
            bdd cond = bddfalse;
            for (auto &label: transition.labels) { cond |= spot_helper::LabelToBdd(label, atomic_propositions); }
            if (IsAcceptingState(transition.to)) {
                graph->new_edge(transition.from, transition.to, cond, {0});
            } else {
                graph->new_edge(transition.from, transition.to, cond);
            }
        }

        return graph;
    }

    void
    FiniteStateAutomaton::Init() {
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

    void
    FiniteStateAutomaton::ReadSpotHoa(const std::string &filepath) {
        spot::parsed_aut_ptr pa = spot::parse_aut(filepath, spot::make_bdd_dict());
        ERL_ASSERTM(pa != nullptr, "failed to parse the HOA file");
        ERL_ASSERTM(!pa->format_errors(std::cerr), "HOA format error");
        ERL_ASSERTM(!pa->aborted, "HOA parsing aborted");
        ERL_ASSERTM(pa->aut != nullptr, "spot error: pa->aut is nullptr");
        ERL_ASSERTM(pa->aut->num_sets() == 1, "only support single set of accepting states.");

        const spot::bdd_dict_ptr &bdd_dict = pa->aut->get_dict();

        m_setting_ = std::make_shared<Setting>();
        m_setting_->num_states = pa->aut->num_states();
        m_setting_->initial_state = pa->aut->get_init_state_number();
        m_setting_->atomic_propositions.resize(pa->aut->ap().size());
        // extract atomic propositions
        std::vector<bdd> atomic_propositions;
        atomic_propositions.reserve(pa->aut->ap().size());
        bdd ap_vars = bddtrue;
        for (const spot::formula &ap: pa->aut->ap()) {
            int index = bdd_dict->varnum(ap);
            ERL_ASSERTM(index >= 0, "spot error: index < 0");
            m_setting_->atomic_propositions[index] = ap.ap_name();
            atomic_propositions.emplace_back(spot::formula_to_bdd(ap, bdd_dict, this));
            ap_vars &= atomic_propositions.back();
        }
        // extract transitions and accepting states
        std::unordered_set<uint32_t> accepting_states;
        std::vector<std::tuple<uint32_t, uint32_t, std::set<uint32_t>>> transitions;
        transitions.resize(m_setting_->num_states * m_setting_->num_states);
        for (uint32_t s = 0; s < m_setting_->num_states; ++s) {
            auto out_edges = pa->aut->out(s);
            for (auto &t: out_edges) {
                if (t.acc.count() > 0) { accepting_states.insert(t.dst); }
                ERL_ASSERTM(spot::bdd_to_formula(t.cond, bdd_dict).is_ltl_formula(), "Only support LTL formula.");
                auto &[from, to, labels] = transitions[s * m_setting_->num_states + t.dst];
                from = t.src;
                to = t.dst;
                std::vector<uint32_t> new_labels = spot_helper::BddToLabels(t.cond, ap_vars);
                for (auto &label: new_labels) { labels.insert(label); }
            }
        }
        bdd_dict->unregister_all_my_variables(this);  // unregister all variables to please spot library
        m_setting_->accepting_states.insert(m_setting_->accepting_states.end(), accepting_states.begin(), accepting_states.end());
        for (auto &[from, to, labels]: transitions) {
            if (labels.empty()) { continue; }
            m_setting_->transitions.emplace_back(from, to, labels);
        }
        std::sort(m_setting_->accepting_states.begin(), m_setting_->accepting_states.end(), std::greater<>());
        for (auto &transition: m_setting_->transitions) { std::sort(transition.labels.begin(), transition.labels.end(), std::greater<>()); }
    }

    void
    FiniteStateAutomaton::ReadBoostDot(const std::string &filepath) {
        std::ifstream dot_file(filepath);
        ERL_ASSERTM(dot_file.is_open(), "failed to open the dot file");
        BoostGraph graph(0);
        boost::dynamic_properties properties;
        // use ref_property_map to turn a graph property into a property map
        boost::ref_property_map<BoostGraph *, std::string> graph_name = boost::get_property(graph, boost::graph_name);
        boost::property_map<BoostGraph, boost::vertex_index_t>::type vertex_index = boost::get(boost::vertex_index, graph);
        boost::property_map<BoostGraph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, graph);
        boost::property_map<BoostGraph, boost::vertex_color_t>::type vertex_label = boost::get(boost::vertex_color, graph);
        boost::property_map<BoostGraph, boost::edge_color_t>::type edge_label = boost::get(boost::edge_color, graph);
        properties.property("name", graph_name);
        properties.property("node_index", vertex_index);
        properties.property("node_id", vertex_name);
        properties.property("label", vertex_label);
        properties.property("label", edge_label);
        boost::read_graphviz(dot_file, graph, properties);

        m_setting_ = std::make_shared<Setting>();
        m_setting_->num_states = boost::num_vertices(graph);
        // get atomic propositions from graph name
        std::string graph_name_str = graph_name[&graph];
        std::stringstream ss(graph_name_str);
        std::string token;
        while (std::getline(ss, token, ' ')) { m_setting_->atomic_propositions.emplace_back(token); }
        // iterate over vertices
        bool initial_state_found = false;
        for (auto v = boost::vertices(graph); v.first != v.second; ++v.first) {
            uint32_t state = vertex_index[*v.first];
            std::string label = vertex_label[*v.first];
            if (label.find("initial") != std::string::npos) {
                ERL_ASSERTM(!initial_state_found, "multiple initial states");
                m_setting_->initial_state = state;
                initial_state_found = true;
            } else if (label.find("accepting") != std::string::npos) {
                m_setting_->accepting_states.emplace_back(state);
            }
        }
        // iterate over edges
        std::vector<std::tuple<uint32_t, uint32_t, std::set<uint32_t>>> transitions(m_setting_->num_states * m_setting_->num_states);
        for (auto e = boost::edges(graph); e.first != e.second; ++e.first) {
            uint32_t from = vertex_index[boost::source(*e.first, graph)];
            uint32_t to = vertex_index[boost::target(*e.first, graph)];
            uint32_t label = edge_label[*e.first];
            std::size_t index = from * m_setting_->num_states + to;
            auto &[from_, to_, labels] = transitions[index];
            from_ = from;
            to_ = to;
            labels.insert(label);
        }
        // construct transitions
        for (auto &[from, to, labels]: transitions) {
            if (labels.empty()) { continue; }
            m_setting_->transitions.emplace_back(from, to, labels);
        }
        std::sort(m_setting_->accepting_states.begin(), m_setting_->accepting_states.end(), std::greater<>());
        for (auto &transition: m_setting_->transitions) { std::sort(transition.labels.begin(), transition.labels.end(), std::greater<>()); }
    }
}  // namespace erl::env

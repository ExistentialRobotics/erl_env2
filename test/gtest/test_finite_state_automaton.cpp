#include <gtest/gtest.h>
#include <string>
#include <filesystem>
#include <boost/graph/graph_traits.hpp>
#include "erl_env/finite_state_automaton.hpp"

TEST(FiniteStateAutomaton, Test) {

    std::filesystem::path path = __FILE__;
    path = path.parent_path();
    path = path / "fsa.yaml";

    auto setting = std::make_shared<erl::env::FiniteStateAutomaton::Setting>();
    setting->FromYamlFile(path);
    std::cout << *setting << std::endl;

    EXPECT_EQ(setting->num_states, 9);
    EXPECT_EQ(setting->initial_state, 0);
    EXPECT_EQ(setting->accepting_states.size(), 1);
    EXPECT_EQ(setting->accepting_states[0], 7);
    EXPECT_EQ(setting->atomic_propositions.size(), 5);
    EXPECT_EQ(setting->atomic_propositions[0], "phi1");
    EXPECT_EQ(setting->atomic_propositions[1], "phi2");
    EXPECT_EQ(setting->atomic_propositions[2], "phi3");
    EXPECT_EQ(setting->atomic_propositions[3], "phi4");
    EXPECT_EQ(setting->atomic_propositions[4], "phi5");
    EXPECT_EQ(setting->transitions.size(), 38);
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>>> expected_transitions = {
        {{0, 0}, {12, 8, 4, 0}},
        {{0, 1}, {14, 10, 6, 2}},
        {{0, 2}, {20, 16}},
        {{0, 3}, {22, 18}},
        {{0, 4}, {24}},
        {{0, 5}, {26}},
        {{0, 6}, {28}},
        {{0, 7}, {30}},
        {{0, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{1, 1}, {14, 12, 10, 8, 6, 4, 2, 0}},
        {{1, 3}, {22, 20, 18, 16}},
        {{1, 5}, {26, 24}},
        {{1, 7}, {30, 28}},
        {{1, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{2, 2}, {20, 16, 4, 0}},
        {{2, 3}, {22, 18, 6, 2}},
        {{2, 4}, {24, 8}},
        {{2, 5}, {26, 10}},
        {{2, 6}, {28, 12}},
        {{2, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{3, 3}, {22, 20, 18, 16, 6, 4, 2, 0}},
        {{3, 5}, {26, 24, 10, 8}},
        {{3, 7}, {30, 28, 14, 12}},
        {{3, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{4, 4}, {24, 16, 8, 0}},
        {{4, 5}, {26, 18, 10, 2}},
        {{4, 6}, {28, 20, 12, 4}},
        {{4, 7}, {30, 22, 14, 6}},
        {{4, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{5, 5}, {26, 24, 18, 16, 10, 8, 2, 0}},
        {{5, 7}, {30, 28, 22, 20, 14, 12, 6, 4}},
        {{5, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{6, 6}, {28, 24, 20, 16, 12, 8, 4, 0}},
        {{6, 7}, {30, 26, 22, 18, 14, 10, 6, 2}},
        {{6, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{7, 7}, {30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0}},
        {{7, 8}, {31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1}},
        {{8, 8}, {31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}},
    };
    for (int i = 0; i < 38; ++i) {
        EXPECT_EQ(setting->transitions[i].from, expected_transitions[i].first.first);
        EXPECT_EQ(setting->transitions[i].to, expected_transitions[i].first.second);
        EXPECT_EQ(setting->transitions[i].labels.size(), expected_transitions[i].second.size());
        for (std::size_t j = 0; j < expected_transitions[i].second.size(); ++j) {
            EXPECT_EQ(setting->transitions[i].labels[j], expected_transitions[i].second[j]);
        }
    }

    erl::env::FiniteStateAutomaton fsa(setting);
    EXPECT_EQ(fsa.GetAlphabetSize(), 1 << 5);

    // check levels
    std::vector<std::vector<uint32_t>> expected_levels = {
        {7},
        {0, 1, 3, 4, 5, 6},
        {2},
    };
    auto fsa_levels = fsa.GetLevels();
    EXPECT_EQ(fsa_levels.size(), expected_levels.size());
    for (std::size_t i = 0; i < expected_levels.size(); ++i) {
        EXPECT_EQ(fsa_levels[i].size(), expected_levels[i].size());
        for (std::size_t j = 0; j < expected_levels[i].size(); ++j) {
            if (fsa_levels[i][j] != expected_levels[i][j]) { std::cout << "i: " << i << ", j: " << j << std::endl; }
            EXPECT_EQ(fsa_levels[i][j], expected_levels[i][j]);
        }
    }
    std::vector<std::vector<bool>> expected_levels_b = {
        {false, false, false, false, false, false, false, true, false},
        {true, true, false, true, true, true, true, false, false},
        {false, false, true, false, false, false, false, false, false},
    };
    auto fsa_levels_b = fsa.GetLevelsB();
    EXPECT_EQ(fsa_levels_b.size(), expected_levels_b.size());
    for (std::size_t i = 0; i < expected_levels_b.size(); ++i) {
        EXPECT_EQ(fsa_levels_b[i].size(), expected_levels_b[i].size());
        for (std::size_t j = 0; j < expected_levels_b[i].size(); ++j) {
            if (fsa_levels_b[i][j] != expected_levels_b[i][j]) { std::cout << "i: " << i << ", j: " << j << std::endl; }
            EXPECT_EQ(fsa_levels_b[i][j], expected_levels_b[i][j]);
        }
    }

    // check sink state
    std::vector<bool> expected_sink_states = {false, false, false, false, false, false, false, false, true};
    auto fsa_sink_states = fsa.GetSinkStates();
    EXPECT_EQ(fsa_sink_states.size(), expected_sink_states.size());
    for (std::size_t i = 0; i < expected_sink_states.size(); ++i) { EXPECT_EQ(fsa_sink_states[i], expected_sink_states[i]); }

    // as boost graph, save as a dot file
    // https://github.com/boostorg/graph/blob/develop/example/quick_tour.cpp
    using Graph = erl::env::FiniteStateAutomaton::BoostGraph;
    using EdgeLabelProperty = erl::env::FiniteStateAutomaton::BoostEdgeLabelProperty;
    auto graph = fsa.AsBoostGraph();
    boost::property_map<Graph, boost::vertex_index_t>::type vertex_id = boost::get(boost::vertex_index, graph);
    boost::property_map<Graph, EdgeLabelProperty>::type edge_label = boost::get(EdgeLabelProperty::edge_color, graph);
    for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
        std::cout << boost::get(vertex_id, boost::source(*ep.first, graph)) << " -(" << boost::get(edge_label, *ep.first) << ")-> "
                  << boost::get(vertex_id, boost::target(*ep.first, graph)) << std::endl;
    }

    fsa.SaveAsBoostGraphDotFile("fsa.dot");

    spot::print_hoa(std::cout, fsa.AsSpotGraph());
}

using namespace boost;

template<class Graph>
struct exercise_vertex {
    explicit exercise_vertex(Graph& g_)
        : g(g_) {}

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

    void
    operator()(const Vertex& v) const {
        using namespace boost;
        typename property_map<Graph, vertex_index_t>::type vertex_id = get(vertex_index, g);
        std::cout << "vertex: " << get(vertex_id, v) << std::endl;

        // Write out the outgoing edges
        std::cout << "\tout-edges: ";
        typename graph_traits<Graph>::out_edge_iterator out_i, out_end;
        typename graph_traits<Graph>::edge_descriptor e;
        for (tie(out_i, out_end) = out_edges(v, g); out_i != out_end; ++out_i) {
            e = *out_i;
            Vertex src = source(e, g), targ = target(e, g);
            std::cout << "(" << get(vertex_id, src) << "," << get(vertex_id, targ) << ") ";
        }
        std::cout << std::endl;

        // Write out the incoming edges
        std::cout << "\tin-edges: ";
        typename graph_traits<Graph>::in_edge_iterator in_i, in_end;
        for (tie(in_i, in_end) = in_edges(v, g); in_i != in_end; ++in_i) {
            e = *in_i;
            Vertex src = source(e, g), targ = target(e, g);
            std::cout << "(" << get(vertex_id, src) << "," << get(vertex_id, targ) << ") ";
        }
        std::cout << std::endl;

        // Write out all adjacent vertices
        std::cout << "\tadjacent vertices: ";
        typename graph_traits<Graph>::adjacency_iterator ai, ai_end;
        for (tie(ai, ai_end) = adjacent_vertices(v, g); ai != ai_end; ++ai) std::cout << get(vertex_id, *ai) << " ";
        std::cout << std::endl;
    }

    Graph& g;
};

TEST(BoostGraph, QuickTour) {
    // create a typedef for the Graph type
    typedef adjacency_list<vecS, vecS, bidirectionalS, no_property, property<edge_weight_t, float>> Graph;

    // Make convenient labels for the vertices
    enum { A, B, C, D, E, N };

    const int num_vertices = N;
    const char* name = "ABCDE";

    // writing out the edges in the graph
    typedef std::pair<int, int> Edge;
    Edge edge_array[] = {
        Edge(A, B),
        Edge(A, D),
        Edge(C, A),
        Edge(D, C),
        Edge(C, E),
        Edge(B, D),
        Edge(D, E),
    };
    const int num_edges = sizeof(edge_array) / sizeof(edge_array[0]);

    // average transmission delay (in milliseconds) for each connection
    float transmission_delay[] = {1.2, 4.5, 2.6, 0.4, 5.2, 1.8, 3.3, 9.1};

    // declare a graph object, adding the edges and edge properties
#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
    // VC++ can't handle the iterator constructor
    Graph g(num_vertices);
    property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, g);
    for (std::size_t j = 0; j < num_edges; ++j) {
        graph_traits<Graph>::edge_descriptor e;
        bool inserted;
        tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
        weightmap[e] = transmission_delay[j];
    }
#else
    Graph g(edge_array, edge_array + num_edges, transmission_delay, num_vertices);
#endif

    boost::property_map<Graph, vertex_index_t>::type vertex_id = get(vertex_index, g);
    boost::property_map<Graph, edge_weight_t>::type trans_delay = get(edge_weight, g);

    std::cout << "vertices(g) = ";
    typedef graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first) std::cout << name[get(vertex_id, *vp.first)] << " ";
    std::cout << std::endl;

    std::cout << "edges(g) = ";
    graph_traits<Graph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        std::cout << "(" << name[get(vertex_id, source(*ei, g))] << "," << name[get(vertex_id, target(*ei, g))] << ") ";
    std::cout << std::endl;

    std::for_each(vertices(g).first, vertices(g).second, exercise_vertex<Graph>(g));

    std::map<std::string, std::string> graph_attr, vertex_attr, edge_attr;
    graph_attr["size"] = "3,3";
    graph_attr["rankdir"] = "LR";
    graph_attr["ratio"] = "fill";
    vertex_attr["shape"] = "circle";

    boost::write_graphviz(
        std::cout,
        g,
        make_label_writer(name),
        make_label_writer(trans_delay),
        make_graph_attributes_writer(graph_attr, vertex_attr, edge_attr));
}

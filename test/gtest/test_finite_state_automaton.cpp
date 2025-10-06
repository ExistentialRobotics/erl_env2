#include "erl_common/test_helper.hpp"
#include "erl_env/finite_state_automaton.hpp"

#include <boost/graph/graph_traits.hpp>

#include <filesystem>
#include <string>

TEST(ERL_ENV, FiniteStateAutomaton) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::env;

    std::filesystem::path fsa_yaml_path = gtest_src_dir / "fsa.yaml";

    auto setting = std::make_shared<FiniteStateAutomaton::Setting>();
    ASSERT_TRUE(setting->FromYamlFile(fsa_yaml_path));
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
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>>>
        expected_transitions = {
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
            {{8, 8}, {31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
                      15, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,  0}},
        };
    for (int i = 0; i < 38; ++i) {
        EXPECT_EQ(setting->transitions[i].from, expected_transitions[i].first.first);
        EXPECT_EQ(setting->transitions[i].to, expected_transitions[i].first.second);
        EXPECT_EQ(setting->transitions[i].labels.size(), expected_transitions[i].second.size());
        for (std::size_t j = 0; j < expected_transitions[i].second.size(); ++j) {
            EXPECT_EQ(setting->transitions[i].labels[j], expected_transitions[i].second[j]);
        }
    }

    FiniteStateAutomaton fsa(setting);
    EXPECT_EQ(fsa.GetAlphabetSize(), 1 << 5);

    // check levels
    std::vector<std::vector<uint32_t>> expected_levels = {
        {7},
        {0, 1, 3, 4, 5, 6},
        {2},
    };
    const auto& fsa_levels = fsa.GetLevels();
    EXPECT_EQ(fsa_levels.size(), expected_levels.size());
    for (std::size_t i = 0; i < expected_levels.size(); ++i) {
        EXPECT_EQ(fsa_levels[i].size(), expected_levels[i].size());
        for (std::size_t j = 0; j < expected_levels[i].size(); ++j) {
            if (fsa_levels[i][j] != expected_levels[i][j]) {
                std::cout << "i: " << i << ", j: " << j << std::endl;
            }
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
            if (fsa_levels_b[i][j] != expected_levels_b[i][j]) {
                std::cout << "i: " << i << ", j: " << j << std::endl;
            }
            EXPECT_EQ(fsa_levels_b[i][j], expected_levels_b[i][j]);
        }
    }

    // check sink state
    std::vector<bool> expected_sink_states =
        {false, false, false, false, false, false, false, false, true};
    auto fsa_sink_states = fsa.GetSinkStates();
    EXPECT_EQ(fsa_sink_states.size(), expected_sink_states.size());
    for (std::size_t i = 0; i < expected_sink_states.size(); ++i) {
        EXPECT_EQ(fsa_sink_states[i], expected_sink_states[i]);
    }

    setting->AsBoostGraphDotFile("fsa.dot");            // as boost graph, save as a dot file
    setting->AsSpotGraphHoaFile("fsa.hoa", false);      // as spot graph, save as a hoa file
    setting->AsSpotGraphDotFile("fsa.hoa.dot", false);  // as spot graph, save as a dot file
    std::string gt_str = setting->AsYamlString();
    EXPECT_EQ(
        gt_str,
        FiniteStateAutomaton::Setting(
            "fsa.hoa",
            FiniteStateAutomaton::Setting::FileType::kSpotHoa,
            false)
            .AsYamlString());
    EXPECT_EQ(
        gt_str,
        FiniteStateAutomaton::Setting(
            "fsa.dot",
            FiniteStateAutomaton::Setting::FileType::kBoostDot,
            false)
            .AsYamlString());

    // test spot file generated by ChatGPT
    auto fsa_hoa_gpt_setting = std::make_shared<FiniteStateAutomaton::Setting>(
        gtest_src_dir / "automaton.aut",
        FiniteStateAutomaton::Setting::FileType::kSpotHoa,
        false);

    // auto fsa_hoa_gpt = std::make_shared<FiniteStateAutomaton>(fsa_hoa_gpt_setting);
    // EXPECT_EQ(fsa_hoa_gpt_setting->num_states, 4);
    // EXPECT_EQ(fsa_hoa_gpt_setting->initial_state, 0);
    // EXPECT_EQ(fsa_hoa_gpt_setting->accepting_states.size(), 1);
    // EXPECT_EQ(fsa_hoa_gpt_setting->accepting_states[0], 3);
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions.size(), 8);
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[0], "p5");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[1], "p1");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[2], "p2");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[3], "p3");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[4], "p6");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[5], "p7");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[6], "p8");
    // EXPECT_EQ(fsa_hoa_gpt_setting->atomic_propositions[7], "p4");

    // as spot graph, save as a dot file
    fsa_hoa_gpt_setting->AsSpotGraphDotFile("fsa_hoa_gpt.dot", false);
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
        for (tie(ai, ai_end) = adjacent_vertices(v, g); ai != ai_end; ++ai)
            std::cout << get(vertex_id, *ai) << " ";
        std::cout << std::endl;
    }

    Graph& g;
};

TEST(BoostGraph, QuickTour) {
    // create a typedef for the Graph type
    typedef adjacency_list<vecS, vecS, bidirectionalS, no_property, property<edge_weight_t, float>>
        Graph;

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
    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
        std::cout << name[get(vertex_id, *vp.first)] << " ";
    std::cout << std::endl;

    std::cout << "edges(g) = ";
    graph_traits<Graph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        std::cout << "(" << name[get(vertex_id, source(*ei, g))] << ","
                  << name[get(vertex_id, target(*ei, g))] << ") ";
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

TEST(BoostGraph, ReadGraphViz) {
    // Vertex properties
    typedef property<vertex_name_t, std::string, property<vertex_color_t, float>> vertex_p;
    // Edge properties
    typedef property<edge_weight_t, double> edge_p;
    // Graph properties
    typedef property<graph_name_t, std::string> graph_p;
    // adjacency_list-based type
    typedef adjacency_list<vecS, vecS, directedS, vertex_p, edge_p, graph_p> graph_t;

    // Construct an empty graph and prepare the dynamic_property_maps.
    graph_t graph(0);
    dynamic_properties dp;

    property_map<graph_t, vertex_name_t>::type name = get(vertex_name, graph);
    dp.property("node_id", name);

    property_map<graph_t, vertex_color_t>::type mass = get(vertex_color, graph);
    dp.property("mass", mass);

    property_map<graph_t, edge_weight_t>::type weight = get(edge_weight, graph);
    dp.property("weight", weight);

    // Use ref_property_map to turn a graph property into a property map
    boost::ref_property_map<graph_t*, std::string> gname(get_property(graph, graph_name));
    dp.property("name", gname);

    // Sample graph as an std::istream;
    std::istringstream gvgraph("digraph { graph [name=\"graphname\"]  a  c e [mass = 6.66] }");

    bool status = read_graphviz(gvgraph, graph, dp, "node_id");

    for (auto v = boost::vertices(graph); v.first != v.second; ++v.first) {
        std::cout << name[*v.first] << std::endl;
    }

    EXPECT_EQ(status, true);
}

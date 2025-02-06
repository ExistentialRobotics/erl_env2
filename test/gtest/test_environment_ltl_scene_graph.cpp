#include "erl_common/test_helper.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"

TEST(ERL_ENV, EnvironmentLTLSceneGraph) {
    using namespace erl::env;

    std::filesystem::path path = __FILE__;
    auto data_dir = path.parent_path();
    path = data_dir / "building.yaml";
    // load the building
    auto building = std::make_shared<scene_graph::Building>();
    ASSERT_TRUE(building->FromYamlFile(path.string()));

    // load the env setting
    auto setting = std::make_shared<EnvironmentLTLSceneGraph::Setting>();
    setting->data_dir = data_dir.string();
    setting->shape = Eigen::Matrix2Xd(2, 360);
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(360, 0, 2 * M_PI);
    double r = 0.15;
    for (int i = 0; i < 360; ++i) {
        setting->shape(0, i) = r * cos(angles[i]);
        setting->shape(1, i) = r * sin(angles[i]);
    }
    // load the finite state automaton setting from spot hoa file
    setting->fsa = std::make_shared<FiniteStateAutomaton::Setting>(data_dir / "automaton.aut", FiniteStateAutomaton::Setting::FileType::kSpotHoa);
    // load the atomic propositions
    setting->LoadAtomicPropositions(data_dir / "ap_desc.yaml");

    // create the environment
    EnvironmentLTLSceneGraph env_scene_graph(building, setting);
}

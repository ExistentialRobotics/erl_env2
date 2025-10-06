#include "erl_common/test_helper.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"

TEST(ERL_ENV, EnvironmentLTLSceneGraph) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::env;
    using Dtype = float;
    using Env = EnvironmentLTLSceneGraph<Dtype>;

    std::filesystem::path path = gtest_src_dir / "building.yaml";
    // load the building
    auto building = std::make_shared<scene_graph::Building>();
    ASSERT_TRUE(building->FromYamlFile(path.string()));

    // load the env setting
    auto setting = std::make_shared<Env::Setting>();
    setting->data_dir = gtest_src_dir.string();
    setting->robot_metric_contour = Eigen::Matrix2X<Dtype>(2, 360);
    Eigen::VectorX<Dtype> angles = Eigen::VectorX<Dtype>::LinSpaced(360, 0, 2 * M_PI);
    for (int i = 0; i < 360; ++i) {
        constexpr Dtype r = 0.15f;
        setting->robot_metric_contour.col(i) << r * cos(angles[i]), r * sin(angles[i]);
    }
    // load the finite state automaton setting from spot hoa file
    setting->fsa = std::make_shared<FiniteStateAutomaton::Setting>(
        gtest_src_dir / "automaton.aut",
        FiniteStateAutomaton::Setting::FileType::kSpotHoa,
        false);
    // load the atomic propositions
    setting->LoadAtomicPropositions(gtest_src_dir / "ap_desc.yaml");

    // create the environment
    Env env_scene_graph(building, setting);
}

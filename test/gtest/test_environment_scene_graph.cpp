#include "erl_common/test_helper.hpp"
#include "erl_env/environment_scene_graph.hpp"

#include <filesystem>

TEST(ERL_ENV, EnvironmentSceneGraph) {

    std::filesystem::path path = __FILE__;
    auto data_dir = path.parent_path();
    path = data_dir / "building.yaml";
    auto building = std::make_shared<erl::env::scene_graph::Building>();
    ASSERT_TRUE(building->FromYamlFile(path.string()));

    auto setting = std::make_shared<erl::env::EnvironmentSceneGraph::Setting>();
    setting->data_dir = data_dir.string();
    setting->shape = Eigen::Matrix2Xd(2, 360);
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(360, 0, 2 * M_PI);
    double r = 0.15;
    for (int i = 0; i < 360; ++i) {
        setting->shape(0, i) = r * cos(angles[i]);
        setting->shape(1, i) = r * sin(angles[i]);
    }
    erl::env::EnvironmentSceneGraph env_scene_graph(building, setting);
}

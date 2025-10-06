#include "erl_common/test_helper.hpp"
#include "erl_env/environment_scene_graph.hpp"

#include <filesystem>

TEST(ERL_ENV, EnvironmentSceneGraph) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::env;
    using Dtype = float;
    using Env = EnvironmentSceneGraph<Dtype>;

    std::filesystem::path path = gtest_src_dir / "building.yaml";
    auto building = std::make_shared<scene_graph::Building>();
    ASSERT_TRUE(building->FromYamlFile(path.string()));

    auto setting = std::make_shared<Env::Setting>();
    setting->data_dir = gtest_src_dir.string();
    setting->robot_metric_contour = Eigen::Matrix2X<Dtype>(2, 360);
    Eigen::VectorX<Dtype> angles = Eigen::VectorX<Dtype>::LinSpaced(360, 0, 2 * M_PI);
    for (int i = 0; i < 360; ++i) {
        constexpr Dtype r = 0.15f;
        setting->robot_metric_contour.col(i) << r * cos(angles[i]), r * sin(angles[i]);
    }
    Env env_scene_graph(building, setting);
}

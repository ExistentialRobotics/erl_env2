#include "erl_common/grid_map.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_env/environment_se2.hpp"

TEST(ERL_ENV, EnvironmentSe2) {
    GTEST_PREPARE_OUTPUT_DIR();

    using namespace erl::common;
    using namespace erl::env;
    using Dtype = float;
    using Env = EnvironmentSe2<Dtype>;

    auto motion_primitives = LoadYamlSequenceFromFile<DdcMotionPrimitive<Dtype>>(
        gtest_src_dir / "ddc_motion_primitives.yaml",
        true);
    auto grid_map_info = std::make_shared<GridMapInfo2D<Dtype>>(
        Eigen::Vector2<Dtype>(0, 0),
        Eigen::Vector2<Dtype>(10, 10),
        Eigen::Vector2<Dtype>(0.1, 0.1),
        Eigen::Vector2i(10, 10));
    auto grid_map = std::make_shared<GridMap<uint8_t, Dtype, 2>>(grid_map_info, 0);
    auto env_setting = std::make_shared<Env::Setting>();
    auto env = std::make_shared<Env>(grid_map, env_setting);
    auto grid = env->MetricToGrid(Eigen::Vector3<Dtype>(0, 0, 0));
    std::cout << grid << std::endl;
}

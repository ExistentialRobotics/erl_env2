#include "erl_common/test_helper.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_env/environment_se2.hpp"

TEST(ERL_ENV, EnvironmentSe2) {
    using namespace erl::common;
    using namespace erl::env;

    std::filesystem::path path = __FILE__;
    auto data_dir = path.parent_path();

    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(data_dir / "ddc_motion_primitives.yaml");
    auto grid_map_info = std::make_shared<GridMapInfo2D>(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10), Eigen::Vector2d(0.1, 0.1), Eigen::Vector2i(10, 10));
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, 0);
    auto env_setting = std::make_shared<EnvironmentSe2::Setting>();
    auto env = std::make_shared<EnvironmentSe2>(grid_map, env_setting);
    auto grid = env->MetricToGrid(Eigen::Vector3d(0, 0, 0));
    std::cout << grid << std::endl;
}

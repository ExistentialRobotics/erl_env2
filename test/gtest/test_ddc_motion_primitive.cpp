#include "erl_common/test_helper.hpp"
#include "erl_env/ddc_motion_primitive.hpp"

TEST(DDCMotionPrimitiveTest, LoadFromFile) {
    GTEST_PREPARE_OUTPUT_DIR();
    using namespace erl::env;
    using Control = DifferentialDriveControl<float>;

    const std::filesystem::path file = gtest_src_dir / "ddc_motion_primitives.yaml";

    auto motion_primitives =
        erl::common::LoadYamlSequenceFromFile<DdcMotionPrimitive<float>>(file, true);

    constexpr int kNumMotionPrimitives = 6;
    ASSERT_EQ(motion_primitives.size(), kNumMotionPrimitives);

    std::vector<DdcMotionPrimitive<float>> expected_motion_primitives = {
        {{Control{1.0f, 0.0f}}, {1.0f}, {1.0f}},
        {{Control{1.0f, -1.0f}}, {1.0f}, {1.0f}},
        {{Control{1.0f, 1.0f}}, {1.0f}, {1.0f}},
        {{Control{1.0f, -2.0f}}, {1.0f}, {1.0f}},
        {{Control{1.0f, 2.0f}}, {1.0f}, {1.0f}},
        {{Control{1.0f, 0.0f}, Control{0.0f, 1.0f}}, {1.0f, 1.0f}, {1.0f, 1.0f}}};

    for (int i = 0; i < kNumMotionPrimitives; ++i) {
        ASSERT_EQ(motion_primitives[i].controls, expected_motion_primitives[i].controls);
        ASSERT_EQ(motion_primitives[i].durations, expected_motion_primitives[i].durations);
        ASSERT_EQ(motion_primitives[i].costs, expected_motion_primitives[i].costs);
    }
}

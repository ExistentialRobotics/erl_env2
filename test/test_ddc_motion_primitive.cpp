#include <gtest/gtest.h>

#include "erl_env/ddc_motion_primitive.hpp"

TEST(DDCMotionPrimitiveTest, LoadFromFile) {
    auto motion_primitives = erl::env::LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_YAML_PATH);
    constexpr int kNumMotionPrimitives = 6;
    ASSERT_EQ(motion_primitives.size(), kNumMotionPrimitives);

    std::vector<erl::env::DdcMotionPrimitive> expected_motion_primitives = {
        {{erl::env::DifferentialDriveControl{1.0, 0.0}}, {1.0}, {1.0}},
        {{erl::env::DifferentialDriveControl{1.0, -1.0}}, {1.0}, {1.0}},
        {{erl::env::DifferentialDriveControl{1.0, 1.0}}, {1.0}, {1.0}},
        {{erl::env::DifferentialDriveControl{1.0, -2.0}}, {1.0}, {1.0}},
        {{erl::env::DifferentialDriveControl{1.0, 2.0}}, {1.0}, {1.0}},
        {{erl::env::DifferentialDriveControl{1.0, 0.0}, erl::env::DifferentialDriveControl{0.0, 1.0}}, {1.0, 1.0}, {1.0, 1.0}}};

    for (int i = 0; i < kNumMotionPrimitives; ++i) {
        ASSERT_EQ(motion_primitives[i].controls, expected_motion_primitives[i].controls);
        ASSERT_EQ(motion_primitives[i].durations, expected_motion_primitives[i].durations);
        ASSERT_EQ(motion_primitives[i].costs, expected_motion_primitives[i].costs);
    }
}

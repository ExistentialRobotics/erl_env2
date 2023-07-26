#include <gtest/gtest.h>
#include <cmath>

#include "erl_env/differential_drive_model.hpp"

inline double
RestrictAngle(double phi, double minrange = -M_PI, double maxrange = M_PI) {
    // return minrange + std::fmod( (phi - minrange), (maxrange - minrange));
    // NOTE!!: fmod does not behave like MATLAB mod!
    double x = phi - minrange;
    double y = maxrange - minrange;
    return minrange + x - y * std::floor(x / y);
}

inline std::array<double, 3>
DdMotionModel(const std::array<double, 3>& x, const std::array<double, 2>& u, double t) {
    std::array<double, 3> nx{};
    double tw = t * u[1];
    nx[2] = erl::common::ClipAngle(x[2] + tw);
    if (std::abs(tw) < 0.0001) {
        nx[0] = x[0] + t * u[0] * std::cos(nx[2]);
        nx[1] = x[1] + t * u[0] * std::sin(nx[2]);
    } else {
        nx[0] = x[0] + u[0] / u[1] * (std::sin(nx[2]) - std::sin(x[2]));
        nx[1] = x[1] + u[0] / u[1] * (std::cos(x[2]) - std::cos(nx[2]));
    }
    return nx;
}

TEST(DifferentialDriveModelTest, LinearOnly) {

    auto expected = DdMotionModel({0, 0, 0}, {1, 0}, 1);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(0, 0, 0, 1, 0, 1, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

TEST(DifferentialDriveModelTest, AngularOnly) {

    auto expected = DdMotionModel({0, 0, 0}, {0, 1}, 1);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(0, 0, 0, 0, 1, 1, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

TEST(DifferentialDriveModelTest, LinearAndAngular) {

    auto expected = DdMotionModel({0, 0, 0}, {1, 1}, 1);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(0, 0, 0, 1, 1, 1, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

TEST(DifferentialDriveModelTest, VeryShortDuration) {

    // EXPECT_EQ(0.00001, RestrictAngle(0.00001));  // RestrictAngle is not accurate when the angle is very small

    auto expected = DdMotionModel({0, 0, 0}, {1, 1}, 0.00001);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(0, 0, 0, 1, 1, 0.00001, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

TEST(DifferentialDriveModelTest, LongerDuration) {

    auto expected = DdMotionModel({0, 0, 0}, {1, 1}, 2);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(0, 0, 0, 1, 1, 2, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

TEST(DifferentialDriveModelTest, NonZeroInitialPose) {

    auto expected = DdMotionModel({1, 2, 3}, {1, 1}, 2);
    std::array<double, 3> actual{};
    erl::env::DifferentialDriveKinematic(1, 2, 3, 1, 1, 2, actual[0], actual[1], actual[2]);

    EXPECT_EQ(actual, expected);
}

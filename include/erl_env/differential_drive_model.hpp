#pragma once

#include "erl_common/angle_utils.hpp"
#include "erl_common/yaml.hpp"

#include <cmath>

namespace erl::env {

    template<typename Dtype>
    struct DifferentialDriveControl : public common::Yamlable<DifferentialDriveControl<Dtype>> {
        Dtype linear_v = 0.0f;
        Dtype angular_v = 0.0f;

        ERL_REFLECT_SCHEMA(
            DifferentialDriveControl,
            ERL_REFLECT_MEMBER(DifferentialDriveControl, linear_v),
            ERL_REFLECT_MEMBER(DifferentialDriveControl, angular_v));

        DifferentialDriveControl() = default;

        DifferentialDriveControl(Dtype linear_v, Dtype angular_v)
            : linear_v(linear_v),
              angular_v(angular_v) {}

        bool
        operator==(const DifferentialDriveControl &other) const {
            return linear_v == other.linear_v && angular_v == other.angular_v;
        }
    };

    using DifferentialDriveControlf = DifferentialDriveControl<float>;
    using DifferentialDriveControld = DifferentialDriveControl<double>;

    /**
     * Kinematic model of differential drive robot.
     * @tparam Dtype data type, float or double.
     * @param x car x position
     * @param y car y position
     * @param theta car heading angle
     * @param linear_v linear velocity
     * @param angular_v angular velocity
     * @param t time duration
     * @param new_x new x position after t
     * @param new_y new y position after t
     * @param new_theta new heading angle after t
     */
    template<typename Dtype>
    void
    DifferentialDriveKinematic(
        const Dtype x,
        const Dtype y,
        const Dtype theta,
        const Dtype linear_v,
        const Dtype angular_v,
        const Dtype t,
        Dtype &new_x,
        Dtype &new_y,
        Dtype &new_theta) {

        auto w = angular_v * t;
        new_theta = common::WrapAnglePi<Dtype>(theta + w);
        if (std::abs(w) < 1.e-6f) {
            new_x = x + t * linear_v * std::cos(new_theta);
            new_y = y + t * linear_v * std::sin(new_theta);
        } else {
            new_x = x + linear_v / angular_v * (std::sin(new_theta) - std::sin(theta));
            new_y = y + linear_v / angular_v * (std::cos(theta) - std::cos(new_theta));
        }
    }
}  // namespace erl::env

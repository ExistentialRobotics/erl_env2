#pragma once

#include "erl_common/angle_utils.hpp"
#include "erl_common/yaml.hpp"

#include <cmath>
#include <memory>

namespace erl::env {

    template<typename Dtype>
    struct DifferentialDriveControl {
        Dtype linear_v = 0.;
        Dtype angular_v = 0.;

        DifferentialDriveControl() = default;

        DifferentialDriveControl(Dtype linear_v, Dtype angular_v)
            : linear_v(linear_v), angular_v(angular_v) {}

        bool
        operator==(const DifferentialDriveControl &other) const {
            return linear_v == other.linear_v && angular_v == other.angular_v;
        }

        struct YamlConvertImpl {
            static YAML::Node
            encode(const DifferentialDriveControl &control) {
                YAML::Node node;
                ERL_YAML_SAVE_ATTR(node, control, linear_v);
                ERL_YAML_SAVE_ATTR(node, control, angular_v);
                return node;
            }

            static bool
            decode(const YAML::Node &node, DifferentialDriveControl &control) {
                if (!node.IsMap()) { return false; }
                ERL_YAML_LOAD_ATTR(node, control, linear_v);
                ERL_YAML_LOAD_ATTR(node, control, angular_v);
                return true;
            }
        };
    };

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

template<>
struct YAML::convert<erl::env::DifferentialDriveControl<float>>
    : public erl::env::DifferentialDriveControl<float>::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::DifferentialDriveControl<double>>
    : public erl::env::DifferentialDriveControl<double>::YamlConvertImpl {};

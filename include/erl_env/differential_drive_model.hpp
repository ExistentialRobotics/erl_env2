#pragma once

#include <memory>
#include <cmath>
#include "erl_common/angle_utils.hpp"
#include "erl_common/yaml.hpp"

namespace erl::env {

    struct DifferentialDriveControl {
        double linear_v = 0.;
        double angular_v = 0.;

        DifferentialDriveControl() = default;

        DifferentialDriveControl(double linear_v, double angular_v)
            : linear_v(linear_v),
              angular_v(angular_v) {}

        bool operator==(const DifferentialDriveControl &other) const {
            return linear_v == other.linear_v && angular_v == other.angular_v;
        }
    };

    inline void
    DifferentialDriveKinematic(double x, double y, double theta, double linear_v, double angular_v, double t, double &new_x, double &new_y, double &new_theta) {

        auto w = angular_v * t;
        new_theta = common::ClipAngle(theta + w);
        if (std::abs(w) < 0.0001) {
            new_x = x + t * linear_v * std::cos(new_theta);
            new_y = y + t * linear_v * std::sin(new_theta);
        } else {
            new_x = x + linear_v / angular_v * (std::sin(new_theta) - std::sin(theta));
            new_y = y + linear_v / angular_v * (std::cos(theta) - std::cos(new_theta));
        }
    }
}  // namespace erl::env

namespace YAML {

    template<>
    struct convert<erl::env::DifferentialDriveControl> {
        static Node encode(const erl::env::DifferentialDriveControl &control) {
            Node node;
            node["linear_v"] = control.linear_v;
            node["angular_v"] = control.angular_v;
            return node;
        }

        static bool decode(const Node &node, erl::env::DifferentialDriveControl &control) {
            if (!node.IsMap()) { return false; }
            if (!node["linear_v"]) { return false; }
            if (!node["angular_v"]) { return false; }
            control.linear_v = node["linear_v"].as<double>();
            control.angular_v = node["angular_v"].as<double>();
            return true;
        }
    };

    inline Emitter & operator<<(Emitter &out, const erl::env::DifferentialDriveControl &control) {
        out << BeginMap;
        out << Key << "linear_v" << Value << control.linear_v;
        out << Key << "angular_v" << Value << control.angular_v;
        out << EndMap;
        return out;
    }
}

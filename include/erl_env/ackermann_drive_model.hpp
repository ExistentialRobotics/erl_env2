#pragma once

#include "differential_drive_model.hpp"

namespace erl::env {

    struct AckermannDriveControl {
        double linear_v = 0.;
        double wheel_base = 1.;  // distance between front and rear wheels
        double steering_angle = 0.;
    };

    inline void
    AckermannDriveKinematic(
        double x,
        double y,
        double theta,
        double linear_v,
        double wheel_base,
        double steering_angle,
        double t,
        double &new_x,
        double &new_y,
        double &new_theta) {
        DifferentialDriveKinematic(
            x,
            y,
            theta,
            linear_v,
            linear_v / wheel_base * std::tan(steering_angle),
            t,
            new_x,
            new_y,
            new_theta);
    }

}  // namespace erl::env

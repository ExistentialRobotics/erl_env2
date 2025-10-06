#pragma once

#include "differential_drive_model.hpp"

namespace erl::env {

    template<typename Dtype>
    struct AckermannDriveControl {
        Dtype linear_v = 0.;
        Dtype wheel_base = 1.;  // distance between front and rear wheels
        Dtype steering_angle = 0.;
    };

    /**
     * Kinematic model of Ackermann drive robot.
     * @tparam Dtype data type, float or double.
     * @param x car x position
     * @param y car y position
     * @param theta car heading angle
     * @param linear_v linear velocity
     * @param wheel_base length between front and rear wheels
     * @param steering_angle steering angle of front wheels
     * @param t time duration
     * @param new_x new x position after t
     * @param new_y new y position after t
     * @param new_theta new heading angle after t
     */
    template<typename Dtype>
    void
    AckermannDriveKinematic(
        const Dtype x,
        const Dtype y,
        const Dtype theta,
        const Dtype linear_v,
        const Dtype wheel_base,
        const Dtype steering_angle,
        const Dtype t,
        Dtype &new_x,
        Dtype &new_y,
        Dtype &new_theta) {
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

#pragma once

#include "differential_drive_model.hpp"
#include "motion_primitive.hpp"

namespace erl::env {
    template<typename Dtype>
    using DdcMotionPrimitive = MotionPrimitive<DifferentialDriveControl<Dtype>, Dtype, 3>;

    using DdcMotionPrimitivef = DdcMotionPrimitive<float>;
    using DdcMotionPrimtived = DdcMotionPrimitive<double>;
}  // namespace erl::env

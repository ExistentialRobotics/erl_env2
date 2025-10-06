#pragma once

#include "differential_drive_model.hpp"
#include "motion_primitive.hpp"

namespace erl::env {
    template<typename Dtype>
    using DdcMotionPrimitive = MotionPrimitive<DifferentialDriveControl<Dtype>, Dtype, 3>;
}  // namespace erl::env

template<>
struct YAML::convert<erl::env::DdcMotionPrimitive<double>>
    : public erl::env::DdcMotionPrimitive<double>::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::DdcMotionPrimitive<float>>
    : public erl::env::DdcMotionPrimitive<float>::YamlConvertImpl {};

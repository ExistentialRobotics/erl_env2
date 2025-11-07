#pragma once

#include "environment_2d.hpp"
#include "pybind11_environment_type.hpp"

#include "erl_common/pybind11_yaml.hpp"

template<typename Env>
void
BindEnvironment2DExtra(py::class_<Env, typename Env::Super, std::shared_ptr<Env>> &env) {
    using Setting = typename Env::Setting;
    using GridMap = typename Env::GridMap;
    using GridMapInfo = typename GridMap::Info;
    using Cost = typename Env::Cost;

    env.def(
           py::init<
               const std::shared_ptr<GridMap> &,
               std::shared_ptr<Setting>,
               std::shared_ptr<Cost>>(),
           py::arg("grid_map"),
           py::arg("setting"),
           py::arg("cost_func"))
        .def(
            py::init<
                std::shared_ptr<GridMapInfo>,
                cv::Mat,
                std::shared_ptr<Setting>,
                std::shared_ptr<Cost>>(),
            py::arg("grid_map_info"),
            py::arg("cost_map"),
            py::arg("setting"),
            py::arg("cost_func"));

    BindYamlable<decltype(env), Setting>(env, "Setting")
        .def(
            "set_grid_motion_primitive",
            &Setting::SetGridMotionPrimitive,
            py::arg("max_axis_step"),
            py::arg("allow_diagonal"));
}

template<typename Dtype, typename MapDtype>
void
BindEnvironment2D(py::module &m, const char *env_name) {
    using Env = erl::env::Environment2D<Dtype, MapDtype>;
    BindEnvironmentType<Env>(m, env_name, BindEnvironment2DExtra<Env>);
}

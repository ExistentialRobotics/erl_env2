#pragma once

#include "environment_state.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_common/string_utils.hpp"

template<typename Dtype, int Dim>
void
BindEnvironmentState(py::module &m, const char *env_state_name) {

    using namespace erl::env;
    using EnvState = EnvironmentState<Dtype, Dim>;
    using MetricState = typename EnvState::MetricState;
    using GridState = typename EnvState::GridState;

    py::class_<EnvState>(m, env_state_name)
        .def(py::init<>())
        .def(py::init<MetricState>(), py::arg("metric_state"))
        .def(py::init<GridState>(), py::arg("grid_state"))
        .def(py::init<MetricState, GridState>(), py::arg("metric_state"), py::arg("grid_state"))
        .def_readwrite("metric", &EnvState::metric)
        .def_readwrite("grid", &EnvState::grid)
        .def("__repr__", [](const EnvState &s) {
            std::ostringstream oss;
            oss << type_name<EnvState>() << "(metric=" << s.metric.transpose()
                << ", grid=" << s.grid.transpose() << ")";
            return oss.str();
        });
}

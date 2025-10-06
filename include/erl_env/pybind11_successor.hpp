#pragma once

#include "environment_base.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindSuccessor(py::module& m, const char* successor_name) {
    using namespace erl::env;
    using Successor_t = Successor<Dtype, Dim>;

    py::class_<Successor_t>(m, successor_name)
        .def(
            py::init<const EnvironmentState<Dtype, Dim>&, Dtype, long, long>(),
            py::arg("state"),
            py::arg("cost"),
            py::arg("action_idx"),
            py::arg("env_id"))
        .def_readonly("state", &Successor_t::state)
        .def_readonly("cost", &Successor_t::cost)
        .def_readonly("action_idx", &Successor_t::action_idx)
        .def_readonly("env_id", &Successor_t::env_id);
}

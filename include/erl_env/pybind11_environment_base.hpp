#pragma once

#include "environment_base.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindEnvironmentBase(py::module& m, const char* env_name) {
    using namespace erl::env;
    using EnvironmentBase_t = EnvironmentBase<Dtype, Dim>;

    py::class_<EnvironmentBase_t, std::shared_ptr<EnvironmentBase_t>>(m, env_name)
        .def(py::init<long>(), py::arg("env_id") = 0)
        .def_property("env_id", &EnvironmentBase_t::GetEnvId, &EnvironmentBase_t::SetEnvId);

    // virtual methods:
    // .def_property_readonly("state_space_size", &EnvironmentBase_t::GetStateSpaceSize)
    // .def_property_readonly("action_space_size", &EnvironmentBase_t::GetActionSpaceSize)
    // .def(
    //     "forward_action",
    //     &EnvironmentBase_t::ForwardAction,
    //     py::arg("env_state"),
    //     py::arg("action_idx"))
    // .def("get_successors", &EnvironmentBase_t::GetSuccessors, py::arg("env_state"))
    // .def("in_state_space", &EnvironmentBase_t::InStateSpace, py::arg("env_state"))
    // .def("state_hashing", &EnvironmentBase_t::StateHashing, py::arg("env_state"))
    // .def("metric_to_grid", &EnvironmentBase_t::MetricToGrid, py::arg("metric_state"))
    // .def("grid_to_metric", &EnvironmentBase_t::GridToMetric, py::arg("grid_state"))
    // .def("sample_valid_states", &EnvironmentBase_t::SampleValidStates, py::arg("num_samples"));
}

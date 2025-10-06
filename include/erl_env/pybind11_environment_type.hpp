#pragma once

#include "erl_common/pybind11.hpp"

template<typename Env>
void
BindEnvironmentType(
    py::module &m,
    const char *env_name,
    std::function<void(py::class_<Env, typename Env::Super, std::shared_ptr<Env>> &)> extra_defs =
        nullptr) {

    py::class_<Env, typename Env::Super, std::shared_ptr<Env>> env(m, env_name);
    if (extra_defs) { extra_defs(env); }
    env.def_property_readonly("setting", &Env::GetSetting)
        .def_property_readonly("state_space_size", &Env::GetStateSpaceSize)
        .def_property_readonly("action_space_size", &Env::GetActionSpaceSize)
        .def("forward_action", &Env::ForwardAction, py::arg("env_state"), py::arg("action_idx"))
        .def("get_successors", &Env::GetSuccessors, py::arg("env_state"))
        .def("in_state_space", &Env::InStateSpace, py::arg("env_state"))
        .def("state_hashing", &Env::StateHashing, py::arg("env_state"))
        .def("metric_to_grid", &Env::MetricToGrid, py::arg("metric_state"))
        .def("grid_to_metric", &Env::GridToMetric, py::arg("grid_state"))
        .def("sample_valid_states", &Env::SampleValidStates, py::arg("num_samples"));
}

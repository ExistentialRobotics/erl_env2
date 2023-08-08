#include "erl_common/pybind11.hpp"
#include "erl_common/string_utils.hpp"
#include "erl_env/environment_state.hpp"
#include "erl_env/cost.hpp"
#include "erl_env/environment_base.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_se2.hpp"
#include "erl_env/environment_anchor.hpp"
#include "erl_env/environment_grid_anchor.hpp"

using namespace erl::common;
using namespace erl::env;

// trampoline class for allowing inheriting and overriding from Python
// https://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining--functions-and-inheritance

template<class Base = CostBase>
class PyCostBase : virtual public Base {
public:
    using Base::Base;

    double
    operator()(const EnvironmentState &state1, const EnvironmentState &state2) const override {
        PYBIND11_OVERRIDE_PURE_NAME(double, Base, "__call__", operator(), state1, state2);
    }
};

template<class EnvBase = EnvironmentBase>
class PyEnvBase : public EnvBase {
public:
    using EnvBase::EnvBase;  // Inherit constructors

    [[nodiscard]] std::size_t
    GetStateSpaceSize() const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "get_state_space_size", GetStateSpaceSize);
    }

    [[nodiscard]] std::size_t
    GetActionSpaceSize() const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "get_action_space_size", GetActionSpaceSize);
    }

    [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
    ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<std::shared_ptr<EnvironmentState>>, EnvBase, "forward_action", ForwardAction, env_state, action_coords);
    }

    [[nodiscard]] std::vector<Successor>
    GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<Successor>, EnvBase, "get_successors", GetSuccessors, env_state);
    }

    [[nodiscard]] bool
    InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(bool, EnvBase, "in_state_space", InStateSpace, env_state);
    }

    [[nodiscard]] uint32_t
    StateHashing(const std::shared_ptr<EnvironmentState> &env_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(uint32_t, EnvBase, "state_hashing", StateHashing, env_state);
    }

    [[nodiscard]] Eigen::VectorXi
    MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(Eigen::VectorXi, EnvBase, "metric_to_grid", MetricToGrid, metric_state);
    }

    [[nodiscard]] Eigen::VectorXd
    GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(Eigen::VectorXd, EnvBase, "grid_to_metric", GridToMetric, grid_state);
    }

    [[nodiscard]] cv::Mat
    ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override {
        PYBIND11_OVERRIDE_PURE_NAME(cv::Mat, EnvBase, "show_paths", ShowPaths, paths);
    }
};

template<class EnvBase = EnvironmentAnchor>
class PyEnvAnchor : public PyEnvBase<EnvBase> {
public:
    using PyEnvBase<EnvBase>::PyEnvBase;

    [[nodiscard]] std::size_t
    GetStateSpaceSize() const override {
        PYBIND11_OVERRIDE_NAME(std::size_t, EnvBase, "get_state_space_size", GetStateSpaceSize);
    }

    [[nodiscard]] std::size_t
    GetActionSpaceSize() const override {
        PYBIND11_OVERRIDE_NAME(std::size_t, EnvBase, "get_action_space_size", GetActionSpaceSize);
    }

    [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
    ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
        PYBIND11_OVERRIDE_NAME(std::vector<std::shared_ptr<EnvironmentState>>, EnvBase, "forward_action", ForwardAction, env_state, action_coords);
    }

    [[nodiscard]] std::vector<Successor>
    GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override {
        PYBIND11_OVERRIDE_NAME(std::vector<Successor>, EnvBase, "get_successors", GetSuccessors, env_state);
    }

    [[nodiscard]] bool
    InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
        PYBIND11_OVERRIDE_NAME(bool, EnvBase, "in_state_space", InStateSpace, env_state);
    }
};

static void
BindCosts(py::module &m) {
    py::class_<CostBase, PyCostBase<>, std::shared_ptr<CostBase>>(m, ERL_AS_STRING(CostBase))
        .def(py::init_alias<>())
        .def("__call__", &CostBase::operator(), py::arg("state1"), py::arg("state2"));

    py::class_<EuclideanDistanceCost, CostBase, std::shared_ptr<EuclideanDistanceCost>>(m, ERL_AS_STRING(EuclideanDistanceCost)).def(py::init<>());
    py::class_<ManhattanDistanceCost, CostBase, std::shared_ptr<ManhattanDistanceCost>>(m, ERL_AS_STRING(ManhattanDistanceCost)).def(py::init<>());
}

static void
BindEnvironments(py::module &m) {

    py::class_<EnvironmentBase, PyEnvBase<>, std::shared_ptr<EnvironmentBase>>(m, ERL_AS_STRING(EnvironmentBase))
        .def(py::init_alias<std::shared_ptr<CostBase>, double>(), py::arg("distance_cost_func").none(false), py::arg("time_step"))
        .def_property_readonly("state_space_size", &EnvironmentBase::GetStateSpaceSize)
        .def_property_readonly("action_space_size", &EnvironmentBase::GetActionSpaceSize)
        .def("forward_action", &EnvironmentBase::ForwardAction, py::arg("env_state"), py::arg("action_coords"))
        .def_property_readonly("distance_cost_func", &EnvironmentBase::GetDistanceCostFunc)
        .def_property_readonly("time_step", &EnvironmentBase::GetTimeStep)
        .def("get_successors", &EnvironmentBase::GetSuccessors, py::arg("env_state").none(false))
        .def("in_state_space", &EnvironmentBase::InStateSpace, py::arg("env_state").none(false))
        .def("state_hashing", &EnvironmentBase::StateHashing, py::arg("env_state").none(false))
        .def("metric_to_grid", &EnvironmentBase::MetricToGrid, py::arg("metric_state"))
        .def("grid_to_metric", &EnvironmentBase::GridToMetric, py::arg("grid_state"))
        .def("show_paths", &EnvironmentBase::ShowPaths, py::arg("paths"));

    auto env_2d = py::class_<Environment2D, EnvironmentBase, std::shared_ptr<Environment2D>>(m, ERL_AS_STRING(Environment2D));

    py::enum_<Environment2D::Action>(env_2d, "Action", py::arithmetic(), "Action in 2D grid environment.")
        .value(Environment2D::GetActionName(Environment2D::Action::kForward), Environment2D::Action::kForward)
        .value(Environment2D::GetActionName(Environment2D::Action::kBack), Environment2D::Action::kBack)
        .value(Environment2D::GetActionName(Environment2D::Action::kRight), Environment2D::Action::kRight)
        .value(Environment2D::GetActionName(Environment2D::Action::kLeft), Environment2D::Action::kLeft)
        .value(Environment2D::GetActionName(Environment2D::Action::kForwardRight), Environment2D::Action::kForwardRight)
        .value(Environment2D::GetActionName(Environment2D::Action::kForwardLeft), Environment2D::Action::kForwardLeft)
        .value(Environment2D::GetActionName(Environment2D::Action::kBackRight), Environment2D::Action::kBackRight)
        .value(Environment2D::GetActionName(Environment2D::Action::kBackLeft), Environment2D::Action::kBackLeft)
        .export_values();

    py::class_<Environment2D::Setting, YamlableBase, std::shared_ptr<Environment2D::Setting>>(env_2d, "Setting")
        .def_readwrite("allow_diagonal", &Environment2D::Setting::allow_diagonal)
        .def_readwrite("step_size", &Environment2D::Setting::step_size)
        .def_readwrite("down_sampled", &Environment2D::Setting::down_sampled)
        .def_readwrite("obstacle_threshold", &Environment2D::Setting::obstacle_threshold)
        .def_readwrite("add_map_cost", &Environment2D::Setting::add_map_cost)
        .def_readwrite("map_cost_factor", &Environment2D::Setting::map_cost_factor)
        .def_readwrite("shape", &Environment2D::Setting::shape);

    env_2d
        .def(
            py::init<const std::shared_ptr<GridMapUnsigned2D> &, std::shared_ptr<Environment2D::Setting>, std::shared_ptr<CostBase>>(),
            py::arg("grid_map"),
            py::arg("setting") = nullptr,
            py::arg("distance_cost_func") = nullptr)
        .def_property_readonly("setting", &Environment2D::GetSetting)
        .def_static("get_action_from_name", &Environment2D::GetActionFromName, py::arg("action_name"));

    py::class_<DifferentialDriveControl>(m, "DifferentialDriveControl", "Differential drive control.")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("linear_velocity"), py::arg("angular_velocity"))
        .def_readwrite("linear_v", &DifferentialDriveControl::linear_v, "Linear velocity.")
        .def_readwrite("angular_v", &DifferentialDriveControl::angular_v, "Angular velocity.");

    py::class_<DdcMotionPrimitive>(m, "DdcMotionPrimitive")
        .def(py::init<>())
        .def(
            py::init<std::vector<DifferentialDriveControl>, std::vector<double>, std::vector<double>>(),
            py::arg("controls"),
            py::arg("durations"),
            py::arg("costs"))
        .def_readwrite("controls", &DdcMotionPrimitive::controls)
        .def_readwrite("durations", &DdcMotionPrimitive::durations)
        .def_readwrite("costs", &DdcMotionPrimitive::costs)
        .def("compute_trajectories", &DdcMotionPrimitive::ComputeTrajectorySegments, py::arg("state"), py::arg("dt"), py::arg("motion_model_function"))
        .def(
            "compute_trajectory",
            py::overload_cast<
                const Eigen::Ref<const Eigen::VectorXd> &,
                std::size_t,
                double,
                const std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd> &, const DifferentialDriveControl &, double)> &>(
                &DdcMotionPrimitive::ComputeTrajectorySegment,
                py::const_),
            py::arg("state"),
            py::arg("control_idx"),
            py::arg("dt"),
            py::arg("motion_model_function"));

    m.def("load_ddc_motion_primitives_from_yaml", &LoadDdcMotionPrimitivesFromYaml, py::arg("filename"));

    auto env_se2 = py::class_<EnvironmentSe2, EnvironmentBase, std::shared_ptr<EnvironmentSe2>>(m, ERL_AS_STRING(EnvironmentSe2));

    py::class_<EnvironmentSe2::Setting, YamlableBase, std::shared_ptr<EnvironmentSe2::Setting>>(env_se2, "Setting")
        .def_readwrite("time_step", &EnvironmentSe2::Setting::time_step)
        .def_readwrite("motion_primitives", &EnvironmentSe2::Setting::motion_primitives)
        .def_readwrite("num_orientations", &EnvironmentSe2::Setting::num_orientations)
        .def_readwrite("obstacle_threshold", &EnvironmentSe2::Setting::obstacle_threshold)
        .def_readwrite("add_map_cost", &EnvironmentSe2::Setting::add_map_cost)
        .def_readwrite("map_cost_factor", &EnvironmentSe2::Setting::map_cost_factor)
        .def_readwrite("shape", &EnvironmentSe2::Setting::shape);

    env_se2
        .def(
            py::init<const std::shared_ptr<GridMapUnsigned2D> &, std::shared_ptr<EnvironmentSe2::Setting>>(),
            py::arg("grid_map"),
            py::arg("setting") = nullptr)
        .def_property_readonly("setting", &EnvironmentSe2::GetSetting)
        .def_static("motion_model", &EnvironmentSe2::MotionModel, py::arg("metric_state"), py::arg("control"), py::arg("t"));

    py::class_<EnvironmentAnchor, EnvironmentBase, PyEnvAnchor<>, std::shared_ptr<EnvironmentAnchor>>(m, ERL_AS_STRING(EnvironmentAnchor))
        .def(py::init_alias<std::vector<std::shared_ptr<EnvironmentBase>>>(), py::arg("environments"));

    py::class_<EnvironmentGridAnchor2D, EnvironmentAnchor, std::shared_ptr<EnvironmentGridAnchor2D>>(m, ERL_AS_STRING(EnvironmentGridAnchor2D))
        .def(
            py::init<std::vector<std::shared_ptr<EnvironmentBase>>, std::shared_ptr<GridMapInfo2D>>(),
            py::arg("environments"),
            py::arg("grid_map_info").none(false));

    py::class_<EnvironmentGridAnchor3D, EnvironmentAnchor, std::shared_ptr<EnvironmentGridAnchor3D>>(m, ERL_AS_STRING(EnvironmentGridAnchor3D))
        .def(
            py::init<std::vector<std::shared_ptr<EnvironmentBase>>, std::shared_ptr<GridMapInfo3D>>(),
            py::arg("environments"),
            py::arg("grid_map_info").none(false));
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_env";

    py::class_<EnvironmentState, std::shared_ptr<EnvironmentState>>(m, ERL_AS_STRING(EnvironmentState))
        .def_readwrite("metric", &EnvironmentState::metric)
        .def_readwrite("grid", &EnvironmentState::grid);
    py::class_<Successor>(m, ERL_AS_STRING(Successor))
        .def_readwrite("env_state", &Successor::env_state)
        .def_readwrite("cost", &Successor::cost)
        .def_readwrite("action_coords", &Successor::action_coords);

    BindCosts(m);
    BindEnvironments(m);
}

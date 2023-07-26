#include "erl_common/pybind11.hpp"
#include "erl_common/string_utils.hpp"
#include "erl_env/environment_state.hpp"
#include "erl_env/cost.hpp"
#include "erl_env/environment_base.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_se2.hpp"

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
class PyEnvBase : virtual public EnvBase {
public:
    using EnvBase::EnvBase;  // Inherit constructors

    [[nodiscard]] std::size_t
    GetStateSpaceSize() const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "get_state_space_size", GetStateSpaceSize);
    }

    [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
    ForwardAction(const std::shared_ptr<const EnvironmentState> &state, std::size_t action_index, double dt) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<std::shared_ptr<EnvironmentState>>, EnvBase, "forward_action", ForwardAction, state, action_index, dt);
    }

    [[nodiscard]] std::vector<Successor>
    GetSuccessors(const std::shared_ptr<const EnvironmentState> &state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<Successor>, EnvBase, "get_successors", GetSuccessors, state);
    }

    [[nodiscard]] bool
    IsReachable(const std::vector<std::shared_ptr<EnvironmentState>> &trajectory) const override {
        PYBIND11_OVERRIDE_PURE_NAME(bool, EnvBase, "is_reachable", IsReachable, trajectory);
    }

    [[nodiscard]] std::size_t
    GridStateHashing(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "grid_state_hashing", GridStateHashing, grid_state);
    }

    [[nodiscard]] Eigen::VectorXi
    MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(Eigen::VectorXi, EnvBase, "metric_to_grid", MetricToGrid, metric_state);
    }

    [[nodiscard]] Eigen::VectorXd
    GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(Eigen::VectorXd, EnvBase, "grid_to_metric", GridToMetric, grid_state);
    }

    [[nodiscard]] std::size_t
    ActionCoordsToActionIndex(const std::vector<std::size_t> &action_coords) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "action_coords_to_action_index", ActionCoordsToActionIndex, action_coords);
    }

    [[nodiscard]] std::vector<std::size_t>
    ActionIndexToActionCoords(std::size_t action_idx) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<std::size_t>, EnvBase, "action_index_to_action_coords", ActionIndexToActionCoords, action_idx);
    }

    void
    PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override {
        PYBIND11_OVERRIDE_PURE_NAME(void, EnvBase, "place_robot", PlaceRobot, metric_state);
    }

    void
    Reset() override {
        PYBIND11_OVERRIDE_PURE_NAME(void, EnvBase, "reset", Reset);
    }

    void
    SHowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
        PYBIND11_OVERRIDE_NAME(void, EnvBase, "show_paths", ShowPaths, paths);
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
        .def(py::init_alias<>())
        .def("forward_action", &EnvironmentBase::ForwardAction, py::arg("state"), py::arg("action_index"), py::arg("dt"))
        .def("get_successors", &EnvironmentBase::GetSuccessors, py::arg("state"))
        .def("is_reachable", &EnvironmentBase::IsReachable, py::arg("trajectory"))
        .def("grid_state_hashing", &EnvironmentBase::StateHashing, py::arg("grid_state"))
        .def("metric_to_grid", &EnvironmentBase::MetricToGrid, py::arg("metric_state"))
        .def("grid_to_metric", &EnvironmentBase::GridToMetric, py::arg("grid_state"))
        .def("action_coords_to_action_index", &EnvironmentBase::ActionCoordsToActionIndex, py::arg("action_coords"))
        .def("action_index_to_action_coords", &EnvironmentBase::ActionIndexToActionCoords, py::arg("action_index"));

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

    env_2d
        .def(
            py::init<bool, int, std::shared_ptr<CostBase>, const std::shared_ptr<GridMapUnsigned2D> &, uint8_t, bool, double>(),
            py::arg("allow_diagonal"),
            py::arg("step_size"),
            py::arg("cost_func"),
            py::arg("grid_map"),
            py::arg("obstacle_threshold"),
            py::arg("add_map_cost") = false,
            py::arg("map_cost_factor") = 1.0)
        .def(
            py::init<
                bool,
                int,
                const std::shared_ptr<CostBase> &,
                const std::shared_ptr<GridMapUnsigned2D> &,
                uint8_t,
                double,
                const Eigen::Ref<const Eigen::Matrix2Xd> &,
                bool,
                double>(),
            py::arg("allow_diagonal"),
            py::arg("step_size"),
            py::arg("cost_func"),
            py::arg("grid_map"),
            py::arg("obstacle_threshold"),
            py::arg("inflate_scale"),
            py::arg("shape_metric_vertices"),
            py::arg("add_map_cost") = false,
            py::arg("map_cost_factor") = 1.0)
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

    py::class_<EnvironmentSe2, EnvironmentBase, std::shared_ptr<EnvironmentSe2>>(m, ERL_AS_STRING(EnvironmentSe2))
        .def(
            py::init<double, std::vector<DdcMotionPrimitive>, const std::shared_ptr<GridMapUnsigned2D> &, uint8_t, int, bool, double>(),
            py::arg("collision_check_dt"),
            py::arg("motion_primitives"),
            py::arg("grid_map"),
            py::arg("obstacle_threshold"),
            py::arg("num_orientations"),
            py::arg("add_map_cost") = false,
            py::arg("map_cost_factor") = 1.0)
        .def(
            py::init<
                double,
                std::vector<DdcMotionPrimitive>,
                const std::shared_ptr<GridMapUnsigned2D> &,
                uint8_t,
                int,
                double,
                const Eigen::Ref<const Eigen::Matrix2Xd> &,
                bool,
                double>(),
            py::arg("collision_check_dt"),
            py::arg("motion_primitives"),
            py::arg("grid_map"),
            py::arg("obstacle_threshold"),
            py::arg("num_orientations"),
            py::arg("inflate_scale"),
            py::arg("shape_metric_vertices"),
            py::arg("add_map_cost") = false,
            py::arg("map_cost_factor") = 1.0)
        .def_static("motion_model", &EnvironmentSe2::MotionModel, py::arg("state"), py::arg("control"), py::arg("t"));

    // py::class_<EnvironmentGridSe2, EnvironmentBase, std::shared_ptr<EnvironmentGridSe2>>(m, ERL_AS_STRING(EnvironmentGridSe2))
    //     .def(py::init<int, const std::shared_ptr<GridMapUnsigned2D> &, int>(), py::arg("step_size"), py::arg("grid_map"), py::arg("num_orientations"))
    //     .def(
    //         py::init<int, const std::shared_ptr<GridMapUnsigned2D> &, int, double, const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));
    // py::class_<EnvironmentGridSe2, EnvironmentBase, std::shared_ptr<EnvironmentGridSe2>>(m, ERL_AS_STRING(EnvironmentGridSe2))
    //     .def(
    //         py::init<double, double, double, double, double, double, double, double, double, double, int, const std::shared_ptr<GridMapUnsigned2D> &, int>(),
    //         py::arg("linear_velocity_min"),
    //         py::arg("linear_velocity_max"),
    //         py::arg("linear_velocity_step"),
    //         py::arg("euclidean_square_distance_cost_weight"),
    //         py::arg("angular_velocity_min"),
    //         py::arg("angular_velocity_max"),
    //         py::arg("angular_velocity_step"),
    //         py::arg("angular_square_distance_cost_weight"),
    //         py::arg("duration_step"),
    //         py::arg("duration"),
    //         py::arg("max_step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"))
    //     .def(
    //         py::init<
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             int,
    //             double,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("linear_velocity_min"),
    //         py::arg("linear_velocity_max"),
    //         py::arg("linear_velocity_step"),
    //         py::arg("euclidean_square_distance_cost_weight"),
    //         py::arg("angular_velocity_min"),
    //         py::arg("angular_velocity_max"),
    //         py::arg("angular_velocity_step"),
    //         py::arg("angular_square_distance_cost_weight"),
    //         py::arg("duration_step"),
    //         py::arg("duration"),
    //         py::arg("max_step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_env";

    py::class_<Successor>(m, ERL_AS_STRING(Successor))
        .def_readwrite("env_state", &Successor::env_state)
        .def_readwrite("cost", &Successor::cost)
        .def_readwrite("action_id", &Successor::action_id);

    py::class_<EnvironmentState, std::shared_ptr<EnvironmentState>>(m, ERL_AS_STRING(EnvironmentState))
        .def_readwrite("metric", &EnvironmentState::metric)
        .def_readwrite("grid", &EnvironmentState::grid);

    BindCosts(m);
    BindEnvironments(m);
}

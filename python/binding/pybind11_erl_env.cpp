#include "erl_common/pybind11.hpp"
#include "erl_common/string_utils.hpp"
#include "erl_env/cost.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_anchor.hpp"
#include "erl_env/environment_base.hpp"
#include "erl_env/environment_grid_anchor.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"
#include "erl_env/environment_multi_resolution.hpp"
#include "erl_env/environment_scene_graph.hpp"
#include "erl_env/environment_se2.hpp"
#include "erl_env/environment_state.hpp"

using namespace erl::common;
using namespace erl::env;

// trampoline class for allowing inheriting and overriding from Python
// https://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining--functions-and-inheritance

template<class Base = CostBase>
class PyCostBase : public Base {
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
    ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const override {
        PYBIND11_OVERRIDE_PURE_NAME(cv::Mat, EnvBase, "show_paths", ShowPaths, paths, block);
    }

    [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
    SampleValidStates(int num_samples) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<std::shared_ptr<EnvironmentState>>, EnvBase, "sample_valid_states", SampleValidStates, num_samples);
    }
};

template<class EnvBase = EnvironmentMultiResolution>
class PyEnvMultiResolution : public PyEnvBase<EnvBase> {
public:
    using PyEnvBase<EnvBase>::PyEnvBase;

    [[nodiscard]] std::size_t
    GetNumResolutionLevels() const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::size_t, EnvBase, "get_num_resolution_levels", GetNumResolutionLevels);
    }

    [[nodiscard]] std::vector<Successor>
    GetSuccessorsAtLevel(const std::shared_ptr<EnvironmentState> &state, std::size_t resolution_level) const override {
        PYBIND11_OVERRIDE_PURE_NAME(std::vector<Successor>, EnvBase, "get_successors_at_level", GetSuccessorsAtLevel, state, resolution_level);
    }

    [[nodiscard]] bool
    InStateSpaceAtLevel(const std::shared_ptr<EnvironmentState> &env_state, std::size_t resolution_level) const override {
        PYBIND11_OVERRIDE_PURE_NAME(bool, EnvBase, "in_state_space_at_level", InStateSpaceAtLevel, env_state, resolution_level);
    }
};

static void
BindCosts(py::module &m) {
    py::class_<CostBase, PyCostBase<>, std::shared_ptr<CostBase>>(m, "CostBase")
        .def(py::init_alias<>())
        .def("__call__", &CostBase::operator(), py::arg("state1"), py::arg("state2"));

    py::class_<EuclideanDistanceCost, CostBase, std::shared_ptr<EuclideanDistanceCost>>(m, "EuclideanDistanceCost").def(py::init<>());
    py::class_<ManhattanDistanceCost, CostBase, std::shared_ptr<ManhattanDistanceCost>>(m, "ManhattanDistanceCost").def(py::init<>());
}

static void
BindSceneGraph(py::module &m) {
    py::class_<scene_graph::Node, std::shared_ptr<scene_graph::Node>> node(m, "Node");
    py::enum_<scene_graph::Node::Type>(node, "Type", py::arithmetic(), "Type of scene graph node.")
        .value("kOcc", scene_graph::Node::Type::kOcc)
        .value("kObject", scene_graph::Node::Type::kObject)
        .value("kRoom", scene_graph::Node::Type::kRoom)
        .value("kFloor", scene_graph::Node::Type::kFloor)
        .value("kBuilding", scene_graph::Node::Type::kBuilding)
        .export_values();
    node.def_readwrite("uuid", &scene_graph::Object::uuid)
        .def_readwrite("id", &scene_graph::Object::id)
        .def_readwrite("parent_id", &scene_graph::Object::parent_id)
        .def_readwrite("parent_uuid", &scene_graph::Object::parent_uuid)
        .def_readwrite("type", &scene_graph::Object::type)
        .def_readwrite("name", &scene_graph::Object::name);
    py::class_<scene_graph::Object, YamlableBase, std::shared_ptr<scene_graph::Object>> object(m, "Object");
    object.def_readwrite("action_affordance", &scene_graph::Object::action_affordance)
        .def_readwrite("grid_map_min", &scene_graph::Object::grid_map_min)
        .def_readwrite("grid_map_max", &scene_graph::Object::grid_map_max)
        .def_readwrite("location", &scene_graph::Object::location)
        .def_readwrite("size", &scene_graph::Object::size);
    py::enum_<scene_graph::Object::SOC>(object, "SOC", py::arithmetic(), "Special object category.")
        .value("kGround", scene_graph::Object::SOC::kGround)
        .value("kStairsUp", scene_graph::Object::SOC::kStairsUp)
        .value("kStairsDown", scene_graph::Object::SOC::kStairsDown)
        .value("kWall", scene_graph::Object::SOC::kWall)
        .value("kCeiling", scene_graph::Object::SOC::kCeiling)
        .value("kNA", scene_graph::Object::SOC::kNA)
        .export_values();
    py::class_<scene_graph::Room, YamlableBase, std::shared_ptr<scene_graph::Room>>(m, "Room")
        .def_readwrite("objects", &scene_graph::Room::objects)
        .def_readwrite("num_objects", &scene_graph::Room::num_objects)
        .def_readwrite("connected_room_ids", &scene_graph::Room::connected_room_ids)
        .def_readwrite("connected_room_uuids", &scene_graph::Room::connected_room_uuids)
        .def_readwrite("door_grids", &scene_graph::Room::door_grids)
        .def_readwrite("grid_map_min", &scene_graph::Room::grid_map_min)
        .def_readwrite("grid_map_max", &scene_graph::Room::grid_map_max)
        .def_readwrite("location", &scene_graph::Room::location)
        .def_readwrite("size", &scene_graph::Room::size);
    py::class_<scene_graph::Floor, YamlableBase, std::shared_ptr<scene_graph::Floor>>(m, "Floor")
        .def_readwrite("down_stairs_id", &scene_graph::Floor::down_stairs_id)
        .def_readwrite("up_stairs_id", &scene_graph::Floor::up_stairs_id)
        .def_readwrite("down_stairs_uuid", &scene_graph::Floor::down_stairs_uuid)
        .def_readwrite("up_stairs_uuid", &scene_graph::Floor::up_stairs_uuid)
        .def_readwrite("down_stairs_cost", &scene_graph::Floor::down_stairs_cost)
        .def_readwrite("up_stairs_cost", &scene_graph::Floor::up_stairs_cost)
        .def_readwrite("up_stairs_portal", &scene_graph::Floor::up_stairs_portal)
        .def_readwrite("down_stairs_portal", &scene_graph::Floor::down_stairs_portal)
        .def_readwrite("ground_z", &scene_graph::Floor::ground_z)
        .def_readwrite("room_map", &scene_graph::Floor::room_map)
        .def_readwrite("cat_map", &scene_graph::Floor::cat_map)
        .def_readwrite("rooms", &scene_graph::Floor::rooms)
        .def_readwrite("num_rooms", &scene_graph::Floor::num_rooms)
        .def_readwrite("grid_map_origin", &scene_graph::Floor::grid_map_origin)
        .def_readwrite("grid_map_resolution", &scene_graph::Floor::grid_map_resolution)
        .def_readwrite("grid_map_size", &scene_graph::Floor::grid_map_size);
    py::class_<scene_graph::Building, YamlableBase, std::shared_ptr<scene_graph::Building>>(m, "Building")
        .def_readwrite("floors", &scene_graph::Building::floors)
        .def_readwrite("num_floors", &scene_graph::Building::num_floors)
        .def_readwrite("reference_point", &scene_graph::Building::reference_point)
        .def_readwrite("size", &scene_graph::Building::size)
        .def_readwrite("room_ids", &scene_graph::Building::room_ids)
        .def_readwrite("room_uuids", &scene_graph::Building::room_uuids)
        .def_readwrite("object_ids", &scene_graph::Building::object_ids)
        .def_readwrite("object_uuids", &scene_graph::Building::object_uuids)
        .def_readwrite("id_to_object", &scene_graph::Building::id_to_object)
        .def_readwrite("id_to_room", &scene_graph::Building::id_to_room)
        .def_readwrite("uuid_to_node", &scene_graph::Building::uuid_to_node)
        .def("get_object_node", &scene_graph::Building::GetNode<erl::env::scene_graph::Object>, py::arg("object_id"))
        .def("get_room_node", &scene_graph::Building::GetNode<erl::env::scene_graph::Room>, py::arg("room_id"))
        .def("load_room_map", &scene_graph::Building::LoadRoomMap, py::arg("data_dir"), py::arg("floor_id"))
        .def("load_cat_map", &scene_graph::Building::LoadCatMap, py::arg("data_dir"), py::arg("floor_id"));
}

static void
BindAtomicProposition(py::module &m) {
    py::class_<AtomicProposition, YamlableBase, std::shared_ptr<AtomicProposition>> ap(m, "AtomicProposition");
    py::enum_<AtomicProposition::Type>(ap, "Type", py::arithmetic(), "Type of atomic proposition.")
        .value("kNA", AtomicProposition::Type::kNA)
        .value("kEnterRoom", AtomicProposition::Type::kEnterRoom)
        .value("kReachObject", AtomicProposition::Type::kReachObject)
        .export_values();
    ap.def(py::init<>())
        .def(py::init<AtomicProposition::Type, int, double>(), py::arg("type"), py::arg("uuid"), py::arg("reach_distance"))
        .def_readwrite("type", &AtomicProposition::type)
        .def_readwrite("uuid", &AtomicProposition::uuid)
        .def_readwrite("reach_distance", &AtomicProposition::reach_distance);
}

static void
BindFiniteStateAutomaton(py::module &m) {
    py::class_<FiniteStateAutomaton, std::shared_ptr<FiniteStateAutomaton>> fsa(m, "FiniteStateAutomaton");
    py::class_<FiniteStateAutomaton::Setting, YamlableBase, std::shared_ptr<FiniteStateAutomaton::Setting>> fsa_setting(fsa, "Setting");
    py::class_<FiniteStateAutomaton::Setting::Transition>(fsa_setting, "Transition")
        .def(py::init<>())
        .def(py::init<uint32_t, uint32_t, std::set<uint32_t>>(), py::arg("from"), py::arg("to"), py::arg("labels"))
        .def_readwrite("from", &FiniteStateAutomaton::Setting::Transition::from)
        .def_readwrite("to", &FiniteStateAutomaton::Setting::Transition::to)
        .def_readwrite("labels", &FiniteStateAutomaton::Setting::Transition::labels);
    py::enum_<FiniteStateAutomaton::Setting::FileType>(fsa_setting, "FileType", py::arithmetic(), "Type of finite state automaton file")
        .value("kSpotHoa", FiniteStateAutomaton::Setting::FileType::kSpotHoa)
        .value("kBoostDot", FiniteStateAutomaton::Setting::FileType::kBoostDot)
        .value("kYaml", FiniteStateAutomaton::Setting::FileType::kYaml)
        .export_values();
    fsa_setting.def(py::init<>())
        .def(py::init<const std::string &, FiniteStateAutomaton::Setting::FileType>(), py::arg("filepath"), py::arg("file_type"))
        .def_readwrite("num_states", &FiniteStateAutomaton::Setting::num_states)
        .def_readwrite("initial_state", &FiniteStateAutomaton::Setting::initial_state)
        .def_readwrite("accepting_states", &FiniteStateAutomaton::Setting::accepting_states)
        .def_readwrite("atomic_propositions", &FiniteStateAutomaton::Setting::atomic_propositions)
        .def_readwrite("transitions", &FiniteStateAutomaton::Setting::transitions)
        .def("as_boost_graph_dot_file", &FiniteStateAutomaton::Setting::AsBoostGraphDotFile, py::arg("filepath"))
        .def("from_boost_graph_dot_file", &FiniteStateAutomaton::Setting::FromBoostGraphDotFile, py::arg("filepath"))
        .def("as_spot_graph_hoa_file", &FiniteStateAutomaton::Setting::AsSpotGraphHoaFile, py::arg("filepath"))
        .def("from_spot_graph_hoa_file", &FiniteStateAutomaton::Setting::FromSpotGraphHoaFile, py::arg("filepath"));
}

static void
BindEnvironments(py::module &m) {

    py::class_<EnvironmentBase, PyEnvBase<>, std::shared_ptr<EnvironmentBase>>(m, "EnvironmentBase")
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
        .def("show_paths", &EnvironmentBase::ShowPaths, py::arg("paths"), py::arg("block"));

    auto env_2d = py::class_<Environment2D, EnvironmentBase, std::shared_ptr<Environment2D>>(m, "Environment2D");
    py::class_<Environment2D::Setting, YamlableBase, std::shared_ptr<Environment2D::Setting>>(env_2d, "Setting")
        .def(py::init<>())
        .def_readwrite("motions", &Environment2D::Setting::motions)
        .def_readwrite("grid_stride", &Environment2D::Setting::grid_stride)
        .def_readwrite("obstacle_threshold", &Environment2D::Setting::obstacle_threshold)
        .def_readwrite("add_map_cost", &Environment2D::Setting::add_map_cost)
        .def_readwrite("map_cost_factor", &Environment2D::Setting::map_cost_factor)
        .def_readwrite("shape", &Environment2D::Setting::shape)
        .def("set_grid_motion_primitive", &Environment2D::Setting::SetGridMotionPrimitive, py::arg("max_axis_step"), py::arg("allow_diagonal"));

    env_2d
        .def(
            py::init<const std::shared_ptr<GridMapUnsigned2D> &, std::shared_ptr<Environment2D::Setting>, std::shared_ptr<CostBase>>(),
            py::arg("grid_map"),
            py::arg("setting") = nullptr,
            py::arg("distance_cost_func") = nullptr)
        .def_property_readonly("setting", &Environment2D::GetSetting);

    auto env_se2 = py::class_<EnvironmentSe2, EnvironmentBase, std::shared_ptr<EnvironmentSe2>>(m, "EnvironmentSe2");

    py::class_<EnvironmentSe2::Setting, YamlableBase, std::shared_ptr<EnvironmentSe2::Setting>>(env_se2, "Setting")
        .def(py::init<>())
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

    py::class_<EnvironmentMultiResolution, PyEnvMultiResolution<>, std::shared_ptr<EnvironmentMultiResolution>>(m, "EnvironmentMultiResolution")
        .def(py::init_alias<>())
        .def_property_readonly("num_resolution_levels", &EnvironmentMultiResolution::GetNumResolutionLevels)
        .def("get_successor_at_level", &EnvironmentMultiResolution::GetSuccessorsAtLevel, py::arg("env_state"), py::arg("resolution_level"))
        .def("in_state_space_at_level", &EnvironmentMultiResolution::InStateSpaceAtLevel, py::arg("env_state"), py::arg("resolution_level"));

    py::class_<EnvironmentAnchor, EnvironmentMultiResolution, PyEnvMultiResolution<EnvironmentAnchor>, std::shared_ptr<EnvironmentAnchor>>(
        m,
        "EnvironmentAnchor")
        .def(py::init_alias<std::vector<std::shared_ptr<EnvironmentBase>>>(), py::arg("environments"));

    py::class_<EnvironmentGridAnchor2D, EnvironmentAnchor, std::shared_ptr<EnvironmentGridAnchor2D>>(m, "EnvironmentGridAnchor2D")
        .def(
            py::init<std::vector<std::shared_ptr<EnvironmentBase>>, std::shared_ptr<GridMapInfo2D>>(),
            py::arg("environments"),
            py::arg("grid_map_info").none(false));

    py::class_<EnvironmentGridAnchor3D, EnvironmentAnchor, std::shared_ptr<EnvironmentGridAnchor3D>>(m, "EnvironmentGridAnchor3D")
        .def(
            py::init<std::vector<std::shared_ptr<EnvironmentBase>>, std::shared_ptr<GridMapInfo3D>>(),
            py::arg("environments"),
            py::arg("grid_map_info").none(false));

    py::class_<EnvironmentSceneGraph, EnvironmentMultiResolution, PyEnvMultiResolution<EnvironmentSceneGraph>, std::shared_ptr<EnvironmentSceneGraph>>
        env_graph(m, "EnvironmentSceneGraph");
    py::class_<EnvironmentSceneGraph::Setting, YamlableBase, std::shared_ptr<EnvironmentSceneGraph::Setting>>(env_graph, "Setting")
        .def_readwrite("data_dir", &EnvironmentSceneGraph::Setting::data_dir)
        .def_readwrite("num_threads", &EnvironmentSceneGraph::Setting::num_threads)
        .def_readwrite("allow_diagonal", &EnvironmentSceneGraph::Setting::allow_diagonal)
        .def_readwrite("object_reach_distance", &EnvironmentSceneGraph::Setting::object_reach_distance)
        .def_readwrite("shape", &EnvironmentSceneGraph::Setting::shape)
        .def_readwrite("max_level", &EnvironmentSceneGraph::Setting::max_level);
    env_graph
        .def(
            py::init_alias<std::shared_ptr<scene_graph::Building>, std::shared_ptr<EnvironmentSceneGraph::Setting>>(),
            py::arg("scene_graph"),
            py::arg("setting") = nullptr)
        .def_property_readonly("setting", &EnvironmentSceneGraph::GetSetting)
        .def_property_readonly("grid_map_info", &EnvironmentSceneGraph::GetGridMapInfo);

    py::class_<EnvironmentLTLSceneGraph, EnvironmentSceneGraph, PyEnvMultiResolution<EnvironmentLTLSceneGraph>, std::shared_ptr<EnvironmentLTLSceneGraph>>
        env_ltl_graph(m, "EnvironmentLTLSceneGraph");
    py::class_<EnvironmentLTLSceneGraph::Setting, YamlableBase, std::shared_ptr<EnvironmentLTLSceneGraph::Setting>>(env_ltl_graph, "Setting")
        .def_readwrite("atomic_propositions", &EnvironmentLTLSceneGraph::Setting::atomic_propositions)
        .def_readwrite("fsa", &EnvironmentLTLSceneGraph::Setting::fsa)
        .def("load_atomic_propositions", &EnvironmentLTLSceneGraph::Setting::LoadAtomicPropositions, py::arg("yaml_file"));
    env_ltl_graph.def_property_readonly("finite_state_automaton", &EnvironmentLTLSceneGraph::GetFiniteStateAutomaton)
        .def_property_readonly("setting", &EnvironmentLTLSceneGraph::GetSetting)
        .def_property_readonly("label_maps", &EnvironmentLTLSceneGraph::GetLabelMaps);
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_env";

    py::class_<EnvironmentState, std::shared_ptr<EnvironmentState>>(m, "EnvironmentState")
        .def_readwrite("metric", &EnvironmentState::metric)
        .def_readwrite("grid", &EnvironmentState::grid);
    py::class_<Successor>(m, "Successor")
        .def_readwrite("env_state", &Successor::env_state)
        .def_readwrite("cost", &Successor::cost)
        .def_readwrite("action_coords", &Successor::action_coords);

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

    BindCosts(m);
    BindSceneGraph(m);
    BindAtomicProposition(m);
    BindFiniteStateAutomaton(m);
    BindEnvironments(m);
}

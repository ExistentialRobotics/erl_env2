#pragma once

#include <bitset>
#include "erl_common/yaml.hpp"
#include "erl_common/grid_map_info.hpp"
#include "environment_scene_graph.hpp"
#include "scene_graph.hpp"
#include "finite_state_automaton.hpp"
#include "atomic_proposition.hpp"

namespace erl::env {

    class EnvironmentLTLSceneGraph : public EnvironmentSceneGraph {
    public:
        struct Setting : public common::OverrideYamlable<EnvironmentSceneGraph::Setting, Setting> {
            std::unordered_map<std::string, AtomicProposition> atomic_propositions;
            std::shared_ptr<FiniteStateAutomaton::Setting> fsa;  // finite state automaton

            void
            LoadAtomicPropositions(const std::string &yaml_file) {
                YAML::Node node = YAML::LoadFile(yaml_file);
                YAML::convert<std::unordered_map<std::string, AtomicProposition>>::decode(node, atomic_propositions);
            }
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_ = nullptr;
        std::unordered_map<int, Eigen::MatrixX<uint32_t>> m_label_maps_ = {};

    public:
        EnvironmentLTLSceneGraph(std::shared_ptr<scene_graph::Building> building, const std::shared_ptr<EnvironmentLTLSceneGraph::Setting> &setting)
            : EnvironmentSceneGraph(std::move(building), setting),
              m_setting_(setting) {
            m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
            GenerateLabelMaps();
        }

        [[nodiscard]] inline std::shared_ptr<FiniteStateAutomaton>
        GetFiniteStateAutomaton() const {
            return m_fsa_;
        }

        [[nodiscard]] inline std::unordered_map<int, Eigen::MatrixX<uint32_t>>
        GetLabelMaps() const {
            return m_label_maps_;
        }

        [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override;

        [[nodiscard]] std::vector<Successor>
        GetSuccessorsAtLevel(const std::shared_ptr<EnvironmentState> &env_state, std::size_t resolution_level) const override;

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid.head<3>());
        }

        [[nodiscard]] inline bool
        InStateSpaceAtLevel(const std::shared_ptr<EnvironmentState> &env_state, std::size_t resolution_level) const override {
            if (resolution_level == 0) { return m_grid_map_info_->InGrids(env_state->grid.head<3>()); }
            auto level = scene_graph::Node::Type(resolution_level - 1);
            if (!m_grid_map_info_->InGrids(env_state->grid.head<3>())) { return false; }
            switch (level) {
                case scene_graph::Node::Type::kObject:
                    return !m_object_reached_maps_.at(env_state->grid[2])(env_state->grid[0], env_state->grid[1]).empty();
                case scene_graph::Node::Type::kRoom:
                    return m_room_maps_[env_state->grid[2]].at<int>(env_state->grid[0], env_state->grid[1]) > 0;
                case scene_graph::Node::Type::kNA:
                case scene_graph::Node::Type::kFloor:
                case scene_graph::Node::Type::kBuilding:
                    return true;
                default:
                    throw std::runtime_error("Unknown level.");
            }
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            uint32_t hashing = m_grid_map_info_->GridToIndex(env_state->grid.head<3>(), true);
            hashing = hashing * m_setting_->fsa->num_states + env_state->grid[3];
            return hashing;
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            return Eigen::Vector4i(
                m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
                m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
                m_grid_map_info_->MeterToGridForValue(metric_state[2], 2),
                int(metric_state[3]));
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            return Eigen::Vector4d(
                m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
                m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
                m_grid_map_info_->GridToMeterForValue(grid_state[2], 2),
                grid_state[3]);
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

    protected:
        void
        GenerateLabelMaps();

        inline bool
        EvaluateAtomicProposition(int x, int y, int floor_num, const AtomicProposition &proposition) {
            switch (proposition.type) {
                case AtomicProposition::Type::kNA:
                    return false;
                case AtomicProposition::Type::kEnterRoom:
                    return EvaluateEnterRoom(x, y, floor_num, proposition.uuid);
                case AtomicProposition::Type::kReachObject: {
                    double reach_distance = proposition.reach_distance > 0 ? proposition.reach_distance : m_setting_->object_reach_distance;
                    return EvaluateReachObject(x, y, floor_num, proposition.uuid, reach_distance);
                }
                default:
                    throw std::runtime_error("Unknown atomic proposition type.");
            }
        }

        inline bool
        EvaluateEnterRoom(int x, int y, int floor_num, int uuid) {
            auto room = m_scene_graph_->GetNode<scene_graph::Room>(uuid);
            return m_room_maps_[floor_num].at<int>(x, y) == room->id;
        }

        inline bool
        EvaluateReachObject(int x, int y, int floor_num, int uuid, double reach_distance) {
            auto half_rows = int(reach_distance / m_grid_map_info_->Resolution(0));
            if (half_rows == 0) { half_rows = 1; }
            auto half_cols = int(reach_distance / m_grid_map_info_->Resolution(1));
            if (half_cols == 0) { half_cols = 1; }

            int object_id = m_scene_graph_->GetNode<scene_graph::Object>(uuid)->id;
            auto &cat_map = m_cat_maps_[floor_num];
            int roi_min_x = x - half_rows;
            int roi_min_y = y - half_cols;
            int roi_max_x = x + half_rows;
            int roi_max_y = y + half_cols;
            if (roi_min_x < 0) { roi_min_x = 0; }
            if (roi_min_y < 0) { roi_min_y = 0; }
            if (roi_max_x >= cat_map.rows) { roi_max_x = cat_map.rows - 1; }
            if (roi_max_y >= cat_map.cols) { roi_max_y = cat_map.cols - 1; }
            for (int r = roi_min_x; r <= roi_max_x; ++r) {
                for (int c = roi_min_y; c <= roi_max_y; ++c) {
                    if (cat_map.at<int>(r, c) == object_id) { return true; }
                }
            }
            return false;
        }

    private:
        std::vector<std::shared_ptr<EnvironmentState>>
        ConvertPath(const std::vector<std::array<int, 2>> &path, int floor_num, int cur_q) const {
            std::vector<std::shared_ptr<EnvironmentState>> next_env_states;
            next_env_states.reserve(path.size() + 1);
            for (auto &point: path) {
                auto next_env_state = std::make_shared<EnvironmentState>();
                next_env_state->grid.resize(4);
                int &nx = next_env_state->grid[0];
                int &ny = next_env_state->grid[1];
                int &nz = next_env_state->grid[2];
                int &nq = next_env_state->grid[3];
                nx = point[0];
                ny = point[1];
                nz = floor_num;
                nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(nz)(nx, ny)));
                cur_q = nq;  // update cur_q
                next_env_state->metric = GridToMetric(next_env_state->grid);
                next_env_states.push_back(next_env_state);
            }
            return next_env_states;
        }

        std::vector<std::shared_ptr<EnvironmentState>>
        GetPathToFloor(int xg, int yg, int floor_num, int cur_q, int next_floor_num) const {
            ERL_DEBUG_ASSERT(std::abs(floor_num - next_floor_num) == 1, "floor_num and next_floor_num should differ by 1.");
            auto &path = m_up_stairs_path_maps_.at(floor_num)(xg, yg);
            std::vector<std::shared_ptr<EnvironmentState>> next_env_states = ConvertPath(path, floor_num, cur_q);
            auto next_env_state = std::make_shared<EnvironmentState>();
            auto &floor = m_scene_graph_->floors.at(next_floor_num);
            next_env_state->grid.resize(4);
            int &nx = next_env_state->grid[0];
            int &ny = next_env_state->grid[1];
            int &nz = next_env_state->grid[2];
            int &nq = next_env_state->grid[3];
            if (floor_num < next_floor_num) {  // go upstairs
                nx = floor->down_stairs_portal.value()[0];
                ny = floor->down_stairs_portal.value()[1];
                nz = next_floor_num;
            } else {  // go downstairs
                nx = floor->up_stairs_portal.value()[0];
                ny = floor->up_stairs_portal.value()[1];
                nz = next_floor_num;
            }
            if (!next_env_states.empty()) { cur_q = next_env_states.back()->grid[3]; }
            nq = int(m_fsa_->GetNextState(cur_q, m_label_maps_.at(next_floor_num)(nx, ny)));
            next_env_state->metric = GridToMetric(next_env_state->grid);
            next_env_states.push_back(next_env_state);
            return next_env_states;
        }
    };
}  // namespace erl::env

namespace YAML {
    template<>
    struct convert<erl::env::EnvironmentLTLSceneGraph::Setting> {
        inline static Node
        encode(const erl::env::EnvironmentLTLSceneGraph::Setting &rhs) {
            Node node = convert<erl::env::EnvironmentSceneGraph::Setting>::encode(rhs);
            node["atomic_propositions"] = rhs.atomic_propositions;
            node["fsa"] = rhs.fsa;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::env::EnvironmentLTLSceneGraph::Setting &rhs) {
            if (!convert<erl::env::EnvironmentSceneGraph::Setting>::decode(node, rhs)) { return false; }
            rhs.atomic_propositions = node["atomic_propositions"].as<std::unordered_map<std::string, erl::env::AtomicProposition>>();
            rhs.fsa = node["fsa"].as<std::shared_ptr<erl::env::FiniteStateAutomaton::Setting>>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::env::EnvironmentLTLSceneGraph::Setting &rhs) {
        out << static_cast<const erl::env::EnvironmentSceneGraph::Setting &>(rhs);
        out << BeginMap;
        out << Key << "atomic_propositions" << Value << rhs.atomic_propositions;
        out << Key << "fsa" << Value << rhs.fsa;
        out << EndMap;
        return out;
    }
}  // namespace YAML

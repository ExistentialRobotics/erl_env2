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
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_ = nullptr;
        std::unordered_map<int, Eigen::MatrixX<uint64_t>> m_label_maps_ = {};

    public:
        EnvironmentLTLSceneGraph(std::shared_ptr<scene_graph::Building> building, const std::shared_ptr<EnvironmentLTLSceneGraph::Setting> &setting)
            : EnvironmentSceneGraph(std::move(building), setting),
              m_setting_(setting) {
            m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
            GenerateLabelMaps();
        }

    private:
        void
        GenerateLabelMaps() {
            std::unordered_map<int, Eigen::MatrixX<std::bitset<64>>> label_maps = {};
            for (int i = 0; i < m_scene_graph_->num_floors; ++i) {  // initialize the label maps
                label_maps[i].resize(m_floor_grid_map_info_->Shape(0), m_floor_grid_map_info_->Shape(1));
            }

            // TODO: omp parallel for

            for (int i = 0; i < m_scene_graph_->num_floors; ++i) {  // each floor
                int rows = m_floor_grid_map_info_->Shape(0);
#pragma omp parallel for collapse(2) default(none) shared(rows, label_maps, i)
                for (int r = 0; r < rows; ++r) {  // each row of the map
                    int cols = m_floor_grid_map_info_->Shape(1);
                    for (int c = 0; c < cols; ++c) {  // each column of the map
                        std::size_t num_propositions = m_setting_->fsa->atomic_propositions.size();
                        auto &bitset = label_maps[i](r, c);
                        for (std::size_t j = 0; j < num_propositions; ++j) {  // each atomic proposition
                            auto &proposition = m_setting_->atomic_propositions[m_setting_->fsa->atomic_propositions[j]];
                            bitset.set(j, EvaluateAtomicProposition(r, c, i, proposition));
                        }
                    }
                }
                m_label_maps_[i] = label_maps[i].cast<uint64_t>();
            }
        }

        inline bool
        EvaluateAtomicProposition(int x, int y, int floor_num, const AtomicProposition &proposition) {
            switch (proposition.type) {
                case AtomicProposition::Type::kNA:
                    return false;
                case AtomicProposition::Type::kEnterRoom:
                    return EvaluateEnterRoom(x, y, floor_num, proposition.uuid);
                case AtomicProposition::Type::kReachObject:
                    return EvaluateReachObject(x, y, floor_num, proposition.uuid, proposition.reach_distance);
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
            auto half_rows = int(reach_distance / m_floor_grid_map_info_->Resolution(0));
            if (half_rows == 0) { half_rows = 1; }
            auto half_cols = int(reach_distance / m_floor_grid_map_info_->Resolution(1));
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

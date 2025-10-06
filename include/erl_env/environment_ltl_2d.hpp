#pragma once

// https://github.com/ExistentialRobotics/erl_astar/blob/master/test/old/test_astar_2d_ltl.cpp
// https://github.com/ExistentialRobotics/erl_env/blob/master/include/erl_env/ltl/fsa.h
// https://github.com/ExistentialRobotics/erl_astar/blob/master/data/maps/ltl/mapjie_2d.yaml

#include "cost.hpp"
#include "environment_2d.hpp"
#include "finite_state_automaton.hpp"
#include "motion_primitive.hpp"

#include "erl_common/exception.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/random.hpp"

#include <numeric>

namespace erl::env {

    /**
     * 2D Grid Environment with Linear Temporal Logic. The state is (x, y, ltl_state).
     */
    template<typename Dtype, typename MapDtype = uint8_t>
    class EnvironmentLTL2D : public EnvironmentBase<Dtype, 3> {
    public:
        struct Setting
            : public common::Yamlable<Setting, typename Environment2D<Dtype, MapDtype>::Setting> {
            using Super = typename Environment2D<Dtype, MapDtype>::Setting;

            std::shared_ptr<FiniteStateAutomaton::Setting> fsa =
                std::make_shared<FiniteStateAutomaton::Setting>();

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting) {
                    YAML::Node node = Super::YamlConvertImpl::encode(setting);
                    ERL_YAML_SAVE_ATTR(node, setting, fsa);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, Setting &setting) {
                    if (!Super::YamlConvertImpl::decode(node, setting)) { return false; }
                    if (!ERL_YAML_LOAD_ATTR(node, setting, fsa)) { return false; }
                    return true;
                }
            };
        };

        using Cost = CostBase<Dtype, 2>;
        using GridMap = common::GridMap<MapDtype, Dtype, 2>;
        using GridMapInfo = common::GridMapInfo3D<Dtype>;
        using State = EnvironmentState<Dtype, 3>;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using Successor_t = Successor<Dtype, 3>;

    private:
        std::shared_ptr<Setting> m_setting_;            // environment setting
        std::shared_ptr<GridMap> m_grid_map_ext_;       // grid map
        std::shared_ptr<GridMapInfo> m_grid_map_info_;  // grid map description
        cv::Mat m_original_grid_map_;  // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;           // inflated grid map
        std::vector<Eigen::Matrix2Xi> m_rel_trajectories_;  // rel trajectories of motion primitives
        std::vector<Dtype> m_motion_costs_;                 // cost of each control
        Eigen::MatrixX<std::vector<int>> m_reachable_motions_;  // reachable controls for each grid

        // each element is a |AP|-bit word representing the result of atomic propositions
        Eigen::MatrixX<uint32_t> m_label_map_;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_;

    public:
        EnvironmentLTL2D(
            Eigen::MatrixX<uint32_t> label_map,
            const std::shared_ptr<GridMap> &grid_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<Cost> cost_func)
            : EnvironmentBase<Dtype, 3>(),
              m_setting_(NotNull(std::move(setting), true, "setting is nullptr.")),
              m_grid_map_ext_(NotNull(grid_map, true, "grid_map is nullptr")),
              m_original_grid_map_(
                  grid_map->info->Shape(0),
                  grid_map->info->Shape(1),
                  common::CvMatType<MapDtype, 1>(),
                  m_grid_map_ext_->data.Data().data()),
              m_label_map_(std::move(label_map)) {

            ERL_ASSERTM(
                m_setting_->fsa->atomic_propositions.size() <= 64,
                "Does not support more than 64 atomic propositions.");

            auto num_states = static_cast<int>(m_setting_->fsa->num_states);
            if (num_states % 2 == 0) { num_states += 1; }
            m_grid_map_info_ = std::make_shared<GridMapInfo>(grid_map->info->Extend(
                static_cast<int>(num_states),
                -0.5f,
                static_cast<Dtype>(num_states) - 0.5f,
                2));
            m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());

            // generate 2D obstacle/cost map
            if (m_setting_->robot_metric_contour.cols() > 0) {
                common::InflateWithShape(
                    m_original_grid_map_,
                    grid_map->info,
                    m_setting_->robot_metric_contour,
                    m_grid_map_);
            } else {
                m_original_grid_map_.copyTo(m_grid_map_);
            }

            // compute relative trajectories and costs of each control
            m_rel_trajectories_.reserve(m_setting_->motions.size());
            m_motion_costs_.reserve(m_setting_->motions.size());

            EnvironmentState<Dtype, 2> ref_start, end;
            ref_start.metric = grid_map->info->Center();
            ref_start.grid = grid_map->info->CenterGrid();

            ERL_ASSERTM(cost_func != nullptr, "cost_func is nullptr.");
            const Cost &cost_func_2d = *cost_func;

            for (const Eigen::Vector2i &control: m_setting_->motions) {
                end.grid = ref_start.grid + control;
                end.metric = grid_map->info->GridToMeterForPoints(end.grid);

                Eigen::Matrix2Xi rel_trajectory =
                    grid_map->info->RayCasting(ref_start.metric, end.metric).colwise() -
                    ref_start.grid;
                Dtype control_cost = cost_func_2d(ref_start, end);
                m_rel_trajectories_.push_back(rel_trajectory);
                m_motion_costs_.push_back(control_cost);
            }

            // configure finite state automaton
            ERL_ASSERTM(m_setting_->fsa != nullptr, "setting->fsa is nullptr.");
            m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
            ERL_ASSERTM(
                m_grid_map_.rows == m_label_map_.rows(),
                "label_map and grid_map should have the same number of rows.");
            ERL_ASSERTM(
                m_grid_map_.cols == m_label_map_.cols(),
                "label_map and grid_map should have the same number of columns.");
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<FiniteStateAutomaton>
        GetFiniteStateAutomaton() const {
            return m_fsa_;
        }

        [[nodiscard]] std::shared_ptr<GridMapInfo>
        GetGridMapInfo() const {
            return m_grid_map_info_;
        }

        [[nodiscard]] std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] std::size_t
        GetActionSpaceSize() const override {
            return m_setting_->motions.size();
        }

        [[nodiscard]] std::vector<State>
        ForwardAction(const State &env_state, long action_idx) const override {
            State new_state;
            int &nx_grid = new_state.grid[0];
            int &ny_grid = new_state.grid[1];
            int &nq = new_state.grid[2];
            auto &control = m_setting_->motions[action_idx];

            nx_grid = env_state.grid[0] + control[0];
            ny_grid = env_state.grid[1] + control[1];
            nq = static_cast<int>(
                m_fsa_->GetNextState(env_state.grid[2], m_label_map_(nx_grid, ny_grid)));

            new_state.metric[0] = m_grid_map_info_->GridToMeterAtDim(nx_grid, 0);
            new_state.metric[1] = m_grid_map_info_->GridToMeterAtDim(ny_grid, 1);
            new_state.metric[2] = new_state.grid[2];
            return {new_state};
        }

        [[nodiscard]] std::vector<Successor_t>
        GetSuccessors(const State &env_state) const override {
            if (!InStateSpace(env_state)) { return {}; }

            const int cur_xg = env_state.grid[0];
            const int cur_yg = env_state.grid[1];
            const auto num_motions = static_cast<int>(m_rel_trajectories_.size());
            auto &reachable_motions =
                const_cast<std::vector<int> &>(m_reachable_motions_(cur_xg, cur_yg));
            if (reachable_motions.empty()) {
                for (int motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
                    auto &rel_trajectory = m_rel_trajectories_[motion_idx];
                    bool is_reachable = true;
                    for (long i = 0; i < rel_trajectory.cols(); ++i) {
                        auto wp = rel_trajectory.col(i);
                        int nx_grid = cur_xg + wp[0];
                        if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
                            is_reachable = false;
                            break;
                        }
                        int ny_grid = cur_yg + wp[1];
                        if (ny_grid < 0 || ny_grid >= m_grid_map_info_->Shape(1)) {
                            is_reachable = false;
                            break;
                        }
                        if (m_grid_map_.at<MapDtype>(nx_grid, ny_grid) >=
                            m_setting_->obstacle_threshold) {
                            is_reachable = false;
                            break;
                        }
                    }
                    if (is_reachable) { reachable_motions.push_back(motion_idx); }
                }
                if (reachable_motions.empty()) { reachable_motions.push_back(-1); }
            }
            if (reachable_motions[0] == -1) { return {}; }  // no reachable control

            std::vector<Successor_t> successors;
            successors.clear();
            successors.reserve(reachable_motions.size());
            const int &cur_q = env_state.grid[2];
            for (auto &motion_idx: reachable_motions) {
                ERL_DEBUG_ASSERT(
                    motion_idx >= 0 && motion_idx < num_motions,
                    "Invalid motion index: {}.",
                    motion_idx);

                auto &rel_trajectory = m_rel_trajectories_[motion_idx];
                bool is_reachable = true;

                State next_state;
                int &nx_grid = next_state.grid[0];
                int &ny_grid = next_state.grid[1];
                int &nq = next_state.grid[2];

                const long num_steps = rel_trajectory.cols();
                for (long i = 0; i < num_steps; ++i) {
                    nx_grid = cur_xg + rel_trajectory(0, i);
                    ny_grid = cur_yg + rel_trajectory(1, i);
                    // check LTL
                    nq = static_cast<int>(
                        m_fsa_->GetNextState(cur_q, m_label_map_(nx_grid, ny_grid)));
                    if (m_fsa_->IsSinkState(nq)) {
                        is_reachable = false;  // cannot reach accepting state
                        break;
                    }
                }
                if (!is_reachable) { continue; }

                next_state.metric[0] = m_grid_map_info_->GridToMeterAtDim(nx_grid, 0);
                next_state.metric[1] = m_grid_map_info_->GridToMeterAtDim(ny_grid, 1);
                next_state.metric[2] = nq;
                if (m_setting_->add_map_cost) {
                    const Dtype map_cost =
                        m_setting_->map_cost_factor *
                        static_cast<Dtype>(
                            m_grid_map_.at<MapDtype>(next_state.grid[0], next_state.grid[1]));
                    const Dtype cost = m_motion_costs_[motion_idx] + map_cost;
                    successors.emplace_back(next_state, cost, motion_idx, this->m_env_id_);
                } else {
                    successors.emplace_back(
                        next_state,
                        m_motion_costs_[motion_idx],
                        motion_idx,
                        this->m_env_id_);
                }
            }
            return successors;
        }

        [[nodiscard]] bool
        InStateSpace(const State &env_state) const override {
            return m_grid_map_info_->InGrids(env_state.grid) &&
                   (m_setting_->grid_stride == 1 ||
                    (env_state.grid[0] % m_setting_->grid_stride == 0 &&
                     env_state.grid[1] % m_setting_->grid_stride == 0));
        }

        [[nodiscard]] uint32_t
        StateHashing(const State &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state.grid, true);
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            return {
                m_grid_map_info_->MeterToGridAtDim(metric_state[0], 0),
                m_grid_map_info_->MeterToGridAtDim(metric_state[1], 1),
                static_cast<int>(metric_state[2])};
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            return {
                m_grid_map_info_->GridToMeterAtDim(grid_state[0], 0),
                m_grid_map_info_->GridToMeterAtDim(grid_state[1], 1),
                static_cast<Dtype>(grid_state[2])};
        }

        [[nodiscard]] std::vector<State>
        SampleValidStates(int /*num_samples*/) const override {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };

    extern template class EnvironmentLTL2D<float, uint8_t>;
    extern template class EnvironmentLTL2D<double, uint8_t>;
    extern template class EnvironmentLTL2D<float, float>;
    extern template class EnvironmentLTL2D<double, float>;
    extern template class EnvironmentLTL2D<float, double>;
    extern template class EnvironmentLTL2D<double, double>;

}  // namespace erl::env

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<float>::Setting>
    : public erl::env::EnvironmentLTL2D<float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<double>::Setting>
    : public erl::env::EnvironmentLTL2D<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<float, float>::Setting>
    : public erl::env::EnvironmentLTL2D<float, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<double /*Dtype*/, float /*MapDtype*/>::Setting>
    : public erl::env::EnvironmentLTL2D<double, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<float, double>::Setting>
    // WARNING: map cost will be cast from double to float.
    : public erl::env::EnvironmentLTL2D<float, double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::EnvironmentLTL2D<double, double>::Setting>
    : public erl::env::EnvironmentLTL2D<double, double>::Setting::YamlConvertImpl {};

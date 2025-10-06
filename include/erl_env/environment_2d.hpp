#pragma once

#include "cost.hpp"
#include "environment_base.hpp"

#include "erl_common/grid_map.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/random.hpp"
#include "erl_common/yaml.hpp"

namespace erl::env {

    /**
     *
     * @tparam Dtype data type of the metric space, float or double.
     * @tparam MapDtype data type of the cost map, e.g. uint8_t, float
     */
    template<typename Dtype, typename MapDtype = uint8_t>
    class Environment2D : public EnvironmentBase<Dtype, 2> {

    public:
        struct Setting : public common::Yamlable<Setting> {
            std::vector<Eigen::Vector2i> motions;  // 2d grid motions
            int grid_stride = 1;                   // grid stride in grid space
            MapDtype obstacle_threshold = 1;       // minimum map value to be considered as obstacle
            bool add_map_cost = false;    // indicate whether to add map cost to the successor cost
            Dtype map_cost_factor = 1.0;  // map cost = map_cost_factor * map_cost
            Eigen::Matrix2X<Dtype> robot_metric_contour;  // robot shape in metric space

            void
            SetGridMotionPrimitive(int max_axis_step, bool allow_diagonal) {
                motions.clear();
                ERL_ASSERTM(max_axis_step > 0, "max_axis_step must be positive.");
                int num_controls = max_axis_step * 2 + 1;
                if (allow_diagonal) {
                    num_controls = num_controls * num_controls - 1;
                    motions.reserve(num_controls);
                    for (int i = -max_axis_step; i <= max_axis_step; ++i) {
                        for (int j = -max_axis_step; j <= max_axis_step; ++j) {
                            if (i == 0 && j == 0) { continue; }
                            motions.emplace_back(i, j);
                        }
                    }
                } else {
                    num_controls = num_controls * 2 - 2;
                    motions.reserve(num_controls);
                    for (int i = -max_axis_step; i <= max_axis_step; ++i) {
                        if (i == 0) { continue; }
                        motions.emplace_back(i, 0);
                        motions.emplace_back(0, i);
                    }
                }
            }

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting) {
                    YAML::Node node;
                    ERL_YAML_SAVE_ATTR(node, setting, motions);
                    ERL_YAML_SAVE_ATTR(node, setting, grid_stride);
                    ERL_YAML_SAVE_ATTR(node, setting, obstacle_threshold);
                    ERL_YAML_SAVE_ATTR(node, setting, add_map_cost);
                    ERL_YAML_SAVE_ATTR(node, setting, map_cost_factor);
                    ERL_YAML_SAVE_ATTR(node, setting, robot_metric_contour);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, Setting &setting) {
                    if (!node.IsMap()) { return false; }
                    ERL_YAML_LOAD_ATTR(node, setting, motions);
                    ERL_YAML_LOAD_ATTR(node, setting, grid_stride);
                    ERL_YAML_LOAD_ATTR(node, setting, obstacle_threshold);
                    ERL_YAML_LOAD_ATTR(node, setting, add_map_cost);
                    ERL_YAML_LOAD_ATTR(node, setting, map_cost_factor);
                    ERL_YAML_LOAD_ATTR(node, setting, robot_metric_contour);
                    return true;
                }
            };
        };

        using Super = EnvironmentBase<Dtype, 2>;
        using Cost = CostBase<Dtype, 2>;
        using GridMap = common::GridMap<MapDtype, Dtype, 2>;
        using GridMapInfo = typename GridMap::Info;
        using State = EnvironmentState<Dtype, 2>;
        using MetricState = typename State::MetricState;
        using GridState = typename State::GridState;
        using Successor_t = Successor<Dtype, 2>;

    protected:
        std::shared_ptr<Setting> m_setting_;            // environment setting
        std::shared_ptr<GridMap> m_grid_map_ext_;       // grid map
        std::shared_ptr<GridMapInfo> m_grid_map_info_;  // grid map description
        cv::Mat m_original_grid_map_;  // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;           // inflated grid map
        std::vector<Eigen::Matrix2Xi> m_rel_trajectories_;      // relative trajectories
        std::vector<Dtype> m_motion_costs_;                     // cost of each motion
        Eigen::MatrixX<std::vector<int>> m_reachable_motions_;  // reachable controls for each grid

        // x to the bottom, y to the right, along y first

    public:
        /**
         *
         * @param grid_map grid map
         * @param setting environment setting
         * @param cost_func cost function to compute the cost between two states
         */
        Environment2D(
            const std::shared_ptr<GridMap> &grid_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<Cost> cost_func)
            : EnvironmentBase<Dtype, 2>(),
              m_setting_(NotNull(std::move(setting), true, "setting is nullptr.")),
              // keep grid_map alive
              m_grid_map_ext_(NotNull(grid_map, true, "grid_map is nullptr.")),
              m_grid_map_info_(grid_map->info),
              // external storage from grid_map
              m_original_grid_map_(
                  m_grid_map_info_->Shape(0),
                  m_grid_map_info_->Shape(1),
                  common::CvMatType<MapDtype, 1>(),
                  m_grid_map_ext_->data.Data().data()) {

            m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());
            if (m_setting_->robot_metric_contour.cols() > 0) {
                common::InflateWithShape(
                    m_original_grid_map_,
                    m_grid_map_info_,
                    m_setting_->robot_metric_contour,
                    m_grid_map_);
            } else {
                m_original_grid_map_.copyTo(m_grid_map_);
            }

            ERL_ASSERTM(cost_func != nullptr, "distance_cost_func is nullptr.");
            InitRelTrajectoriesAndCosts(*cost_func);
        }

        Environment2D(
            std::shared_ptr<GridMapInfo> grid_map_info,
            cv::Mat cost_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<Cost> cost_func)
            : EnvironmentBase<Dtype, 2>(),
              m_setting_(NotNull(std::move(setting), true, "setting is nullptr.")),
              m_grid_map_info_(
                  NotNull(std::move(grid_map_info), true, "grid_map_info is nullptr.")),
              m_original_grid_map_(std::move(cost_map)) {

            if (m_original_grid_map_.type() != common::CvMatType<MapDtype, 1>()) {
                ERL_WARN("cost_map type is not {}, converting.", type_name<MapDtype>());
                m_original_grid_map_.convertTo(
                    m_original_grid_map_,
                    common::CvMatType<MapDtype, 1>());
            }

            m_reachable_motions_.resize(m_grid_map_info_->Rows(), m_grid_map_info_->Cols());
            if (m_setting_->robot_metric_contour.cols() > 0) {
                common::InflateWithShape(
                    m_original_grid_map_,
                    m_grid_map_info_,
                    m_setting_->robot_metric_contour,
                    m_grid_map_);
            } else {
                m_original_grid_map_.copyTo(m_grid_map_);
            }

            ERL_ASSERTM(cost_func != nullptr, "distance_cost_func is nullptr.");
            InitRelTrajectoriesAndCosts(*cost_func);
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
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
        ForwardAction(const State &env_state, const long action_idx) const override {
            State new_state;
            new_state.grid = env_state.grid + m_setting_->motions[action_idx];
            new_state.metric = GridToMetric(new_state.grid);
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
                        const int nx_grid = cur_xg + rel_trajectory(0, i);
                        if (nx_grid < 0 || nx_grid >= m_grid_map_info_->Shape(0)) {
                            is_reachable = false;
                            break;
                        }
                        const int ny_grid = cur_yg + rel_trajectory(1, i);
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
            for (auto &motion_idx: reachable_motions) {
                ERL_DEBUG_ASSERT(
                    motion_idx >= 0 && motion_idx < num_motions,
                    "Invalid motion index: {}.",
                    motion_idx);
                State next_state;
                next_state.grid = env_state.grid + m_setting_->motions[motion_idx];
                next_state.metric = GridToMetric(next_state.grid);
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
            return m_grid_map_info_->GridToIndex(env_state.grid, true);  // row-major
        }

        [[nodiscard]] GridState
        MetricToGrid(const MetricState &metric_state) const override {
            return {
                m_grid_map_info_->MeterToGridAtDim(metric_state[0], 0),
                m_grid_map_info_->MeterToGridAtDim(metric_state[1], 1)};
        }

        [[nodiscard]] MetricState
        GridToMetric(const GridState &grid_state) const override {
            return {
                m_grid_map_info_->GridToMeterAtDim(grid_state[0], 0),
                m_grid_map_info_->GridToMeterAtDim(grid_state[1], 1)};
        }

        [[nodiscard]] std::vector<State>
        SampleValidStates(int num_samples) const override {
            ERL_ASSERTM(m_grid_map_.rows * m_grid_map_.cols > 0, "grid map is empty.");
            auto x_rng = std::uniform_int_distribution(0, m_grid_map_.rows - 1);
            auto y_rng = std::uniform_int_distribution(0, m_grid_map_.cols - 1);
            std::vector<State> states;
            states.reserve(num_samples);
            while (states.size() < static_cast<std::size_t>(num_samples)) {
                if (const int x = x_rng(common::g_random_engine),
                    y = y_rng(common::g_random_engine);
                    m_grid_map_.at<MapDtype>(x, y) < m_setting_->obstacle_threshold) {
                    State state;
                    state.grid = Eigen::Vector2i(x, y);
                    state.metric = GridToMetric(state.grid);
                    states.push_back(state);
                }
            }
            return states;
        }

    private:
        void
        InitRelTrajectoriesAndCosts(const Cost &cost_func) {
            // compute relative trajectories and costs of each control
            m_rel_trajectories_.reserve(m_setting_->motions.size());
            m_motion_costs_.reserve(m_setting_->motions.size());
            State ref_start;
            ref_start.metric = m_grid_map_info_->Center();
            ref_start.grid = m_grid_map_info_->CenterGrid();
            for (const Eigen::Vector2i &control: m_setting_->motions) {
                State end;
                end.grid = ref_start.grid + control;
                end.metric = m_grid_map_info_->GridToMeterForPoints(end.grid);
                Eigen::Matrix2Xi rel_trajectory =
                    m_grid_map_info_->RayCasting(ref_start.metric, end.metric);
                rel_trajectory.colwise() -= ref_start.grid;
                Dtype control_cost = cost_func(ref_start, end);
                m_rel_trajectories_.push_back(rel_trajectory);
                m_motion_costs_.push_back(control_cost);
            }
        }
    };

    extern template class Environment2D<float, uint8_t>;
    extern template class Environment2D<double, uint8_t>;
    extern template class Environment2D<float, float>;
    extern template class Environment2D<double, float>;
    extern template class Environment2D<float, double>;
    extern template class Environment2D<double, double>;

}  // namespace erl::env

template<>
struct YAML::convert<erl::env::Environment2D<float>::Setting>
    : public erl::env::Environment2D<float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::Environment2D<double>::Setting>
    : public erl::env::Environment2D<double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::Environment2D<float, float>::Setting>
    : public erl::env::Environment2D<float, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::Environment2D<double /*Dtype*/, float /*MapDtype*/>::Setting>
    : public erl::env::Environment2D<double, float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::Environment2D<float, double>::Setting>
    // WARNING: map cost will be cast from double to float.
    : public erl::env::Environment2D<float, double>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::env::Environment2D<double, double>::Setting>
    : public erl::env::Environment2D<double, double>::Setting::YamlConvertImpl {};

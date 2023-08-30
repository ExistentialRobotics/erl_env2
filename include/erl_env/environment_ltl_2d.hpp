#pragma once

// https://github.com/ExistentialRobotics/erl_astar/blob/master/test/old/test_astar_2d_ltl.cpp
// https://github.com/ExistentialRobotics/erl_env/blob/master/include/erl_env/ltl/fsa.h
// https://github.com/ExistentialRobotics/erl_astar/blob/master/data/maps/ltl/mapjie_2d.yaml

#include <numeric>
#include "erl_common/grid_map.hpp"
#include "erl_common/random.hpp"
#include "cost.hpp"
#include "finite_state_automaton.hpp"
#include "environment_base.hpp"
#include "motion_primitive.hpp"

namespace erl::env {

    /**
     * 2D Grid Environment with Linear Temporal Logic. The state is (x, y, ltl_state).
     */
    class EnvironmentLTL2D : public EnvironmentBase {
    public:
        typedef MotionPrimitive<Eigen::Vector2i> GridMotionPrimitive;  // control signal is 2D-grid movement

        struct Setting : public common::Yamlable<Setting> {
            bool allow_diagonal = true;                          // allow diagonal movement
            int step_size = 1;                                   // step size in grid space
            bool down_sampled = false;                           // indicate whether the state space is down sampled by step_size
            uint8_t obstacle_threshold = 1;                      // minimum map value to be considered as obstacle
            bool add_map_cost = false;                           // indicate whether to add map cost to the successor cost
            double map_cost_factor = 1.0;                        // map cost = map_cost_factor * map_cost
            Eigen::Matrix2Xd shape = {};                         // assume the shape center is at the origin
            std::shared_ptr<FiniteStateAutomaton::Setting> fsa;  // finite state automaton
        };

    private:
        std::shared_ptr<Setting> m_setting_;
        std::shared_ptr<FiniteStateAutomaton> m_fsa_;
        GridMotionPrimitive m_grid_motion_primitive_;             // motion primitives in grid space
        std::vector<double> m_motion_cost_;                       // cost of each motion primitive
        cv::Mat m_original_grid_map_;                             // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;                                      // inflated grid map
        std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_;  // grid map description, x to the bottom, y to the right, along y first
        Eigen::MatrixXi m_label_map_;                             // each element is a |AP|-bit word representing the result of atomic propositions

    public:
        EnvironmentLTL2D(
            Eigen::MatrixXi label_map,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
            std::shared_ptr<Setting> setting,
            std::shared_ptr<CostBase> distance_cost_func = nullptr)
            : EnvironmentBase(std::move(distance_cost_func)),
              m_setting_(std::move(setting)),
              // m_grid_map_info_((assert(grid_map != nullptr), grid_map->info->Extend())),
              m_label_map_(std::move(label_map)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
            ERL_ASSERTM(grid_map != nullptr, "grid_map is nullptr.");

            auto num_states = int(m_setting_->fsa->num_states);
            if (num_states % 2 == 0) { num_states += 1; }
            m_grid_map_info_ = std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(num_states, -0.5, double(num_states) - 0.5, 2));

            // generate motions and compute their costs
            if (m_setting_->allow_diagonal) {
                m_grid_motion_primitive_.controls = {
                    {1, 0},    // kForward
                    {-1, 0},   // kBack
                    {0, 1},    // kRight
                    {0, -1},   // kLeft
                    {1, 1},    // kForwardRight
                    {1, -1},   // kForwardLeft
                    {-1, 1},   // kBackRight
                    {-1, -1},  // kBackLeft
                };
            } else {
                m_grid_motion_primitive_.controls = {
                    {1, 0},   // kForward
                    {-1, 0},  // kBack
                    {0, 1},   // kRight
                    {0, -1},  // kLeft
                };
            }
            m_grid_motion_primitive_.durations.resize(m_grid_motion_primitive_.controls.size(), 1);
            m_grid_motion_primitive_.costs.reserve(m_grid_motion_primitive_.controls.size());
            EnvironmentState state0, state1;
            state0.grid = Eigen::VectorXi::Zero(2);
            state0.metric = grid_map->info->GridToMeterForPoints(state0.grid);
            for (auto &control: m_grid_motion_primitive_.controls) {
                state1.grid = state0.grid + control * m_setting_->step_size;
                state1.metric = grid_map->info->GridToMeterForPoints(state1.grid);
                m_grid_motion_primitive_.costs.push_back((*m_distance_cost_func_)(state0, state1));  // compute distance cost in metric space
            }

            // generate 2D obstacle/cost map
            m_original_grid_map_ = InitializeGridMap2D(grid_map);
            m_original_grid_map_.copyTo(m_grid_map_);
            if (m_setting_->shape.cols() > 0) { InflateGridMap2D(m_original_grid_map_, m_grid_map_, grid_map->info, m_setting_->shape); }

            // configure finite state automaton
            ERL_ASSERTM(m_setting_->fsa != nullptr, "setting->fsa is nullptr.");
            m_fsa_ = std::make_shared<FiniteStateAutomaton>(m_setting_->fsa);
            ERL_ASSERTM(m_grid_map_.rows == m_label_map_.rows(), "label_map and grid_map should have the same number of rows.");
            ERL_ASSERTM(m_grid_map_.cols == m_label_map_.cols(), "label_map and grid_map should have the same number of columns.");
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline std::shared_ptr<FiniteStateAutomaton>
        GetFiniteStateAutomaton() const {
            return m_fsa_;
        }

        [[nodiscard]] inline std::shared_ptr<common::GridMapInfo3D>
        GetGridMapInfo() const {
            return m_grid_map_info_;
        }

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size() * m_setting_->fsa->num_states;
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            return m_grid_motion_primitive_.controls.size();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
            ERL_ASSERTM(action_coords.size() == 1, "Invalid action_coords size: %lu.", action_coords.size());
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->grid.resize(3);
            int &nx_grid = new_state->grid[0];
            int &ny_grid = new_state->grid[1];
            int &nq = new_state->grid[2];
            auto &control = m_grid_motion_primitive_.controls[action_coords[0]];
            nx_grid = env_state->grid[0] + control[0];
            ny_grid = env_state->grid[1] + control[1];
            nq = int(m_fsa_->GetNextState(env_state->grid[2], m_label_map_(nx_grid, ny_grid)));

            new_state->metric.resize(3);
            new_state->metric[0] = m_grid_map_info_->GridToMeterForValue(nx_grid, 0);
            new_state->metric[1] = m_grid_map_info_->GridToMeterForValue(ny_grid, 1);
            new_state->metric[2] = new_state->grid[2];
            return {new_state};
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override {
            if (!InStateSpace(env_state)) { return {}; }

            std::vector<Successor> successors;
            auto num_controls = int(m_grid_motion_primitive_.controls.size());
            successors.reserve(num_controls);
            int &cur_q = env_state->grid[2];
            for (int control_idx = 0; control_idx < num_controls; ++control_idx) {
                auto &direction = m_grid_motion_primitive_.controls[control_idx];
                bool is_reachable = true;
                auto next_state = std::make_shared<EnvironmentState>();
                next_state->grid = env_state->grid;
                int &nx_grid = next_state->grid[0];
                int &ny_grid = next_state->grid[1];
                int &nq = next_state->grid[2];

                for (long i = 0; i < m_setting_->step_size; ++i) {
                    nx_grid += direction[0];
                    if (nx_grid < 0 || nx_grid > m_grid_map_info_->Shape(0)) {
                        is_reachable = false;
                        break;
                    }
                    ny_grid += direction[1];
                    if (ny_grid < 0 || ny_grid > m_grid_map_info_->Shape(1)) {
                        is_reachable = false;
                        break;
                    }
                    // check collision
                    if (m_grid_map_.at<uint8_t>(nx_grid, ny_grid) >= m_setting_->obstacle_threshold) {
                        is_reachable = false;
                        break;
                    }
                    // check LTL
                    nq = int(m_fsa_->GetNextState(cur_q, m_label_map_(nx_grid, ny_grid)));
                    if (m_fsa_->IsSinkState(nq)) {
                        is_reachable = false;
                        break;
                    }
                }
                if (!is_reachable) { continue; }
                next_state->metric.resize(3);
                next_state->metric[0] = m_grid_map_info_->GridToMeterForValue(nx_grid, 0);
                next_state->metric[1] = m_grid_map_info_->GridToMeterForValue(ny_grid, 1);
                next_state->metric[2] = nq;

                if (m_setting_->add_map_cost) {
                    double map_cost = m_setting_->map_cost_factor * double(m_grid_map_.at<uint8_t>(nx_grid, ny_grid));
                    double cost = m_grid_motion_primitive_.costs[control_idx] + map_cost;
                    successors.emplace_back(next_state, cost, std::vector<int>{});
                } else {
                    successors.emplace_back(next_state, m_grid_motion_primitive_.costs[control_idx], std::vector<int>{});
                }
                successors.back().action_coords.reserve(2);  // reserve one more for multi resolution search
                successors.back().action_coords.push_back(control_idx);
            }
            return successors;
        }

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid) &&
                   (!m_setting_->down_sampled || (env_state->grid[0] % m_setting_->step_size == 0 && env_state->grid[1] % m_setting_->step_size == 0));
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            uint32_t hashing = m_grid_map_info_->GridToIndex(env_state->grid, true);
            return hashing;
            // uint32_t hashing = m_grid_map_info_->GridToIndex(env_state->grid.head<2>(), true);
            // hashing = hashing * m_setting_->fsa->num_states + env_state->grid[2];
            // return hashing;
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector3i grid_state(
                m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
                m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
                int(metric_state[2]));
            return grid_state;
        }

        [[nodiscard]] Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector3d metric_state(
                m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
                m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
                double(grid_state[2]));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override {
            cv::Mat img = m_grid_map_ * 255;
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
            std::uniform_int_distribution random_int(0, 255);
            for (const auto &[kGoalIndex, kPath]: paths) {
                std::vector<cv::Point> points;
                auto num_points = kPath.cols();
                points.reserve(num_points);
                for (long i = 0; i < num_points; ++i) {
                    auto grid_point = MetricToGrid(kPath.col(i));
                    points.emplace_back(grid_point[1], grid_point[0]);
                }
                cv::Scalar color(random_int(common::g_random_engine), random_int(common::g_random_engine), random_int(common::g_random_engine));
                cv::polylines(img, points, false, color, 1);
            }
            cv::namedWindow("environment 2d: paths", cv::WINDOW_NORMAL);
            cv::imshow("environment 2d: paths", img);
            cv::waitKey(100);
            return img;
        }
    };
}  // namespace erl::env

namespace YAML {
    template<>
    struct convert<erl::env::EnvironmentLTL2D::Setting> {
        inline static Node
        encode(const erl::env::EnvironmentLTL2D::Setting &rhs) {
            Node node;
            node["allow_diagonal"] = rhs.allow_diagonal;
            node["step_size"] = rhs.step_size;
            node["down_sampled"] = rhs.down_sampled;
            node["obstacle_threshold"] = rhs.obstacle_threshold;
            node["add_map_cost"] = rhs.add_map_cost;
            node["map_cost_factor"] = rhs.map_cost_factor;
            node["shape"] = rhs.shape;
            node["fsa"] = rhs.fsa;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::env::EnvironmentLTL2D::Setting &rhs) {
            rhs.allow_diagonal = node["allow_diagonal"].as<bool>();
            rhs.step_size = node["step_size"].as<int>();
            rhs.down_sampled = node["down_sampled"].as<bool>();
            rhs.obstacle_threshold = node["obstacle_threshold"].as<uint8_t>();
            rhs.add_map_cost = node["add_map_cost"].as<bool>();
            rhs.map_cost_factor = node["map_cost_factor"].as<double>();
            rhs.shape = node["shape"].as<Eigen::Matrix2Xd>();
            rhs.fsa = node["fsa"].as<std::shared_ptr<erl::env::FiniteStateAutomaton::Setting>>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::env::EnvironmentLTL2D::Setting &rhs) {
        out << BeginMap;
        out << Key << "allow_diagonal" << Value << rhs.allow_diagonal;
        out << Key << "step_size" << Value << rhs.step_size;
        out << Key << "down_sampled" << Value << rhs.down_sampled;
        out << Key << "obstacle_threshold" << Value << rhs.obstacle_threshold;
        out << Key << "add_map_cost" << Value << rhs.add_map_cost;
        out << Key << "map_cost_factor" << Value << rhs.map_cost_factor;
        out << Key << "shape" << Value << rhs.shape;
        out << Key << "fsa" << Value << rhs.fsa;
        out << EndMap;
        return out;
    }
}  // namespace YAML

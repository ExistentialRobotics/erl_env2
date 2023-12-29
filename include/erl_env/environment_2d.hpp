#pragma once

#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"
#include "environment_base.hpp"
#include "motion_primitive.hpp"
#include "cost.hpp"

namespace erl::env {

    class Environment2D : public EnvironmentBase {

    public:
        enum class Action {
            /*          Left
             *            |
             *  Back -----|------x  Forward
             *            |
             *            y
             *          Right
             */
            kBack = 0,
            kBackRight = 1,
            kRight = 2,
            kForwardRight = 3,
            kForward = 4,
            kForwardLeft = 5,
            kLeft = 6,
            kBackLeft = 7
        };

        static inline const char *
        GetActionName(const Action &action) {
            static const char *names[] = {
                ERL_AS_STRING(kForward),
                ERL_AS_STRING(kBack),
                ERL_AS_STRING(kRight),
                ERL_AS_STRING(kLeft),
                ERL_AS_STRING(kForwardRight),
                ERL_AS_STRING(kForwardLeft),
                ERL_AS_STRING(kBackRight),
                ERL_AS_STRING(kBackLeft)};

            return names[int(action)];
        }

        static inline Action
        GetActionFromName(const std::string &action_name) {
            if (action_name == ERL_AS_STRING(kForward)) { return Action::kForward; }
            if (action_name == ERL_AS_STRING(kBack)) { return Action::kBack; }
            if (action_name == ERL_AS_STRING(kRight)) { return Action::kRight; }
            if (action_name == ERL_AS_STRING(kLeft)) { return Action::kLeft; }
            if (action_name == ERL_AS_STRING(kForwardRight)) { return Action::kForwardRight; }
            if (action_name == ERL_AS_STRING(kForwardLeft)) { return Action::kForwardLeft; }
            if (action_name == ERL_AS_STRING(kBackRight)) { return Action::kBackRight; }
            if (action_name == ERL_AS_STRING(kBackLeft)) { return Action::kBackLeft; }

            throw std::runtime_error("Unknown Action: " + action_name);
        }

        typedef MotionPrimitive<Eigen::Vector2i> GridMotionPrimitive;  // control signal is 2D-grid movement

        struct Setting : common::Yamlable<Setting> {
            bool allow_diagonal = true;      // allow diagonal movement
            int step_size = 1;               // step size in grid space
            bool down_sampled = false;       // indicate whether the state space is down sampled by step_size
            uint8_t obstacle_threshold = 1;  // minimum map value to be considered as obstacle
            bool add_map_cost = false;       // indicate whether to add map cost to the successor cost
            double map_cost_factor = 1.0;    // map cost = map_cost_factor * map_cost
            Eigen::Matrix2Xd shape = {};     // assume the shape center is at the origin
        };

    protected:
        std::shared_ptr<Setting> m_setting_;                      // environment setting
        GridMotionPrimitive m_grid_motion_primitive_;             // motion primitives in grid space
        std::vector<double> m_motion_cost_;                       // cost of each motion primitive
        cv::Mat m_original_grid_map_;                             // original grid map, where each cell is a scaled cost value
        cv::Mat m_grid_map_;                                      // inflated grid map
        std::shared_ptr<common::GridMapInfo2D> m_grid_map_info_;  // grid map description, x to the bottom, y to the right, along y first

    public:
        explicit Environment2D(
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,  // x to the bottom, y to the right, along y first
            std::shared_ptr<Setting> setting = nullptr,
            std::shared_ptr<CostBase> distance_cost_func = nullptr);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            return m_grid_motion_primitive_.controls.size();
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override {
            ERL_ASSERTM(action_coords.size() == 1, "Invalid action_coords size: %lu.", action_coords.size());
            auto new_state = std::make_shared<EnvironmentState>();
            new_state->grid = env_state->grid + m_setting_->step_size * m_grid_motion_primitive_.controls[action_coords[0]];
            new_state->metric = GridToMetric(new_state->grid);
            return {new_state};
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override;

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid) &&
                   (!m_setting_->down_sampled || (env_state->grid[0] % m_setting_->step_size == 0 && env_state->grid[1] % m_setting_->step_size == 0));
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            return m_grid_map_info_->GridToIndex(env_state->grid, true);  // row-major
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector2i grid_state(m_grid_map_info_->MeterToGridForValue(metric_state[0], 0), m_grid_map_info_->MeterToGridForValue(metric_state[1], 1));
            return grid_state;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector2d metric_state(m_grid_map_info_->GridToMeterForValue(grid_state[0], 0), m_grid_map_info_->GridToMeterForValue(grid_state[1], 1));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override;
    };

}  // namespace erl::env

namespace YAML {

    template<>
    struct convert<erl::env::Environment2D::Setting> {
        inline static Node
        encode(const erl::env::Environment2D::Setting &rhs) {
            Node node;
            node["allow_diagonal"] = rhs.allow_diagonal;
            node["step_size"] = rhs.step_size;
            node["down_sampled"] = rhs.down_sampled;
            node["obstacle_threshold"] = rhs.obstacle_threshold;
            node["add_map_cost"] = rhs.add_map_cost;
            node["map_cost_factor"] = rhs.map_cost_factor;
            node["shape"] = rhs.shape;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::env::Environment2D::Setting &rhs) {
            rhs.allow_diagonal = node["allow_diagonal"].as<bool>();
            rhs.step_size = node["step_size"].as<int>();
            rhs.down_sampled = node["down_sampled"].as<bool>();
            rhs.obstacle_threshold = node["obstacle_threshold"].as<uint8_t>();
            rhs.add_map_cost = node["add_map_cost"].as<bool>();
            rhs.map_cost_factor = node["map_cost_factor"].as<double>();
            rhs.shape = node["shape"].as<Eigen::Matrix2Xd>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::env::Environment2D::Setting &rhs) {
        out << BeginMap;
        out << Key << "allow_diagonal" << Value << rhs.allow_diagonal;
        out << Key << "step_size" << Value << rhs.step_size;
        out << Key << "down_sampled" << Value << rhs.down_sampled;
        out << Key << "obstacle_threshold" << Value << rhs.obstacle_threshold;
        out << Key << "add_map_cost" << Value << rhs.add_map_cost;
        out << Key << "map_cost_factor" << Value << rhs.map_cost_factor;
        out << Key << "shape" << Value << rhs.shape;
        out << EndMap;
        return out;
    }
}  // namespace YAML

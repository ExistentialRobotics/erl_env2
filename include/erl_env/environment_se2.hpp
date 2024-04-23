#pragma once

#include <map>

#include "ddc_motion_primitive.hpp"
#include "environment_base.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"

namespace erl::env {

    class EnvironmentSe2 : public EnvironmentBase {

    public:
        struct Setting : public common::Yamlable<Setting> {
            double time_step = 0.05;
            std::vector<DdcMotionPrimitive> motion_primitives;
            int num_orientations = 16;
            double cost_theta_weight = 0.0;
            uint8_t obstacle_threshold = 1;
            bool add_map_cost = false;
            double map_cost_factor = 1.0;
            Eigen::Matrix2Xd shape = {};
        };

    protected:
        std::shared_ptr<Setting> m_setting_;
        cv::Mat m_original_grid_map_;
        std::vector<cv::Mat> m_inflated_grid_maps_;
        std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_;
        // [theta][motion_primitive][control][trajectory_waypoint_index]
        std::vector<std::vector<std::vector<std::vector<std::shared_ptr<EnvironmentState>>>>> m_rel_trajectories_;

        /*
         * for each orientation
         *     for each unique relative successor (distinguished by grid state hashing)
         *         controls that lead to the successor, sorted by cost (ascending)
         */
        struct RelSuccessorInfo {
            std::shared_ptr<EnvironmentState> rel_state = nullptr;
            std::vector<std::size_t> orders;
            std::vector<int> motion_indices;
            std::vector<int> control_indices;
            std::vector<std::vector<int>> action_coords;
            std::vector<double> costs;
        };

        std::vector<std::vector<RelSuccessorInfo>> m_rel_successors_;

    public:
        /**
         * @brief Construct a new Environment Se2 object that assumes the robot is a point.
         * @param collision_check_dt
         * @param motion_primitives
         * @param grid_map
         * @param num_orientations
         * @param add_map_cost
         * @param map_cost_factor
         */
        explicit EnvironmentSe2(const std::shared_ptr<common::GridMapUnsigned2D> &grid_map, std::shared_ptr<Setting> setting = nullptr);

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
            std::size_t size = 0;
            for (auto &motion_primitive: m_setting_->motion_primitives) { size += motion_primitive.controls.size(); }
            return size;
        }

        inline static Eigen::VectorXd
        MotionModel(const Eigen::Ref<const Eigen::VectorXd> &metric_state, const DifferentialDriveControl &control, double t) {
            Eigen::VectorXd next_state(3);

            DifferentialDriveKinematic(
                metric_state[0],
                metric_state[1],
                metric_state[2],
                control.linear_v,
                control.angular_v,
                t,
                next_state[0],
                next_state[1],
                next_state[2]);
            return next_state;
        }

        [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &env_state, const std::vector<int> &action_coords) const override;

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &env_state) const override;

        [[nodiscard]] inline bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &env_state) const override {
            return m_grid_map_info_->InGrids(env_state->grid);
        }

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const override {
            return GridStateHashingImpl(env_state->grid);
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            ERL_DEBUG_ASSERT(m_grid_map_info_, "Not supported when not initialized with grid_map_info.");
            Eigen::Vector3i grid_state(
                m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
                m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
                m_grid_map_info_->MeterToGridForValue(metric_state[2], 2));
            return grid_state;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            ERL_DEBUG_ASSERT(m_grid_map_info_, "Not supported when not initialized with grid_map_info.");
            Eigen::Vector3d metric_state(
                m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
                m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
                m_grid_map_info_->GridToMeterForValue(grid_state[2], 2));
            return metric_state;
        }

        [[nodiscard]] cv::Mat
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths, bool block) const override;

    private:
        [[nodiscard]] inline std::size_t
        GridStateHashingImpl(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const {  // this can be called in the constructor
            return m_grid_map_info_->GridToIndex(grid_state, true);
        }
    };

}  // namespace erl::env

namespace YAML {
    template<>
    struct convert<erl::env::EnvironmentSe2::Setting> {
        inline static Node
        encode(const erl::env::EnvironmentSe2::Setting &rhs) {
            Node node;
            node["time_step"] = rhs.time_step;
            node["motion_primitives"] = rhs.motion_primitives;
            node["num_orientations"] = rhs.num_orientations;
            node["obstacle_threshold"] = rhs.obstacle_threshold;
            node["add_map_cost"] = rhs.add_map_cost;
            node["map_cost_factor"] = rhs.map_cost_factor;
            node["shape"] = rhs.shape;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::env::EnvironmentSe2::Setting &rhs) {
            rhs.time_step = node["time_step"].as<double>();
            rhs.motion_primitives = node["motion_primitives"].as<std::vector<erl::env::DdcMotionPrimitive>>();
            rhs.num_orientations = node["num_orientations"].as<int>();
            rhs.obstacle_threshold = node["obstacle_threshold"].as<uint8_t>();
            rhs.add_map_cost = node["add_map_cost"].as<bool>();
            rhs.map_cost_factor = node["map_cost_factor"].as<double>();
            rhs.shape = node["shape"].as<Eigen::Matrix2Xd>();
            return true;
        }
    };
}  // namespace YAML

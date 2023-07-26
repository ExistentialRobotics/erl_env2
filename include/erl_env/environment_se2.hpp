#pragma once

#include <map>

#include "ddc_motion_primitive.hpp"
#include "environment_base.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/grid_map_info.hpp"

namespace erl::env {

    class EnvironmentSe2 : virtual public EnvironmentBase {

    protected:
        cv::Mat m_original_grid_map_;
        std::vector<cv::Mat> m_inflated_grid_maps_;
        Eigen::Matrix2Xd m_shape_metric_vertices_;
        std::vector<DdcMotionPrimitive> m_motion_primitives_;
        std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_;
        std::size_t m_max_num_controls_ = 0;
        uint8_t m_obstacle_threshold_ = 0;
        bool m_add_map_cost_ = false;
        double m_map_cost_factor_ = 1.0;
        // std::size_t m_max_num_successors_ = 0;
        bool m_already_reset_ = true;

        // clang-format off
        /*
         * for each motion primitive:
         *      for each control: matrix of relative metric trajectory starting from [0, 0, 0], where each column is a state vector.
         */
        std::vector<std::vector<Eigen::MatrixXd>> m_metric_rel_trajectories_;

        /*
         * for each orientation theta
         *      for each motion primitive
         *          for each control: matrix of relative grid trajectory starting from [x_center, y_center, theta], where each column is a state vector.
         */
        std::vector<std::vector<std::vector<std::vector<std::shared_ptr<EnvironmentState>>>>> m_grid_rel_trajectories_;
        // ^^^^^^^^    ^^^^^^^^       ^^^^^      ^^^^^^
        //  theta   motion primitive  control    trajectory
        // clang-format on

        /*
         * for each orientation
         *     for each unique relative successor (distinguished by grid state hashing)
         *         controls that lead to the successor, sorted by cost (ascending)
         */
        struct GridRelSuccessorInfo {
            std::shared_ptr<EnvironmentState> rel_state = nullptr;
            std::vector<std::size_t> orders;
            std::vector<std::size_t> motion_indices;
            std::vector<std::size_t> control_indices;
            std::vector<std::size_t> action_indices;
            std::vector<double> costs;
        };

        std::vector<std::vector<GridRelSuccessorInfo>> m_grid_rel_successors_;
        // std::vector<std::map<std::size_t, GridRelSuccessorInfo>> m_grid_rel_successors_;
        //                   ^^^^^^^^^^^
        //            hashing of ref grid state

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
        EnvironmentSe2(
            double collision_check_dt,
            std::vector<DdcMotionPrimitive> motion_primitives,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
            uint8_t obstacle_threshold,
            int num_orientations,
            bool add_map_cost = false,
            double map_cost_factor = 1.0);

        /**
         * @brief Construct a new Environment Se2 object that assumes the robot is a polygon.
         * @param collision_check_dt
         * @param motion_primitives
         * @param grid_map
         * @param num_orientations
         * @param inflate_scale
         * @param shape_metric_vertices
         * @param add_map_cost
         * @param map_cost_factor
         */
        EnvironmentSe2(
            double collision_check_dt,
            std::vector<DdcMotionPrimitive> motion_primitives,
            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
            uint8_t obstacle_threshold,
            int num_orientations,
            double inflate_scale,
            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices,
            bool add_map_cost = false,
            double map_cost_factor = 1.0);

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            return m_grid_map_info_->Size();
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            return m_motion_primitives_.size() * m_max_num_controls_;
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
        ForwardAction(const std::shared_ptr<const EnvironmentState> &state, std::size_t action_index) const override;

        // [[nodiscard]] inline Eigen::MatrixXd
        // ForwardActionInMetricSpace(const Eigen::Ref<const Eigen::VectorXd> &current_metric_state, std::size_t action_index, double dt) const override {
        //     auto action_coords = ActionIndexToActionCoords(action_index);
        //     auto motion_idx = action_coords[0];
        //     auto control_idx = action_coords[1];
        //
        //     auto metric_trajectory = m_motion_primitives_[motion_idx].ComputeTrajectorySegment(current_metric_state, control_idx, dt, MotionModel);
        //     return metric_trajectory;
        // }
        //
        // [[nodiscard]] inline Eigen::MatrixXi
        // ForwardActionInGridSpace(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state, std::size_t action_index) const override {
        //     auto action_coords = ActionIndexToActionCoords(action_index);
        //     auto &motion_idx = action_coords[0];
        //     auto &control_idx = action_coords[1];
        //
        //     auto &grid_rel_trajectory = m_grid_rel_trajectories_[current_grid_state[2]][motion_idx][control_idx];
        //     long num_trajectory_states = grid_rel_trajectory.cols();
        //     Eigen::Matrix3Xi grid_trajectory(3, num_trajectory_states);
        //     for (int i = 0; i < num_trajectory_states; i++) {
        //         grid_trajectory(0, i) = grid_rel_trajectory(0, i) + current_grid_state[0];
        //         grid_trajectory(1, i) = grid_rel_trajectory(1, i) + current_grid_state[1];
        //         grid_trajectory(2, i) = grid_rel_trajectory(2, i) + current_grid_state[2];
        //     }
        //
        //     return grid_trajectory;
        // }

        // [[nodiscard]] std::vector<Successor>
        // GetSuccessorsInMetricSpace(const Eigen::Ref<const Eigen::VectorXd> &current_metric_state) const override;

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &state) const override;

        [[nodiscard]] bool
        IsReachable(const std::vector<std::shared_ptr<EnvironmentState>> &trajectory) const override {
            return std::all_of(trajectory.begin(), trajectory.end(), [this](const auto &state) {
                auto &grid_state = state->grid;
                return m_grid_map_info_->InGrids(grid_state) &&
                       (m_inflated_grid_maps_[grid_state[2]].template at<uint8_t>(grid_state[0], grid_state[1]) < m_obstacle_threshold_);
            });
        }

        [[nodiscard]] inline std::size_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const override {
            return GridStateHashingImpl(state->grid);
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

        [[nodiscard]] inline std::size_t
        ActionCoordsToActionIndex(const std::vector<std::size_t> &action_coords) const override {
            return action_coords[0] * m_max_num_controls_ + action_coords[1];  // action_coords = [motion_id, control_id]
        }

        [[nodiscard]] inline std::vector<std::size_t>
        ActionIndexToActionCoords(std::size_t action_idx) const override {
            auto motion_id = action_idx / m_max_num_controls_;
            auto control_id = action_idx % m_max_num_controls_;
            return {motion_id, control_id};
        }

        void
        PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override;

        void
        Reset() override;

        void
        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override;

    private:
        [[nodiscard]] inline std::size_t
        GridStateHashingImpl(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const {  // this can be called in the constructor
            return m_grid_map_info_->GridToIndex(grid_state, true);
        }
    };

}  // namespace erl::env

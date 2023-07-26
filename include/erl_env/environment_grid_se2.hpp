//#pragma once
//
//#include <opencv2/imgproc.hpp>
//#include <memory>
//#include "environment_base.hpp"
//#include "erl_common/grid_map.hpp"
//#include "erl_common/grid_map_info.hpp"
//#include "erl_common/angle_utils.hpp"
//#include "motion_primitive.hpp"
//
//namespace erl::env {
//
//    class EnvironmentGridSe2 : virtual public EnvironmentBase {
//
//    public:
//        enum class GridSe2Action {  // 3 x 3 x 3 - 1 = 26 actions
//            kForward = 0,           // (x + s, y, theta), allowed only when theta is 0
//            kBack = 1,              // (x - s, y, theta), allowed only when theta is 0
//            // kRight = 2,                   // (x, y + s, theta), forbid drifting
//            // kLeft = 3,                    // (x, y - s, theta), forbid drifting
//            kTurnLeft = 4,                // (x, y, theta + s)
//            kTurnRight = 5,               // (x, y, theta - s)
//            // kForwardRight = 6,            // (x + s, y + s, theta)
//            // kForwardLeft = 7,             // (x + s, y - s, theta)
//            // kBackRight = 8,               // (x - s, y + s, theta)
//            // kBackLeft = 9,                // (x - s, y - s, theta)
//            // kForwardTurnLeft = 10,        // (x + s, y, theta + s)
//            // kForwardTurnRight = 11,       // (x + s, y, theta - s)
//            // kBackTurnLeft = 12,           // (x - s, y, theta + s)
//            // kBackTurnRight = 13,          // (x - s, y, theta - s)
//            // kRightTurnLeft = 14,          // (x, y + s, theta + s)
//            // kRightTurnRight = 15,         // (x, y + s, theta - s)
//            // kLeftTurnLeft = 16,           // (x, y - s, theta + s)
//            // kLeftTurnRight = 17,          // (x, y - s, theta - s)
//            // kForwardRightTurnLeft = 18,   // (x + s, y + s, theta + s)
//            // kForwardRightTurnRight = 19,  // (x + s, y + s, theta - s)
//            // kForwardLeftTurnLeft = 20,    // (x + s, y - s, theta + s)
//            // kForwardLeftTurnRight = 21,   // (x + s, y - s, theta - s)
//            // kBackRightTurnLeft = 22,      // (x - s, y + s, theta + s)
//            // kBackRightTurnRight = 23,     // (x - s, y + s, theta - s)
//            // kBackLeftTurnLeft = 24,       // (x - s, y - s, theta + s)
//            // kBackLeftTurnRight = 25       // (x - s, y - s, theta - s)
//        };
//
//        static inline const char *
//        GetGridSe2ActionName(const GridSe2Action &action) {
//            static const char *names[] = {
//                ERL_AS_STRING(kForward),
//                ERL_AS_STRING(kBack),
//                ERL_AS_STRING(kRight),
//                ERL_AS_STRING(kLeft),
//                ERL_AS_STRING(kTurnLeft),
//                ERL_AS_STRING(kTurnRight),
//                ERL_AS_STRING(kForwardRight),
//                ERL_AS_STRING(kForwardLeft),
//                ERL_AS_STRING(kBackRight),
//                ERL_AS_STRING(kBackLeft),
//                ERL_AS_STRING(kForwardTurnLeft),
//                ERL_AS_STRING(kForwardTurnRight),
//                ERL_AS_STRING(kBackTurnLeft),
//                ERL_AS_STRING(kBackTurnRight),
//                ERL_AS_STRING(kRightTurnLeft),
//                ERL_AS_STRING(kRightTurnRight),
//                ERL_AS_STRING(kLeftTurnLeft),
//                ERL_AS_STRING(kLeftTurnRight),
//                ERL_AS_STRING(kForwardRightTurnLeft),
//                ERL_AS_STRING(kForwardRightTurnRight),
//                ERL_AS_STRING(kForwardLeftTurnLeft),
//                ERL_AS_STRING(kForwardLeftTurnRight),
//                ERL_AS_STRING(kBackRightTurnLeft),
//                ERL_AS_STRING(kBackRightTurnRight),
//                ERL_AS_STRING(kBackLeftTurnLeft),
//                ERL_AS_STRING(kBackLeftTurnRight)};
//            return names[static_cast<int>(action)];
//        }
//
//        static inline GridSe2Action
//        GetGridSe2ActionFromName(const std::string &action_name) {
//            if (action_name == ERL_AS_STRING(kForward)) {
//                return GridSe2Action::kForward;
//            } else if (action_name == ERL_AS_STRING(kBack)) {
//                return GridSe2Action::kBack;
//                // } else if (action_name == ERL_AS_STRING(kRight)) {
//                //     return GridSe2Action::kRight;
//                // } else if (action_name == ERL_AS_STRING(kLeft)) {
//                //     return GridSe2Action::kLeft;
//            } else if (action_name == ERL_AS_STRING(kTurnLeft)) {
//                return GridSe2Action::kTurnLeft;
//            } else if (action_name == ERL_AS_STRING(kTurnRight)) {
//                return GridSe2Action::kTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kForwardRight)) {
//                //     return GridSe2Action::kForwardRight;
//                // } else if (action_name == ERL_AS_STRING(kForwardLeft)) {
//                //     return GridSe2Action::kForwardLeft;
//                // } else if (action_name == ERL_AS_STRING(kBackRight)) {
//                //     return GridSe2Action::kBackRight;
//                // } else if (action_name == ERL_AS_STRING(kBackLeft)) {
//                //     return GridSe2Action::kBackLeft;
//                // } else if (action_name == ERL_AS_STRING(kForwardTurnLeft)) {
//                //     return GridSe2Action::kForwardTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kForwardTurnRight)) {
//                //     return GridSe2Action::kForwardTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kBackTurnLeft)) {
//                //     return GridSe2Action::kBackTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kBackTurnRight)) {
//                //     return GridSe2Action::kBackTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kRightTurnLeft)) {
//                //     return GridSe2Action::kRightTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kRightTurnRight)) {
//                //     return GridSe2Action::kRightTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kLeftTurnLeft)) {
//                //     return GridSe2Action::kLeftTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kLeftTurnRight)) {
//                //     return GridSe2Action::kLeftTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kForwardRightTurnLeft)) {
//                //     return GridSe2Action::kForwardRightTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kForwardRightTurnRight)) {
//                //     return GridSe2Action::kForwardRightTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kForwardLeftTurnLeft)) {
//                //     return GridSe2Action::kForwardLeftTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kForwardLeftTurnRight)) {
//                //     return GridSe2Action::kForwardLeftTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kBackRightTurnLeft)) {
//                //     return GridSe2Action::kBackRightTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kBackRightTurnRight)) {
//                //     return GridSe2Action::kBackRightTurnRight;
//                // } else if (action_name == ERL_AS_STRING(kBackLeftTurnLeft)) {
//                //     return GridSe2Action::kBackLeftTurnLeft;
//                // } else if (action_name == ERL_AS_STRING(kBackLeftTurnRight)) {
//                //     return GridSe2Action::kBackLeftTurnRight;
//            } else {
//                throw std::runtime_error("Invalid action name: " + action_name);
//            }
//        }
//
//        typedef MotionPrimitive<Eigen::Vector3i> GridMotionPrimitive;
//
//    protected:
//        GridMotionPrimitive m_grid_motion_primitive_;
//        cv::Mat m_original_grid_map_;
//        std::vector<cv::Mat> m_inflated_grid_maps_;
//        std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_;
//        Eigen::Matrix2Xd m_shape_metric_vertices_;
//        bool m_already_reset_ = true;
//
//    public:
//        EnvironmentGridSe2(int step_size, const std::shared_ptr<common::GridMapUnsigned2D> &grid_map, int num_orientations);
//
//        EnvironmentGridSe2(
//            int step_size,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            int num_orientations,
//            double inflate_scale,
//            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices);
//
//        [[nodiscard]] inline Eigen::MatrixXd
//        ForwardActionInMetricSpace(const Eigen::Ref<const Eigen::VectorXd> &current_metric_state, std::size_t action_index, double) const override {
//            auto &control = m_grid_motion_primitive_.controls[action_index];
//            double l = control[0] * m_grid_map_info_->Resolution(0);
//            double theta = current_metric_state[2];
//            double x = current_metric_state[0] + l * std::cos(theta);
//            double y = current_metric_state[1] + l * std::sin(theta);
//            theta += control[2] * m_grid_map_info_->Resolution(2);
//            theta = common::ClipAngle(theta);
//            return Eigen::Vector3d(x, y, theta);
//            // return GridToMetric(MetricToGrid(current_metric_state) + m_grid_motion_primitive_.controls[action_index]);
//        }
//
//        [[nodiscard]] inline Eigen::MatrixXi
//        ForwardActionInGridSpace(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state, std::size_t action_index) const override {
//            return MetricToGrid(ForwardActionInMetricSpace(GridToMetric(current_grid_state), action_index, 0));
//        }
//
//        [[nodiscard]] std::vector<Successor>
//        GetSuccessors(const std::shared_ptr<const EnvironmentState> &state) const override;
//
//        [[nodiscard]] inline bool
//        IsMetricStateCollided(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
//            return IsGridStateCollided(MetricToGrid(metric_state));
//        }
//
//        [[nodiscard]] inline bool
//        IsGridStateCollided(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
//            return m_inflated_grid_maps_[grid_state[2]].at<uint8_t>(grid_state[0], grid_state[1]) > 0;  // at(row, col)
//        }
//
//        [[nodiscard]] inline std::size_t
//        GridStateHashing(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
//            return m_grid_map_info_->GridToIndex(grid_state, true);  // row-major
//        }
//
//        [[nodiscard]] inline Eigen::VectorXi
//        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
//            Eigen::Vector3i grid_state(
//                m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
//                m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
//                m_grid_map_info_->MeterToGridForValue(metric_state[2], 2));
//            return grid_state;
//        }
//
//        [[nodiscard]] inline Eigen::VectorXd
//        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
//            Eigen::Vector3d metric_state(
//                m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
//                m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
//                m_grid_map_info_->GridToMeterForValue(grid_state[2], 2));
//            return metric_state;
//        }
//
//        [[nodiscard]] inline std::size_t
//        ActionCoordsToActionIndex(const std::vector<std::size_t> &action_coords) const override {
//            return action_coords[0];
//        }
//
//        [[nodiscard]] inline std::vector<std::size_t>
//        ActionIndexToActionCoords(std::size_t action_idx) const override {
//            return {action_idx};
//        }
//
//        void PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override;
//
//        void Reset() override;
//
//        void
//        ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override;
//    };
//
//}  // namespace erl::env
//
//// #pragma once
////
//// #include <opencv2/imgproc.hpp>
//// #include <memory>
//// #include "environment_base.hpp"
//// #include "erl_common/grid_map.hpp"
//// #include "erl_common/grid_map_info.hpp"
//// #include "motion_primitive.hpp"
//// #include "differential_drive_model.hpp"
////
//// namespace erl::env {
////
////     class EnvironmentGridSe2 : virtual public EnvironmentBase {
////
////     public:
////         typedef MotionPrimitive<Eigen::Vector3i> GridMotionPrimitive;
////
////     protected:
////         std::vector<std::vector<GridMotionPrimitive>> m_grid_motion_primitives_ = {};
////         int m_max_num_motion_primitives_ = 0;
////         cv::Mat m_original_grid_map_ = {};
////         std::vector<cv::Mat> m_inflated_grid_maps_ = {};
////         std::shared_ptr<common::GridMapInfo3D> m_grid_map_info_ = nullptr;
////         Eigen::Matrix2Xd m_shape_metric_vertices_ = {};
////
////     public:
////         static std::vector<std::vector<GridMotionPrimitive>>
////         GenerateGridSe2MotionPrimitives(
////             double linear_velocity_min,
////             double linear_velocity_max,
////             double linear_velocity_step,
////             double euclidean_square_distance_cost_weight,
////             double angular_velocity_min,
////             double angular_velocity_max,
////             double angular_velocity_step,
////             double angular_square_distance_cost_weight,
////             double duration_step,
////             double duration,
////             int max_grid_step,
////             const std::shared_ptr<common::GridMapInfo3D> &grid_map_info);
////
////         EnvironmentGridSe2(
////             double linear_velocity_min,
////             double linear_velocity_max,
////             double linear_velocity_step,
////             double euclidean_square_distance_cost_weight,
////             double angular_velocity_min,
////             double angular_velocity_max,
////             double angular_velocity_step,
////             double angular_square_distance_cost_weight,
////             double duration_step,
////             double duration,
////             int max_step_size,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations);
////
////         EnvironmentGridSe2(
////             double linear_velocity_min,
////             double linear_velocity_max,
////             double linear_velocity_step,
////             double euclidean_square_distance_cost_weight,
////             double angular_velocity_min,
////             double angular_velocity_max,
////             double angular_velocity_step,
////             double angular_square_distance_cost_weight,
////             double duration_step,
////             double duration,
////             int max_step_size,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations,
////             double inflate_scale,
////             const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices);
////
////         EnvironmentGridSe2(
////             std::vector<std::vector<GridMotionPrimitive>> grid_motion_primitives,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations);
////
////         EnvironmentGridSe2(
////             std::vector<std::vector<GridMotionPrimitive>> grid_motion_primitives,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations,
////             double inflate_scale,
////             const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices);
////
////         [[nodiscard]] inline Eigen::MatrixXd
////         ForwardActionInMetricSpace(const Eigen::Ref<const Eigen::VectorXd> &current_metric_state, std::size_t action_index, double) const override {
////             int theta_g = m_grid_map_info_->MeterToGridForValue(current_metric_state[2], 2);
////             int action_theta_g = int(action_index) / m_max_num_motion_primitives_;
////             ERL_ASSERTM(theta_g == action_theta_g, "The action index is not consistent with the current orientation.\n");
////             int motion_index = int(action_index) - theta_g * m_max_num_motion_primitives_;
////             auto &control = m_grid_motion_primitives_[theta_g][motion_index].controls[0];
////
////             Eigen::Vector3i next_grid_state(
////                 m_grid_map_info_->MeterToGridForValue(current_metric_state[0], 0) + control[0],
////                 m_grid_map_info_->MeterToGridForValue(current_metric_state[1], 1) + control[1],
////                 theta_g + control[2]);
////
////             return GridToMetric(next_grid_state);
////         }
////
////         [[nodiscard]] inline Eigen::MatrixXi
////         ForwardActionInGridSpace(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state, std::size_t action_index) const override {
////             int action_theta_g = int(action_index) / m_max_num_motion_primitives_;
////             ERL_ASSERTM(current_grid_state[2] == action_theta_g, "The action index is not consistent with the current orientation.\n");
////
////             int motion_index = int(action_index) - action_theta_g * m_max_num_motion_primitives_;
////             auto &control = m_grid_motion_primitives_[action_theta_g][motion_index].controls[0];
////             return current_grid_state + control;
////         }
////
////         [[nodiscard]] inline std::vector<Successor>
////         GetSuccessorsInMetricSpace(const Eigen::Ref<const Eigen::VectorXd> &) const override {
////             throw NotImplemented(__PRETTY_FUNCTION__);
////         }
////
////         [[nodiscard]] std::vector<Successor>
////         GetSuccessors(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state) const override;
////
////         [[nodiscard]] inline bool
////         IsMetricStateCollided(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
////             return IsGridStateCollided(MetricToGrid(metric_state));
////         }
////
////         [[nodiscard]] inline bool
////         IsGridStateCollided(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
////             return m_inflated_grid_maps_[grid_state[2]].at<uint8_t>(grid_state[0], grid_state[1]) > 0;  // at(row, col)
////         }
////
////         [[nodiscard]] inline std::size_t
////         GridStateHashing(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
////             return m_grid_map_info_->GridToIndex(grid_state, true);  // row-major
////         }
////
////         [[nodiscard]] inline Eigen::VectorXi
////         MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
////             Eigen::Vector3i grid_state(
////                 m_grid_map_info_->MeterToGridForValue(metric_state[0], 0),
////                 m_grid_map_info_->MeterToGridForValue(metric_state[1], 1),
////                 m_grid_map_info_->MeterToGridForValue(metric_state[2], 2));
////             return grid_state;
////         }
////
////         [[nodiscard]] inline Eigen::VectorXd
////         GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
////             Eigen::Vector3d metric_state(
////                 m_grid_map_info_->GridToMeterForValue(grid_state[0], 0),
////                 m_grid_map_info_->GridToMeterForValue(grid_state[1], 1),
////                 m_grid_map_info_->GridToMeterForValue(grid_state[2], 2));
////             return metric_state;
////         }
////
////         [[nodiscard]] inline std::size_t
////         ActionCoordsToActionIndex(const std::vector<std::size_t> &action_coords) const override {
////             auto &kThetaG = action_coords[0];
////             auto &kMotionIndex = action_coords[1];
////             return kThetaG * m_max_num_motion_primitives_ + kMotionIndex;
////         }
////
////         [[nodiscard]] inline std::vector<std::size_t>
////         ActionIndexToActionCoords(std::size_t action_idx) const override {
////             return {action_idx / m_max_num_motion_primitives_, action_idx % m_max_num_motion_primitives_};
////         }
////
////         void
////         ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const override;
////     };
////
//// }  // namespace erl::env

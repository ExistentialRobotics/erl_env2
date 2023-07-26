//#include "erl_env/environment_grid_se2.hpp"
//
//namespace erl::env {
//
//    EnvironmentGridSe2::EnvironmentGridSe2(int step_size, const std::shared_ptr<common::GridMapUnsigned2D> &grid_map, int num_orientations)
//        : m_original_grid_map_(InitializeGridMap2D(grid_map)),
//          m_grid_map_info_(std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(num_orientations, -M_PI, M_PI, 2))) {
//
//        // the agent is a point, all orientations share the same map. No copy here, just a reference.
//        m_inflated_grid_maps_.resize(num_orientations, m_original_grid_map_);
//
//        m_grid_motion_primitive_.controls = {
//            {+step_size, 0, 0},                    // kForward
//            {-step_size, 0, 0},                    // kBack
//            {0, +step_size, 0},                    // kRight
//            {0, -step_size, 0},                    // kLeft
//            {0, 0, +step_size},                    // kTurnLeft
//            {0, 0, -step_size},                    // kTurnRight
//            {+step_size, +step_size, 0},           // kForwardRight
//            {+step_size, -step_size, 0},           // kForwardLeft
//            {-step_size, +step_size, 0},           // kBackRight
//            {-step_size, -step_size, 0},           // kBackLeft
//            {+step_size, 0, +step_size},           // kForwardTurnLeft
//            {+step_size, 0, -step_size},           // kForwardTurnRight
//            {-step_size, 0, +step_size},           // kBackTurnLeft
//            {-step_size, 0, -step_size},           // kBackTurnRight
//            {0, +step_size, +step_size},           // kRightTurnLeft
//            {0, +step_size, -step_size},           // kRightTurnRight
//            {0, -step_size, +step_size},           // kLeftTurnLeft
//            {0, -step_size, -step_size},           // kLeftTurnRight
//            {+step_size, +step_size, +step_size},  // kForwardRightTurnLeft
//            {+step_size, +step_size, -step_size},  // kForwardRightTurnRight
//            {+step_size, -step_size, +step_size},  // kForwardLeftTurnLeft
//            {+step_size, -step_size, -step_size},  // kForwardLeftTurnRight
//            {-step_size, +step_size, +step_size},  // kBackRightTurnLeft
//            {-step_size, +step_size, -step_size},  // kBackRightTurnRight
//            {-step_size, -step_size, +step_size},  // kBackLeftTurnLeft
//            {-step_size, -step_size, -step_size},  // kBackLeftTurnRight
//        };
//
//        m_grid_motion_primitive_.durations.resize(m_grid_motion_primitive_.controls.size(), 1);
//        m_grid_motion_primitive_.costs.reserve(m_grid_motion_primitive_.controls.size());
//        Eigen::Vector3d weight(2, 2, 1);                              // turn cost is smaller
//        Eigen::Vector3d bias(-0.5 * step_size, -0.5 * step_size, 0);  // encourage forward motion
//        for (auto &control: m_grid_motion_primitive_.controls) {
//            m_grid_motion_primitive_.costs.push_back(((control.cast<double>() + bias).cwiseProduct(weight)).norm());
//        }
//    }
//
//    EnvironmentGridSe2::EnvironmentGridSe2(
//        int step_size,
//        const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//        int num_orientations,
//        double inflate_scale,
//        const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
//        : EnvironmentGridSe2(step_size, grid_map, num_orientations) {
//
//        if (shape_metric_vertices.cols() == 0) {
//            ERL_WARN("shape_metric_vertices is empty, no inflation");
//            return;
//        }
//        if (inflate_scale <= 0) {
//            ERL_WARN("inflate_scale <= 0, no inflation");
//            return;
//        }
//        m_shape_metric_vertices_ = shape_metric_vertices.array() * inflate_scale;
//
//        // inflate grid map for different orientations
//        cv::Mat inflated_map;
//        for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
//            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
//            Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_shape_metric_vertices_;
//            InflateGridMap2D(m_original_grid_map_, inflated_map, grid_map->info, vertices);
//            m_inflated_grid_maps_[theta_g] = inflated_map.clone();
//        }
//    }
//
//    std::vector<Successor>
//    EnvironmentGridSe2::GetSuccessors(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state) const {
//
//        static GridSe2Action allowed_actions[] = {
//            GridSe2Action::kForward,
//            GridSe2Action::kBack,
//            GridSe2Action::kTurnLeft,
//            GridSe2Action::kTurnRight,
//            // GridSe2Action::kForwardRight,
//            // GridSe2Action::kForwardLeft,
//            // GridSe2Action::kBackRight,
//            // GridSe2Action::kBackLeft,
//            // GridSe2Action::kForwardTurnLeft,
//            // GridSe2Action::kForwardTurnRight,
//            // GridSe2Action::kBackTurnLeft,
//            // GridSe2Action::kBackTurnRight,
//            // GridSe2Action::kRightTurnLeft,
//            // GridSe2Action::kRightTurnRight,
//            // GridSe2Action::kLeftTurnLeft,
//            // GridSe2Action::kLeftTurnRight,
//            // GridSe2Action::kForwardRightTurnLeft,
//            // GridSe2Action::kForwardRightTurnRight,
//            // GridSe2Action::kForwardLeftTurnLeft,
//            // GridSe2Action::kForwardLeftTurnRight,
//            // GridSe2Action::kBackRightTurnLeft,
//            // GridSe2Action::kBackRightTurnRight,
//            // GridSe2Action::kBackLeftTurnLeft,
//            // GridSe2Action::kBackLeftTurnRight,
//        };
//        static std::size_t num_controls = sizeof(allowed_actions) / sizeof(GridSe2Action);
//
//        std::vector<Successor> successors;
//        successors.clear();
//        successors.reserve(num_controls);
//        for (std::size_t control_idx = 0; control_idx < num_controls; control_idx++) {
//            auto &action = allowed_actions[control_idx];
//            Eigen::Vector3i next_grid_state = ForwardActionInGridSpace(current_grid_state, std::size_t(action));
//            if (next_grid_state == current_grid_state) { continue; }        // discard control that does not move
//            if (!m_grid_map_info_->InGrids(next_grid_state)) { continue; }  // discard control that goes out of the map
//            if (IsGridStateCollided(next_grid_state)) { continue; }         // at(row, col), discard control that collides
//
//            // std::cout << "action: " << GetGridSe2ActionName(action) << ", current grid state:" << current_grid_state.transpose()
//            //           << ", next grid state:" << next_grid_state.transpose() << std::endl;
//            auto next_metric_state = GridToMetric(next_grid_state);
//            successors.emplace_back(
//                next_metric_state,
//                next_grid_state,
//                m_grid_motion_primitive_.costs[control_idx],
//                ActionCoordsToActionIndex({std::size_t(action)}));
//        }
//        return successors;
//    }
//
//    void
//    EnvironmentGridSe2::PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) {
//        m_already_reset_ = false;
//
//        if (m_shape_metric_vertices_.cols() == 0) {
//            Eigen::VectorXi grid_state = MetricToGrid(metric_state);
//            int num_orientations = m_grid_map_info_->Shape(2);
//            int block_half_size = 1;
//            for (int i = -block_half_size; i <= block_half_size; ++i) {
//                for (int j = -block_half_size; j <= block_half_size; ++j) {
//                    for (int k = 0; k < num_orientations; ++k) {
//                        int x = grid_state[0] + i;
//                        int y = grid_state[1] + j;
//                        if (x < 0 || x >= m_grid_map_info_->Shape(0) || y < 0 || y >= m_grid_map_info_->Shape(1)) { continue; }
//                        m_inflated_grid_maps_[k].at<uint8_t>(x, y) = 0;
//                    }
//                }
//            }
//            return;
//        }
//
//        Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(metric_state[2]).toRotationMatrix();
//        Eigen::Vector2d translation_vector = metric_state.head<2>();
//        Eigen::Matrix2Xd vertices = (rotation_matrix * m_shape_metric_vertices_).colwise() + translation_vector;
//        auto &grid_map = m_inflated_grid_maps_[m_grid_map_info_->MeterToGridForValue(metric_state[2], 2)];
//        std::vector<std::vector<cv::Point>> contour(1);
//        auto &contour_points = contour[0];
//        for (int i = 0; i < vertices.cols(); ++i) {
//            contour_points.emplace_back(m_grid_map_info_->MeterToGridForValue(vertices(1, i), 1), m_grid_map_info_->MeterToGridForValue(vertices(0, i), 0));
//        }
//        cv::drawContours(grid_map, contour, 0, cv::Scalar(0), cv::FILLED);
//    }
//
//    void
//    EnvironmentGridSe2::Reset() {
//        if (m_already_reset_) { return; }
//
//        m_already_reset_ = true;
//
//        int num_orientations = m_grid_map_info_->Shape(2);
//        m_inflated_grid_maps_.clear();
//        m_inflated_grid_maps_.resize(num_orientations);
//
//        if (m_shape_metric_vertices_.cols() == 0) {
//            for (int i = 0; i < num_orientations; ++i) { m_original_grid_map_.copyTo(m_inflated_grid_maps_[i]); }
//            return;
//        }
//
//        for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
//            double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
//            Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_shape_metric_vertices_;
//            InflateGridMap2D(
//                m_original_grid_map_,
//                m_inflated_grid_maps_[theta_g],
//                std::make_shared<common::GridMapInfo2D>(m_grid_map_info_->Squeeze(2)),
//                vertices);
//        }
//    }
//
//    void
//    EnvironmentGridSe2::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
//        cv::Mat img = m_original_grid_map_ * 255;
//        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
//        for (const auto &[kGoalIndex, kPath]: paths) {
//            std::vector<cv::Point> points;
//            auto num_points = kPath.cols();
//            points.reserve(num_points);
//            for (long i = 0; i < num_points; ++i) {
//                const double &kX = kPath(0, i);
//                const double &kY = kPath(1, i);
//                const double &kTheta = kPath(2, i);
//                points.emplace_back(m_grid_map_info_->MeterToGridForValue(kY, 1), m_grid_map_info_->MeterToGridForValue(kX, 0));
//                cv::Point2i arrow_point(
//                    m_grid_map_info_->MeterToGridForValue(kY + 0.1 * std::cos(kTheta), 1),
//                    m_grid_map_info_->MeterToGridForValue(kX + 0.1 * std::sin(kTheta), 0));
//                cv::arrowedLine(img, points.back(), arrow_point, cv::Scalar(0, 255, 255), 1, cv::LINE_AA, 0, 0.01);
//            }
//            cv::polylines(img, points, false, cv::Scalar(0, 255, 255), 1);
//        }
//        cv::imshow("environment_grid_se2: paths", img);
//        cv::waitKey(0);
//    }
//}  // namespace erl::env
//
//// #include "erl_env/environment_grid_se2.hpp"
////
//// namespace erl::env {
////
////     std::vector<std::vector<EnvironmentGridSe2::GridMotionPrimitive>>
////     EnvironmentGridSe2::GenerateGridSe2MotionPrimitives(
////         double linear_velocity_min,
////         double linear_velocity_max,
////         double linear_velocity_step,
////         double euclidean_square_distance_cost_weight,
////         double angular_velocity_min,
////         double angular_velocity_max,
////         double angular_velocity_step,
////         double angular_square_distance_cost_weight,
////         double duration_step,
////         double duration,
////         int max_grid_step,
////         const std::shared_ptr<common::GridMapInfo3D> &grid_map_info) {
////
////         // we need a grid_map_info that contains all trajectories that are max_grid_step away from the center
////         // so that we can hash the grid state correctly
////         auto grid_map_info_with_max_step = grid_map_info;
////         int x_g = grid_map_info->CenterGrid().x();
////         int y_g = grid_map_info->CenterGrid().y();
////         if ((x_g < max_grid_step + 1) || (y_g < max_grid_step + 1)) {
////             const double &kXRes = grid_map_info->Resolution(0);
////             const double &kYRes = grid_map_info->Resolution(1);
////             const double &kThetaRes = grid_map_info->Resolution(2);
////             double x_min = grid_map_info->Center().x() - (max_grid_step + 1) * kXRes;
////             double y_min = grid_map_info->Center().y() - (max_grid_step + 1) * kYRes;
////             double x_max = grid_map_info->Center().x() + (max_grid_step + 1) * kXRes;
////             double y_max = grid_map_info->Center().y() + (max_grid_step + 1) * kYRes;
////             grid_map_info_with_max_step = std::make_shared<common::GridMapInfo3D>(
////                 Eigen::Vector3d(x_min, y_min, -M_PI),
////                 Eigen::Vector3d(x_max, y_max, M_PI),
////                 Eigen::Vector3d(kXRes, kYRes, kThetaRes),
////                 Eigen::Vector3i(0, 0, 0));
////         }
////
////         double x = grid_map_info_with_max_step->Center().x();
////         double y = grid_map_info_with_max_step->Center().y();
////         x_g = grid_map_info_with_max_step->CenterGrid().x();
////         y_g = grid_map_info_with_max_step->CenterGrid().y();
////         int num_orientations = grid_map_info_with_max_step->Shape(2);
////         std::vector<std::unordered_set<int>> grid_motion_primitive_hash_tables(num_orientations);
////         std::vector<std::vector<GridMotionPrimitive>> grid_motion_primitive_controls(num_orientations);
////
////         auto num_linear_velocities = int((linear_velocity_max - linear_velocity_min) / linear_velocity_step) + 1;
////         auto num_angular_velocities = int((angular_velocity_max - angular_velocity_min) / angular_velocity_step) + 1;
////         auto num_ts = int(duration / duration_step) + 1;
////         double t, new_x, new_y, new_theta;
////
////         for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
////             double theta = grid_map_info_with_max_step->GridToMeterForValue(theta_g, 2);
////             auto &grid_motion_primitive_hash_table = grid_motion_primitive_hash_tables[theta_g];
////
////             for (int i = 0; i < num_linear_velocities; ++i) {
////                 double v = linear_velocity_min + i * linear_velocity_step;
////                 for (int j = 0; j < num_angular_velocities; ++j) {
////                     if (i == 0 && j == 0) { continue; }
////                     double w = angular_velocity_min + j * angular_velocity_step;
////                     for (int k = 1; k <= num_ts; ++k) {
////                         t = k * duration_step;
////                         DifferentialDriveKinematic(x, y, theta, v, w, t, new_x, new_y, new_theta);
////                         new_theta = common::ClipAngle(new_theta);
////
////                         // skip if the step is too large or the motion is too small to be recognized by the grid map
////                         Eigen::Vector3d next_metric_state = Eigen::Vector3d(new_x, new_y, new_theta);
////                         Eigen::Vector3i next_grid_state = grid_map_info_with_max_step->MeterToGridForPoints(next_metric_state);
////
////                         if (next_grid_state[0] == x_g && next_grid_state[1] == y_g && next_grid_state[2] == theta_g) { continue; }  // too small to recognize
////                         if (std::abs(next_grid_state[0] - x_g) > max_grid_step || std::abs(next_grid_state[1] - y_g) > max_grid_step) { continue; }
////                         int grid_step_theta = std::abs(next_grid_state[2] - theta_g);
////                         grid_step_theta = std::min(grid_step_theta, num_orientations - grid_step_theta);
////                         if (grid_step_theta > max_grid_step) { continue; }             // too large
////                         if (!grid_map_info_with_max_step->InGrids(next_grid_state)) {  // out of grid map
////                             ERL_WARN(
////                                 "v=%f, w=%f, t=%f causes the next state %s out of grid map.\n",
////                                 v,
////                                 w,
////                                 t,
////                                 common::EigenToNumPyFmtString(next_metric_state.transpose()).c_str());
////                             continue;
////                         }
////
////                         auto hashing = grid_map_info_with_max_step->GridToIndex(next_grid_state, true);
////                         grid_motion_primitive_hash_table.insert(hashing);
////                     }
////                 }
////             }
////
////             auto &grid_motion_primitive = grid_motion_primitive_controls[theta_g];
////             grid_motion_primitive.reserve(grid_motion_primitive_hash_table.size());
////             for (auto &kHashing: grid_motion_primitive_hash_table) {
////                 Eigen::Vector3i control = grid_map_info_with_max_step->IndexToGrid(kHashing, true);
////                 control[0] -= x_g;      // relative x
////                 control[1] -= y_g;      // relative y
////                 control[2] -= theta_g;  // relative orientation
////                 double dx = control[0] * grid_map_info_with_max_step->Resolution(0);
////                 double dy = control[1] * grid_map_info_with_max_step->Resolution(1);
////                 double euclidean_square_distance = dx * dx + dy * dy;
////                 int angular_distance_g = std::abs(control[2]);
////                 angular_distance_g = std::min(angular_distance_g, num_orientations - angular_distance_g);
////                 double angular_square_distance = angular_distance_g * grid_map_info_with_max_step->Resolution(2);
////                 angular_square_distance *= angular_square_distance;
////                 double cost = std::sqrt(
////                     euclidean_square_distance * euclidean_square_distance_cost_weight + angular_square_distance * angular_square_distance_cost_weight);
////                 grid_motion_primitive.push_back({{control}, {1.}, {cost}});  // control, duration, cost
////             }
////
////             ERL_ASSERTM(!grid_motion_primitive.empty(), "No motion primitive found for orientation %f.\n", theta);
////         }
////
////         return grid_motion_primitive_controls;
////     }
////
////     EnvironmentGridSe2::EnvironmentGridSe2(
////         double linear_velocity_min,
////         double linear_velocity_max,
////         double linear_velocity_step,
////         double euclidean_square_distance_cost_weight,
////         double angular_velocity_min,
////         double angular_velocity_max,
////         double angular_velocity_step,
////         double angular_square_distance_cost_weight,
////         double duration_step,
////         double duration,
////         int max_step_size,
////         const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////         int num_orientations)
////         : EnvironmentGridSe2({}, grid_map, num_orientations) {
////
////         m_grid_motion_primitives_ = GenerateGridSe2MotionPrimitives(
////             linear_velocity_min,
////             linear_velocity_max,
////             linear_velocity_step,
////             euclidean_square_distance_cost_weight,
////             angular_velocity_min,
////             angular_velocity_max,
////             angular_velocity_step,
////             angular_square_distance_cost_weight,
////             duration_step,
////             duration,
////             max_step_size,
////             m_grid_map_info_);
////
////         m_max_num_motion_primitives_ = 0;
////         for (auto &grid_motion_primitive: m_grid_motion_primitives_) {
////             m_max_num_motion_primitives_ = std::max(m_max_num_motion_primitives_, int(grid_motion_primitive.size()));
////         }
////     }
////
////     EnvironmentGridSe2::EnvironmentGridSe2(
////         double linear_velocity_min,
////         double linear_velocity_max,
////         double linear_velocity_step,
////         double euclidean_square_distance_cost_weight,
////         double angular_velocity_min,
////         double angular_velocity_max,
////         double angular_velocity_step,
////         double angular_square_distance_cost_weight,
////         double duration_step,
////         double duration,
////         int max_step_size,
////         const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////         int num_orientations,
////         double inflate_scale,
////         const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
////         : EnvironmentGridSe2({}, grid_map, num_orientations, inflate_scale, shape_metric_vertices) {
////
////         m_grid_motion_primitives_ = GenerateGridSe2MotionPrimitives(
////             linear_velocity_min,
////             linear_velocity_max,
////             linear_velocity_step,
////             euclidean_square_distance_cost_weight,
////             angular_velocity_min,
////             angular_velocity_max,
////             angular_velocity_step,
////             angular_square_distance_cost_weight,
////             duration_step,
////             duration,
////             max_step_size,
////             m_grid_map_info_);
////
////         m_max_num_motion_primitives_ = 0;
////         for (auto &grid_motion_primitive: m_grid_motion_primitives_) {
////             m_max_num_motion_primitives_ = std::max(m_max_num_motion_primitives_, int(grid_motion_primitive.size()));
////         }
////     }
////
////     EnvironmentGridSe2::EnvironmentGridSe2(
////         std::vector<std::vector<GridMotionPrimitive>> grid_motion_primitives,
////         const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////         int num_orientations)
////         : m_grid_motion_primitives_(std::move(grid_motion_primitives)),
////           m_original_grid_map_(InitializeGridMap2D(grid_map)),
////           m_grid_map_info_(std::make_shared<common::GridMapInfo3D>(grid_map->info->Extend(num_orientations, -M_PI, M_PI, 2))) {
////
////         m_max_num_motion_primitives_ = 0;
////         for (auto &grid_motion_primitive: m_grid_motion_primitives_) {
////             m_max_num_motion_primitives_ = std::max(m_max_num_motion_primitives_, int(grid_motion_primitive.size()));
////         }
////         // the agent is a point, all orientations share the same map. No copy here, just a reference.
////         m_inflated_grid_maps_.resize(num_orientations, m_original_grid_map_);
////     }
////
////     EnvironmentGridSe2::EnvironmentGridSe2(
////         std::vector<std::vector<GridMotionPrimitive>> grid_motion_primitives,
////         const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////         int num_orientations,
////         double inflate_scale,
////         const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
////         : EnvironmentGridSe2(std::move(grid_motion_primitives), grid_map, num_orientations) {
////
////         m_max_num_motion_primitives_ = 0;
////         for (auto &grid_motion_primitive: m_grid_motion_primitives_) {
////             m_max_num_motion_primitives_ = std::max(m_max_num_motion_primitives_, int(grid_motion_primitive.size()));
////         }
////
////         if (shape_metric_vertices.cols() == 0) {
////             ERL_WARN("shape_metric_vertices is empty, no inflation");
////             return;
////         }
////         if (inflate_scale <= 0) {
////             ERL_WARN("inflate_scale <= 0, no inflation");
////             return;
////         }
////         m_shape_metric_vertices_ = shape_metric_vertices.array() * inflate_scale;
////
////         // inflate grid map for different orientations
////         cv::Mat inflated_map;
////         for (int theta_g = 0; theta_g < num_orientations; ++theta_g) {
////             double theta = m_grid_map_info_->GridToMeterForValue(theta_g, 2);
////             Eigen::Matrix2Xd vertices = Eigen::Rotation2Dd(theta).toRotationMatrix() * m_shape_metric_vertices_;
////             InflateGridMap2D(m_original_grid_map_, inflated_map, grid_map->info, vertices);
////             m_inflated_grid_maps_[theta_g] = inflated_map.clone();
////         }
////     }
////
////     std::vector<Successor>
////     EnvironmentGridSe2::GetSuccessors(const Eigen::Ref<const Eigen::VectorXi> &current_grid_state) const {
////         std::size_t theta_g = current_grid_state[2];
////         auto &motion_primitives = m_grid_motion_primitives_[theta_g];
////         std::size_t num_motions = motion_primitives.size();
////         std::vector<Successor> successors;
////         successors.clear();
////         successors.reserve(num_motions);
////
////         for (std::size_t motion_idx = 0; motion_idx < num_motions; ++motion_idx) {
////             auto &motion_primitive = motion_primitives[motion_idx];
////             Eigen::Vector3i next_grid_state = current_grid_state + motion_primitive.controls[0];
////             if (!m_grid_map_info_->InGrids(next_grid_state)) { continue; }  // discard control that goes out of the map
////             if (IsGridStateCollided(next_grid_state)) { continue; }         // at(row, col), discard control that collides
////
////             auto next_metric_state = GridToMetric(next_grid_state);
////             successors.push_back({next_metric_state, next_grid_state, motion_primitive.costs[0], ActionCoordsToActionIndex({theta_g, motion_idx})});
////         }
////
////         return successors;
////     }
////
////     void
////     EnvironmentGridSe2::ShowPaths(const std::map<int, Eigen::MatrixXd> &paths) const {
////         cv::Mat img = m_original_grid_map_ * 255;
////         cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
////         for (const auto &[kGoalIndex, kPath]: paths) {
////             std::vector<cv::Point> points;
////             auto num_points = kPath.cols();
////             points.reserve(num_points);
////             for (long i = 0; i < num_points; ++i) {
////                 const double &kX = kPath(0, i);
////                 const double &kY = kPath(1, i);
////                 const double &kTheta = kPath(2, i);
////                 points.emplace_back(m_grid_map_info_->MeterToGridForValue(kY, 1), m_grid_map_info_->MeterToGridForValue(kX, 0));
////                 cv::Point2i arrow_point(
////                     m_grid_map_info_->MeterToGridForValue(kY + 0.1 * std::cos(kTheta), 1),
////                     m_grid_map_info_->MeterToGridForValue(kX + 0.1 * std::sin(kTheta), 0));
////                 cv::arrowedLine(img, points.back(), arrow_point, cv::Scalar(0, 255, 255), 1, cv::LINE_AA, 0, 0.01);
////             }
////             cv::polylines(img, points, false, cv::Scalar(0, 255, 255), 1);
////         }
////         cv::imshow("environment_grid_se2: paths", img);
////         cv::waitKey(0);
////     }
//// }  // namespace erl::env

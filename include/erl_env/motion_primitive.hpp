#pragma once

#include <functional>
#include <vector>

// #include "state.hpp"

namespace erl::env {

    /**
     * A motion primitive is defined by a sequence of controls
     * @tparam State
     * @tparam Control
     */
    template<typename Control>
    struct MotionPrimitive {

        std::vector<Control> controls;  // [u1, u2, ...]
        std::vector<double> durations;  // [t1, t2, ...]: Apply u1 for t1, then u2 for t2, then ...
        std::vector<double> costs;      // [c1, c2, ...]: cumulated cost at each time step

        MotionPrimitive() = default;

        MotionPrimitive(
            std::vector<Control> controls,
            std::vector<double> durations,
            std::vector<double> costs)
            : controls(std::move(controls)),
              durations(std::move(durations)),
              costs(std::move(costs)) {

            ERL_DEBUG_ASSERT(
                std::none_of(
                    this->durations.begin(),
                    this->durations.end(),
                    [](double d) { return d <= 0.; }),
                "All durations must be positive.");
            ERL_DEBUG_ASSERT(
                std::none_of(
                    this->costs.begin(),
                    this->costs.end(),
                    [](double c) { return c <= 0.; }),
                "All costs must be positive.");
        }

        [[nodiscard]] inline std::vector<Eigen::MatrixXd>
        ComputeTrajectorySegments(
            Eigen::VectorXd state,
            double dt,
            const std::function<
                Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd> &, const Control &, double)>
                &motion_model_function) const {

            std::vector<Eigen::MatrixXd> trajectories;
            trajectories.resize(controls.size());
            for (std::size_t control_idx = 0; control_idx < controls.size(); ++control_idx) {
                auto &trajectory = trajectories[control_idx];
                ComputeTrajectorySegment(state, control_idx, dt, motion_model_function, trajectory);
                state = trajectory.rightCols<1>();  // terminal state of the current control, also
                                                    // the initial state of the next control
            }
            return trajectories;
        }

        [[nodiscard]] inline Eigen::MatrixXd
        ComputeTrajectorySegment(
            const Eigen::Ref<const Eigen::VectorXd> &state,
            std::size_t control_idx,
            double dt,
            const std::function<
                Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd> &, const Control &, double)>
                &motion_model_function) const {

            Eigen::MatrixXd trajectory;
            ComputeTrajectorySegment(state, control_idx, dt, motion_model_function, trajectory);
            return trajectory;
        }

        inline void
        ComputeTrajectorySegment(
            const Eigen::Ref<const Eigen::VectorXd> &state,
            std::size_t control_idx,
            double dt,
            const std::function<
                Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd> &, const Control &, double)>
                &motion_model_function,
            Eigen::MatrixXd &trajectory) const {

            int num_steps = static_cast<int>(durations[control_idx] / dt) + 1;
            dt = durations[control_idx] / num_steps;
            trajectory.resize(state.size(), num_steps);

            auto &control = controls[control_idx];
            double t = dt;
            for (int i = 0; i < num_steps; ++i) {
                trajectory.col(i) = motion_model_function(state, control, t);
                t += dt;
            }
        }
    };

}  // namespace erl::env

#pragma once

#include <functional>
#include <vector>

namespace erl::env {

    /**
     * A motion primitive is defined by a sequence of controls
     * @tparam Control
     */
    template<typename Control, typename Dtype, int Dim>
    struct MotionPrimitive {

        using MetricState = Eigen::Vector<Dtype, Dim>;
        using MetricTrajectory = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using MotionModelFunction =
            std::function<MetricState(const MetricState &, const Control &, Dtype)>;

        std::vector<Control> controls;  // [u1, u2, ...]
        std::vector<Dtype> durations;   // [t1, t2, ...]: Apply u1 for t1, then u2 for t2, then ...
        std::vector<Dtype> costs;       // [c1, c2, ...]: cumulated cost at each time step

        MotionPrimitive() = default;

        MotionPrimitive(
            std::vector<Control> controls,
            std::vector<Dtype> durations,
            std::vector<Dtype> costs)
            : controls(std::move(controls)),
              durations(std::move(durations)),
              costs(std::move(costs)) {

            ERL_DEBUG_ASSERT(
                std::none_of(
                    this->durations.begin(),
                    this->durations.end(),
                    [](Dtype d) { return d <= 0.f; }),
                "All durations must be positive.");
            ERL_DEBUG_ASSERT(
                std::none_of(
                    this->costs.begin(),
                    this->costs.end(),
                    [](Dtype c) { return c <= 0.f; }),
                "All costs must be positive.");
        }

        [[nodiscard]] std::vector<MetricTrajectory>
        ComputeTrajectorySegments(
            MetricState state,
            Dtype dt,
            const MotionModelFunction &motion_model_function) const {

            std::vector<MetricTrajectory> trajectories;
            trajectories.resize(controls.size());
            for (std::size_t control_idx = 0; control_idx < controls.size(); ++control_idx) {
                auto &trajectory = trajectories[control_idx];
                ComputeTrajectorySegment(state, control_idx, dt, motion_model_function, trajectory);
                // terminal state of the current control, also the initial state of the next control
                state = trajectory.template rightCols<1>();
            }
            return trajectories;
        }

        /**
         * Compute the trajectory segment for a single control with fixed time step.
         * @param state: initial state
         * @param control_idx: index of the control to apply
         * @param dt: time step
         * @param motion_model_function: function that defines the motion model
         * @return trajectory: each column is a state. The initial state is not included.
         */
        [[nodiscard]] MetricTrajectory
        ComputeTrajectorySegment(
            const Eigen::Ref<const MetricState> &state,
            const std::size_t control_idx,
            const Dtype dt,
            const MotionModelFunction &motion_model_function) const {

            MetricTrajectory trajectory;
            ComputeTrajectorySegment(state, control_idx, dt, motion_model_function, trajectory);
            return trajectory;
        }

        /**
         * Compute the trajectory segment for a single control with fixed time step.
         * @param state: initial state
         * @param control_idx: index of the control to apply
         * @param dt: time step
         * @param motion_model_function: function that defines the motion model
         * @param trajectory: output trajectory, each column is a state. The initial state is not
         * included.
         */
        void
        ComputeTrajectorySegment(
            const Eigen::Ref<const MetricState> &state,
            const std::size_t control_idx,
            Dtype dt,
            const MotionModelFunction &motion_model_function,
            MetricTrajectory &trajectory) const {

            int num_steps = static_cast<int>(durations[control_idx] / dt) + 1;
            dt = durations[control_idx] / num_steps;
            trajectory.resize(state.size(), num_steps);

            auto &control = controls[control_idx];
            Dtype t = dt;
            for (int i = 0; i < num_steps; ++i) {
                trajectory.col(i) = motion_model_function(state, control, t);
                t += dt;
            }
        }

        struct YamlConvertImpl {
            static YAML::Node
            encode(const MotionPrimitive &primitive) {
                YAML::Node node;
                ERL_YAML_SAVE_ATTR(node, primitive, controls);
                ERL_YAML_SAVE_ATTR(node, primitive, durations);
                ERL_YAML_SAVE_ATTR(node, primitive, costs);
                return node;
            }

            static bool
            decode(const YAML::Node &node, MotionPrimitive &primitive) {
                if (!node.IsMap()) { return false; }
                ERL_YAML_LOAD_ATTR(node, primitive, controls);
                ERL_YAML_LOAD_ATTR(node, primitive, durations);
                ERL_YAML_LOAD_ATTR(node, primitive, costs);
                return true;
            }
        };
    };

}  // namespace erl::env

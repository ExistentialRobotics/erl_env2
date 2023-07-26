#pragma once

#include "erl_common/exception.hpp"
#include "erl_common/grid_map.hpp"
#include "environment_state.hpp"
#include "cost.hpp"

#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <list>
#include <map>

namespace erl::env {

    struct Successor {
        std::shared_ptr<EnvironmentState> env_state = nullptr;
        double cost = 0.0;
        std::size_t action_id = -1;

        Successor() = default;

        Successor(std::shared_ptr<EnvironmentState> state, double cost, std::size_t action_id)
            : env_state(std::move(state)),
              cost(cost),
              action_id(action_id) {
            ERL_ASSERTM(env_state != nullptr, "state is nullptr");
        }

        Successor(Eigen::VectorXd env_metric_state, Eigen::VectorXi env_grid_state, double cost, std::size_t action_id)
            : env_state(std::make_shared<EnvironmentState>(std::move(env_metric_state), std::move(env_grid_state))),
              cost(cost),
              action_id(action_id) {}
    };

    /**
     * @brief EnvironmentBase is a virtual interface for search-based planning on a metric space. Thus, the EnvironmentBase includes a set of map parameters
     * such as grid cell resolution, and a collision checker. The internal state representation is discrete grid coordinate.
     */
    class EnvironmentBase {

    protected:
        std::shared_ptr<CostBase> m_distance_cost_func_;
        double m_dt_ = 0.01;  // 10ms

    public:
        explicit EnvironmentBase(std::shared_ptr<CostBase> distance_cost_func, double dt = 0.01)
            : m_distance_cost_func_(std::move(distance_cost_func)),
              m_dt_(dt) {
            if (m_distance_cost_func_ == nullptr) {
                ERL_INFO("distance_cost_func is nullptr, use Euclidean distance as default cost function.");
                m_distance_cost_func_ = std::make_shared<EuclideanDistanceCost>();
            }
        }

        virtual ~EnvironmentBase() = default;

        [[nodiscard]] virtual std::size_t
        GetStateSpaceSize() const = 0;

        [[nodiscard]] virtual std::size_t
        GetActionSpaceSize() const = 0;

        [[nodiscard]] virtual std::vector<std::shared_ptr<EnvironmentState>>
        ForwardAction(const std::shared_ptr<const EnvironmentState> &state, std::size_t action_index) const = 0;

        [[nodiscard]] inline std::shared_ptr<CostBase>
        GetDistanceCostFunc() const {
            return m_distance_cost_func_;
        }

        [[nodiscard]] inline double
        GetTimeStep() const {
            return m_dt_;
        }

        [[nodiscard]] virtual std::vector<Successor>
        GetSuccessors(const std::shared_ptr<EnvironmentState> &state) const = 0;

        [[nodiscard]] virtual bool
        InStateSpace(const std::shared_ptr<EnvironmentState> &state) const {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

        [[nodiscard]] virtual bool
        IsReachable(const std::vector<std::shared_ptr<EnvironmentState>> &trajectory) const = 0;

        [[nodiscard]] virtual std::size_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const = 0;

        [[nodiscard]] virtual Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const = 0;

        [[nodiscard]] virtual Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const = 0;

        [[nodiscard]] virtual std::size_t
        ActionCoordsToActionIndex(const std::vector<std::size_t> &action_coords) const = 0;

        [[nodiscard]] virtual std::vector<std::size_t>
        ActionIndexToActionCoords(std::size_t action_idx) const = 0;

        virtual void
        PlaceRobot(const Eigen::Ref<const Eigen::VectorXd> &metric_state) = 0;

        virtual void
        Reset() = 0;

        virtual void
        ShowPaths(const std::map<int, Eigen::MatrixXd> &) const {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

    protected:
        static cv::Mat
        InitializeGridMap2D(const std::shared_ptr<common::GridMapUnsigned2D> &grid_map) {
            cv::Mat grid_map_mat(grid_map->info->Shape(0), grid_map->info->Shape(1), CV_8UC1, cv::Scalar(0));  // x to the bottom, y to the right, along y first
            int size = grid_map->info->Size();
            auto begin = grid_map->data.GetDataPtr();
            auto end = begin + size;
            std::copy(begin, end, grid_map_mat.data);  // both erl::common::GridMapUnsigned2D and cv::Mat are row-major.
            return grid_map_mat;
        }

        static void
        InflateGridMap2D(
            const cv::Mat &original_grid_map,
            cv::Mat &inflated_grid_map,
            const std::shared_ptr<common::GridMapInfo2D> &grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices);
    };

}  // namespace erl::env

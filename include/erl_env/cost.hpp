#pragma once

#include "erl_common/assert.hpp"
#include "erl_common/grid_map.hpp"
#include "environment_state.hpp"

namespace erl::env {

    struct CostBase {
        virtual ~CostBase() = default;

        virtual double
        operator()(const EnvironmentState& state1, const EnvironmentState& state2) const = 0;
    };

    struct EuclideanDistanceCost : public CostBase {
        inline double
        operator()(const EnvironmentState& state1, const EnvironmentState& state2) const override {
            ERL_ASSERTM(state1.metric.size() == state2.metric.size(), "state dimension is not equal to goal dimension.");
            long n = state1.metric.size();
            double distance = 0.0;
            for (long i = 0; i < n; ++i) {
                double diff = std::abs(state1.metric[i] - state2.metric[i]);
                distance += diff * diff;
            }
            distance = std::sqrt(distance);
            return distance;
        }
    };

    struct ManhattanDistanceCost : public CostBase {
        inline double
        operator()(const EnvironmentState& state1, const EnvironmentState& state2) const override {
            ERL_ASSERTM(state1.metric.size() == state2.metric.size(), "state dimension is not equal to goal dimension.");
            long n = state1.metric.size();
            double distance = 0.0;
            for (long i = 0; i < n; ++i) {
                double diff = std::abs(state1.metric[i] - state2.metric[i]);
                distance += diff;
            }
            return distance;
        }
    };

    template<typename DataType, int Dim>
    struct MapCost : public CostBase {

        typedef common::GridMap<DataType, Dim> MapType;
        MapType map;

        explicit MapCost(MapType map_in)
            : map(std::move(map_in)) {}

        inline double
        operator()(const EnvironmentState&, const EnvironmentState& state2) const override {
            return map.data(state2.grid);
        }
    };

    struct CostSum : public CostBase {

        std::vector<std::shared_ptr<CostBase>> costs;

        explicit CostSum(std::vector<std::shared_ptr<CostBase>> costs_in)
            : costs(std::move(costs_in)) {}

        inline double
        operator()(const EnvironmentState& state1, const EnvironmentState& state2) const override {
            double cost = 0.0;
            for (auto& cost_func: costs) { cost += (*cost_func)(state1, state2); }
            return cost;
        }
    };

}  // namespace erl::env

#pragma once

#include "environment_state.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/factory_pattern.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/serialization.hpp"

namespace erl::env {

    template<typename Dtype, int Dim>
    struct CostBase {
        using Factory = common::FactoryPattern<CostBase>;
        using State = EnvironmentState<Dtype, Dim>;
        using MetricState = typename State::MetricState;

        virtual ~CostBase() = default;

        [[nodiscard]] std::string
        GetCostType() const {
            return type_name(*this);
        }

        template<typename Derived>
        static std::enable_if_t<std::is_base_of_v<CostBase, Derived>, bool>
        Register(const std::string &cost_type = "") {
            return Factory::GetInstance().template Register<Derived>(cost_type, []() {
                return std::make_shared<Derived>();
            });
        }

        virtual Dtype
        operator()(const State &state1, const State &state2) const = 0;

        [[nodiscard]] virtual bool
        Write(std::ostream &s) const {
            return s.good();
        }

        [[nodiscard]] virtual bool
        Read(std::istream &s) {
            return s.good();
        }

        Dtype
        Calc(const MetricState &state1, const MetricState &state2) const {
            State s1, s2;
            s1.metric = state1;
            s2.metric = state2;
            return (*this)(s1, s2);
        }
    };

    template<typename Dtype, int Dim>
    struct EuclideanDistanceCost : public CostBase<Dtype, Dim> {
        using State = typename CostBase<Dtype, Dim>::State;

        Dtype
        operator()(const State &state1, const State &state2) const override {
            return (state1.metric - state2.metric).norm();
        }
    };

    template<typename Dtype>
    struct Se2Cost : public CostBase<Dtype, 3> {
        using State = typename CostBase<Dtype, 3>::State;

        Dtype w_theta = 0;  // weight for the heading difference

        explicit Se2Cost(Dtype w_theta_in = 1.0)
            : w_theta(w_theta_in) {}

        Dtype
        operator()(const State &state1, const State &state2) const override {
            Dtype diff_x = state1.metric[0] - state2.metric[0];
            Dtype diff_y = state1.metric[1] - state2.metric[1];
            Dtype diff_theta = std::abs(state1.metric[2] - state2.metric[2]);
            diff_theta = common::WrapAngleTwoPi(diff_theta);
            diff_theta = std::min(diff_theta, static_cast<Dtype>(2 * M_PI - diff_theta));
            Dtype distance =
                std::sqrt(diff_x * diff_x + diff_y * diff_y + w_theta * diff_theta * diff_theta);
            return distance;
        }

        [[nodiscard]] bool
        Write(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&w_theta), sizeof(w_theta));
            return s.good();
        }

        [[nodiscard]] bool
        Read(std::istream &s) override {
            s.read(reinterpret_cast<char *>(&w_theta), sizeof(w_theta));
            return s.good();
        }
    };

    template<typename Dtype, int Dim>
    struct ManhattanDistanceCost : public CostBase<Dtype, Dim> {
        using State = typename CostBase<Dtype, Dim>::State;

        Dtype
        operator()(const State &state1, const State &state2) const override {
            const long n = Dim == Eigen::Dynamic ? state1.metric.size() : Dim;
            Dtype distance = 0.0f;
            for (long i = 0; i < n; ++i) {
                Dtype diff = std::abs(state1.metric[i] - state2.metric[i]);
                distance += diff;
            }
            return distance;
        }
    };

    template<typename Dtype, int Dim>
    using L1Cost = ManhattanDistanceCost<Dtype, Dim>;  // alias

    template<typename DataType, int Dim>
    struct MapCost : public CostBase<DataType, Dim> {
        using MapType = common::GridMap<DataType, DataType, Dim>;
        using State = typename CostBase<DataType, Dim>::State;

        MapType map;

        explicit MapCost(MapType map_in)
            : map(std::move(map_in)) {}

        DataType
        operator()(const State &, const State &state2) const override {
            return map.data[state2.grid];
        }

        [[nodiscard]] bool
        Write(std::ostream &s) const override {
            using namespace common::serialization;
            static const TokenWriteFunctionPairs<MapCost> token_function_pairs = {
                {
                    "map",
                    [](const MapCost *self, std::ostream &stream) {
                        return self->map.Write(stream);
                    },
                },
            };
            return WriteTokens(s, this, token_function_pairs);
        }

        [[nodiscard]] bool
        Read(std::istream &s) override {
            using namespace common::serialization;
            static const TokenReadFunctionPairs<MapCost> token_function_pairs = {
                {
                    "map",
                    [](MapCost *self, std::istream &stream) { return self->map.Read(stream); },
                },
            };
            return ReadTokens(s, this, token_function_pairs);
        }
    };

    template<typename Dtype, int Dim>
    struct CostSum : public CostBase<Dtype, Dim> {
        using State = typename CostBase<Dtype, Dim>::State;

        std::vector<std::shared_ptr<CostBase<Dtype, Dim>>> costs;

        explicit CostSum(std::vector<std::shared_ptr<CostBase<Dtype, Dim>>> costs_in)
            : costs(std::move(costs_in)) {}

        Dtype
        operator()(const State &state1, const State &state2) const override {
            Dtype cost = 0.0f;
            for (auto &cost_func: costs) { cost += (*cost_func)(state1, state2); }
            return cost;
        }

        [[nodiscard]] bool
        Write(std::ostream &s) const override {
            using namespace common::serialization;
            static const TokenWriteFunctionPairs<CostSum> token_function_pairs = {
                {
                    "costs",
                    [](const CostSum *self, std::ostream &stream) {
                        const std::size_t num_costs = self->costs.size();
                        stream.write(reinterpret_cast<const char *>(&num_costs), sizeof(num_costs));
                        for (const auto &cost: self->costs) {
                            const std::string cost_type = cost->GetCostType();
                            stream << cost_type << '\n';
                            if (!cost->Write(stream)) { return false; }
                        }
                        return stream.good();
                    },
                },
            };
            return WriteTokens(s, this, token_function_pairs);
        }

        [[nodiscard]] bool
        Read(std::istream &s) override {
            using namespace common::serialization;
            static const TokenReadFunctionPairs<CostSum> token_function_pairs = {
                {
                    "costs",
                    [](CostSum *self, std::istream &stream) {
                        std::size_t num_costs = 0;
                        stream.read(reinterpret_cast<char *>(&num_costs), sizeof(num_costs));
                        self->costs.clear();
                        for (std::size_t i = 0; i < num_costs; ++i) {
                            std::string cost_type;
                            std::getline(stream, cost_type);
                            auto cost =
                                CostBase<Dtype, Dim>::Factory::GetInstance().Create(cost_type);
                            if (cost == nullptr) {
                                ERL_WARN("Failed to create cost of type {}.", cost_type);
                                return false;
                            }
                            if (!cost->Read(stream)) { return false; }
                            self->costs.push_back(cost);
                        }
                        return stream.good();
                    },
                },
            };
            return ReadTokens(s, this, token_function_pairs);
        }
    };

}  // namespace erl::env

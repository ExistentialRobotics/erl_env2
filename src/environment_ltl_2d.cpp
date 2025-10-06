#include "erl_env/environment_ltl_2d.hpp"

namespace erl::env {
    template class EnvironmentLTL2D<float, uint8_t>;
    template class EnvironmentLTL2D<double, uint8_t>;
    template class EnvironmentLTL2D<float, float>;
    template class EnvironmentLTL2D<double, float>;
    template class EnvironmentLTL2D<float, double>;
    template class EnvironmentLTL2D<double, double>;
}  // namespace erl::env

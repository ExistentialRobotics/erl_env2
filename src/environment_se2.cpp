#include "erl_env/environment_se2.hpp"

namespace erl::env {
    template class EnvironmentSe2<float, uint8_t>;
    template class EnvironmentSe2<double, uint8_t>;
    template class EnvironmentSe2<float, float>;
    template class EnvironmentSe2<double, float>;
    template class EnvironmentSe2<float, double>;
    template class EnvironmentSe2<double, double>;
}  // namespace erl::env

#include "erl_env/environment_2d.hpp"

namespace erl::env {
    template class Environment2D<float, uint8_t>;
    template class Environment2D<double, uint8_t>;
    template class Environment2D<float, float>;
    template class Environment2D<double, float>;
    template class Environment2D<float, double>;
    template class Environment2D<double, double>;
}  // namespace erl::env

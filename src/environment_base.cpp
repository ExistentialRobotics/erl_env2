#include "erl_env/environment_base.hpp"

namespace erl::env {
    template class EnvironmentBase<float, 2>;
    template class EnvironmentBase<double, 2>;
    template class EnvironmentBase<float, 3>;
    template class EnvironmentBase<double, 3>;
    template class EnvironmentBase<float, 4>;
    template class EnvironmentBase<double, 4>;
}  // namespace erl::env

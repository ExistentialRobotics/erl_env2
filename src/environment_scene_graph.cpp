#include "erl_env/environment_scene_graph.hpp"

namespace erl::env {
    template class EnvironmentSceneGraph<float, 3>;
    template class EnvironmentSceneGraph<double, 3>;
    template class EnvironmentSceneGraph<float, 4>;
    template class EnvironmentSceneGraph<double, 4>;
}  // namespace erl::env

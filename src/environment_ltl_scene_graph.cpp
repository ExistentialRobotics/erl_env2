#include "erl_env/environment_ltl_scene_graph.hpp"

namespace Eigen::internal {

    template<>
    struct cast_impl<std::bitset<32>, uint32_t> {
        EIGEN_DEVICE_FUNC
        static uint32_t
        run(const std::bitset<32> &x) {
            return static_cast<uint32_t>(x.to_ulong());
        }
    };

    template<>
    struct cast_impl<uint32_t, std::bitset<32>> {
        EIGEN_DEVICE_FUNC
        static std::bitset<32>
        run(const uint32_t &x) {
            return {x};
        }
    };
}  // namespace Eigen::internal

namespace erl::env {
    template class EnvironmentLTLSceneGraph<float>;
    template class EnvironmentLTLSceneGraph<double>;
}  // namespace erl::env

#include "erl_env/pybind11_environment_2d.hpp"

void
BindEnvironment2D(py::module &m) {
    BindEnvironment2D<float, uint8_t>(m, "Environment2D_f_u8");
    BindEnvironment2D<double, uint8_t>(m, "Environment2D_d_u8");
    BindEnvironment2D<float, float>(m, "Environment2D_f_f");
    BindEnvironment2D<double, float>(m, "Environment2D_d_f");
    BindEnvironment2D<float, double>(m, "Environment2D_f_d");
    BindEnvironment2D<double, double>(m, "Environment2D_d_d");
}

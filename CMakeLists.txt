cmake_minimum_required(VERSION 3.16)

project(erl_env
    LANGUAGES CXX
    VERSION 0.2.0
    DESCRIPTION "erl_env is a C++ library of environment abstraction for planning algorithms.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if(NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif()

erl_project_setup(ERL_PACKAGES erl_common)
include(cmake/erl_env_dependencies.cmake)
erl_setup_ros(
    LIBRARIES ${PROJECT_NAME}
    CFG_EXTRAS erl_env_dependencies.cmake)

####################################################################################################
# BUILD
####################################################################################################

########################
# define the libraries #
########################
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
erl_collect_targets(LIBRARIES ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${spot_INCLUDE_DIRS})
erl_target_dependencies(
    ${PROJECT_NAME} PUBLIC
    ${spot_LIBRARIES} absl::flat_hash_map
)

###################
# Python bindings #
###################
erl_add_pybind_module(
    PYBIND_MODULE_NAME py${PROJECT_NAME}
    PYBIND_SRC_DIR ${${PROJECT_NAME}_PYTHON_BINDING_DIR}
    LIBRARIES ${PROJECT_NAME})
erl_add_python_package()

####################################################################################################
# INSTALL
####################################################################################################
erl_install(
    EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES}
    LIBRARIES ${${PROJECT_NAME}_COLLECTED_LIBRARIES}
    PYBIND_MODULES py${PROJECT_NAME})

####################################################################################################
# TEST
####################################################################################################
erl_add_tests(LIBRARIES ${PROJECT_NAME})

####################################################################################################
# export variables
####################################################################################################
erl_mark_project_found(${PROJECT_NAME})

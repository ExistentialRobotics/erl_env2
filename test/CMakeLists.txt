link_libraries(${PROJECT_NAME}::${PROJECT_NAME} GTest::gtest_main)

add_test(NAME test_explorer_vedo_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/../python/${PROJECT_NAME}/env/data/house_expo/explorer_vedo.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../python)
set_tests_properties(test_explorer_vedo_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

# test quadtree with HouseExpo data
add_test(NAME test_quadtree_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/test_quadtree.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set_tests_properties(test_quadtree_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

# test GridMapInfo2D, GridMapDrawer2D, Lidar2D, LidarFrame2D
add_test(NAME test_play_trajectory_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/../python/${PROJECT_NAME}/env/data/house_expo/play_trajectory.py --draw-trajectory
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../python)
set_tests_properties(test_play_trajectory_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

# test LogOddMap2D
add_test(NAME test_play_trajectory_vedo_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/../python/${PROJECT_NAME}/env/data/house_expo/play_trajectory_vedo.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../python)
set_tests_properties(test_play_trajectory_vedo_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

# test loading DdcMotionPrimitive
add_executable(test_ddc_motion_primitive test_ddc_motion_primitive.cpp)
target_compile_definitions(test_ddc_motion_primitive PRIVATE
        -DDDC_MOTION_PRIMITIVES_YAML_PATH="${CMAKE_CURRENT_LIST_DIR}/ddc_motion_primitives.yaml")
gtest_discover_tests(test_ddc_motion_primitive)

# test differential drive model
add_executable(test_differential_drive_model test_differential_drive_model.cpp)
gtest_discover_tests(test_differential_drive_model)

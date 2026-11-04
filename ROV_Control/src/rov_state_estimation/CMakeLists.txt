cmake_minimum_required(VERSION 3.16)
project(rov_state_estimation VERSION 0.1.0 LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies rclcpp rcl_interfaces geometry_msgs sensor_msgs std_srvs diagnostic_msgs
  tf2_ros ustrov_state_estimation)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(rov_hippo_fusion SHARED src/frame_adapter.cpp src/hippo_fusion.cpp src/fusion_node.cpp)
target_compile_features(rov_hippo_fusion PUBLIC cxx_std_17)
target_include_directories(rov_hippo_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rov_hippo_fusion PUBLIC ${dependencies})
target_link_libraries(rov_hippo_fusion PUBLIC Eigen3::Eigen)
# 与上游 Eigen 编译定义保持一致；几何函数本身没有 ROS 调用。
target_compile_definitions(rov_hippo_fusion PUBLIC
  EIGEN_MATRIXBASE_PLUGIN=<ustrov_state_estimation/matrix_base_addon.h>)
target_compile_options(rov_hippo_fusion PRIVATE -Wall -Wextra -Wpedantic)
add_executable(hippo_fusion_node src/fusion_node_main.cpp)
target_link_libraries(hippo_fusion_node PRIVATE rov_hippo_fusion)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hippo_fusion test/test_hippo_fusion.cpp TIMEOUT 90)
  target_link_libraries(test_hippo_fusion rov_hippo_fusion)
  ament_add_gtest(test_fusion_ros test/test_fusion_ros.cpp TIMEOUT 90)
  target_link_libraries(test_fusion_ros rov_hippo_fusion)
  set_tests_properties(test_fusion_ros PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=174;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
install(TARGETS rov_hippo_fusion hippo_fusion_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()

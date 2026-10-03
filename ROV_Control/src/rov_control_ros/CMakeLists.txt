cmake_minimum_required(VERSION 3.16)
project(rov_control_ros VERSION 0.1.0 LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rov_control_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 库保存适配/节点实现，main 仅负责生命周期；测试直接链接同一份实现。
add_library(preview_support src/state_adapter.cpp src/trajectory_preview_node.cpp)
target_compile_features(preview_support PUBLIC cxx_std_17)
set_target_properties(preview_support PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(preview_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(preview_support PUBLIC rov_control_core::rov_control_core)
ament_target_dependencies(preview_support PUBLIC rclcpp rcl_interfaces nav_msgs geometry_msgs diagnostic_msgs std_srvs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(preview_support PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
add_executable(trajectory_preview_node src/preview_main.cpp)
target_link_libraries(trajectory_preview_node PRIVATE preview_support)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  foreach(test_name state_adapter preview_math)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} PRIVATE preview_support GTest::gtest_main)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}")
    add_test(NAME ${test_name} COMMAND test_${test_name}
      "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/${test_name}.xml")
  endforeach()
endif()

install(TARGETS preview_support trajectory_preview_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()

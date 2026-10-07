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
find_package(ustrov_control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

# C1 复用已有状态适配；不把输入试验逻辑放进只读预览节点。
add_library(actuation_support src/hippo_command.cpp src/actuation_probe_node.cpp)
target_link_libraries(actuation_support PUBLIC preview_support)
ament_target_dependencies(actuation_support PUBLIC ustrov_control_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(actuation_support PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
add_executable(actuation_probe_node src/probe_main.cpp)
target_link_libraries(actuation_probe_node PRIVATE actuation_support)

# 水平到点使用独立节点，复用状态适配与显示消息计算，不运行预览节点。
add_library(point_tracking_support src/point_tracking_node.cpp src/point_tracking_parameters.cpp)
target_link_libraries(point_tracking_support PUBLIC preview_support)
ament_target_dependencies(point_tracking_support PUBLIC ustrov_control_msgs tf2_ros)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(point_tracking_support PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
add_executable(point_tracking_node src/tracking_main.cpp)
target_link_libraries(point_tracking_node PRIVATE point_tracking_support)

# 硬件工具显式开启，已有纯仿真工作区不必为了预览而安装 PX4 消息。
option(ROV_BUILD_PX4_MOTOR_PROBE "Build the PX4 individual motor probe" OFF)
if(ROV_BUILD_PX4_MOTOR_PROBE)
  find_package(px4_msgs REQUIRED)
  add_library(motor_probe_support src/motor_probe_node.cpp)
  target_compile_features(motor_probe_support PUBLIC cxx_std_17)
  set_target_properties(motor_probe_support PROPERTIES CXX_EXTENSIONS OFF)
  target_include_directories(motor_probe_support PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(motor_probe_support PUBLIC
    rclcpp rcl_interfaces diagnostic_msgs std_srvs ustrov_control_msgs px4_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motor_probe_support PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  endif()
  add_executable(motor_probe_node src/motor_probe_main.cpp)
  target_link_libraries(motor_probe_node PRIVATE motor_probe_support)
  install(TARGETS motor_probe_support motor_probe_node
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

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
  find_package(rosgraph_msgs REQUIRED)
  foreach(test_name hippo_command actuation_probe)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} PRIVATE actuation_support GTest::gtest_main)
    ament_target_dependencies(test_${test_name} PUBLIC rosgraph_msgs)
    add_test(NAME ${test_name} COMMAND test_${test_name}
      "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/${test_name}.xml")
  endforeach()
  set_tests_properties(actuation_probe PROPERTIES TIMEOUT 90)
  add_executable(test_point_tracking test/test_point_tracking.cpp)
  target_link_libraries(test_point_tracking PRIVATE point_tracking_support GTest::gtest_main)
  ament_target_dependencies(test_point_tracking PUBLIC rosgraph_msgs)
  add_test(NAME point_tracking COMMAND test_point_tracking
    "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/point_tracking.xml")
  set_tests_properties(point_tracking PROPERTIES TIMEOUT 90)
  if(ROV_BUILD_PX4_MOTOR_PROBE)
    # 测试使用实际 USTROV bridge 可执行程序 + C++ 模拟 PX4；不连接串口/Agent。
    find_package(ustrov_core REQUIRED)
    find_program(PX4_BRIDGE_EXECUTABLE px4_actuator_bridge
      HINTS "${ustrov_core_DIR}/../../../lib/ustrov_core" REQUIRED)
    add_executable(test_motor_probe test/test_motor_probe.cpp)
    target_link_libraries(test_motor_probe PRIVATE motor_probe_support GTest::gtest_main)
    target_compile_definitions(test_motor_probe PRIVATE PX4_BRIDGE_EXECUTABLE="${PX4_BRIDGE_EXECUTABLE}")
    add_test(NAME motor_probe COMMAND test_motor_probe
      "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/motor_probe.xml")
    set_tests_properties(motor_probe PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE
      ENVIRONMENT "ROS_DOMAIN_ID=124;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()
endif()

install(TARGETS preview_support trajectory_preview_node actuation_support actuation_probe_node
  point_tracking_support point_tracking_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()

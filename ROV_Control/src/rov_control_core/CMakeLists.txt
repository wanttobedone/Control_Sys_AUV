cmake_minimum_required(VERSION 3.16)
project(rov_control_core VERSION 0.1.0 LANGUAGES CXX)

# 数学核心只有 C++/Eigen 依赖，不包含 ROS 消息和节点。
# 默认 ON：由 colcon 调用，通过 ament 安装并导出给后续 ROS 包使用。
# 设为 OFF：直接用 cmake 编译同一份源码，便于独立理解、测试算法。
option(ROV_CORE_WITH_AMENT "Package the library with ROS 2 ament" ON)
if(ROV_CORE_WITH_AMENT)
  find_package(ament_cmake REQUIRED)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 三个实现文件编译成一个库；头文件中的类声明供调用方包含。
# 带命名空间的别名使依赖关系明确：调用方链接 rov_control_core::rov_control_core。
add_library(${PROJECT_NAME}
  src/planning/workspace_bounds.cpp
  src/planning/line_path.cpp
  src/planning/quintic_trajectory.cpp
)
add_library(rov_control_core::rov_control_core ALIAS ${PROJECT_NAME})
# std::optional、std::clamp 等需要 C++17；不使用编译器私有语言扩展。
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
# 构建阶段从源码 include/ 找头文件，安装后从安装目录的 include/ 查找。
# PUBLIC 表示库自身与链接本库的调用方都需要这些头文件和 Eigen。
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # 提示隐式转换、变量遮蔽等问题，避免数学代码中难以察觉的类型错误。
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

# CTest 负责运行测试程序；GoogleTest 负责程序内部的 21 个测试用例。
# 因此 colcon 汇总可能显示 22 项：1 个 CTest 项加 21 个 GoogleTest 用例。
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_path_trajectory test/test_path_trajectory.cpp)
  target_link_libraries(test_path_trajectory PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_path_trajectory PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  # 保存 XML，供 colcon test-result 展示每个用例的通过/失败结果。
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}")
  add_test(NAME path_trajectory COMMAND test_path_trajectory
    "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/path_trajectory.xml")
endif()

if(ROV_CORE_WITH_AMENT)
  # 导出库、头文件及 Eigen 依赖，后续包 find_package(rov_control_core) 后即可链接。
  install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()

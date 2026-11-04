cmake_minimum_required(VERSION 3.16)
project(rov_localization VERSION 0.3.0 LANGUAGES CXX)

# 与 rov_control_core 一样：colcon 默认注册 ament；独立标定时可关闭 ROS 包装。
option(ROV_LOCALIZATION_WITH_AMENT "Register this library as a ROS 2 package" ON)
if(ROV_LOCALIZATION_WITH_AMENT)
  find_package(ament_cmake REQUIRED)
endif()
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d highgui)
# Ubuntu 24.04的3.3.0 Debian包导出的CMake目标可能含/usr/lib/lib/...错误路径。
# 优先使用同一个包的pkg-config信息；用户从源码安装且没有.pc时再走CONFIG。
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(APRILTAG QUIET IMPORTED_TARGET apriltag>=3)
endif()
if(TARGET PkgConfig::APRILTAG)
  set(ROV_APRILTAG_TARGET PkgConfig::APRILTAG)
  set(ROV_APRILTAG_VERSION ${APRILTAG_VERSION})
else()
  find_package(apriltag 3 REQUIRED CONFIG)
  set(ROV_APRILTAG_TARGET apriltag::apriltag)
  set(ROV_APRILTAG_VERSION ${apriltag_VERSION})
endif()
find_path(ROV_APRILTAG_INCLUDE_DIR apriltag.h HINTS ${APRILTAG_INCLUDE_DIRS}
  PATH_SUFFIXES apriltag)
if(NOT ROV_APRILTAG_INCLUDE_DIR)
  message(FATAL_ERROR "AprilTag headers missing: install libapriltag-dev")
endif()

add_library(${PROJECT_NAME} SHARED src/apriltag_detector.cpp src/camera_calibrator.cpp
  src/tag_pose_estimator.cpp)
add_library(rov_localization::rov_localization ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
# 第三方C头允许柔性数组等用法；只把该库头标成SYSTEM，自研代码仍开启严格警告。
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${ROV_APRILTAG_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC opencv_core opencv_imgproc opencv_videoio opencv_calib3d
  PRIVATE ${ROV_APRILTAG_TARGET})
target_compile_definitions(${PROJECT_NAME} PRIVATE ROV_APRILTAG_VERSION="${ROV_APRILTAG_VERSION}"
  ROV_LOCALIZATION_VERSION="${PROJECT_VERSION}")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()
add_executable(calibrate_camera src/calibrate_camera_main.cpp)
target_link_libraries(calibrate_camera PRIVATE ${PROJECT_NAME})
add_executable(tag_pose_preview src/tag_pose_preview_main.cpp)
target_link_libraries(tag_pose_preview PRIVATE ${PROJECT_NAME} opencv_highgui)
target_compile_definitions(tag_pose_preview PRIVATE ROV_LOCALIZATION_VERSION="${PROJECT_VERSION}")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tag_pose_preview PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

include(CTest)
# ROS包装独立成库，离线数学库和两个旧CLI不链接rclcpp。
if(ROV_LOCALIZATION_WITH_AMENT)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  add_library(tag_pose_ros SHARED src/tag_pose_node.cpp)
  target_link_libraries(tag_pose_ros PUBLIC ${PROJECT_NAME} opencv_imgcodecs)
  ament_target_dependencies(tag_pose_ros PUBLIC rclcpp rcl_interfaces std_msgs sensor_msgs
    geometry_msgs diagnostic_msgs tf2 tf2_ros)
  add_executable(tag_pose_node src/tag_pose_node_main.cpp)
  target_link_libraries(tag_pose_node PRIVATE tag_pose_ros)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(tag_pose_ros PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  endif()
  install(TARGETS tag_pose_ros tag_pose_node
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(test_camera_calibration test/test_camera_calibration.cpp)
  target_link_libraries(test_camera_calibration PRIVATE ${PROJECT_NAME} GTest::gtest_main
    ${ROV_APRILTAG_TARGET})
  target_include_directories(test_camera_calibration PRIVATE ${ROV_APRILTAG_INCLUDE_DIR})
  # Debian把图案生成后的image_u8_destroy放在独立utils库；源码安装通常在主库内。
  find_library(ROV_APRILTAG_UTILS_LIBRARY NAMES apriltag-utils)
  if(ROV_APRILTAG_UTILS_LIBRARY)
    target_link_libraries(test_camera_calibration PRIVATE ${ROV_APRILTAG_UTILS_LIBRARY})
  endif()
  target_compile_definitions(test_camera_calibration PRIVATE
    ROV_CALIBRATION_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/calibration_options.yaml")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}")
  add_test(NAME camera_calibration COMMAND test_camera_calibration
    "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/camera_calibration.xml")
  set_tests_properties(camera_calibration PROPERTIES TIMEOUT 120)

  add_executable(test_tag_pose test/test_tag_pose.cpp)
  target_link_libraries(test_tag_pose PRIVATE ${PROJECT_NAME} GTest::gtest_main ${ROV_APRILTAG_TARGET})
  target_include_directories(test_tag_pose PRIVATE ${ROV_APRILTAG_INCLUDE_DIR})
  if(ROV_APRILTAG_UTILS_LIBRARY)
    target_link_libraries(test_tag_pose PRIVATE ${ROV_APRILTAG_UTILS_LIBRARY})
  endif()
  target_compile_definitions(test_tag_pose PRIVATE
    ROV_TAG_POSE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/tag_pose_options.yaml"
    ROV_TAG_POSE_EXECUTABLE="$<TARGET_FILE:tag_pose_preview>")
  add_dependencies(test_tag_pose tag_pose_preview)
  add_test(NAME tag_pose COMMAND test_tag_pose
    "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/tag_pose.xml")
  set_tests_properties(tag_pose PROPERTIES TIMEOUT 60)

  if(ROV_LOCALIZATION_WITH_AMENT)
    find_package(tf2_msgs REQUIRED)
    add_executable(test_tag_pose_ros test/test_tag_pose_ros.cpp)
    target_link_libraries(test_tag_pose_ros PRIVATE tag_pose_ros GTest::gtest_main
      ${ROV_APRILTAG_TARGET})
    ament_target_dependencies(test_tag_pose_ros PUBLIC tf2_msgs)
    target_include_directories(test_tag_pose_ros PRIVATE ${ROV_APRILTAG_INCLUDE_DIR})
    if(ROV_APRILTAG_UTILS_LIBRARY)
      target_link_libraries(test_tag_pose_ros PRIVATE ${ROV_APRILTAG_UTILS_LIBRARY})
    endif()
    target_compile_definitions(test_tag_pose_ros PRIVATE
      ROV_TAG_POSE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/tag_pose_options.yaml")
    add_test(NAME tag_pose_ros COMMAND test_tag_pose_ros
      "--gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/tag_pose_ros.xml")
    # 软件测试与真实DDS域隔离；夹具只发布合成JPEG，不含执行器代码。
    set_tests_properties(tag_pose_ros PROPERTIES TIMEOUT 90
      ENVIRONMENT "ROS_DOMAIN_ID=173;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()
endif()

install(TARGETS ${PROJECT_NAME} calibrate_camera tag_pose_preview EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(ROV_LOCALIZATION_WITH_AMENT)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(OpenCV)
  ament_package()
endif()

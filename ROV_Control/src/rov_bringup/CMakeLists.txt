cmake_minimum_required(VERSION 3.16)
project(rov_bringup VERSION 0.1.0 LANGUAGES NONE)
find_package(ament_cmake REQUIRED)
# 本包没有运行代码，只安装启动入口、参数与显示配置。
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()

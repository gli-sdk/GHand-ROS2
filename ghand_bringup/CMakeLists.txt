cmake_minimum_required(VERSION 3.8)
project(ghand_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(ghand_bringup_launch)


install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/wait_for_control DESTINATION lib/${PROJECT_NAME})

ament_package()

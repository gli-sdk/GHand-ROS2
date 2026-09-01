cmake_minimum_required(VERSION 3.8)
project(ghand_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  ghand_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ghand_controllers SHARED
  src/joint_status_broadcaster.cpp
  src/tactile_broadcaster.cpp
)
set_target_properties(ghand_controllers PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../ghand_msgs/lib"
)
target_include_directories(ghand_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ghand_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rosidl_typesupport_cpp normally dlopens the selected RMW implementation via
# LD_LIBRARY_PATH. Processes with network capabilities run in secure-loader
# mode, so keep the Fast DDS implementation as an explicit dependency instead.
target_link_options(ghand_controllers PRIVATE "LINKER:--no-as-needed")
target_link_libraries(ghand_controllers PRIVATE
  ghand_msgs::ghand_msgs__rosidl_typesupport_fastrtps_cpp
)

pluginlib_export_plugin_description_file(controller_interface ghand_controllers.xml)

install(TARGETS ghand_controllers
  EXPORT export_ghand_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ghand_controllers DESTINATION include)

ament_export_targets(export_ghand_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

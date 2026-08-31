cmake_minimum_required(VERSION 3.8)
project(hand_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hand_msgs
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

add_library(hand_controllers SHARED
  src/joint_status_broadcaster.cpp
  src/tactile_broadcaster.cpp
)
set_target_properties(hand_controllers PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../hand_msgs/lib"
)
target_include_directories(hand_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hand_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rosidl_typesupport_cpp normally dlopens the selected RMW implementation via
# LD_LIBRARY_PATH. Processes with network capabilities run in secure-loader
# mode, so keep the Fast DDS implementation as an explicit dependency instead.
target_link_options(hand_controllers PRIVATE "LINKER:--no-as-needed")
target_link_libraries(hand_controllers PRIVATE
  hand_msgs::hand_msgs__rosidl_typesupport_fastrtps_cpp
)

pluginlib_export_plugin_description_file(controller_interface hand_controllers.xml)

install(TARGETS hand_controllers
  EXPORT export_hand_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/hand_controllers DESTINATION include)

ament_export_targets(export_hand_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

cmake_minimum_required(VERSION 3.8)
project(hand_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hand_msgs
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
target_include_directories(hand_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hand_controllers>
)
ament_target_dependencies(hand_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface hand_controllers.xml)

install(TARGETS hand_controllers
  EXPORT export_hand_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/hand_controllers)

ament_export_targets(export_hand_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gtest(test_joint_status_broadcaster test/test_joint_status_broadcaster.cpp)
  target_include_directories(test_joint_status_broadcaster PRIVATE include)
  ament_target_dependencies(test_joint_status_broadcaster
    controller_interface
    hand_msgs
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_joint_status_broadcaster hand_controllers)

  ament_add_gtest(test_tactile_broadcaster test/test_tactile_broadcaster.cpp)
  target_include_directories(test_tactile_broadcaster PRIVATE include)
  ament_target_dependencies(test_tactile_broadcaster
    controller_interface
    hand_msgs
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_tactile_broadcaster hand_controllers)
endif()

ament_package()

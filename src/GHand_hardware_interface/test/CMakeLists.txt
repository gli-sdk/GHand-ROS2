ament_add_gtest(test_communication
  test_communication.cpp
  mock_communication.cpp
)

target_include_directories(test_communication PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(test_communication GHand_hardware_interface)

ament_target_dependencies(test_communication
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

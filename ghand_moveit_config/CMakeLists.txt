cmake_minimum_required(VERSION 3.8)
project(ghand_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_generated_config_dir "${CMAKE_CURRENT_BINARY_DIR}/config")
file(MAKE_DIRECTORY "${_generated_config_dir}")
execute_process(
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dual_moveit_config.py"
          --left-srdf "${CMAKE_CURRENT_SOURCE_DIR}/config/ghand5_left.srdf"
          --right-srdf "${CMAKE_CURRENT_SOURCE_DIR}/config/ghand5_right.srdf"
          --srdf-output "${_generated_config_dir}/ghand5_dual.srdf"
          --joint-limits-output "${_generated_config_dir}/dual_joint_limits.yaml"
          --kinematics-output "${_generated_config_dir}/dual_kinematics.yaml"
  RESULT_VARIABLE _dual_moveit_generation_result
)
if(NOT _dual_moveit_generation_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate the dual-hand MoveIt configuration")
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${_generated_config_dir}/ghand5_dual.srdf"
  "${_generated_config_dir}/dual_joint_limits.yaml"
  "${_generated_config_dir}/dual_kinematics.yaml"
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

cmake_minimum_required(VERSION 3.8)
project(ghand_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(ghand_moveit_launch)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

file(GLOB _product_config_dirs LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/config/*")
foreach(_product_config_dir IN LISTS _product_config_dirs)
  if(NOT IS_DIRECTORY "${_product_config_dir}")
    continue()
  endif()
  get_filename_component(_product_name "${_product_config_dir}" NAME)
  if(_product_name STREQUAL "common")
    continue()
  endif()
  set(_generated_config_dir "${CMAKE_CURRENT_BINARY_DIR}/config/${_product_name}")
  file(MAKE_DIRECTORY "${_generated_config_dir}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    scripts/generate_dual_moveit_config.py
    "${_product_config_dir}/${_product_name}_left.srdf"
    "${_product_config_dir}/${_product_name}_right.srdf"
    "${_product_config_dir}/joint_limits.yaml" "${_product_config_dir}/kinematics.yaml")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dual_moveit_config.py"
            --left-srdf "${_product_config_dir}/${_product_name}_left.srdf"
            --right-srdf "${_product_config_dir}/${_product_name}_right.srdf"
            --joint-limits "${_product_config_dir}/joint_limits.yaml"
            --kinematics "${_product_config_dir}/kinematics.yaml"
            --srdf-output "${_generated_config_dir}/${_product_name}_dual.srdf"
            --joint-limits-output "${_generated_config_dir}/dual_joint_limits.yaml"
            --kinematics-output "${_generated_config_dir}/dual_kinematics.yaml"
    RESULT_VARIABLE _dual_moveit_generation_result
  )
  if(NOT _dual_moveit_generation_result EQUAL 0)
    message(FATAL_ERROR "Failed to generate the ${_product_name} dual-hand MoveIt configuration")
  endif()

  install(FILES
    "${_generated_config_dir}/${_product_name}_dual.srdf"
    "${_generated_config_dir}/dual_joint_limits.yaml"
    "${_generated_config_dir}/dual_kinematics.yaml"
    DESTINATION share/${PROJECT_NAME}/config/${_product_name}
  )
endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

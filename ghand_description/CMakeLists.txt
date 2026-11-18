cmake_minimum_required(VERSION 3.8)
project(ghand_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# --------------------------------------------------------------------------
# Source assets
#
# The URDF and meshes live in the ghand_assets repository, cloned as the
# sibling ghand_assets checkout under the workspace src directory.
# ghand_assets is a raw asset repository (no package.xml), so this package
# turns the G5 left- and right-hand descriptions into a standard ROS 2
# description package: it installs the meshes under share/ and rewrites the URDF's bare
# mesh paths into package:// URIs that RViz / Gazebo / robot_state_publisher
# can resolve.
# --------------------------------------------------------------------------
set(GHAND_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ghand_assets/ghand5_system"
    CACHE PATH "Path to the ghand5_system assets directory")

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5")

foreach(_hand_side IN ITEMS left right)
  set(_source_urdf "${GHAND_ASSETS_DIR}/urdf/ghand5_${_hand_side}.urdf")
  if(NOT EXISTS "${_source_urdf}")
    message(FATAL_ERROR
      "GHand ${_hand_side}-hand assets not found at:\n"
      "  ${_source_urdf}\n"
      "Clone ghand_assets from Gitee or GitHub into the workspace src "
      "directory as ghand_assets, or pass "
      "-DGHAND_ASSETS_DIR=<path>.")
  endif()

  set(_g5_preparation_script "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_ghand5_urdf.py")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
               "${_source_urdf}" "${_g5_preparation_script}")
  set(_output_urdf
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5/ghand5_${_hand_side}_description.urdf")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${_g5_preparation_script}"
            "${_source_urdf}" "${_output_urdf}"
    RESULT_VARIABLE _g5_result
  )
  if(NOT _g5_result EQUAL 0)
    message(FATAL_ERROR "Failed to prepare GHand5 ${_hand_side} description")
  endif()
  install(FILES "${_output_urdf}" DESTINATION share/${PROJECT_NAME}/urdf/ghand5)
endforeach()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  scripts/generate_dual_urdf.py scripts/prepare_lite1_urdf.py config/ghandlite1.yaml)
# Generate a single robot description containing both hands. Link and joint
# names are prefixed with left_ / right_, and each palm is mounted to a shared
# world link so TF and ros2_control resources cannot collide.
set(_dual_urdf "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5/ghand5_dual_description.urdf")
execute_process(
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dual_urdf.py"
          --left "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5/ghand5_left_description.urdf"
          --right "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5/ghand5_right_description.urdf"
          --output "${_dual_urdf}"
  RESULT_VARIABLE _dual_generation_result
)
if(NOT _dual_generation_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate the dual-hand URDF")
endif()
install(FILES "${_dual_urdf}" DESTINATION share/${PROJECT_NAME}/urdf/ghand5)

# Collision geometry reuses the visual STL files, so install both hands.
install(DIRECTORY "${GHAND_ASSETS_DIR}/meshes/hands/visual/"
        DESTINATION share/${PROJECT_NAME}/meshes/ghand5/hands/visual)

set(GHAND_LITE1_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ghand_assets/ghand_lite1_system"
    CACHE PATH "Path to the ghand_lite1_system assets directory")
set(_lite1_urdf_dir "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghandlite1")
file(MAKE_DIRECTORY "${_lite1_urdf_dir}")
foreach(_hand_side IN ITEMS left right)
  set(_source_urdf "${GHAND_LITE1_ASSETS_DIR}/urdf/glite1_${_hand_side}.urdf")
  if(NOT EXISTS "${_source_urdf}")
    message(FATAL_ERROR "Lite1 assets missing at ${_source_urdf}. Add the product asset bundle under ghand_assets/ghand_lite1_system or set GHAND_LITE1_ASSETS_DIR.")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_source_urdf}")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_lite1_urdf.py"
            --source "${_source_urdf}"
            --config "${CMAKE_CURRENT_SOURCE_DIR}/config/ghandlite1.yaml"
            --side "${_hand_side}"
            --output "${_lite1_urdf_dir}/ghandlite1_${_hand_side}_description.urdf"
    RESULT_VARIABLE _lite1_result
  )
  if(NOT _lite1_result EQUAL 0)
    message(FATAL_ERROR "Failed to prepare Lite1 ${_hand_side} description")
  endif()
endforeach()
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dual_urdf.py"
          --left "${_lite1_urdf_dir}/ghandlite1_left_description.urdf"
          --right "${_lite1_urdf_dir}/ghandlite1_right_description.urdf"
          --product-name ghandlite1
          --output "${_lite1_urdf_dir}/ghandlite1_dual_description.urdf"
  RESULT_VARIABLE _lite1_result
)
if(NOT _lite1_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate Lite1 dual description")
endif()
install(DIRECTORY "${_lite1_urdf_dir}/" DESTINATION share/${PROJECT_NAME}/urdf/ghandlite1)
install(DIRECTORY "${GHAND_LITE1_ASSETS_DIR}/meshes/hands/visual/"
        DESTINATION share/${PROJECT_NAME}/meshes/ghandlite1/hands/visual)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ghand_description)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------
# Source assets
#
# The URDF and meshes live in the ghand_assets repository, which is fetched
# via `vcs import . < ghand-ros2/ghand.repos` into the sibling
# ghand_assets checkout under the workspace src directory.
# ghand_assets is a raw asset repository (no package.xml), so this package
# turns the G5 left- and right-hand descriptions into a standard ROS 2
# description package: it installs the meshes under share/ and rewrites the URDF's bare
# mesh paths into package:// URIs that RViz / Gazebo / robot_state_publisher
# can resolve.
# --------------------------------------------------------------------------
set(GHAND_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ghand_assets/ghand5_system"
    CACHE PATH "Path to the ghand5_system assets directory")

foreach(_hand_side IN ITEMS left right)
  set(_source_urdf "${GHAND_ASSETS_DIR}/urdf/ghand5_${_hand_side}.urdf")
  if(NOT EXISTS "${_source_urdf}")
    message(FATAL_ERROR
      "GHand ${_hand_side}-hand assets not found at:\n"
      "  ${_source_urdf}\n"
      "Fetch them with `vcs import . < ghand-ros2/ghand.repos` from the "
      "workspace src directory before building, or pass -DGHAND_ASSETS_DIR=<path>.")
  endif()

  # Rewrite bare mesh paths (ghand5_system/meshes/...) into package:// URIs.
  file(READ "${_source_urdf}" _urdf_contents)
  string(REPLACE
    "ghand5_system/meshes/"
    "package://${PROJECT_NAME}/meshes/"
    _urdf_contents "${_urdf_contents}")

  # Gazebo cannot drive joints whose source effort/velocity limits are zero.
  string(REGEX REPLACE "effort=\"0\"" "effort=\"50\"" _urdf_contents "${_urdf_contents}")
  string(REGEX REPLACE "velocity=\"0\"" "velocity=\"50\"" _urdf_contents "${_urdf_contents}")

  # Add damping/friction to suppress high-gain oscillations in Gazebo.
  string(REGEX REPLACE "(<limit[^/]*/>)" "\\1\n      <dynamics damping=\"0.1\" friction=\"0.05\"/>" _urdf_contents "${_urdf_contents}")
  set(_output_urdf
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/ghand5_${_hand_side}_description.urdf")
  file(WRITE "${_output_urdf}" "${_urdf_contents}")
  install(FILES "${_output_urdf}" DESTINATION share/${PROJECT_NAME}/urdf)
endforeach()

# Collision geometry reuses the visual STL files, so install both hands.
install(DIRECTORY "${GHAND_ASSETS_DIR}/meshes/hands/visual/"
        DESTINATION share/${PROJECT_NAME}/meshes/hands/visual)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

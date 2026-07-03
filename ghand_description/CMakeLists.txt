cmake_minimum_required(VERSION 3.8)
project(ghand_description)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------
# Source assets
#
# The URDF and meshes live in the ghand_assets repository, which is fetched
# via `vcs import . < ghand.repos` into the sibling ghand_assets checkout.
# ghand_assets is a raw asset repository (no package.xml), so this package
# turns the G5 right-hand description into a standard ROS 2 description
# package: it installs the meshes under share/ and rewrites the URDF's bare
# mesh paths into package:// URIs that RViz / Gazebo / robot_state_publisher
# can resolve.
# --------------------------------------------------------------------------
set(GHAND_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ghand_assets/ghand5_system"
    CACHE PATH "Path to the ghand5_system assets directory")

if(NOT EXISTS "${GHAND_ASSETS_DIR}/urdf/G5_right.urdf")
  message(FATAL_ERROR
    "ghand5_system assets not found at:\n"
    "  ${GHAND_ASSETS_DIR}\n"
    "Fetch them with `vcs import . < ghand.repos` from the workspace root "
    "before building, or pass -DGHAND_ASSETS_DIR=<path>.")
endif()

# Rewrite bare mesh paths (ghand5_system/meshes/...) into package:// URIs.
file(READ "${GHAND_ASSETS_DIR}/urdf/G5_right.urdf" _urdf_contents)
string(REPLACE
  "ghand5_system/meshes/"
  "package://${PROJECT_NAME}/meshes/"
  _urdf_contents "${_urdf_contents}")

# The source URDF sets effort="0" velocity="0" on every joint limit. Gazebo
# treats that as zero-force/zero-speed joints and the simulated hand cannot
# move. Replace them with reasonable positive values so gz_ros2_control can
# drive the fingers.
string(REGEX REPLACE "effort=\"0\"" "effort=\"50\"" _urdf_contents "${_urdf_contents}")
string(REGEX REPLACE "velocity=\"0\"" "velocity=\"50\"" _urdf_contents "${_urdf_contents}")

# Add damping/friction to every joint to suppress high-gain oscillations in
# Gazebo. The source URDF has no <dynamics>; without damping the fingers jitter
# when position_proportional_gain is raised enough for trajectory tracking.
string(REGEX REPLACE "(<limit[^/]*/>)" "\\1\n      <dynamics damping=\"0.1\" friction=\"0.05\"/>" _urdf_contents "${_urdf_contents}")
file(WRITE
  "${CMAKE_CURRENT_BINARY_DIR}/urdf/G5_right_description.urdf"
  "${_urdf_contents}")

# Install the rewritten URDF under the filename the consuming packages expect.
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/G5_right_description.urdf"
        DESTINATION share/${PROJECT_NAME}/urdf)

# Install the right-hand visual meshes. Collision geometry reuses the same
# STL files, so only the visual/right subtree is needed.
install(DIRECTORY "${GHAND_ASSETS_DIR}/meshes/hands/visual/right"
        DESTINATION share/${PROJECT_NAME}/meshes/hands/visual)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

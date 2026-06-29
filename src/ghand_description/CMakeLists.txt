cmake_minimum_required(VERSION 3.8)
project(ghand_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Hand system asset subdirectory in the external ghand_assets repository.
set(HAND_SYSTEM "ghand5_system" CACHE STRING "Hand system asset subdirectory in ghand_assets")
set(GHAND_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ghand_assets/${HAND_SYSTEM}")

if(NOT EXISTS ${GHAND_ASSETS_DIR})
  message(FATAL_ERROR
    "ghand_assets directory not found: ${GHAND_ASSETS_DIR}\n"
    "Please run: vcs import src < ghand.repos")
endif()

# Install ROS wrapper xacro files from source.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# Copy and patch the assets URDF so it can be xacro-included by the wrapper.
# ghand_assets stores mesh paths relative to its own tree; rewrite them to
# package://ghand_description/meshes so ROS resource resolution works. The
# robot wrapper is stripped so the file can be included inside G5_right.urdf.
set(ASSETS_URDF_SRC "${GHAND_ASSETS_DIR}/urdf/G5_right.urdf")
set(PATCHED_URDF_DIR "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/urdf")
set(PATCHED_URDF "${PATCHED_URDF_DIR}/G5_right_description.urdf")

if(NOT EXISTS ${ASSETS_URDF_SRC})
  message(FATAL_ERROR
    "Assets URDF not found: ${ASSETS_URDF_SRC}\n"
    "Please run: vcs import src < ghand.repos")
endif()

file(MAKE_DIRECTORY ${PATCHED_URDF_DIR})
file(READ ${ASSETS_URDF_SRC} ASSETS_URDF_CONTENT)
string(REPLACE
  "ghand5_system/meshes/hands/visual/right/"
  "package://ghand_description/meshes/"
  ASSETS_URDF_CONTENT ${ASSETS_URDF_CONTENT})
string(REPLACE "<?xml version='1.0' encoding='utf-8'?>" "" ASSETS_URDF_CONTENT ${ASSETS_URDF_CONTENT})
string(REPLACE "<robot name=\"G5-right\">" "" ASSETS_URDF_CONTENT ${ASSETS_URDF_CONTENT})
string(REPLACE "</robot>" "" ASSETS_URDF_CONTENT ${ASSETS_URDF_CONTENT})
file(WRITE ${PATCHED_URDF} ${ASSETS_URDF_CONTENT})

install(FILES ${PATCHED_URDF} DESTINATION share/${PROJECT_NAME}/urdf)

# Copy right-hand meshes into a flat meshes directory to match the patched URDF.
set(ASSETS_MESHES_DIR "${GHAND_ASSETS_DIR}/meshes/hands/visual/right")
set(PATCHED_MESHES_DIR "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/meshes")

if(EXISTS ${ASSETS_MESHES_DIR})
  file(COPY ${ASSETS_MESHES_DIR}/ DESTINATION ${PATCHED_MESHES_DIR})
  install(DIRECTORY ${PATCHED_MESHES_DIR}/ DESTINATION share/${PROJECT_NAME}/meshes)
endif()

ament_package()

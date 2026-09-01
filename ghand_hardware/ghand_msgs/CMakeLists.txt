cmake_minimum_required(VERSION 3.8)
project(ghand_msgs)

# Keep generated message libraries discoverable when a process runs with Linux
# capabilities. In that mode the dynamic loader ignores LD_LIBRARY_PATH. The
# first entry covers libraries installed together under lib; the second lets
# generated Python extension modules find that lib directory.
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../../..")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/JointStatus.msg
  msg/TactileData.msg
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()

cmake_minimum_required(VERSION 3.8)
project(ghand_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    worlds
    scripts
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
foreach(_product IN ITEMS ghand5 ghandlite1)
  set(_dual "${CMAKE_CURRENT_BINARY_DIR}/${_product}/dual_controllers.yaml")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${_product}")
  add_custom_command(OUTPUT "${_dual}"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_dual_controllers.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/config/${_product}/ghand_gazebo_controllers.yaml" "${_dual}"
    DEPENDS scripts/generate_dual_controllers.py "config/${_product}/ghand_gazebo_controllers.yaml"
    VERBATIM)
  add_custom_target(${_product}_dual_controllers ALL DEPENDS "${_dual}")
  install(FILES "${_dual}" DESTINATION share/${PROJECT_NAME}/config/${_product})
endforeach()
ament_package()

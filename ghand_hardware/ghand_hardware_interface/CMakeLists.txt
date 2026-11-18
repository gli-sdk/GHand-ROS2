cmake_minimum_required(VERSION 3.16)
project(ghand_hardware_interface)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ghand_sdk_vendor REQUIRED)
get_filename_component(_sdk_prefix "${ghand_sdk_vendor_DIR}/../../.." ABSOLUTE)
# Force the bundled installation even if an external SDK was previously cached.
set(GHandSDK_DIR "${_sdk_prefix}/lib/cmake/GHandSDK")
find_package(GHandSDK 3.1 REQUIRED CONFIG PATHS "${GHandSDK_DIR}" NO_DEFAULT_PATH)
set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp rclcpp_lifecycle)
foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

# Product metadata comes from the same SDK installation as the linked library.
set(GHAND_SDK_CONFIG_DIR "${_sdk_prefix}/share/ghand-sdk/config")
foreach(_product IN ITEMS ghand5 ghandlite1)
  if(NOT EXISTS "${GHAND_SDK_CONFIG_DIR}/${_product}.json")
    message(FATAL_ERROR "SDK product metadata missing: ${GHAND_SDK_CONFIG_DIR}/${_product}.json")
  endif()
endforeach()
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/sdk_products.hpp"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_sdk_products.py"
    --config-dir "${GHAND_SDK_CONFIG_DIR}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/sdk_products.hpp"
  DEPENDS scripts/generate_sdk_products.py "${GHAND_SDK_CONFIG_DIR}/ghand5.json"
    "${GHAND_SDK_CONFIG_DIR}/ghandlite1.json"
  VERBATIM)
set(ADAPTER_SOURCES src/ghand_hardware.cpp src/ros2_control_adapter.cpp
  src/sdk_session.cpp src/communication/communication.cpp src/communication/product_config.cpp
  "${CMAKE_CURRENT_BINARY_DIR}/generated/sdk_products.hpp")
add_library(ghand_hardware_interface SHARED ${ADAPTER_SOURCES})
target_include_directories(ghand_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(ghand_hardware_interface PUBLIC GHand::ghand)
set_target_properties(ghand_hardware_interface PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../../ghand_sdk_vendor/lib")
ament_target_dependencies(ghand_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(hardware_interface hand_hardware_interface.xml)
install(TARGETS ghand_hardware_interface EXPORT export_ghand_hardware_interface
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/ghand_joint_controller_gui DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_ghand_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} ghand_sdk_vendor GHandSDK)
ament_package()

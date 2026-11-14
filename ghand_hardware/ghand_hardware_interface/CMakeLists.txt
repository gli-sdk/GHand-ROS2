cmake_minimum_required(VERSION 3.8)
project(ghand_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Optional communication backends.
# GHAND_ENABLE_<BACKEND> can be AUTO (default, detect), ON (require), or OFF (disable).
set(GHAND_ENABLE_ETHERCAT AUTO CACHE STRING "Enable EtherCAT backend (AUTO/ON/OFF)")
set_property(CACHE GHAND_ENABLE_ETHERCAT PROPERTY STRINGS AUTO ON OFF)
set(GHAND_ENABLE_ZLG_CANFD AUTO CACHE STRING "Enable legacy ZLG CANFD backend (AUTO/ON/OFF)")
set_property(CACHE GHAND_ENABLE_ZLG_CANFD PROPERTY STRINGS AUTO ON OFF)

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  if(NOT "${GHAND_ENABLE_ETHERCAT}" STREQUAL "OFF")
    pkg_check_modules(SOEM soem QUIET)
  endif()
  pkg_check_modules(LIBMODBUS libmodbus QUIET)
endif()

# SOEM upstream does not ship a .pc file; fall back to its CMake config.
# SOEM installs soemConfig.cmake to <prefix>/cmake instead of the standard
# <prefix>/lib/cmake/soem, so probe common prefixes explicitly.
if(NOT "${GHAND_ENABLE_ETHERCAT}" STREQUAL "OFF")
  if(NOT SOEM_FOUND)
    set(SOEM_CONFIG_SEARCH_PATHS "")
    set(SOEM_PREFIX_PATHS ${CMAKE_PREFIX_PATH})
    if(DEFINED ENV{CMAKE_PREFIX_PATH})
      string(REPLACE ":" ";" SOEM_ENV_PREFIX_PATHS
        "$ENV{CMAKE_PREFIX_PATH}")
      list(APPEND SOEM_PREFIX_PATHS ${SOEM_ENV_PREFIX_PATHS})
    endif()
    foreach(PREFIX IN LISTS SOEM_PREFIX_PATHS)
      list(APPEND SOEM_CONFIG_SEARCH_PATHS "${PREFIX}/cmake")
    endforeach()
    find_package(soem QUIET PATHS
      ${SOEM_CONFIG_SEARCH_PATHS}
      /usr/local/cmake
      /usr/cmake
      $ENV{HOME}/SOEM/build/install/cmake
      NO_DEFAULT_PATH)
  endif()
  if("${GHAND_ENABLE_ETHERCAT}" STREQUAL "ON" AND NOT SOEM_FOUND AND NOT soem_FOUND)
    message(FATAL_ERROR "GHAND_ENABLE_ETHERCAT=ON but SOEM was not found")
  endif()
endif()

# Locate the workspace vendor directory robustly from the package source location.
get_filename_component(GHAND_WORKSPACE_ROOT "${PROJECT_SOURCE_DIR}/../.." ABSOLUTE)
set(GHAND_VENDOR_DIR "${GHAND_WORKSPACE_ROOT}/vendor" CACHE PATH "Root of vendored third-party libraries")

# Prefer the vendored ZLG USB-CANFD library (VCI API: libusbcanfd + libzuds).
# Falls back to a system libzlgcan.so if present.
set(VENDOR_ZLG_DIR "${GHAND_VENDOR_DIR}/usbcanfd_libusb_x64_1.0.12_251029")
set(VENDOR_ZLG_INCLUDE_DIR "${GHAND_VENDOR_DIR}/dist/include/zlgcan")
if(NOT "${GHAND_ENABLE_ZLG_CANFD}" STREQUAL "OFF")
  find_library(ZLGCAN_LIBRARY
    NAMES usbcanfd libusbcanfd usbcanfd.so.1.0.12 libusbcanfd.so.1.0.12
    HINTS ${VENDOR_ZLG_DIR} /usr/lib /usr/local/lib
    NO_DEFAULT_PATH)
  find_library(ZLGUDS_LIBRARY NAMES zuds libzuds HINTS ${VENDOR_ZLG_DIR} /usr/lib /usr/local/lib NO_DEFAULT_PATH)
  if("${GHAND_ENABLE_ZLG_CANFD}" STREQUAL "ON" AND
    (NOT ZLGCAN_LIBRARY OR NOT ZLGUDS_LIBRARY OR
    NOT EXISTS ${VENDOR_ZLG_INCLUDE_DIR}/zcan.h))
    message(FATAL_ERROR "GHAND_ENABLE_ZLG_CANFD=ON but ZLG CAN library was not found")
  endif()
else()
  set(ZLGCAN_LIBRARY "")
  set(ZLGUDS_LIBRARY "")
endif()

add_library(ghand_hardware_interface SHARED
  src/ghand_hardware.cpp
  src/communication/communication.cpp
  src/communication/modbus_codec.cpp
  src/communication/canfd_protocol.cpp
  src/communication/ethercat_driver.cpp
  src/communication/rs485_driver.cpp
  src/communication/canfd_driver.cpp
  src/communication/zqwl_canfd_driver.cpp
)

target_include_directories(ghand_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if((SOEM_FOUND OR soem_FOUND) AND NOT "${GHAND_ENABLE_ETHERCAT}" STREQUAL "OFF")
  if(SOEM_FOUND)
    target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${SOEM_INCLUDE_DIRS})
    target_link_libraries(ghand_hardware_interface PRIVATE ${SOEM_LIBRARIES})
  elseif(soem_FOUND AND TARGET soem)
    target_link_libraries(ghand_hardware_interface PRIVATE soem)
  endif()
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_SOEM)
  message(STATUS "EtherCAT backend enabled")
else()
  if("${GHAND_ENABLE_ETHERCAT}" STREQUAL "ON")
    message(FATAL_ERROR "GHAND_ENABLE_ETHERCAT=ON but SOEM was not found")
  endif()
  message(WARNING "SOEM not found; EtherCAT backend will be disabled at runtime")
endif()

if(LIBMODBUS_FOUND)
  target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${LIBMODBUS_INCLUDE_DIRS})
  target_link_libraries(ghand_hardware_interface PRIVATE ${LIBMODBUS_LIBRARIES})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_LIBMODBUS)
else()
  message(WARNING "libmodbus not found; RS485 backend will be disabled at runtime")
endif()

if(ZLGCAN_LIBRARY AND ZLGUDS_LIBRARY AND EXISTS ${VENDOR_ZLG_INCLUDE_DIR}/zcan.h)
  target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${VENDOR_ZLG_INCLUDE_DIR})
  target_link_libraries(ghand_hardware_interface PRIVATE ${ZLGCAN_LIBRARY} ${ZLGUDS_LIBRARY})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_ZLG_CAN)
  # At build time use the source vendor directory; after install use a path relative to the plugin.
  install(FILES ${ZLGCAN_LIBRARY} ${ZLGUDS_LIBRARY}
    DESTINATION lib/${PROJECT_NAME})
  set_target_properties(ghand_hardware_interface PROPERTIES
    BUILD_RPATH "${VENDOR_ZLG_DIR}"
    INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}"
    BUILD_WITH_INSTALL_RPATH FALSE
  )
  message(STATUS "Found vendored ZLG CAN library: ${ZLGCAN_LIBRARY}")
elseif(ZLGCAN_LIBRARY)
  target_link_libraries(ghand_hardware_interface PRIVATE ${ZLGCAN_LIBRARY})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_ZLG_CAN)
  message(STATUS "Found system ZLG CAN library: ${ZLGCAN_LIBRARY}")
else()
  if("${GHAND_ENABLE_ZLG_CANFD}" STREQUAL "ON")
    message(FATAL_ERROR "GHAND_ENABLE_ZLG_CANFD=ON but ZLG CAN library was not found")
  endif()
  message(STATUS "ZLG CAN library not found; optional zlg_canfd backend disabled")
endif()

message(STATUS "GHand-Cpp-SDK compatible ZQWL CANFD backend enabled")

ament_target_dependencies(ghand_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface hand_hardware_interface.xml)

install(TARGETS ghand_hardware_interface
  EXPORT export_ghand_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/ghand_joint_controller_gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_ghand_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  add_executable(test_modbus_codec
    test/test_modbus_codec.cpp
    src/communication/modbus_codec.cpp
  )
  target_include_directories(test_modbus_codec PRIVATE include)
  add_test(NAME test_modbus_codec COMMAND test_modbus_codec)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zqwl_canfd test/test_zqwl_canfd.cpp)
  if(TARGET test_zqwl_canfd)
    target_link_libraries(test_zqwl_canfd ghand_hardware_interface util)
    ament_target_dependencies(test_zqwl_canfd rclcpp)
  endif()
endif()
ament_package()

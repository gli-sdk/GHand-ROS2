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

# Optional communication backends
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(SOEM soem QUIET)
  pkg_check_modules(LIBMODBUS libmodbus QUIET)
endif()

# SOEM upstream does not ship a .pc file; fall back to its CMake config.
# SOEM installs soemConfig.cmake to <prefix>/cmake instead of the standard
# <prefix>/lib/cmake/soem, so probe common prefixes explicitly.
if(NOT SOEM_FOUND)
  find_package(soem QUIET PATHS
    ${CMAKE_PREFIX_PATH}/cmake
    /usr/local/cmake
    /usr/cmake
    $ENV{HOME}/SOEM/build/install/cmake
    NO_DEFAULT_PATH)
endif()

# Prefer the vendored ZLG USB-CANFD library (VCI API: libusbcanfd + libzuds).
# Falls back to a system libzlgcan.so if present.
set(VENDOR_ZLG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/usbcanfd_libusb_x64_1.0.12_251029")
find_library(ZLGCAN_LIBRARY NAMES usbcanfd libusbcanfd usbcanfd.so.1.0.12 libusbcanfd.so.1.0.12 HINTS ${VENDOR_ZLG_DIR} /usr/lib /usr/local/lib NO_DEFAULT_PATH)
find_library(ZLGUDS_LIBRARY NAMES zuds libzuds HINTS ${VENDOR_ZLG_DIR} /usr/lib /usr/local/lib NO_DEFAULT_PATH)
set(ZLGCAN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/dist/include/zlgcan")

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

if(SOEM_FOUND)
  target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${SOEM_INCLUDE_DIRS})
  target_link_libraries(ghand_hardware_interface PRIVATE ${SOEM_LIBRARIES})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_SOEM)
elseif(soem_FOUND AND TARGET soem)
  target_link_libraries(ghand_hardware_interface PRIVATE soem)
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_SOEM)
else()
  message(WARNING "SOEM not found; EtherCAT backend will be disabled at runtime")
endif()

if(LIBMODBUS_FOUND)
  target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${LIBMODBUS_INCLUDE_DIRS})
  target_link_libraries(ghand_hardware_interface PRIVATE ${LIBMODBUS_LIBRARIES})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_LIBMODBUS)
else()
  message(WARNING "libmodbus not found; RS485 backend will be disabled at runtime")
endif()

if(ZLGCAN_LIBRARY AND ZLGUDS_LIBRARY AND EXISTS ${ZLGCAN_INCLUDE_DIR}/zcan.h)
  target_include_directories(ghand_hardware_interface SYSTEM PRIVATE ${ZLGCAN_INCLUDE_DIR})
  target_link_libraries(ghand_hardware_interface PRIVATE ${ZLGCAN_LIBRARY} ${ZLGUDS_LIBRARY})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_ZLG_CAN)
  # Embed the vendored library path so the plugin can be loaded without extra LD_LIBRARY_PATH.
  set_target_properties(ghand_hardware_interface PROPERTIES
    BUILD_RPATH "${VENDOR_ZLG_DIR}"
    INSTALL_RPATH "${VENDOR_ZLG_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  message(STATUS "Found vendored ZLG CAN library: ${ZLGCAN_LIBRARY}")
elseif(ZLGCAN_LIBRARY)
  target_link_libraries(ghand_hardware_interface PRIVATE ${ZLGCAN_LIBRARY})
  target_compile_definitions(ghand_hardware_interface PRIVATE GHAND_HAS_ZLG_CAN)
  message(STATUS "Found system ZLG CAN library: ${ZLGCAN_LIBRARY}")
else()
  message(WARNING "ZLG CAN library not found; CANFD backend will be disabled at runtime")
endif()

ament_target_dependencies(ghand_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface hand_hardware_interface.xml)

install(TARGETS ghand_hardware_interface
  EXPORT export_ghand_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_ghand_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

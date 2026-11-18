# Always expose the SDK installed with this package, never a developer checkout.
get_filename_component(_ghand_vendor_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(GHandSDK_DIR "${_ghand_vendor_prefix}/lib/cmake/GHandSDK")
find_package(GHandSDK 3.1 REQUIRED CONFIG PATHS "${GHandSDK_DIR}" NO_DEFAULT_PATH)
unset(_ghand_vendor_prefix)

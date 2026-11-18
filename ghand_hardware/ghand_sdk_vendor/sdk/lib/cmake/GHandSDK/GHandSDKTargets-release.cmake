#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "GHand::ghand" for configuration "Release"
set_property(TARGET GHand::ghand APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(GHand::ghand PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libghand.so.3.1.0"
  IMPORTED_SONAME_RELEASE "libghand.so.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS GHand::ghand )
list(APPEND _IMPORT_CHECK_FILES_FOR_GHand::ghand "${_IMPORT_PREFIX}/lib/libghand.so.3.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

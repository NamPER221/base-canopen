#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "canopen::canopen" for configuration ""
set_property(TARGET canopen::canopen APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(canopen::canopen PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcanopen.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS canopen::canopen )
list(APPEND _IMPORT_CHECK_FILES_FOR_canopen::canopen "${_IMPORT_PREFIX}/lib/libcanopen.a" )

# Import target "canopen::canopen_kinematics" for configuration ""
set_property(TARGET canopen::canopen_kinematics APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(canopen::canopen_kinematics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcanopen_kinematics.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS canopen::canopen_kinematics )
list(APPEND _IMPORT_CHECK_FILES_FOR_canopen::canopen_kinematics "${_IMPORT_PREFIX}/lib/libcanopen_kinematics.a" )

# Import target "canopen::canopen_zlac" for configuration ""
set_property(TARGET canopen::canopen_zlac APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(canopen::canopen_zlac PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcanopen_zlac.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS canopen::canopen_zlac )
list(APPEND _IMPORT_CHECK_FILES_FOR_canopen::canopen_zlac "${_IMPORT_PREFIX}/lib/libcanopen_zlac.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

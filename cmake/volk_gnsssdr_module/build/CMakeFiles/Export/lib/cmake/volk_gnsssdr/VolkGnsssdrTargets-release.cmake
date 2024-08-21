#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "VolkGnsssdr::volk_gnsssdr" for configuration "Release"
set_property(TARGET VolkGnsssdr::volk_gnsssdr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(VolkGnsssdr::volk_gnsssdr PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvolk_gnsssdr.so.0.0.19"
  IMPORTED_SONAME_RELEASE "libvolk_gnsssdr.so.0.0.19"
  )

list(APPEND _IMPORT_CHECK_TARGETS VolkGnsssdr::volk_gnsssdr )
list(APPEND _IMPORT_CHECK_FILES_FOR_VolkGnsssdr::volk_gnsssdr "${_IMPORT_PREFIX}/lib/libvolk_gnsssdr.so.0.0.19" )

# Import target "VolkGnsssdr::volk_gnsssdr_static" for configuration "Release"
set_property(TARGET VolkGnsssdr::volk_gnsssdr_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(VolkGnsssdr::volk_gnsssdr_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvolk_gnsssdr.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS VolkGnsssdr::volk_gnsssdr_static )
list(APPEND _IMPORT_CHECK_FILES_FOR_VolkGnsssdr::volk_gnsssdr_static "${_IMPORT_PREFIX}/lib/libvolk_gnsssdr.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

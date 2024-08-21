# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

get_filename_component(VOLK_GNSSSDR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

if(NOT TARGET VolkGnsssdr::volkgnsssdr)
  include("${VOLK_GNSSSDR_CMAKE_DIR}/VolkGnsssdrTargets.cmake")
endif()

# set VOLKGNSSSDR_FOUND to be set globally, for whether a compatible Volk GNSS-SDR
# was found -- could be a correct enough version or any version depending
# on how find_package was called.
if(NOT TARGET Volkgnsssdr::volkgnsssdr)
  set(VOLKGNSSSDR_FOUND FALSE)
else()
  set(VOLKGNSSSDR_FOUND TRUE)
endif()

# cache whether a compatible VolkGnsssdr was found for
# use anywhere in the calling project
set(VOLKGNSSSDR_FOUND ${VOLKGNSSSDR_FOUND} CACHE BOOL "Whether a compatible Volk GNSS-SDR was found" FORCE)

if(VOLKGNSSSDR_FOUND)
  # use the new target library, regardless of whether new or old style
  # we still need to set a variable with the library name so that there
  # is a variable to reference in the using-project's cmake scripts!
  set(VOLK_GNSSSDR_LIBRARIES Volkgnsssdr::volkgnsssdr CACHE STRING "Volk GNSS-SDR Library" FORCE)

  # INTERFACE_INCLUDE_DIRECTORIES should always be set
  get_target_property(VOLK_GNSSSDR_INCLUDE_DIRS Volkgnsssdr::volkgnsssdr INTERFACE_INCLUDE_DIRECTORIES)
  set(VOLK_GNSSSDR_INCLUDE_DIRS ${VOLK_GNSSSDR_INCLUDE_DIRS} CACHE STRING "Volk GNSS-SDR Include Directories" FORCE)

  # for backward compatibility with old-CMake non-target project finding
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(VOLKGNSSSDR DEFAULT_MSG VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
  mark_as_advanced(VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
endif()

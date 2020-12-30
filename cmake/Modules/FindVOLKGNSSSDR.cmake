# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Volkgnsssdr::volkgnsssdr
#


########################################################################
# Find VOLK (Vector-Optimized Library of Kernels) GNSS-SDR library
########################################################################
if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_VOLK_GNSSSDR QUIET volk_gnsssdr)

if(NOT VOLKGNSSSDR_ROOT)
    set(VOLKGNSSSDR_ROOT_USER_PROVIDED /usr/local)
else()
    set(VOLKGNSSSDR_ROOT_USER_PROVIDED ${VOLKGNSSSDR_ROOT})
endif()
if(DEFINED ENV{VOLKGNSSSDR_ROOT})
    set(VOLKGNSSSDR_ROOT_USER_PROVIDED
        ${VOLKGNSSSDR_ROOT_USER_PROVIDED}
        $ENV{VOLKGNSSSDR_ROOT}
    )
endif()
if(DEFINED ENV{VOLK_GNSSSDR_DIR})
    set(VOLKGNSSSDR_ROOT_USER_PROVIDED
        ${VOLKGNSSSDR_ROOT_USER_PROVIDED}
        $ENV{VOLK_GNSSSDR_DIR}
    )
endif()
if(GNURADIO_INSTALL_PREFIX)
    set(VOLKGNSSSDR_ROOT_USER_PROVIDED
        ${VOLKGNSSSDR_ROOT_USER_PROVIDED}
        ${GNURADIO_INSTALL_PREFIX}
    )
endif()

find_path(VOLK_GNSSSDR_INCLUDE_DIRS
    NAMES volk_gnsssdr/volk_gnsssdr.h
    HINTS ${PC_VOLK_GNSSSDR_INCLUDEDIR}
    PATHS ${VOLKGNSSSDR_ROOT_USER_PROVIDED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(VOLK_GNSSSDR_LIBRARIES
    NAMES volk_gnsssdr
    HINTS ${PC_VOLK_GNSSSDR_LIBDIR}
    PATHS ${VOLKGNSSSDR_ROOT_USER_PROVIDED}/lib
          ${VOLKGNSSSDR_ROOT_USER_PROVIDED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VOLKGNSSSDR DEFAULT_MSG VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
mark_as_advanced(VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)

if(PC_VOLK_GNSSSDR_VERSION)
    set(VOLKGNSSSDR_VERSION ${PC_VOLK_GNSSSDR_VERSION})
endif()

if(VOLKGNSSSDR_FOUND AND VOLKGNSSSDR_VERSION)
    set_package_properties(VOLKGNSSSDR PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels for GNSS-SDR (found: v${VOLKGNSSSDR_VERSION})."
    )
else()
    set_package_properties(VOLKGNSSSDR PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels for GNSS-SDR."
    )
endif()

if(NOT ORC_FOUND)
    find_package(ORC QUIET)
endif()
if(ORC_FOUND)
    set(VOLK_GNSSSDR_LINK_LIBRARIES ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES})
    set(VOLK_GNSSSDR_INCLUDE_DIRS ${VOLK_GNSSSDR_INCLUDE_DIRS} ${ORC_INCLUDE_DIRS})
else()
    set(VOLK_GNSSSDR_LINK_LIBRARIES ${VOLK_GNSSSDR_LIBRARIES})
endif()

if(VOLKGNSSSDR_FOUND AND NOT TARGET Volkgnsssdr::volkgnsssdr)
    add_library(Volkgnsssdr::volkgnsssdr SHARED IMPORTED)
    set_target_properties(Volkgnsssdr::volkgnsssdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${VOLK_GNSSSDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${VOLK_GNSSSDR_LINK_LIBRARIES}"
    )
endif()

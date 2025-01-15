# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Iio::ad9361
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_LIBAD9361 libad9361)

if(NOT LIBAD9361_ROOT)
    set(LIBAD9361_ROOT_USER_DEFINED /usr/local)
else()
    set(LIBAD9361_ROOT_USER_DEFINED ${LIBAD9361_ROOT})
endif()
if(DEFINED ENV{LIBAD9361_ROOT})
    set(LIBAD9361_ROOT_USER_DEFINED
        ${LIBAD9361_ROOT_USER_DEFINED}
        $ENV{LIBAD9361_ROOT}
    )
endif()
set(LIBAD9361_ROOT_USER_DEFINED
    ${LIBAD9361_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(LIBAD9361_INCLUDE_DIRS
    NAMES ad9361.h
    HINTS ${PC_LIBAD9361_INCLUDEDIR}
    PATHS ${LIBAD9361_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(LIBAD9361_LIBRARIES
    NAMES ad9361
    HINTS ${PC_LIBAD9361_LIBDIR}
    PATHS ${LIBAD9361_ROOT_USER_DEFINED}/lib
          ${LIBAD9361_ROOT_USER_DEFINED}/lib64
          ${GNSSSDR_LIB_PATHS}
          /Library/Frameworks/ad9361.framework
)

if(LIBAD9361_LIBRARIES AND APPLE)
    if(LIBAD9361_LIBRARIES MATCHES "framework")
        set(LIBAD9361_LIBRARIES ${LIBAD9361_LIBRARIES}/ad9361)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBAD9361 DEFAULT_MSG LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)

if(PC_LIBAD9361_VERSION)
    set(LIBAD9361_VERSION ${PC_LIBAD9361_VERSION})
endif()

if(LIBAD9361_FOUND AND LIBAD9361_VERSION)
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers (found: v${LIBAD9361_VERSION})"
    )
else()
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers"
    )
endif()

set_package_properties(LIBAD9361 PROPERTIES
    URL "https://github.com/analogdevicesinc/libad9361-iio"
)

if(LIBAD9361_FOUND AND NOT TARGET Iio::ad9361)
    add_library(Iio::ad9361 SHARED IMPORTED)
    set_target_properties(Iio::ad9361 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBAD9361_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBAD9361_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBAD9361_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)

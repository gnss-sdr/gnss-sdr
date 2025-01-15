# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Gnuradio::iio
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

pkg_check_modules(PC_IIO gnuradio-iio)

if(NOT GRIIO_ROOT)
    set(GRIIO_ROOT_USER_DEFINED /usr)
else()
    set(GRIIO_ROOT_USER_DEFINED ${GRIIO_ROOT})
endif()
if(DEFINED ENV{GRIIO_ROOT})
    set(GRIIO_ROOT_USER_DEFINED
        ${GRIIO_ROOT_USER_DEFINED}
        $ENV{GRIIO_ROOT}
    )
endif()
if(DEFINED ENV{IIO_DIR})
    set(GRIIO_ROOT_USER_DEFINED
        ${GRIIO_ROOT_USER_DEFINED}
        $ENV{IIO_DIR}
    )
endif()
set(GRIIO_ROOT_USER_DEFINED
    ${GRIIO_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)


find_path(IIO_INCLUDE_DIRS
    NAMES gnuradio/iio/api.h
    HINTS ${PC_IIO_INCLUDEDIR}
    PATHS ${GRIIO_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

if(IIO_INCLUDE_DIRS)
    set(GR_IIO_INCLUDE_HAS_GNURADIO TRUE)
else()
    find_path(IIO_INCLUDE_DIRS
        NAMES iio/api.h
        HINTS ${PC_IIO_INCLUDEDIR}
        PATHS ${GRIIO_ROOT_USER_DEFINED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )
    set(GR_IIO_INCLUDE_HAS_GNURADIO FALSE)
endif()

find_library(IIO_LIBRARIES
    NAMES gnuradio-iio
    HINTS ${PC_IIO_LIBDIR}
    PATHS ${GRIIO_ROOT_USER_DEFINED}/lib
          ${GRIIO_ROOT_USER_DEFINED}/lib64
          ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRIIO DEFAULT_MSG IIO_LIBRARIES IIO_INCLUDE_DIRS)

if(PC_IIO_VERSION)
    set(GRIIO_VERSION ${PC_IIO_VERSION})
endif()

set_package_properties(GRIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/gr-iio"
)
if(GRIIO_FOUND AND GRIIO_VERSION)
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio (found: v${GRIIO_VERSION})"
    )
else()
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio"
    )
endif()

if(GRIIO_FOUND AND NOT TARGET Gnuradio::iio)
    add_library(Gnuradio::iio SHARED IMPORTED)
    set_target_properties(Gnuradio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${IIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${IIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${IIO_LIBRARIES}"
    )
endif()

mark_as_advanced(IIO_LIBRARIES IIO_INCLUDE_DIRS GR_IIO_INCLUDE_HAS_GNURADIO)

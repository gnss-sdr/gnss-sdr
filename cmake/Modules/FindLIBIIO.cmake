# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Iio::iio
#
# GNSS-SDR uses the libiio v0.x API. LIBIIO_FOUND is only set if the header and
# library found build and link a v0.x program. LIBIIO_IS_V1 is set to TRUE if
# a libiio v1.x installation was detected instead.
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(PC_LIBIIO libiio)

if(NOT LIBIIO_ROOT)
    set(LIBIIO_ROOT_USER_DEFINED /usr)
else()
    set(LIBIIO_ROOT_USER_DEFINED ${LIBIIO_ROOT})
endif()
if(DEFINED ENV{LIBIIO_ROOT})
    set(LIBIIO_ROOT_USER_DEFINED
        ${LIBIIO_ROOT_USER_DEFINED}
        $ENV{LIBIIO_ROOT}
    )
endif()
set(LIBIIO_ROOT_USER_DEFINED
    ${LIBIIO_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(
    LIBIIO_INCLUDE_DIRS
    NAMES iio.h
    HINTS ${PC_LIBIIO_INCLUDEDIR}
    PATHS ${LIBIIO_ROOT_USER_DEFINED}/include
          ${GNSSSDR_INCLUDE_PATHS}
)

find_library(
    LIBIIO_LIBRARIES
    NAMES iio libiio.so.0
    HINTS ${PC_LIBIIO_LIBDIR}
    PATHS ${LIBIIO_ROOT_USER_DEFINED}/lib
          ${LIBIIO_ROOT_USER_DEFINED}/lib64
          ${GNSSSDR_LIB_PATHS}
          /Library/Frameworks/iio.framework/
)

if(LIBIIO_LIBRARIES AND APPLE)
    if(LIBIIO_LIBRARIES MATCHES "framework")
        set(LIBIIO_LIBRARIES ${LIBIIO_LIBRARIES}/iio)
    endif()
endif()

# libiio v1.x installs its header as iio/iio.h and replaces most of the v0.x
# API. Its libiio.so.0 compatibility layer is runtime-only (no v0.x header).
# Check that the header and library found build and link a v0.x program, so a
# v0.x header from one prefix is not paired with a v1.x library from another.
unset(LIBIIO_HAS_V0_API CACHE)
if(LIBIIO_INCLUDE_DIRS AND LIBIIO_LIBRARIES)
    include(CheckCXXSourceCompiles)
    set(_LIBIIO_REQUIRED_INCLUDES_SAVED "${CMAKE_REQUIRED_INCLUDES}")
    set(_LIBIIO_REQUIRED_LIBRARIES_SAVED "${CMAKE_REQUIRED_LIBRARIES}")
    set(CMAKE_REQUIRED_INCLUDES ${LIBIIO_INCLUDE_DIRS})
    set(CMAKE_REQUIRED_LIBRARIES ${LIBIIO_LIBRARIES})
    check_cxx_source_compiles("
        #include <iio.h>
        int main()
        {
            struct iio_context *ctx = iio_create_context_from_uri(\"ip:localhost\");
            struct iio_device *dev = iio_context_find_device(ctx, \"cf-ad9361-lpc\");
            struct iio_buffer *buf = iio_device_create_buffer(dev, 1024, false);
            iio_buffer_destroy(buf);
            iio_context_destroy(ctx);
        }"
        LIBIIO_HAS_V0_API
    )
    set(CMAKE_REQUIRED_INCLUDES "${_LIBIIO_REQUIRED_INCLUDES_SAVED}")
    set(CMAKE_REQUIRED_LIBRARIES "${_LIBIIO_REQUIRED_LIBRARIES_SAVED}")
endif()

set(LIBIIO_IS_V1 FALSE)
if(NOT LIBIIO_HAS_V0_API)
    find_path(
        LIBIIO_V1_INCLUDE_DIR
        NAMES iio/iio.h
        HINTS ${PC_LIBIIO_INCLUDEDIR}
        PATHS ${LIBIIO_ROOT_USER_DEFINED}/include
              ${GNSSSDR_INCLUDE_PATHS}
    )
    if(LIBIIO_V1_INCLUDE_DIR)
        set(LIBIIO_IS_V1 TRUE)
    elseif(PC_LIBIIO_VERSION AND NOT (PC_LIBIIO_VERSION VERSION_LESS 1))
        set(LIBIIO_IS_V1 TRUE)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBIIO DEFAULT_MSG LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS LIBIIO_HAS_V0_API)

if(PC_LIBIIO_VERSION)
    set(LIBIIO_VERSION ${PC_LIBIIO_VERSION})
endif()

if(LIBIIO_FOUND AND LIBIIO_VERSION)
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices (found: v${LIBIIO_VERSION})"
    )
else()
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices"
    )
endif()

set_package_properties(LIBIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/libiio"
)

if(LIBIIO_FOUND AND NOT TARGET Iio::iio)
    add_library(Iio::iio SHARED IMPORTED)
    set_target_properties(Iio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBIIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBIIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBIIO_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS LIBIIO_V1_INCLUDE_DIR)

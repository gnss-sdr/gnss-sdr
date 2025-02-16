# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


# - Try to find Googletest source code
#
# The following environment variable is optionally searched for:
# GTEST_DIR: Base directory where Googletest source code is found.
#
# The following are set after configuration is done:
# GOOGLETEST_FOUND
# LIBGTEST_DEV_DIR
# GTEST_INCLUDE_DIRS

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(PC_GTEST gtest)

if(NOT GTEST_DIR)
    set(GTEST_DIR_USER_PROVIDED /usr/local)
else()
    set(GTEST_DIR_USER_PROVIDED ${GTEST_DIR})
endif()
if(DEFINED ENV{GTEST_DIR})
    set(GTEST_DIR_USER_PROVIDED
        ${GTEST_DIR_USER_PROVIDED}
        $ENV{GTEST_DIR}
    )
endif()

find_path(LIBGTEST_DEV_DIR
    NAMES src/gtest-all.cc
    PATHS
        ${GTEST_DIR_USER_PROVIDED}
        ${GTEST_DIR_USER_PROVIDED}/googletest
        /usr/src/googletest/googletest
        /usr/src/gtest
        ${GNSSSDR_INCLUDE_PATHS}/gtest
        ${GNSSSDR_INCLUDE_PATHS}/googletest
        ${CMAKE_SYSTEM_PREFIX_PATH}/src/googletest/googletest
        ${CMAKE_SYSTEM_PREFIX_PATH}/src/gtest-1.7.0
        ${CMAKE_SYSTEM_PREFIX_PATH}/opt/googletest/include/googletest/googletest
)

find_path(GTEST_INCLUDE_DIRS
    NAMES gtest/gtest.h
    HINTS ${PC_GTEST_INCLUDEDIR}
    PATHS
        ${GTEST_DIR_USER_PROVIDED}/googletest/include
        ${GNSSSDR_INCLUDE_PATHS}
        ${CMAKE_SYSTEM_PREFIX_PATH}/src/gtest-1.7.0/include
        ${CMAKE_SYSTEM_PREFIX_PATH}/opt/googletest/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GOOGLETEST DEFAULT_MSG LIBGTEST_DEV_DIR GTEST_INCLUDE_DIRS)

if(GOOGLETEST_FOUND AND PC_GTEST_VERSION)
    set(GOOGLETEST_VERSION ${PC_GTEST_VERSION})
    set_package_properties(GOOGLETEST PROPERTIES
        DESCRIPTION "Source code of Google's Testing Framework (found: v${GOOGLETEST_VERSION})"
    )
else()
    set_package_properties(GOOGLETEST PROPERTIES
        DESCRIPTION "Source code of Google's Testing Framework"
    )
endif()

set_package_properties(GOOGLETEST PROPERTIES
    URL "https://github.com/google/googletest"
)

mark_as_advanced(LIBGTEST_DEV_DIR GTEST_INCLUDE_DIRS)

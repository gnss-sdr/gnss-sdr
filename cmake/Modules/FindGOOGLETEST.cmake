# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later


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

pkg_check_modules(PC_GTEST gtest)

find_path(LIBGTEST_DEV_DIR
    NAMES src/gtest-all.cc
    PATHS
        ${GTEST_DIR}
        ${GTEST_DIR}/googletest
        /usr/src/googletest/googletest
        /usr/src/gtest
        /usr/include/gtest
        /usr/local/src/googletest/googletest
        /opt/local/src/gtest-1.7.0
)

find_path(GTEST_INCLUDE_DIRS
    NAMES gtest/gtest.h
    HINTS ${PC_GTEST_INCLUDEDIR}
    PATHS
        ${GTEST_DIR}/googletest/include
        /usr/include
        /usr/local/include
        /opt/local/src/gtest-1.7.0/include
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

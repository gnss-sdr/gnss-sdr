# Copyright (C) 2011-2019 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.


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

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_GTEST gtest)

find_path(LIBGTEST_DEV_DIR
    NAMES src/gtest-all.cc
    PATHS
        ${GTEST_DIR}
        ${GTEST_DIR}/googletest
        /usr/src/googletest/googletest
        /usr/local/src/googletest/googletest
        /usr/src/gtest
        /usr/include/gtest
        /opt/local/src/gtest-1.7.0
)

find_path(GTEST_INCLUDE_DIRS
    NAMES gtest/gtest.h
    PATHS
        ${GTEST_DIR}/googletest/include
        /usr/include
        /opt/local/src/gtest-1.7.0/include
        ${PC_GTEST_INCLUDEDIR}
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

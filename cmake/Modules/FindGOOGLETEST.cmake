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


find_path(LIBGTEST_DEV_DIR
    NAMES src/gtest-all.cc
    PATHS
        ${GTEST_DIR}
        ${GTEST_DIR}/googletest
        /usr/src/googletest/googletest
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
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GOOGLETEST DEFAULT_MSG LIBGTEST_DEV_DIR GTEST_INCLUDE_DIRS)
mark_as_advanced(LIBGTEST_DEV_DIR GTEST_INCLUDE_DIRS)

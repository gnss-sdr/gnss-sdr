# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
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

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_ORC "orc-0.4 > 0.4.22")

find_program(ORCC_EXECUTABLE orcc
    HINTS ${PC_ORC_TOOLSDIR}
    PATHS /usr/bin
          /usr/local/bin
          /opt/local/bin
          ${ORC_ROOT}/bin
          ${CMAKE_INSTALL_PREFIX}/bin
)

find_path(ORC_INCLUDE_DIR
    NAMES orc/orc.h
    HINTS ${PC_ORC_INCLUDEDIR}
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
          ${ORC_ROOT}/include/orc-0.4
          ${CMAKE_INSTALL_PREFIX}/include/orc-0.4
)

find_path(ORC_LIBRARY_DIR
    NAMES ${CMAKE_SHARED_LIBRARY_PREFIX}orc-0.4${CMAKE_SHARED_LIBRARY_SUFFIX}
    HINTS ${PC_ORC_LIBDIR}
    PATHS /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
          /usr/lib/i386-linux-gnu
          /usr/lib/arm-linux-gnueabihf
          /usr/lib/arm-linux-gnueabi
          /usr/lib/aarch64-linux-gnu
          /usr/lib/mipsel-linux-gnu
          /usr/lib/mips-linux-gnu
          /usr/lib/mips64el-linux-gnuabi64
          /usr/lib/powerpc-linux-gnu
          /usr/lib/powerpc64-linux-gnu
          /usr/lib/powerpc64le-linux-gnu
          /usr/lib/hppa-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
          ${ORC_ROOT}/lib
          $ENV{ORC_ROOT}/lib
          ${ORC_ROOT}/lib${LIB_SUFFIX}
          ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}
)

find_library(ORC_LIB orc-0.4
    HINTS ${PC_ORC_LIBRARY_DIRS}
    PATHS ${ORC_ROOT}/lib${LIB_SUFFIX}
          ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}
)

if(PC_ORC_VERSION)
    set(ORC_VERSION ${PC_ORC_VERSION})
endif()

list(APPEND ORC_LIBRARY ${ORC_LIB})

set(ORC_INCLUDE_DIRS ${ORC_INCLUDE_DIR})
set(ORC_LIBRARIES ${ORC_LIBRARY})
set(ORC_LIBRARY_DIRS ${ORC_LIBRARY_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORC "orc files" ORC_LIBRARY ORC_INCLUDE_DIR ORCC_EXECUTABLE)

set_package_properties(ORC PROPERTIES
    URL "https://gstreamer.freedesktop.org/modules/orc.html"
)

if(ORC_FOUND AND ORC_VERSION)
    set_package_properties(ORC PROPERTIES
        DESCRIPTION "The Optimized Inner Loops Runtime Compiler (found: v${ORC_VERSION})"
    )
else()
    set_package_properties(ORC PROPERTIES
        DESCRIPTION "The Optimized Inner Loops Runtime Compiler"
    )
endif()

mark_as_advanced(ORC_INCLUDE_DIR ORC_LIBRARY ORCC_EXECUTABLE)

# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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
#

find_package(GPSTK QUIET)
if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
    include(GNUInstallDirs)
    string(REGEX REPLACE /[^/]*$ "" LIBDIR ${CMAKE_INSTALL_LIBDIR})
    set(GPSTK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX} )
    set(GPSTK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include )
endif(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)

set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} ${GPSTK_INCLUDE_DIR}/gpstk)

include_directories(
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${GFlags_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GPSTK_INCLUDE_DIR}/gpstk
    ${GPSTK_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")

add_executable(rinex2assist ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

target_link_libraries(rinex2assist
    ${Boost_LIBRARIES}
    ${GPSTK_LIBRARY}
    ${GFlags_LIBS}
    gnss_sp_libs
    gnss_rx)

if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
    add_dependencies(rinex2assist gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
endif(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)

add_custom_command(TARGET rinex2assist POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rinex2assist>
                   ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:rinex2assist>)

install(TARGETS rinex2assist
                RUNTIME DESTINATION bin
                COMPONENT "rinex2assist"
)

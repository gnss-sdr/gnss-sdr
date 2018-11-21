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

add_definitions(-DGNSS_SDR_VERSION="${VERSION}")

set(RTKLIB_LIB_SOURCES
    rtklib_rtkcmn.cc
    rtklib_ephemeris.cc
    rtklib_preceph.cc
    rtklib_sbas.cc
    rtklib_ionex.cc
    rtklib_pntpos.cc
    rtklib_ppp.cc
    rtklib_tides.cc
    rtklib_lambda.cc
    rtklib_rtkpos.cc
    rtklib_conversions.cc
    rtklib_stream.cc
    rtklib_rtksvr.cc
    rtklib_solution.cc
    rtklib_rtcm.cc
    rtklib_rtcm2.cc
    rtklib_rtcm3.cc
)

set(RTKLIB_LIB_HEADERS
    rtklib_rtkcmn.h
    rtklib_ephemeris.h
    rtklib_preceph.h
    rtklib_sbas.h
    rtklib_ionex.h
    rtklib_pntpos.h
    rtklib_ppp.h
    rtklib_tides.h
    rtklib_lambda.h
    rtklib_rtkpos.h
    rtklib_conversions.h
    rtklib_stream.h
    rtklib_rtksvr.h
    rtklib_solution.h
    rtklib_rtcm.h
    rtklib_rtcm2.h
    rtklib_rtcm3.h
    rtklib.h
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${CMAKE_SOURCE_DIR}/src/core/interfaces
    ${CMAKE_SOURCE_DIR}/src/core/receiver
    ${Boost_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

list(SORT RTKLIB_LIB_HEADERS)
list(SORT RTKLIB_LIB_SOURCES)

add_library(rtklib_lib ${RTKLIB_LIB_SOURCES} ${RTKLIB_LIB_HEADERS})
source_group(Headers FILES ${RTKLIB_LIB_HEADERS})
add_dependencies(rtklib_lib glog-${glog_RELEASE})

if(OS_IS_MACOSX)
    set(MAC_LIBRARIES "-framework Accelerate")
endif(OS_IS_MACOSX)

target_link_libraries(
    rtklib_lib
    ${Boost_LIBRARIES}
    ${GFlags_LIBS}
    ${GLOG_LIBRARIES}
    ${BLAS}
    ${LAPACK}
    ${MAC_LIBRARIES}
)

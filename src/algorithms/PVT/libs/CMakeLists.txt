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

add_definitions( -DGNSS_SDR_VERSION="${VERSION}" )

set(PVT_LIB_SOURCES
    pvt_solution.cc
    ls_pvt.cc
    hybrid_ls_pvt.cc
    kml_printer.cc
    gpx_printer.cc
    rinex_printer.cc
    nmea_printer.cc
    rtcm_printer.cc
    geojson_printer.cc
    rtklib_solver.cc
    pvt_conf.cc
)

set(PVT_LIB_HEADERS
    pvt_solution.h
    ls_pvt.h
    hybrid_ls_pvt.h
    kml_printer.h
    gpx_printer.h
    rinex_printer.h
    nmea_printer.h
    rtcm_printer.h
    geojson_printer.h
    rtklib_solver.h
    pvt_conf.h
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${CMAKE_SOURCE_DIR}/src/core/interfaces
    ${CMAKE_SOURCE_DIR}/src/core/receiver
    ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/adapters
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs/rtklib
    ${Boost_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${MATIO_INCLUDE_DIRS}
)

list(SORT PVT_LIB_HEADERS)
list(SORT PVT_LIB_SOURCES)

add_library(pvt_lib ${PVT_LIB_SOURCES} ${PVT_LIB_HEADERS})
source_group(Headers FILES ${PVT_LIB_HEADERS})

if(MATIO_FOUND)
    add_dependencies(pvt_lib
        glog-${glog_RELEASE}
        armadillo-${armadillo_RELEASE}
    )
else(MATIO_FOUND)
    add_dependencies(pvt_lib
        glog-${glog_RELEASE}
        armadillo-${armadillo_RELEASE}
        matio-${GNSSSDR_MATIO_LOCAL_VERSION}
    )
endif(MATIO_FOUND)

target_link_libraries(
    pvt_lib
    rtklib_lib
    gnss_sdr_flags
    gnss_sp_libs
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${BLAS}
    ${LAPACK}
    ${MATIO_LIBRARIES}
)

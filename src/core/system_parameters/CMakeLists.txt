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


set(SYSTEM_PARAMETERS_SOURCES
    gnss_satellite.cc
    gnss_signal.cc
    gps_navigation_message.cc
    gps_ephemeris.cc
    gps_iono.cc
    gps_almanac.cc
    gps_utc_model.cc
    gps_acq_assist.cc
    agnss_ref_time.cc
    agnss_ref_location.cc
    galileo_utc_model.cc
    galileo_ephemeris.cc
    galileo_almanac.cc
    galileo_almanac_helper.cc
    galileo_iono.cc
    galileo_navigation_message.cc
    sbas_ephemeris.cc
    galileo_fnav_message.cc
    gps_cnav_ephemeris.cc
    gps_cnav_navigation_message.cc
    gps_cnav_iono.cc
    gps_cnav_utc_model.cc
    rtcm.cc
    glonass_gnav_ephemeris.cc
    glonass_gnav_almanac.cc
    glonass_gnav_utc_model.cc
    glonass_gnav_navigation_message.cc
)

set(SYSTEM_PARAMETERS_HEADERS
    gnss_satellite.h
    gnss_signal.h
    gps_navigation_message.h
    gps_ephemeris.h
    gps_iono.h
    gps_almanac.h
    gps_utc_model.h
    gps_acq_assist.h
    agnss_ref_time.h
    agnss_ref_location.h
    galileo_utc_model.h
    galileo_ephemeris.h
    galileo_almanac.h
    galileo_almanac_helper.h
    galileo_iono.h
    galileo_navigation_message.h
    sbas_ephemeris.h
    galileo_fnav_message.h
    gps_cnav_ephemeris.h
    gps_cnav_navigation_message.h
    gps_cnav_iono.h
    gps_cnav_utc_model.h
    rtcm.h
    glonass_gnav_ephemeris.h
    glonass_gnav_almanac.h
    glonass_gnav_utc_model.h
    glonass_gnav_navigation_message.h
    display.h
    Galileo_E1.h
    Galileo_E5a.h
    GLONASS_L1_L2_CA.h
    gnss_frequencies.h
    gnss_obs_codes.h
    gnss_synchro.h
    GPS_CNAV.h
    GPS_L1_CA.h
    GPS_L2C.h
    GPS_L5.h
    MATH_CONSTANTS.h
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/receiver
    ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs/rtklib
    ${GLOG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
)

list(SORT SYSTEM_PARAMETERS_HEADERS)
list(SORT SYSTEM_PARAMETERS_SOURCES)

add_library(gnss_system_parameters
    ${SYSTEM_PARAMETERS_SOURCES}
    ${SYSTEM_PARAMETERS_HEADERS}
)

source_group(Headers FILES ${SYSTEM_PARAMETERS_HEADERS})

add_dependencies(gnss_system_parameters rtklib_lib glog-${glog_RELEASE})

target_link_libraries(gnss_system_parameters rtklib_lib ${Boost_LIBRARIES})

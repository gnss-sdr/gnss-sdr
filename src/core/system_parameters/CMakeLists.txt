# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(SYSTEM_PARAMETERS_SOURCES
    gnss_almanac.cc
    gnss_ephemeris.cc
    gnss_satellite.cc
    gnss_signal.cc
    gps_navigation_message.cc
    gps_ephemeris.cc
    galileo_utc_model.cc
    galileo_ephemeris.cc
    galileo_almanac_helper.cc
    galileo_cnav_message.cc
    galileo_fnav_message.cc
    galileo_has_data.cc
    galileo_inav_message.cc
    galileo_reduced_ced.cc
    beidou_dnav_navigation_message.cc
    beidou_dnav_ephemeris.cc
    sbas_ephemeris.cc
    gps_cnav_navigation_message.cc
    glonass_gnav_ephemeris.cc
    glonass_gnav_utc_model.cc
    glonass_gnav_navigation_message.cc
    reed_solomon.cc
    osnma_data.cc
    osnma_dsm_reader.cc
)

set(SYSTEM_PARAMETERS_HEADERS
    gnss_almanac.h
    gnss_ephemeris.h
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
    Galileo_CNAV.h
    Galileo_FNAV.h
    Galileo_INAV.h
    galileo_iono.h
    galileo_cnav_message.h
    galileo_fnav_message.h
    galileo_has_data.h
    galileo_inav_message.h
    galileo_reduced_ced.h
    sbas_ephemeris.h
    gps_cnav_ephemeris.h
    gps_cnav_navigation_message.h
    gps_cnav_iono.h
    gps_cnav_utc_model.h
    glonass_gnav_ephemeris.h
    glonass_gnav_almanac.h
    glonass_gnav_utc_model.h
    glonass_gnav_navigation_message.h
    beidou_dnav_navigation_message.h
    beidou_dnav_ephemeris.h
    beidou_dnav_iono.h
    beidou_dnav_almanac.h
    beidou_dnav_utc_model.h
    display.h
    Galileo_E1.h
    Galileo_E5a.h
    Galileo_E5b.h
    Galileo_E6.h
    GLONASS_L1_L2_CA.h
    gnss_frequencies.h
    gnss_obs_codes.h
    gnss_synchro.h
    GPS_CNAV.h
    GPS_L1_CA.h
    GPS_L2C.h
    GPS_L5.h
    Beidou_B1I.h
    Beidou_B3I.h
    Beidou_DNAV.h
    MATH_CONSTANTS.h
    reed_solomon.h
    galileo_has_page.h
    Galileo_OSNMA.h
    osnma_data.h
    osnma_dsm_reader.h
)

list(SORT SYSTEM_PARAMETERS_HEADERS)
list(SORT SYSTEM_PARAMETERS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_system_parameters STATIC)
    target_sources(core_system_parameters
        PRIVATE
            ${SYSTEM_PARAMETERS_SOURCES}
        PUBLIC
            ${SYSTEM_PARAMETERS_HEADERS}
    )
else()
    source_group(Headers FILES ${SYSTEM_PARAMETERS_HEADERS})
    add_library(core_system_parameters
        ${SYSTEM_PARAMETERS_SOURCES}
        ${SYSTEM_PARAMETERS_HEADERS}
    )
endif()

target_link_libraries(core_system_parameters
    PUBLIC
        Boost::date_time
        Boost::serialization
    PRIVATE
        Pugixml::pugixml
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(core_system_parameters PRIVATE Gflags::gflags Glog::glog)
    target_compile_definitions(core_system_parameters PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(core_system_parameters PRIVATE absl::flags absl::log)
endif()

# for gnss_sdr_make_unique.h
target_include_directories(core_system_parameters
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(core_system_parameters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET core_system_parameters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

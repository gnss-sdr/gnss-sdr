# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${GNSSSDR_SOURCE_DIR}/docs/protobuf/monitor_pvt.proto)
protobuf_generate_cpp(PROTO_SRCS2 PROTO_HDRS2 ${GNSSSDR_SOURCE_DIR}/docs/protobuf/gps_ephemeris.proto)
protobuf_generate_cpp(PROTO_SRCS3 PROTO_HDRS3 ${GNSSSDR_SOURCE_DIR}/docs/protobuf/galileo_ephemeris.proto)

set(PVT_LIB_SOURCES
    an_packet_printer.cc
    pvt_solution.cc
    geojson_printer.cc
    gpx_printer.cc
    kml_printer.cc
    nmea_printer.cc
    rinex_printer.cc
    rtcm_printer.cc
    rtcm.cc
    rtklib_solver.cc
    monitor_pvt_udp_sink.cc
    monitor_ephemeris_udp_sink.cc
    has_simple_printer.cc
    geohash.cc
)

set(PVT_LIB_HEADERS
    an_packet_printer.h
    pvt_conf.h
    pvt_solution.h
    geojson_printer.h
    gpx_printer.h
    kml_printer.h
    nmea_printer.h
    rinex_printer.h
    rtcm_printer.h
    rtcm.h
    rtklib_solver.h
    monitor_pvt_udp_sink.h
    monitor_pvt.h
    serdes_monitor_pvt.h
    serdes_galileo_eph.h
    serdes_gps_eph.h
    monitor_ephemeris_udp_sink.h
    has_simple_printer.h
    geohash.h
)

list(SORT PVT_LIB_HEADERS)
list(SORT PVT_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(pvt_libs STATIC)
    target_sources(pvt_libs
        PRIVATE
            ${PROTO_SRCS}
            ${PROTO_SRCS2}
            ${PROTO_SRCS3}
            ${PROTO_HDRS}
            ${PROTO_HDRS2}
            ${PROTO_HDRS3}
            ${PVT_LIB_SOURCES}
        PUBLIC
            ${PVT_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${PVT_LIB_HEADERS} ${PROTO_HDRS} ${PROTO_HDRS2} ${PROTO_HDRS3})
    add_library(pvt_libs
        ${PVT_LIB_SOURCES}
        ${PROTO_SRCS}
        ${PROTO_SRCS2}
        ${PROTO_SRCS3}
        ${PVT_LIB_HEADERS}
        ${PROTO_HDRS}
        ${PROTO_HDRS2}
        ${PROTO_HDRS3}
    )
endif()

if(CMAKE_ANDROID_ARCH_ABI)
        target_compile_definitions(pvt_libs PUBLIC -DANDROID=1)
endif()

target_link_libraries(pvt_libs
    PUBLIC
        Boost::date_time
        protobuf::libprotobuf
        core_system_parameters
        algorithms_libs_rtklib
    PRIVATE
        algorithms_libs
        Gflags::gflags
        Glog::glog
        Matio::matio
)

get_filename_component(PROTO_INCLUDE_HEADERS_DIR ${PROTO_HDRS} DIRECTORY)

# for concurrent_queue.h
target_include_directories(pvt_libs
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/receiver
)

# Do not apply clang-tidy fixes to protobuf generated headers
target_include_directories(pvt_libs
    SYSTEM PUBLIC
        ${PROTO_INCLUDE_HEADERS_DIR}
)

target_compile_definitions(pvt_libs PRIVATE -DGNSS_SDR_VERSION="${VERSION}")

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(pvt_libs
        PUBLIC
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

# Fix for Boost Asio < 1.70
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(pvt_libs
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(pvt_libs
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

set_property(TARGET pvt_libs APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_SOURCE_DIR}/docs/protobuf/monitor_pvt.proto)

set(PVT_LIB_SOURCES
    pvt_solution.cc
    ls_pvt.cc
    hybrid_ls_pvt.cc
    kml_printer.cc
    gpx_printer.cc
    rinex_printer.cc
    nmea_printer.cc
    rtcm_printer.cc
    rtcm.cc
    geojson_printer.cc
    rtklib_solver.cc
    pvt_conf.cc
    monitor_pvt_udp_sink.cc
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
    rtcm.h
    geojson_printer.h
    rtklib_solver.h
    pvt_conf.h
    monitor_pvt_udp_sink.h
    monitor_pvt.h
    serdes_monitor_pvt.h
)

list(SORT PVT_LIB_HEADERS)
list(SORT PVT_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(pvt_libs STATIC)
    target_sources(pvt_libs
        PRIVATE
            ${PROTO_SRCS}
            ${PROTO_HDRS}
            ${PVT_LIB_SOURCES}
        PUBLIC
            ${PVT_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${PVT_LIB_HEADERS} ${PROTO_HDRS})
    add_library(pvt_libs ${PVT_LIB_SOURCES} ${PROTO_SRCS} ${PVT_LIB_HEADERS} ${PROTO_HDRS})
endif()

if(${FILESYSTEM_FOUND})
    target_compile_definitions(pvt_libs PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(pvt_libs PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(pvt_libs PRIVATE std::filesystem)
else()
    target_link_libraries(pvt_libs PRIVATE Boost::filesystem Boost::system)
endif()

target_link_libraries(pvt_libs
    PUBLIC
        Armadillo::armadillo
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

get_filename_component(PROTO_INCLUDE_HEADERS ${PROTO_HDRS} DIRECTORY)

target_include_directories(pvt_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/receiver
        SYSTEM ${PROTO_INCLUDE_HEADERS}
)

target_compile_definitions(pvt_libs PRIVATE -DGNSS_SDR_VERSION="${VERSION}")

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(pvt_libs
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

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

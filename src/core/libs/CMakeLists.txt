# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

add_subdirectory(supl)

set(CORE_LIBS_SOURCES
    ini.cc
    INIReader.cc
    string_converter.cc
    gnss_sdr_supl_client.cc
    gnss_sdr_sample_counter.cc
    channel_status_msg_receiver.cc
    channel_event.cc
    command_event.cc
)

set(CORE_LIBS_HEADERS
    ini.h
    INIReader.h
    string_converter.h
    gnss_sdr_supl_client.h
    gnss_sdr_sample_counter.h
    channel_status_msg_receiver.h
    channel_event.h
    command_event.h
)

if(ENABLE_FPGA)
    set(CORE_LIBS_SOURCES
        ${CORE_LIBS_SOURCES}
        gnss_sdr_fpga_sample_counter.cc
        gnss_sdr_time_counter.cc
    )
    set(CORE_LIBS_HEADERS
        ${CORE_LIBS_HEADERS}
        gnss_sdr_fpga_sample_counter.h
        gnss_sdr_time_counter.h
    )
endif()

if(ENABLE_FPGA OR ENABLE_AD9361)
    set(CORE_LIBS_SOURCES
        ${CORE_LIBS_SOURCES}
        uio_fpga.cc
    )
    set(CORE_LIBS_HEADERS
        ${CORE_LIBS_HEADERS}
        uio_fpga.h
    )
endif()

list(SORT CORE_LIBS_HEADERS)
list(SORT CORE_LIBS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_libs STATIC)
    target_sources(core_libs
        PRIVATE
            ${CORE_LIBS_SOURCES}
        PUBLIC
            ${CORE_LIBS_HEADERS}
    )
else()
    source_group(Headers FILES ${CORE_LIBS_HEADERS})
    add_library(core_libs ${CORE_LIBS_SOURCES} ${CORE_LIBS_HEADERS})
endif()

target_link_libraries(core_libs
    PUBLIC
        Gnuradio::blocks
        Gnuradio::runtime
        Gnuradio::pmt
        core_libs_supl
        core_system_parameters
        pvt_libs
    PRIVATE
        Boost::serialization
        Gflags::gflags
        Glog::glog
        Pugixml::pugixml
)

if(USE_GENERIC_LAMBDAS AND NOT GNURADIO_USES_STD_POINTERS)
    target_link_libraries(core_libs PUBLIC Boost::headers)
else()
    target_link_libraries(core_libs PRIVATE Boost::headers)
endif()

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(core_libs
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_include_directories(core_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(core_libs
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(core_libs
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(USE_BOOST_BIND_PLACEHOLDERS)
    target_compile_definitions(core_libs
        PRIVATE
            -DUSE_BOOST_BIND_PLACEHOLDERS=1
    )
endif()

if(ENABLE_FPGA OR ENABLE_AD9361)
    if(FILESYSTEM_FOUND)
        target_compile_definitions(core_libs PRIVATE -DHAS_STD_FILESYSTEM=1)
        if(find_experimental)
            target_compile_definitions(core_libs PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
        endif()
        target_link_libraries(core_libs PRIVATE std::filesystem)
    else()
        target_link_libraries(core_libs PRIVATE Boost::filesystem Boost::system)
    endif()
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(core_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET core_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core/interfaces>
)

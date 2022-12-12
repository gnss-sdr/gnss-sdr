# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(ENABLE_RAW_UDP AND PCAP_FOUND)
    list(APPEND OPT_DRIVER_SOURCES gr_complex_ip_packet_source.cc)
    list(APPEND OPT_DRIVER_HEADERS gr_complex_ip_packet_source.h)
endif()

set(SIGNAL_SOURCE_GR_BLOCKS_SOURCES
    fifo_reader.cc
    unpack_byte_2bit_samples.cc
    unpack_byte_2bit_cpx_samples.cc
    unpack_byte_4bit_samples.cc
    unpack_intspir_1bit_samples.cc
    rtl_tcp_signal_source_c.cc
    unpack_2bit_samples.cc
    unpack_spir_gss6450_samples.cc
    labsat23_source.cc
    ${OPT_DRIVER_SOURCES}
)

set(SIGNAL_SOURCE_GR_BLOCKS_HEADERS
    fifo_reader.h
    unpack_byte_2bit_samples.h
    unpack_byte_2bit_cpx_samples.h
    unpack_byte_4bit_samples.h
    unpack_intspir_1bit_samples.h
    rtl_tcp_signal_source_c.h
    unpack_2bit_samples.h
    unpack_spir_gss6450_samples.h
    labsat23_source.h
    ${OPT_DRIVER_HEADERS}
)

list(SORT SIGNAL_SOURCE_GR_BLOCKS_HEADERS)
list(SORT SIGNAL_SOURCE_GR_BLOCKS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(signal_source_gr_blocks STATIC)
    target_sources(signal_source_gr_blocks
        PRIVATE
            ${SIGNAL_SOURCE_GR_BLOCKS_SOURCES}
        PUBLIC
            ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS}
    )
else()
    source_group(Headers FILES ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})
    add_library(signal_source_gr_blocks
        ${SIGNAL_SOURCE_GR_BLOCKS_SOURCES}
        ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS}
    )
endif()

target_link_libraries(signal_source_gr_blocks
    PUBLIC
        signal_source_libs
        Boost::thread
    PRIVATE
        algorithms_libs
        core_libs
        Gflags::gflags
        Glog::glog
)

target_include_directories(signal_source_gr_blocks
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/receiver
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_source_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(ENABLE_RAW_UDP AND PCAP_FOUND)
    target_link_libraries(signal_source_gr_blocks
        PUBLIC
            Pcap::pcap
    )
endif()

# Fix for Boost Asio < 1.70
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(signal_source_gr_blocks
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(signal_source_gr_blocks
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(signal_source_gr_blocks
        PUBLIC
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

if(USE_BOOST_BIND_PLACEHOLDERS)
    target_compile_definitions(signal_source_gr_blocks
        PRIVATE
            -DUSE_BOOST_BIND_PLACEHOLDERS=1
    )
endif()

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(signal_source_gr_blocks
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(signal_source_gr_blocks
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER "1.57.99")
    target_compile_definitions(signal_source_gr_blocks
        PRIVATE
            -DHAS_BOOST_ENDIAN=1
    )
    target_link_libraries(signal_source_gr_blocks
        PRIVATE
            Boost::headers
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_source_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_source_gr_blocks APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

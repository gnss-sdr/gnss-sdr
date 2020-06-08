# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_library(obs_gr_blocks STATIC)
    target_sources(obs_gr_blocks
        PRIVATE
            hybrid_observables_gs.cc
        PUBLIC
            hybrid_observables_gs.h
    )
else()
    source_group(Headers FILES hybrid_observables_gs.h)
    add_library(obs_gr_blocks hybrid_observables_gs.cc hybrid_observables_gs.h)
endif()

if(${FILESYSTEM_FOUND})
    target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(obs_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(obs_gr_blocks PRIVATE Boost::filesystem)
endif()

target_include_directories(obs_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

target_link_libraries(obs_gr_blocks
    PUBLIC
        Boost::headers
        Gnuradio::blocks
        observables_libs
    PRIVATE
        algorithms_libs
        core_system_parameters
        Gflags::gflags
        Glog::glog
        Matio::matio
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(obs_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(CMAKE_VERSION VERSION_GREATER 3.1 AND NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)))
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(obs_gr_blocks
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(obs_gr_blocks
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.72.99)
    target_compile_definitions(obs_gr_blocks
        PRIVATE
            -DBOOST_173_OR_GREATER=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obs_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET obs_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
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

target_include_directories(obs_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(obs_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(USE_GENERIC_LAMBDAS)
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

if(USE_BOOST_BIND_PLACEHOLDERS)
    target_compile_definitions(obs_gr_blocks
        PRIVATE
            -DUSE_BOOST_BIND_PLACEHOLDERS=1
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

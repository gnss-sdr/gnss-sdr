# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
    add_library(pvt_gr_blocks STATIC)
    target_sources(pvt_gr_blocks
        PRIVATE
            rtklib_pvt_gs.cc
        PUBLIC
            rtklib_pvt_gs.h
    )
else()
    source_group(Headers FILES rtklib_pvt_gs.h)
    add_library(pvt_gr_blocks rtklib_pvt_gs.cc rtklib_pvt_gs.h)
endif()

target_link_libraries(pvt_gr_blocks
    PUBLIC
        algorithms_libs_rtklib
        Boost::date_time
        Gnuradio::pmt
        Gnuradio::runtime
    PRIVATE
        algorithms_libs
        pvt_libs
        Gflags::gflags
        Glog::glog
        Boost::serialization
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(pvt_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_include_directories(pvt_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(pvt_gr_blocks
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(pvt_gr_blocks
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(PMT_USES_BOOST_ANY)
    target_compile_definitions(pvt_gr_blocks
        PRIVATE
            -DPMT_USES_BOOST_ANY=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

if(USE_OLD_BOOST_MATH_COMMON_FACTOR)
    target_compile_definitions(pvt_gr_blocks
        PRIVATE
            -DUSE_OLD_BOOST_MATH_COMMON_FACTOR=1
    )
endif()

if(USE_BOOST_BIND_PLACEHOLDERS)
    target_compile_definitions(pvt_gr_blocks
        PRIVATE
            -DUSE_BOOST_BIND_PLACEHOLDERS=1
    )
endif()

if(${has_put_time})
    target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_PUT_TIME=1)
endif()

set_property(TARGET pvt_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

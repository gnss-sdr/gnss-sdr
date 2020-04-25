# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(PVT_GR_BLOCKS_SOURCES
    rtklib_pvt_gs.cc
)

set(PVT_GR_BLOCKS_HEADERS
    rtklib_pvt_gs.h
)

source_group(Headers FILES ${PVT_GR_BLOCKS_HEADERS})

add_library(pvt_gr_blocks ${PVT_GR_BLOCKS_SOURCES} ${PVT_GR_BLOCKS_HEADERS})

if(${FILESYSTEM_FOUND})
    target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(pvt_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(pvt_gr_blocks PRIVATE Boost::filesystem Boost::system)
endif()

target_link_libraries(pvt_gr_blocks
    PUBLIC
        algorithms_libs_rtklib
        core_system_parameters
        Boost::date_time
        Gnuradio::pmt
        Gnuradio::runtime
    PRIVATE
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

if(CMAKE_VERSION VERSION_GREATER 3.1)
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

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

if(Boost_VERSION_STRING VERSION_LESS 1.58.0)
    target_compile_definitions(pvt_gr_blocks PRIVATE -DOLD_BOOST=1)
endif()

# Check if we have std::put_time (Workaround for gcc < 5.0)
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
    #include <iomanip>
    int main()
    { std::put_time(nullptr, \"\"); }"
    has_put_time
)
if(${has_put_time})
    target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_PUT_TIME=1)
endif()

set_property(TARGET pvt_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

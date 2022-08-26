# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
    add_library(pvt_adapters STATIC)
    target_sources(pvt_adapters
        PRIVATE
            rtklib_pvt.cc
        PUBLIC
            rtklib_pvt.h
    )
else()
    source_group(Headers FILES rtklib_pvt.h)
    add_library(pvt_adapters rtklib_pvt.cc rtklib_pvt.h)
endif()

target_link_libraries(pvt_adapters
    PUBLIC
        pvt_gr_blocks
    PRIVATE
        gnss_sdr_flags
        Glog::glog
        pvt_libs
)

target_include_directories(pvt_adapters
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

if((NOT (CMAKE_VERSION VERSION_LESS "3.1")) AND NOT (CMAKE_CXX_STANDARD VERSION_LESS 17))
    target_compile_definitions(pvt_adapters
        PRIVATE
            -DUSE_STD_COMMON_FACTOR=1
    )
else()
    if(USE_OLD_BOOST_MATH_COMMON_FACTOR)
        target_compile_definitions(pvt_adapters
            PRIVATE
                -DUSE_OLD_BOOST_MATH_COMMON_FACTOR=1
        )
    endif()
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET pvt_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

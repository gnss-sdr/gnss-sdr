# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(OBSERVABLES_LIB_HEADERS ${OBSERVABLES_LIB_HEADERS} obs_conf.h)
set(OBSERVABLES_LIB_SOURCES ${OBSERVABLES_LIB_SOURCES} obs_conf.cc)

list(SORT OBSERVABLES_LIB_HEADERS)
list(SORT OBSERVABLES_LIB_SOURCES)

source_group(Headers FILES ${OBSERVABLES_LIB_HEADERS})

add_library(observables_libs
    ${OBSERVABLES_LIB_SOURCES}
    ${OBSERVABLES_LIB_HEADERS}
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(observables_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET observables_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

file(GLOB ASN_RRLP_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp/*.c")
list(SORT ASN_RRLP_SOURCES)
file(GLOB ASN_SUPL_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl/*.c")
list(SORT ASN_SUPL_SOURCES)

set(SUPL_SOURCES
    supl.c
)

if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        set(MY_C_FLAGS "${MY_C_FLAGS} -Wno-parentheses-equality")
    endif()
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MY_C_FLAGS}")

add_library(core_libs_supl STATIC ${ASN_RRLP_SOURCES} ${ASN_SUPL_SOURCES} ${SUPL_SOURCES})

if(OPENSSL_FOUND)
    target_compile_definitions(core_libs_supl PUBLIC -DUSE_OPENSSL_FALLBACK=1)
endif()

target_link_libraries(core_libs_supl
    PUBLIC
        ${GNUTLS_LIBRARIES}
        ${GNUTLS_OPENSSL_LIBRARY}
)

target_include_directories(core_libs_supl
    PUBLIC
        ${GNUTLS_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
        ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
)

set_target_properties(core_libs_supl PROPERTIES
    LINKER_LANGUAGE C
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR};${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl;${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp;${GNUTLS_INCLUDE_DIR}"
)

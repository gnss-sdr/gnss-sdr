# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

file(GLOB ASN_RRLP_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp/*.c")
list(SORT ASN_RRLP_SOURCES)
file(GLOB ASN_RRLP_HEADERS "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp/*.h")
list(SORT ASN_RRLP_HEADERS)
file(GLOB ASN_SUPL_SOURCES "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl/*.c")
list(SORT ASN_SUPL_SOURCES)
file(GLOB ASN_SUPL_HEADERS "${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl/*.h")
list(SORT ASN_SUPL_HEADERS)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_libs_supl STATIC)
    target_sources(core_libs_supl
        PRIVATE
            ${ASN_RRLP_SOURCES}
            ${ASN_SUPL_SOURCES}
            ${ASN_RRLP_HEADERS}
            ${ASN_SUPL_HEADERS}
            ${CMAKE_CURRENT_LIST_DIR}/supl.c
        PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/supl.h
    )
else()
    add_library(core_libs_supl
        STATIC
            ${ASN_RRLP_SOURCES}
            ${ASN_SUPL_SOURCES}
            supl.c
    )
endif()

if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        set(MY_C_FLAGS -Wno-parentheses-equality)
        target_compile_options(core_libs_supl PUBLIC $<$<COMPILE_LANGUAGE:C>:${MY_C_FLAGS}>)
    endif()
endif()

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

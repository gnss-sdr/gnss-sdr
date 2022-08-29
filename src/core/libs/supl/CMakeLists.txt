# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

file(GLOB ASN_RRLP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/asn-rrlp/*.c")
list(SORT ASN_RRLP_SOURCES)
file(GLOB ASN_RRLP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/asn-rrlp/*.h")
list(SORT ASN_RRLP_HEADERS)
file(GLOB ASN_SUPL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/asn-supl/*.c")
list(SORT ASN_SUPL_SOURCES)
file(GLOB ASN_SUPL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/asn-supl/*.h")
list(SORT ASN_SUPL_HEADERS)
file(GLOB ASN_TYPES_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/types/*.c")
list(SORT ASN_TYPES_SOURCES)
file(GLOB ASN_TYPES_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/types/*.h")
list(SORT ASN_TYPES_HEADERS)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_libs_supl STATIC)
    target_sources(core_libs_supl
        PRIVATE
            ${ASN_RRLP_SOURCES}
            ${ASN_SUPL_SOURCES}
            ${ASN_TYPES_SOURCES}
            ${ASN_RRLP_HEADERS}
            ${ASN_SUPL_HEADERS}
            ${ASN_TYPES_HEADERS}
            ${CMAKE_CURRENT_LIST_DIR}/supl.c
        PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/supl.h
    )
else()
    add_library(core_libs_supl
        STATIC
            ${ASN_RRLP_SOURCES}
            ${ASN_SUPL_SOURCES}
            ${ASN_TYPES_SOURCES}
            supl.c
    )
endif()

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU" AND (CMAKE_VERSION VERSION_GREATER "3.3"))
    set(MY_C_FLAGS -Wno-unused-parameter)
    target_compile_options(core_libs_supl PUBLIC $<$<COMPILE_LANGUAGE:C>:${MY_C_FLAGS}>)
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
        ${CMAKE_CURRENT_SOURCE_DIR}/asn-supl
        ${CMAKE_CURRENT_SOURCE_DIR}/asn-rrlp
        ${CMAKE_CURRENT_SOURCE_DIR}/types
)

# This is ancient and unmaintained code. As the project migrates to modern
# compilers, non-standards-conforming constructs will be flagged as errors. This
# should be upgraded, but until then, allow non-standards-conforming constructs
if(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
    set_target_properties(core_libs_supl
        PROPERTIES C_EXTENSIONS ON
    )
endif()

set_property(TARGET core_libs_supl
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/asn-supl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/asn-rrlp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/types>
        $<BUILD_INTERFACE:${GNUTLS_INCLUDE_DIR}>
)

set_property(TARGET core_libs_supl
    APPEND PROPERTY LINKER_LANGUAGE C
)

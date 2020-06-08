# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES
    cnav_msg.c
    bits.c
    edc.c
    viterbi27.c
)

set(TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS
    cnav_msg.h
    bits.h
    edc.h
    swift_common.h
    fec.h
)

list(SORT TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS)
list(SORT TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES)

if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_library(telemetry_decoder_libswiftcnav STATIC)
    target_sources(telemetry_decoder_libswiftcnav
        PRIVATE
            ${TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES}
        PUBLIC
            ${TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS}
    )
else()
    source_group(Headers FILES ${TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS})
    add_library(telemetry_decoder_libswiftcnav STATIC
        ${TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES}
        ${TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS}
    )
endif()

set_property(TARGET telemetry_decoder_libswiftcnav
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

set_target_properties(telemetry_decoder_libswiftcnav
    PROPERTIES LINKER_LANGUAGE C
)

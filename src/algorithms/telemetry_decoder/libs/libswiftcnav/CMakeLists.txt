# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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

if(USE_CMAKE_TARGET_SOURCES)
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

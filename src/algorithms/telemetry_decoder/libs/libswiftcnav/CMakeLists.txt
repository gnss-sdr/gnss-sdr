# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
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

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
)

list(SORT TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS)
list(SORT TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES)

add_library(telemetry_decoder_libswiftcnav STATIC ${TELEMETRY_DECODER_LIBSWIFTCNAV_SOURCES} ${TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS})
source_group(Headers FILES ${TELEMETRY_DECODER_LIBSWIFTCNAV_HEADERS})
set_target_properties(telemetry_decoder_libswiftcnav PROPERTIES LINKER_LANGUAGE C)

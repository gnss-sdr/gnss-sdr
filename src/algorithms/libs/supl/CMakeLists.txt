# Copyright (C) 2012-2013  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

file(GLOB  ASN_RRLP_SOURCES "${CMAKE_SOURCE_DIR}/src/algorithms/libs/supl/asn-rrlp/*.c")
file(GLOB  ASN_SUPL_SOURCES "${CMAKE_SOURCE_DIR}/src/algorithms/libs/supl/asn-supl/*.c")

set (SUPL_SOURCES
	supl.c
	)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs/supl
	${CMAKE_SOURCE_DIR}/src/algorithms/libs/supl/asn-rrlp
	${CMAKE_SOURCE_DIR}/src/algorithms/libs/supl/asn-supl
	)
	
	

add_library (supl_library STATIC ${ASN_RRLP_SOURCES} ${ASN_SUPL_SOURCES} ${SUPL_SOURCES})
target_link_libraries (supl_library ssl) 
SET_TARGET_PROPERTIES(supl_library PROPERTIES LINKER_LANGUAGE C)
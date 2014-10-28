# Copyright (C) 2012-2014  (see AUTHORS file for a list of contributors)
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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

###############################################################################
# Volk_gnsssdr module
#In order to use volk_gnsssr module it is necessary to add:
# 1) include_directories(..${VOLK_GNSSSDR_INCLUDE_DIRS}..)
# 2) target_link_libraries(..${VOLK_GNSSSDR_LIBRARIES}..)
###############################################################################

message(STATUS " START OF: Setup volk_gnsssdr as a subproject.")

set(VOLK_GNSSSDR_BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr/lib)
list(APPEND CMAKE_C_FLAGS "-std=c11")

add_subdirectory(${VOLK_GNSSSDR_BASE_PATH})

set(VOLK_GNSSSDR_INCLUDE_DIRS
     ${VOLK_GNSSSDR_BASE_PATH}/include
     ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr/include
     CACHE INTERNAL ""
)

set(VOLK_GNSSSDR_LIBRARIES
     #Path to libs of volk_gnsssdr target: ${VOLK_GNSSSDR_BASE_PATH}/lib/Debug/libvolk_gnsssdr.dylib
     volk_gnsssdr
     CACHE INTERNAL ""
)

message(STATUS " * INCLUDES: ${VOLK_GNSSSDR_INCLUDE_DIRS} ")
message(STATUS " * LIBS: ${VOLK_GNSSSDR_LIBRARIES} ")
message(STATUS " END OF: Setup volk_gnsssdr as a subproject.")


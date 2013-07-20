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

set(FRONT_END_CAL_SOURCES 
	front_end_cal.cc
)

include_directories(
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/core/libs   
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
	 ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${GNURADIO_BLOCKS_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}  
     ${Boost_INCLUDE_DIRS}
)

if(OS_IS_MACOSX)
     # Tell the linker where the libraries installed by MacPorts are
     link_directories( /opt/local/lib )
endif(OS_IS_MACOSX)

add_library(front_end_cal_lib ${FRONT_END_CAL_SOURCES})
target_link_libraries(front_end_cal_lib ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES} ${GFlags_LIBS} ${GLOG_LIBRARIES} ${ARMADILLO_LIBRARIES} gnss_rx gnss_sp_libs)

add_definitions( -DGNSS_SDR_VERSION="${VERSION}" )

add_executable(front-end-cal ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)
target_link_libraries(front-end-cal  ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES}  ${Boost_LIBRARIES} ${GFlags_LIBS} ${GLOG_LIBRARIES} ${ARMADILLO_LIBRARIES} gnss_rx gnss_sp_libs front_end_cal_lib)

install(TARGETS front-end-cal 
        DESTINATION ${CMAKE_SOURCE_DIR}/install
        )

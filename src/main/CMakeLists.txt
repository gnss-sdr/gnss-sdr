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


include_directories(
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_CORE_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}  
)

add_executable(gnss-sdr ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)
target_link_libraries(gnss-sdr ${Boost_LIBRARIES} ${GLOG_LIBRARIES} ${GFlags_LIBS} ${ARMADILLO_LIBRARIES} gnss_sp_libs gnss_rx)

install(TARGETS gnss-sdr
        DESTINATION ${CMAKE_SOURCE_DIR}/install
        )
        
       
        

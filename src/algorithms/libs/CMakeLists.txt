# Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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

if(OPENCL_FOUND)
    set(GNSS_SPLIBS_SOURCES
         galileo_e1_signal_processing.cc
         gnss_sdr_valve.cc
         gnss_signal_processing.cc
         gps_sdr_signal_processing.cc
         nco_lib.cc
         fix_fft.cc
         pass_through.cc
         fft_execute.cc # Needs OpenCL
         fft_setup.cc # Needs OpenCL
         fft_kernelstring.cc # Needs OpenCL
         galileo_e5_signal_processing.cc
    )
else(OPENCL_FOUND)
    set(GNSS_SPLIBS_SOURCES
         galileo_e1_signal_processing.cc
         gnss_sdr_valve.cc
         gnss_signal_processing.cc
         gps_sdr_signal_processing.cc
         nco_lib.cc
         fix_fft.cc
         pass_through.cc
         galileo_e5_signal_processing.cc
    )
endif(OPENCL_FOUND)

include_directories(
     $(CMAKE_CURRENT_SOURCE_DIR)
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
)

if(OPENCL_FOUND)
    include_directories( ${OPENCL_INCLUDE_DIRS} )
    if(OS_IS_MACOSX)
         set(OPT_LIBRARIES ${OPT_LIBRARIES} "-framework OpenCL")
    else(OS_IS_MACOSX)
         set(OPT_LIBRARIES ${OPT_LIBRARIES} ${OPENCL_LIBRARIES})
    endif(OS_IS_MACOSX)
endif(OPENCL_FOUND)

file(GLOB GNSS_SPLIBS_HEADERS "*.h")
add_library(gnss_sp_libs ${GNSS_SPLIBS_SOURCES} ${GNSS_SPLIBS_HEADERS})
source_group(Headers FILES ${GNSS_SPLIBS_HEADERS})

target_link_libraries(gnss_sp_libs ${GNURADIO_RUNTIME_LIBRARIES} 
                                   ${GNURADIO_BLOCKS_LIBRARIES} 
                                   ${GNURADIO_FFT_LIBRARIES} 
                                   ${GNURADIO_FILTER_LIBRARIES} 
                                   ${OPT_LIBRARIES} 
                                   gnss_rx
)

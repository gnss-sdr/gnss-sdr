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

set(GNSS_RECEIVER_SOURCES 
     control_thread.cc 
     control_message_factory.cc 
     file_configuration.cc 
     gnss_block_factory.cc
     gnss_flowgraph.cc
     in_memory_configuration.cc
)

include_directories(
     $(CMAKE_CURRENT_SOURCE_DIR)
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/libs
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
	 ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/output_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/conditioner/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/data_type_adapter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/observables/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/observables/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${ARMADILLO_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
)


#Enable GN3S module if the flag is present
if( $ENV{GN3S_DRIVER} )
    message(STATUS "Support for RF front-end GN3S v2 enabled" )
    add_definitions(-DGN3S_DRIVER=1)
endif( $ENV{GN3S_DRIVER} )
if( GN3S_DRIVER )
    message(STATUS "Support for RF front-end GN3S v2 enabled" )
    add_definitions(-DGN3S_DRIVER=1)
endif( GN3S_DRIVER )

#Enable RTL-SDR module if the flag is present
if( $ENV{RTLSDR_DRIVER} )
    message(STATUS "Support for RF front-end based on RTL dongle enabled" )
    add_definitions(-DRTLSDR_DRIVER=1)
endif( $ENV{RTLSDR_DRIVER} )
if( RTLSDR_DRIVER )
    message(STATUS "Support for RF front-end based on RTL dongle enabled" )
    add_definitions(-DRTLSDR_DRIVER=1)
endif( RTLSDR_DRIVER )

#Enable OpenCL if found in the system
if ( OPENCL_FOUND )
    message(STATUS "Adding processing blocks implemented using OpenCL" )
    add_definitions(-DOPENCL=1)
endif( OPENCL_FOUND )


add_library(gnss_rx ${GNSS_RECEIVER_SOURCES})

target_link_libraries(gnss_rx ${Boost_LIBRARIES} 
                              ${ARMADILLO_LIBRARIES} 
                              ${GNURADIO_RUNTIME_LIBRARIES} 
                              ${GNURADIO_BLOCKS_LIBRARIES} 
                              ${GNURADIO_FFT_LIBRARIES} 
                              ${GNURADIO_FILTER_LIBRARIES} 
                              gnss_system_parameters 
                              gnss_sp_libs 
                              signal_source_adapters 
                              datatype_adapters 
                              input_filter_adapters 
                              conditioner_adapters 
                              resampler_adapters 
                              acq_adapters 
                              tracking_lib 
                              tracking_adapters 
                              channel_adapters 
                              telemetry_decoder_adapters 
                              obs_adapters 
                              pvt_adapters 
                              pvt_lib 
                              out_adapters 
                              rx_core_lib
)

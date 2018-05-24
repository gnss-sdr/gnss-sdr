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


if(ENABLE_RAW_UDP)
   # - Try to find libpcap include dirs and libraries
   #
   # Usage of this module as follows:
   #
   #     find_package(PCAP)
   #
   # Variables used by this module, they can change the default behaviour and need
   # to be set before calling find_package:
   #
   #  PCAP_ROOT_DIR             Set this variable to the root installation of
   #                            libpcap if the module has problems finding the
   #                            proper installation path.
   #
   # Variables defined by this module:
   #
   #  PCAP_FOUND                System has libpcap, include and library dirs found
   #  PCAP_INCLUDE_DIR          The libpcap include directories.
   #  PCAP_LIBRARY              The libpcap library (possibly includes a thread
   #                            library e.g. required by pf_ring's libpcap)
   #  HAVE_PF_RING              If a found version of libpcap supports PF_RING
   find_package(PCAP)
   if(NOT PCAP_FOUND)
      message(FATAL_ERROR "PCAP required to compile custom UDP packet sample source (ENABLE_RAW_UDP)")
   endif()
   get_filename_component(PCAP_LIBRARY_DIRS ${PCAP_LIBRARY} DIRECTORY CACHE)
   set(OPT_LIBRARIES ${OPT_LIBRARIES} ${PCAP_LIBRARIES})
   set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS})
   set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} gr_complex_ip_packet_source.cc)
endif(ENABLE_RAW_UDP)

set(SIGNAL_SOURCE_GR_BLOCKS_SOURCES
     unpack_byte_2bit_samples.cc
     unpack_byte_2bit_cpx_samples.cc
     unpack_byte_4bit_samples.cc
     unpack_intspir_1bit_samples.cc
     rtl_tcp_signal_source_c.cc
     unpack_2bit_samples.cc
     unpack_spir_gss6450_samples.cc
     labsat23_source.cc
     ${OPT_DRIVER_SOURCES}
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${OPT_DRIVER_INCLUDE_DIRS}
)

file(GLOB SIGNAL_SOURCE_GR_BLOCKS_HEADERS "*.h")
list(SORT SIGNAL_SOURCE_GR_BLOCKS_HEADERS)
add_library(signal_source_gr_blocks ${SIGNAL_SOURCE_GR_BLOCKS_SOURCES} ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})
target_link_libraries(signal_source_gr_blocks 
    signal_source_lib 
    ${GNURADIO_RUNTIME_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OPT_LIBRARIES}
)
add_dependencies(signal_source_gr_blocks glog-${glog_RELEASE})

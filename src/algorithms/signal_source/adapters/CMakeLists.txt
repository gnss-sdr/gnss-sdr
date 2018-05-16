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

file(GLOB SIGNAL_SOURCE_ADAPTER_HEADERS "*.h")
list(SORT SIGNAL_SOURCE_ADAPTER_HEADERS)

# Optional drivers

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
   set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} custom_udp_signal_source.cc)
   
endif(ENABLE_RAW_UDP)

if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
   find_package(iio REQUIRED)
   if(NOT IIO_FOUND)
      message(STATUS "gnuradio-iio not found, its installation is required.")
      message(STATUS "Please build and install the following projects:")
      message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
      message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
      message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
      message(FATAL_ERROR "gnuradio-iio is required for building gnss-sdr with this option enabled.")
   endif(NOT IIO_FOUND)
   set(OPT_LIBRARIES ${OPT_LIBRARIES} ${IIO_LIBRARIES})
   set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${IIO_INCLUDE_DIRS})
endif(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)

if(ENABLE_AD9361)
   find_package(libiio REQUIRED)
   if(NOT LIBIIO_FOUND)
      message(STATUS "libiio not found, its installation is required.")
      message(STATUS "Please build and install the following projects:")
      message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
      message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
      message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
      message(FATAL_ERROR "libiio is required for building gnss-sdr with this option enabled.")
   endif(NOT LIBIIO_FOUND)
   set(OPT_LIBRARIES ${OPT_LIBRARIES} ${LIBIIO_LIBRARIES})
   set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${LIBIIO_INCLUDE_DIRS})
endif(ENABLE_AD9361)


if(ENABLE_PLUTOSDR)
    ##############################################
    # ADALM-PLUTO (Analog Devices Inc.)
    ##############################################
    if(IIO_FOUND)
        set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} plutosdr_signal_source.cc)
    endif(IIO_FOUND)
endif(ENABLE_PLUTOSDR)


if(ENABLE_FMCOMMS2)
    ###############################################
    # FMCOMMS2 based SDR Hardware
    ###############################################
    if(IIO_FOUND)
        set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} fmcomms2_signal_source.cc)
    endif(IIO_FOUND)
endif(ENABLE_FMCOMMS2)

if(ENABLE_AD9361)
    ###############################################
    # AD9361 DIRECT TO FPGA Hardware
    ###############################################
    if(LIBIIO_FOUND)
        set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} ad9361_fpga_signal_source.cc)
    endif(LIBIIO_FOUND)
endif(ENABLE_AD9361)



if(ENABLE_GN3S)
     ##############################################
     # GN3S (USB dongle)
     ##############################################
     find_package(GrGN3S REQUIRED)
     if(NOT GR_GN3S_FOUND)
         message(" gr-gn3s not found, install it from https://github.com/gnss-sdr/gr-gn3s ")
         message(FATAL_ERROR "gr-gn3s required for building gnss-sdr with this option enabled")
     endif(NOT GR_GN3S_FOUND)
     set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GR_GN3S_LIBRARIES})
     set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${GR_GN3S_INCLUDE_DIRS})
     set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} gn3s_signal_source.cc)
endif(ENABLE_GN3S)


if(ENABLE_FLEXIBAND)
     ##############################################
     # TELEORBIT FLEXIBAND FRONTEND ADAPTER
     ##############################################
     find_package(Teleorbit REQUIRED)
     if(NOT TELEORBIT_FOUND)
         message(FATAL_ERROR "Teleorbit Flexiband GNU Radio driver required to build gnss-sdr with the optional FLEXIBAND adapter")
     endif(NOT TELEORBIT_FOUND)

     # Set up variables
     set(FLEXIBAND_DRIVER_INCLUDE_DIRS
         ${OPT_DRIVER_INCLUDE_DIRS}
         ${TELEORBIT_INCLUDE_DIR}/teleorbit
     )
     set(OPT_LIBRARIES ${OPT_LIBRARIES} ${TELEORBIT_LIBRARIES})
     set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${FLEXIBAND_DRIVER_INCLUDE_DIRS})
     set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} flexiband_signal_source.cc)
endif(ENABLE_FLEXIBAND)


if(ENABLE_ARRAY)
     ##############################################
     # DBFCTTC GNSS EXPERIMENTAL ARRAY PROTOTYPE
     ##############################################
     find_package(GrDbfcttc REQUIRED)
     if(NOT GR_DBFCTTC_FOUND)
         message(" gr-dbfcttc not found, install it from https://github.com/gnss-sdr/gr-dbfcttc ")
         message(FATAL_ERROR "gr-dbfcttc required for building gnss-sdr with this option enabled")
     endif(NOT GR_DBFCTTC_FOUND)
     set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GR_DBFCTTC_LIBRARIES})
     set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${GR_DBFCTTC_INCLUDE_DIRS})
     set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} raw_array_signal_source.cc)
endif(ENABLE_ARRAY)


if(ENABLE_OSMOSDR)
    ################################################################################
    # OsmoSDR - http://sdr.osmocom.org/trac/
    ################################################################################
    if(NOT GROSMOSDR_FOUND)
        if(ENABLE_PACKAGING)
            list(REMOVE_ITEM SIGNAL_SOURCE_ADAPTER_HEADERS ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters/osmosdr_signal_source.h)
        endif(ENABLE_PACKAGING)
    else(NOT GROSMOSDR_FOUND)
        # set OSMO include dirs
        set(OSMO_DRIVER_INCLUDE_DIRS
            ${OPT_DRIVER_INCLUDE_DIRS}
            ${GROSMOSDR_INCLUDE_DIR}/osmosdr
           )
        set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} osmosdr_signal_source.cc)
        set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GROSMOSDR_LIBRARIES})
        set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${OSMO_DRIVER_INCLUDE_DIRS})
    endif(NOT GROSMOSDR_FOUND)
endif(ENABLE_OSMOSDR)

if(ENABLE_UHD AND GNURADIO_UHD_LIBRARIES_gnuradio-uhd)
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} uhd_signal_source.cc)
    set(OPT_LIBRARIES ${OPT_LIBRARIES} ${UHD_LIBRARIES} ${GNURADIO_UHD_LIBRARIES})
    set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${UHD_INCLUDE_DIRS})
else(ENABLE_UHD AND GNURADIO_UHD_LIBRARIES_gnuradio-uhd)
    list(REMOVE_ITEM SIGNAL_SOURCE_ADAPTER_HEADERS ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters/uhd_signal_source.h)
endif(ENABLE_UHD AND GNURADIO_UHD_LIBRARIES_gnuradio-uhd)


set(SIGNAL_SOURCE_ADAPTER_SOURCES file_signal_source.cc
                                  gen_signal_source.cc
                                  nsr_file_signal_source.cc
                                  spir_file_signal_source.cc
                                  spir_gss6450_file_signal_source.cc
                                  rtl_tcp_signal_source.cc
                                  labsat_signal_source.cc
                                  ${OPT_DRIVER_SOURCES}
)

if(PC_GNURADIO_RUNTIME_VERSION VERSION_GREATER 3.7.3)
     set(SIGNAL_SOURCE_ADAPTER_SOURCES ${SIGNAL_SOURCE_ADAPTER_SOURCES}
         two_bit_cpx_file_signal_source.cc
         two_bit_packed_file_signal_source.cc )
endif(PC_GNURADIO_RUNTIME_VERSION VERSION_GREATER 3.7.3)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${OPT_DRIVER_INCLUDE_DIRS}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/gnuradio_blocks
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
)

if(ARCH_64BITS)
    add_definitions(-DARCH_64BITS=1)
endif(ARCH_64BITS)

add_definitions(-DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}")

add_library(signal_source_adapters ${SIGNAL_SOURCE_ADAPTER_SOURCES} ${SIGNAL_SOURCE_ADAPTER_HEADERS})
source_group(Headers FILES ${SIGNAL_SOURCE_ADAPTER_HEADERS})
target_link_libraries(signal_source_adapters signal_source_gr_blocks
                                             ${Boost_LIBRARIES}
                                             ${GNURADIO_PMT_LIBRARIES}
                                             ${GNURADIO_BLOCKS_LIBRARIES}
                                             ${OPT_LIBRARIES}
                                             gnss_sp_libs
)

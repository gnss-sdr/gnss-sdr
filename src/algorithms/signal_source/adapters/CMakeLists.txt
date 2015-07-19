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

# Optional drivers

if(ENABLE_GN3S)
     ##############################################
     # GN3S (USB dongle)
     ##############################################

     # find_package(GrGN3S)
     # if not found, build it with ExternalPackage_Add
     include(ExternalProject)
     ExternalProject_Add(
          gr-gn3s
          SOURCE_DIR ${CMAKE_SOURCE_DIR}/drivers/gr-gn3s
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../../../gr-gn3s
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )

     # Set up variables
     set(GRGN3S_INCLUDE_DIRS ${GRGN3S_DIR}/include ${GRGN3S_DIR} ${GRGN3S_DIR}/src)
     set(GRGN3S_LIBRARIES
         "${CMAKE_CURRENT_BINARY_DIR}/../../../../gr-gn3s/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gr-gn3s${CMAKE_SHARED_LIBRARY_SUFFIX}"
     )
     set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GRGN3S_LIBRARIES})
     set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${GRGN3S_INCLUDE_DIRS})
     set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} gn3s_signal_source.cc)

     #  Copy GN3S firmware binary file to install folder
     message(STATUS "Copying the GN3S firmware binary file to install folder")
     file(COPY ${CMAKE_SOURCE_DIR}/firmware/GN3S_v2/bin/gn3s_firmware.ihx
          DESTINATION ${CMAKE_SOURCE_DIR}/install/
     )
endif(ENABLE_GN3S)

if(ENABLE_FLEXIBAND)
     ##############################################
     # TELEORBIT FLEXIBAND FRONTEND ADAPTER
     ##############################################
     if(OS_IS_MACOSX)
          set(MACOSX_ARGS "-DCMAKE_CXX_COMPILER=/usr/bin/clang++")
     endif(OS_IS_MACOSX)
    find_package(teleorbit REQUIRED)
    if(NOT TELEORBIT_FOUND)
        message(FATAL_ERROR "Teleorbit Flexiband GNURadio driver required to build gnss-sdr with the optional FLEXIBAND adapter")
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
     # GRDBFCTTC GNSS EXPERIMENTAL ARRAY PROTOTYPE
     ##############################################
     if(OS_IS_MACOSX)
          set(MACOSX_ARGS "-DCMAKE_CXX_COMPILER=/usr/bin/clang++")
     endif(OS_IS_MACOSX)
     # find_package(GrDbfcttc)
     # if not found, build it with ExternalPackage_Add
     include(ExternalProject)
     ExternalProject_Add(
          gr-dbfcttc
          SOURCE_DIR ${CMAKE_SOURCE_DIR}/drivers/gr-dbfcttc
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../../../gr-dbfcttc
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          CMAKE_ARGS ${MACOSX_ARGS}
          INSTALL_COMMAND ""
     )

     # Set up variables
     set(GRDBFCTTC_INCLUDE_DIRS ${GRDBFCTTC_DIR}/include ${GRDBFCTTC_DIR} ${GRDBFCTTC_DIR}/src)
     set(GRDBFCTTC_LIBRARIES
         "${CMAKE_CURRENT_BINARY_DIR}/../../../../gr-dbfcttc/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gnuradio-dbfcttc${CMAKE_SHARED_LIBRARY_SUFFIX}"
     )
     set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GRDBFCTTC_LIBRARIES})
     set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${GRDBFCTTC_INCLUDE_DIRS})
     set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} raw_array_signal_source.cc)
endif(ENABLE_ARRAY)


if(ENABLE_OSMOSDR)
    ################################################################################
    # OsmoSDR - http://sdr.osmocom.org/trac/
    ################################################################################
    find_package(GrOsmoSDR REQUIRED)
    if(NOT GROSMOSDR_FOUND)
        message(FATAL_ERROR "gr-osmosdr required to build gnss-sdr with the optional OSMOSDR driver")
    endif(NOT GROSMOSDR_FOUND)

    # set OSMO include dirs
    set(OSMO_DRIVER_INCLUDE_DIRS
        ${OPT_DRIVER_INCLUDE_DIRS}
        ${GROSMOSDR_INCLUDE_DIR}/osmosdr
    )
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} osmosdr_signal_source.cc)
    set(OPT_LIBRARIES ${OPT_LIBRARIES} ${GROSMOSDR_LIBRARIES})
    set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${OSMO_DRIVER_INCLUDE_DIRS})
endif(ENABLE_OSMOSDR)

if(ENABLE_UHD)
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} uhd_signal_source.cc)
    set(OPT_LIBRARIES ${OPT_LIBRARIES} ${UHD_LIBRARIES} ${GNURADIO_UHD_LIBRARIES})
    set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${UHD_INCLUDE_DIRS})
endif(ENABLE_UHD)


set(SIGNAL_SOURCE_ADAPTER_SOURCES file_signal_source.cc
                                  gen_signal_source.cc
                                  nsr_file_signal_source.cc
                                  spir_file_signal_source.cc
				  				  rtl_tcp_signal_source.cc
                                  ${OPT_DRIVER_SOURCES}
)

if(PC_GNURADIO_RUNTIME_VERSION VERSION_GREATER 3.7.3)
     set(SIGNAL_SOURCE_ADAPTER_SOURCES ${SIGNAL_SOURCE_ADAPTER_SOURCES} two_bit_cpx_file_signal_source.cc )
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

file(GLOB SIGNAL_SOURCE_ADAPTER_HEADERS "*.h")
add_library(signal_source_adapters ${SIGNAL_SOURCE_ADAPTER_SOURCES} ${SIGNAL_SOURCE_ADAPTER_HEADERS})
source_group(Headers FILES ${SIGNAL_SOURCE_ADAPTER_HEADERS})
target_link_libraries(signal_source_adapters signal_source_gr_blocks
                                             ${Boost_LIBRARIES}
                                             ${GNURADIO_PMT_LIBRARIES}
                                             ${GNURADIO_BLOCKS_LIBRARIES}
                                             ${OPT_LIBRARIES}
                                             gnss_sp_libs
)

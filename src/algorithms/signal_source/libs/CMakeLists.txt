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

if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
   find_package(iio REQUIRED)
   if(NOT IIO_FOUND)
      message(STATUS "gnuradio-iio not found, its installation is required.")
      message(STATUS "Please build and install the following projects:")
      message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
      message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
      message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
      message(FATAL_ERROR "gnuradio-iio required for building gnss-sdr with this option enabled")
   endif(NOT IIO_FOUND)
   set(OPT_LIBRARIES ${OPT_LIBRARIES} ${IIO_LIBRARIES})
   set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${IIO_INCLUDE_DIRS})
   
   find_package(libiio REQUIRED)
   if(NOT LIBIIO_FOUND)
      message(STATUS "gnuradio-iio not found, its installation is required.")
      message(STATUS "Please build and install the following projects:")
      message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
      message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
      message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
      message(FATAL_ERROR "gnuradio-iio required for building gnss-sdr with this option enabled")
   endif(NOT LIBIIO_FOUND)
   set(OPT_LIBRARIES ${OPT_LIBRARIES} ${LIBIIO_LIBRARIES})
   set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${LIBIIO_INCLUDE_DIRS})
   
endif(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)

if(ENABLE_FMCOMMS2)
    ###############################################
    # FMCOMMS2 based SDR Hardware
    ###############################################
    if(IIO_FOUND)
        set(OPT_SIGNAL_SOURCE_LIB_SOURCES ad9361_manager.cc)
    endif(IIO_FOUND)
endif(ENABLE_FMCOMMS2)

set (SIGNAL_SOURCE_LIB_SOURCES
  rtl_tcp_commands.cc
  rtl_tcp_dongle_info.cc
  ${OPT_SIGNAL_SOURCE_LIB_SOURCES})

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${OPT_DRIVER_INCLUDE_DIRS}
)


file(GLOB SIGNAL_SOURCE_LIB_HEADERS "*.h")
list(SORT SIGNAL_SOURCE_LIB_HEADERS)
list(SORT SIGNAL_SOURCE_LIB_SOURCES)
add_library(signal_source_lib ${SIGNAL_SOURCE_LIB_SOURCES} ${SIGNAL_SOURCE_LIB_HEADERS})
source_group(Headers FILES ${SIGNAL_SOURCE_LIB_HEADERS})
add_dependencies(signal_source_lib glog-${glog_RELEASE})
target_link_libraries(signal_source_lib ${OPT_LIBRARIES})


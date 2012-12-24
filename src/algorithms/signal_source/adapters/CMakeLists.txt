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

# Optional drivers

IF($ENV{GN3S_DRIVER})
    message(" GN3S ")
    set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters)
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} gn3s_signal_source.cc)
ENDIF($ENV{GN3S_DRIVER})

IF($ENV{RTLSDR_DRIVER})
    message(" RTLSDR ")
    set(OPT_DRIVER_INCLUDE_DIRS ${OPT_DRIVER_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters)
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} rtlsdr_signal_source.cc)
ENDIF($ENV{RTLSDR_DRIVER})

set(SIGNAL_SOURCE_ADAPTER_SOURCES file_signal_source.cc uhd_signal_source.cc ${OPT_DRIVER_SOURCES})

include_directories(
     $(CMAKE_CURRENT_SOURCE_DIR)
     $(OPT_DRIVER_INCLUDE_DIRS)
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${GNURADIO_CORE_INCLUDE_DIRS}
     ${GNURADIO_GRUEL_INCLUDE_DIRS}
     ${GR_UHD_INCLUDE_DIRS}
     ${UHD_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIR})

add_library(signal_source_adapters ${SIGNAL_SOURCE_ADAPTER_SOURCES})
target_link_libraries(signal_source_adapters gnss_sp_libs ${Boost_LIBRARIES} uhd gnuradio-uhd)
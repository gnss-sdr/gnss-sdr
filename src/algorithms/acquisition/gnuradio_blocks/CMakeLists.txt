# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
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


set(ACQ_GR_BLOCKS_SOURCES
    pcps_acquisition.cc
    pcps_assisted_acquisition_cc.cc
    pcps_acquisition_fine_doppler_cc.cc
    pcps_tong_acquisition_cc.cc
    pcps_cccwsr_acquisition_cc.cc
    pcps_quicksync_acquisition_cc.cc
    galileo_pcps_8ms_acquisition_cc.cc
    galileo_e5a_noncoherent_iq_acquisition_caf_cc.cc
)

set(ACQ_GR_BLOCKS_HEADERS
    pcps_acquisition.h
    pcps_assisted_acquisition_cc.h
    pcps_acquisition_fine_doppler_cc.h
    pcps_tong_acquisition_cc.h
    pcps_cccwsr_acquisition_cc.h
    pcps_quicksync_acquisition_cc.h
    galileo_pcps_8ms_acquisition_cc.h
    galileo_e5a_noncoherent_iq_acquisition_caf_cc.h
)

if(ENABLE_FPGA)
    set(ACQ_GR_BLOCKS_SOURCES ${ACQ_GR_BLOCKS_SOURCES} pcps_acquisition_fpga.cc)
    set(ACQ_GR_BLOCKS_HEADERS ${ACQ_GR_BLOCKS_HEADERS} pcps_acquisition_fpga.h)
endif()

if(OPENCL_FOUND)
    set(ACQ_GR_BLOCKS_SOURCES ${ACQ_GR_BLOCKS_SOURCES} pcps_opencl_acquisition_cc.cc)
    set(ACQ_GR_BLOCKS_HEADERS ${ACQ_GR_BLOCKS_HEADERS} pcps_opencl_acquisition_cc.h)
endif()


if(OPENCL_FOUND)
    include_directories(${OPENCL_INCLUDE_DIRS})
    if(OS_IS_MACOSX)
        set(OPT_LIBRARIES ${OPT_LIBRARIES} "-framework OpenCL")
    else()
        set(OPT_LIBRARIES ${OPT_LIBRARIES} ${OPENCL_LIBRARIES})
    endif()
endif()

list(SORT ACQ_GR_BLOCKS_HEADERS)
list(SORT ACQ_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${ACQ_GR_BLOCKS_HEADERS})

add_library(acq_gr_blocks ${ACQ_GR_BLOCKS_SOURCES} ${ACQ_GR_BLOCKS_HEADERS})

target_link_libraries(acq_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Gnuradio::fft
        Volk::volk
        gnss_system_parameters
        acquisition_lib
        ${OPT_LIBRARIES}
    PRIVATE
        Boost::filesystem
        Gflags::gflags
        Glog::glog
        Matio::matio
        Volkgnsssdr::volkgnsssdr
        gnss_sp_libs
        gnss_rx
)

if(OPENCL_FOUND)
    target_include_directories(acq_gr_blocks PUBLIC ${OPENCL_INCLUDE_DIRS})
endif()

target_include_directories(acq_gr_blocks PUBLIC ${CMAKE_SOURCE_DIR}/src/algorithms/libs)

set_property(TARGET acq_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

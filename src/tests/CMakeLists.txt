# Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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


if(EXISTS $ENV{GTEST_DIR})
   set(GTEST_DIR_LOCAL $ENV{GTEST_DIR})
endif(EXISTS $ENV{GTEST_DIR})
if(GTEST_DIR)
    set(GTEST_DIR_LOCAL ${GTEST_DIR})
endif(GTEST_DIR)
if(NOT GTEST_DIR_LOCAL)
    set(GTEST_DIR_LOCAL false CACHE STRING "Gtest downloaded and built automatically")
else()
	set(GTEST_DIR_LOCAL true CACHE STRING "Gtest already downloaded")
endif(NOT GTEST_DIR_LOCAL)
find_path(GTEST_INCLUDE_DIRS NAMES gtest/gtest.h
          PATHS /usr/include)
if(GTEST_INCLUDE_DIRS)
    set(GTEST_DIR_LOCAL true CACHE STRING "Gtest already downloaded")
endif(GTEST_INCLUDE_DIRS)

if(NOT GTEST_DIR_LOCAL)
     # if GTEST_DIR is not defined, we download and build it
     set(gtest_RELEASE 1.7.0)
     if(OS_IS_MACOSX)
         ExternalProject_Add(
              gtest-${gtest_RELEASE}
              URL http://googletest.googlecode.com/files/gtest-${gtest_RELEASE}.zip
              DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../download/gtest-${gtest_RELEASE}
              URL_MD5 "2d6ec8ccdf5c46b05ba54a9fd1d130d7"
              SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${gtest_RELEASE}
              BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${gtest_RELEASE}
              CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
              UPDATE_COMMAND ""
              PATCH_COMMAND ""
              INSTALL_COMMAND ""
         )
     else(OS_IS_MACOSX)
         ExternalProject_Add(
              gtest-${gtest_RELEASE}
              URL http://googletest.googlecode.com/files/gtest-${gtest_RELEASE}.zip
              DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../download/gtest-${gtest_RELEASE}
              URL_MD5 "2d6ec8ccdf5c46b05ba54a9fd1d130d7"
              SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${gtest_RELEASE}
              BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${gtest_RELEASE}
              UPDATE_COMMAND ""
              PATCH_COMMAND ""
              INSTALL_COMMAND ""
         )
     endif(OS_IS_MACOSX)
     # Set up variables
     # Set recently downloaded and build Googletest root folder
     set(GTEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${gtest_RELEASE}")
     # Source code
     ExternalProject_Get_Property(gtest-${gtest_RELEASE} source_dir)
     set(GTEST_INCLUDE_DIR ${source_dir}/include)
     set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
     # Library
     ExternalProject_Get_Property(gtest-${gtest_RELEASE} binary_dir)
     if(OS_IS_MACOSX)
         if(CMAKE_GENERATOR STREQUAL Xcode)
             set(binary_dir "${binary_dir}/Debug")
         endif(CMAKE_GENERATOR STREQUAL Xcode)
     endif(OS_IS_MACOSX)
     set(GTEST_LIBRARY_PATH "${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a;${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a")
     set(GTEST_LIBRARY gtest-${gtest_RELEASE}) 
     set(GTEST_LIBRARIES 
          ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a 
          ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a 
     ) 
	 set(GTEST_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${gtest_RELEASE}")
else(NOT GTEST_DIR_LOCAL)
     if(GTEST_INCLUDE_DIRS)
          find_path(GTEST_SOURCE_DIR NAMES gtest-all.cc
                    PATHS /usr/src/gtest/src)
          set(GTEST_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS} {GTEST_SOURCE_DIR})
          add_library(gtest ${GTEST_SOURCE_DIR}/gtest-all.cc ${GTEST_SOURCE_DIR}/gtest_main.cc)
          set(GTEST_LIBRARIES gtest)
     else(GTEST_INCLUDE_DIRS)
          # If the variable GTEST_DIR is defined, we use the existing Googletest
          set(GTEST_DIR $ENV{GTEST_DIR})
          set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
          add_library(gtest ${GTEST_DIR}/src/gtest-all.cc ${GTEST_DIR}/src/gtest_main.cc)
          set(GTEST_LIBRARIES gtest)
     endif(GTEST_INCLUDE_DIRS)
endif(NOT GTEST_DIR_LOCAL)


set(GNSS_SDR_TEST_OPTIONAL_LIBS "")
set(GNSS_SDR_TEST_OPTIONAL_HEADERS "")

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        set(GNSS_SDR_TEST_OPTIONAL_LIBS "${GNSS_SDR_TEST_OPTIONAL_LIBS};${GPERFTOOLS_LIBRARIES}")
        set(GNSS_SDR_TEST_OPTIONAL_HEADERS "${GNSS_SDR_TEST_OPTIONAL_HEADERS};${GPERFTOOLS_INCLUDE_DIR}")
    endif(GPERFTOOLS_FOUND)
endif(ENABLE_GPERFTOOLS)

     
include_directories(
     ${GTEST_INCLUDE_DIRECTORIES}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/core/libs   
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-rrlp
     ${CMAKE_SOURCE_DIR}/src/core/libs/supl/asn-supl
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/output_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${GNSS_SDR_TEST_OPTIONAL_HEADERS}
)


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CLANG_FLAGS "-stdlib=libc++ -std=c++11 -lc++")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")


if(OPENCL_FOUND)
    add_definitions(-DOPENCL_BLOCKS_TEST=1)
endif(OPENCL_FOUND)

add_definitions(-DTEST_PATH="${CMAKE_SOURCE_DIR}/src/tests/")


add_executable(run_tests ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)

add_custom_command(TARGET run_tests POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:run_tests>
                       ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:run_tests>)

target_link_libraries(run_tests ${CLANG_FLAGS} 
                                ${Boost_LIBRARIES}  
                                ${GFLAGS_LIBS} 
                                ${GLOG_LIBRARIES} 
                                ${GTEST_LIBRARIES} 
                                ${GNURADIO_RUNTIME_LIBRARIES} 
                                ${GNURADIO_BLOCKS_LIBRARIES} 
                                ${GNURADIO_FILTER_LIBRARIES} 
                                ${GNURADIO_ANALOG_LIBRARIES} 
                                ${ARMADILLO_LIBRARIES}
                                ${VOLK_LIBRARIES}
                                gnss_sp_libs 
                                gnss_rx  
                                signal_generator_blocks
                                signal_generator_adapters
                                out_adapters
                                pvt_gr_blocks
                                ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES}
                                ${GNSS_SDR_TEST_OPTIONAL_LIBS}
)


#########################################################
#  Adding Tests to Ctest
#########################################################
# ToDo

#set(CMAKE_CTEST_COMMAND ctest -V)
#add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
     
add_executable(control_thread_test EXCLUDE_FROM_ALL
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc 
     ${CMAKE_CURRENT_SOURCE_DIR}/control_thread/control_message_factory_test.cc
)

target_link_libraries(control_thread_test ${Boost_LIBRARIES} 
                                          ${GFLAGS_LIBS} 
                                          ${GLOG_LIBRARIES} 
                                          ${GTEST_LIBRARIES} 
                                          gnss_sp_libs 
                                          gnss_rx
                                          )
                                          
add_test(control_thread_test control_thread_test)

add_executable(gnss_block_test EXCLUDE_FROM_ALL
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc 
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/file_signal_source_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/fir_filter_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_acquisition_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_multithread_acquisition_gsoc2013_test.cc
#     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_tong_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_quicksync_acquisition_gsoc2014_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_ambiguous_acquisition_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_ambiguous_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_8ms_ambiguous_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_cccwsr_ambiguous_acquisition_gsoc2013_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_tong_ambiguous_acquisition_gsoc2013_test.cc
     #${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_dll_pll_veml_tracking_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/file_output_filter_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gnss_block_factory_test.cc   
)
target_link_libraries(gnss_block_test ${Boost_LIBRARIES}
                                      ${GFLAGS_LIBS} 
                                      ${GLOG_LIBRARIES} 
                                      ${GTEST_LIBRARIES} 
                                      gnss_sp_libs 
                                      gnss_rx 
                                      signal_generator_blocks
                                      out_adapters
                                      )
                                      
add_test(gnss_block_test gnss_block_test)

 

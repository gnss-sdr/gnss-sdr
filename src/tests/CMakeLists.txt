# Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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


add_subdirectory(unit-tests/signal-processing-blocks/libs)

################################################################################
# Google Test - https://github.com/google/googletest
################################################################################
if(EXISTS $ENV{GTEST_DIR})
   set(GTEST_DIR_LOCAL $ENV{GTEST_DIR})
endif(EXISTS $ENV{GTEST_DIR})
if(GTEST_DIR)
    set(GTEST_DIR_LOCAL ${GTEST_DIR})
endif(GTEST_DIR)
if(NOT GTEST_DIR_LOCAL)
    set(GTEST_DIR_LOCAL false)
else()
    set(GTEST_DIR_LOCAL true)
endif(NOT GTEST_DIR_LOCAL)

if(GTEST_INCLUDE_DIRS)
    set(GTEST_DIR_LOCAL true)
endif(GTEST_INCLUDE_DIRS)

set(GTEST_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
set(TOOLCHAIN_ARG "")

if(NOT ${GTEST_DIR_LOCAL})
     # if GTEST_DIR is not defined, we download and build it
     set(gtest_RELEASE 1.8.0)

     if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
         set(GTEST_COMPILER "")
         set(TOOLCHAIN_ARG -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Toolchains/oe-sdk_cross.cmake)
     endif(EXISTS $ENV{OECORE_TARGET_SYSROOT})

     ExternalProject_Add(
         gtest-${gtest_RELEASE}
         GIT_REPOSITORY https://github.com/google/googletest
         GIT_TAG release-${gtest_RELEASE}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${gtest_RELEASE}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${gtest_RELEASE}
         CMAKE_ARGS ${GTEST_COMPILER} -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF ${TOOLCHAIN_ARG}
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         INSTALL_COMMAND ""
     )
     # Set up variables
     # Set recently downloaded and build Googletest root folder
     set(GTEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${gtest_RELEASE}/googletest")
     # Source code
     ExternalProject_Get_Property(gtest-${gtest_RELEASE} source_dir)
     set(GTEST_INCLUDE_DIR ${source_dir}/googletest/include)
     set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
     # Library
     ExternalProject_Get_Property(gtest-${gtest_RELEASE} binary_dir)
     if(OS_IS_MACOSX)
         if(CMAKE_GENERATOR STREQUAL Xcode)
             set(binary_dir "${binary_dir}/Debug")
         endif(CMAKE_GENERATOR STREQUAL Xcode)
     endif(OS_IS_MACOSX)
     set(GTEST_LIBRARY_PATH "${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a;${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a")
     set(GTEST_LIBRARY gtest-${gtest_RELEASE})
     set(GTEST_LIBRARIES
          ${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a
          ${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a
     )
     set(GTEST_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${gtest_RELEASE}/googletest")
else(NOT ${GTEST_DIR_LOCAL})
     if(GTEST_INCLUDE_DIRS)
          set(GTEST_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS} ${LIBGTEST_DEV_DIR})
          add_library(gtest ${LIBGTEST_DEV_DIR}/src/gtest-all.cc ${LIBGTEST_DEV_DIR}/src/gtest_main.cc)
          set(GTEST_LIBRARIES gtest)
     else(GTEST_INCLUDE_DIRS)
          # If the variable GTEST_DIR is defined, we use the existing Googletest
          set(GTEST_DIR $ENV{GTEST_DIR})
          set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
          add_library(gtest ${GTEST_DIR}/src/gtest-all.cc ${GTEST_DIR}/src/gtest_main.cc)
          set(GTEST_LIBRARIES gtest)
     endif(GTEST_INCLUDE_DIRS)
endif(NOT ${GTEST_DIR_LOCAL})



################################################################################
# Optional libraries
################################################################################
set(GNSS_SDR_TEST_OPTIONAL_LIBS "")
set(GNSS_SDR_TEST_OPTIONAL_HEADERS "")

if(ENABLE_CUDA)
    set(GNSS_SDR_TEST_OPTIONAL_HEADERS ${GNSS_SDR_TEST_OPTIONAL_HEADERS} ${CUDA_INCLUDE_DIRS})
    set(GNSS_SDR_TEST_OPTIONAL_LIBS ${GNSS_SDR_TEST_OPTIONAL_LIBS} ${CUDA_LIBRARIES})
endif(ENABLE_CUDA)

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        set(GNSS_SDR_TEST_OPTIONAL_LIBS "${GNSS_SDR_TEST_OPTIONAL_LIBS};${GPERFTOOLS_LIBRARIES}")
        set(GNSS_SDR_TEST_OPTIONAL_HEADERS "${GNSS_SDR_TEST_OPTIONAL_HEADERS};${GPERFTOOLS_INCLUDE_DIR}")
    endif(GPERFTOOLS_FOUND)
endif(ENABLE_GPERFTOOLS)

if(Boost_VERSION LESS 105000)
     add_definitions(-DOLD_BOOST=1)
endif(Boost_VERSION LESS 105000)

if(OPENSSL_FOUND)
    add_definitions( -DUSE_OPENSSL_FALLBACK=1 )
endif(OPENSSL_FOUND)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CLANG_FLAGS "-stdlib=libc++ -std=c++11 -lc++")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

if(OPENCL_FOUND)
    add_definitions(-DOPENCL_BLOCKS_TEST=1)
endif(OPENCL_FOUND)

if (ENABLE_CUDA)
	add_definitions(-DCUDA_BLOCKS_TEST=1)
endif(ENABLE_CUDA)

################################################################################
# Optional generator
################################################################################
if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA)
   ExternalProject_Add(
      gnss-sim
      GIT_REPOSITORY https://bitbucket.org/jarribas/gnss-simulator
      GIT_TAG master
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gnss-sim
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim
      CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG}
      UPDATE_COMMAND ""
      PATCH_COMMAND ""
      INSTALL_COMMAND ""
   )
   set(SW_GENERATOR_BIN ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim/gnss_sim)
   add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
   add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/brdc3540.14n")
   add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/circle.csv")

   ################################################################################
   # Local installation of GPSTk http://www.gpstk.org/
   ################################################################################
   find_package(GPSTK)
   if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
      set(gpstk_RELEASE "2.9")
      ExternalProject_Add(
         gpstk-${gpstk_RELEASE}
         GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
         GIT_TAG v${gpstk_RELEASE}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${gpstk_RELEASE}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${gpstk_RELEASE}
         CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${gpstk_RELEASE}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
      )
      set(GPSTK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${gpstk_RELEASE}/install/include CACHE PATH "Local GPSTK headers")
      add_library(gpstk UNKNOWN IMPORTED)
      set_property(TARGET gpstk PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${gpstk_RELEASE}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
      add_dependencies(gpstk gpstk-${gpstk_RELEASE})
      set(GPSTK_BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${gpstk_RELEASE}/install/bin/ )
      add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
      set(gpstk_libs gpstk)
   else(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
       set(gpstk_libs ${GPSTK_LIBRARIES})
       set(GPSTK_INCLUDE_DIRS ${GPSTK_INCLUDE_DIR})
       set(GPSTK_BINDIR ${GPSTK_LIBRARY}/../bin/ )
       add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
   endif(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
endif(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA)


if(ENABLE_UNIT_TESTING_EXTRA)
   add_definitions(-DEXTRA_TESTS)
   if(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
      message(STATUS "Downloading some data files for testing...")
      file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/gps_l2c_m_prn7_5msps.dat ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat
           SHOW_PROGRESS
           EXPECTED_HASH MD5=a6fcbefe155137945d3c33c5ef7bd0f9 )
   endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
endif(ENABLE_UNIT_TESTING_EXTRA)
file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)

add_definitions(-DTEST_PATH="${CMAKE_SOURCE_DIR}/thirdparty/")

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
     ${CMAKE_SOURCE_DIR}/src/algorithms/data_type_adapter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/data_type_adapter/gnuradio_blocks
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
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${CMAKE_SOURCE_DIR}/src/tests/unit-tests/signal-processing-blocks/libs
     ${CMAKE_SOURCE_DIR}/src/tests/common-files
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${GNSS_SDR_TEST_OPTIONAL_HEADERS}
     ${GNSS_SDR_TEST_OPTIONAL_HEADERS}
)



################################################################################
# Unit testing
################################################################################
if(ENABLE_UNIT_TESTING)
    if( ${ARMADILLO_VERSION_STRING} STRGREATER "5.300")  # make sure interp1 is present
        add_definitions(-DMODERN_ARMADILLO)
    endif( ${ARMADILLO_VERSION_STRING} STRGREATER "5.300") 
    add_executable(run_tests ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)

    add_custom_command(TARGET run_tests POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:run_tests>
                       ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:run_tests>)

    target_link_libraries(run_tests ${CLANG_FLAGS}
                                ${Boost_LIBRARIES}
                                ${GFlags_LIBS}
                                ${GLOG_LIBRARIES}
                                ${GTEST_LIBRARIES}
                                ${GNURADIO_RUNTIME_LIBRARIES}
                                ${GNURADIO_BLOCKS_LIBRARIES}
                                ${GNURADIO_FILTER_LIBRARIES}
                                ${GNURADIO_ANALOG_LIBRARIES}
                                ${ARMADILLO_LIBRARIES}
                                ${VOLK_LIBRARIES}
                                channel_fsm
                                gnss_sp_libs
                                gnss_rx
                                gnss_system_parameters
                                signal_generator_blocks
                                signal_generator_adapters
                                pvt_gr_blocks
                                signal_processing_testing_lib
                                ${VOLK_GNSSSDR_LIBRARIES}
                                ${GNSS_SDR_TEST_OPTIONAL_LIBS}
    )
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(run_tests gtest-${gtest_RELEASE})
    endif(NOT ${GTEST_DIR_LOCAL})
endif(ENABLE_UNIT_TESTING)



#########################################################
#  Adding Tests to Ctest
#########################################################
set(CMAKE_CTEST_COMMAND ctest -V)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

add_executable(control_thread_test
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_message_factory_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_thread_test.cc
)
if(NOT ${ENABLE_PACKAGING})
     set_property(TARGET control_thread_test PROPERTY EXCLUDE_FROM_ALL TRUE)
endif(NOT ${ENABLE_PACKAGING})

target_link_libraries(control_thread_test ${Boost_LIBRARIES}
                                          ${GFlags_LIBS}
                                          ${GLOG_LIBRARIES}
                                          ${GTEST_LIBRARIES}
                                          gnss_sp_libs
                                          gnss_system_parameters
                                          gnss_rx
                                          ${VOLK_GNSSSDR_LIBRARIES}
                                          )

add_test(control_thread_test control_thread_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(control_thread_test gtest-${gtest_RELEASE})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(control_thread_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})
set_property(TEST control_thread_test PROPERTY TIMEOUT 30)

add_executable(flowgraph_test
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_flowgraph_test.cc
)
if(NOT ${ENABLE_PACKAGING})
     set_property(TARGET flowgraph_test PROPERTY EXCLUDE_FROM_ALL TRUE)
endif(NOT ${ENABLE_PACKAGING})

target_link_libraries(flowgraph_test ${Boost_LIBRARIES}
                                     ${GFlags_LIBS}
                                     ${GLOG_LIBRARIES}
                                     ${GTEST_LIBRARIES}
                                     gnss_sp_libs
                                     gnss_rx
                                     gnss_system_parameters
                                     ${VOLK_GNSSSDR_LIBRARIES}
                                     )

add_test(flowgraph_test flowgraph_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(flowgraph_test gtest-${gtest_RELEASE})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(flowgraph_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})
set_property(TEST flowgraph_test PROPERTY TIMEOUT 30)

add_executable(gnss_block_test
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/fir_filter_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/adapter/pass_through_test.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_block_factory_test.cc
)
if(NOT ${ENABLE_PACKAGING})
     set_property(TARGET gnss_block_test PROPERTY EXCLUDE_FROM_ALL TRUE)
endif(NOT ${ENABLE_PACKAGING})

target_link_libraries(gnss_block_test ${Boost_LIBRARIES}
                                      ${GFlags_LIBS}
                                      ${GLOG_LIBRARIES}
                                      ${GTEST_LIBRARIES}
                                      ${GNURADIO_RUNTIME_LIBRARIES}
                                      ${GNURADIO_BLOCKS_LIBRARIES}
                                      ${GNURADIO_FILTER_LIBRARIES}
                                      ${GNURADIO_ANALOG_LIBRARIES}
                                      gnss_sp_libs
                                      gnss_rx
                                      gnss_system_parameters
                                      # signal_generator_blocks
                                      ${VOLK_GNSSSDR_LIBRARIES}
                                      )

add_test(gnss_block_test gnss_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnss_block_test gtest-${gtest_RELEASE})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnss_block_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

add_executable(gnuradio_block_test
     ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
     ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc
)
if(NOT ${ENABLE_PACKAGING})
     set_property(TARGET gnuradio_block_test PROPERTY EXCLUDE_FROM_ALL TRUE)
endif(NOT ${ENABLE_PACKAGING})

target_link_libraries(gnuradio_block_test ${Boost_LIBRARIES}
                                      ${GFlags_LIBS}
                                      ${GLOG_LIBRARIES}
                                      ${GTEST_LIBRARIES}
                                      ${GNURADIO_RUNTIME_LIBRARIES}
                                      ${GNURADIO_BLOCKS_LIBRARIES}
                                      ${GNURADIO_FILTER_LIBRARIES}
                                      ${GNURADIO_ANALOG_LIBRARIES}
                                      gnss_sp_libs
                                      gnss_rx
                                      gnss_system_parameters
                                      # signal_generator_blocks
                                      ${VOLK_GNSSSDR_LIBRARIES}
                                      )

add_test(gnuradio_block_test gnuradio_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnuradio_block_test gtest-${gtest_RELEASE})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnuradio_block_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

# add_executable(acq_test ${EXCLUDE}
#      ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
#      ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/gps_l1_ca_pcps_acquisition_test.cc
#      ${CMAKE_CURRENT_SOURCE_DIR}/gnss_block/galileo_e1_pcps_ambiguous_acquisition_test.cc
# )
# target_link_libraries(acq_test ${Boost_LIBRARIES}
#                                ${GFlags_LIBS}
#                                ${GLOG_LIBRARIES}
#                                ${GTEST_LIBRARIES}
#                                ${GNURADIO_RUNTIME_LIBRARIES}
#                                ${GNURADIO_BLOCKS_LIBRARIES}
#                                ${GNURADIO_FILTER_LIBRARIES}
#                                ${GNURADIO_ANALOG_LIBRARIES}
#                                gnss_sp_libs
#                                gnss_rx
#                                gnss_system_parameters
#                                signal_generator_blocks
#                                ${VOLK_GNSSSDR_LIBRARIES}
#                                )
# add_test(acq_test acq_test)

add_executable(trk_test
               ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc
               )

if(NOT ${ENABLE_PACKAGING})
    set_property(TARGET trk_test PROPERTY EXCLUDE_FROM_ALL TRUE)
endif(NOT ${ENABLE_PACKAGING})

target_link_libraries(trk_test ${Boost_LIBRARIES}
                               ${GFlags_LIBS}
                               ${GLOG_LIBRARIES}
                               ${GTEST_LIBRARIES}
                               ${GNURADIO_RUNTIME_LIBRARIES}
                               ${GNURADIO_BLOCKS_LIBRARIES}
                               ${GNURADIO_FILTER_LIBRARIES}
                               ${GNURADIO_ANALOG_LIBRARIES}
                               gnss_sp_libs
                               gnss_rx
                               gnss_system_parameters
                               signal_generator_blocks
                               ${VOLK_GNSSSDR_LIBRARIES}
                               )
add_test(trk_test trk_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(trk_test gtest-${gtest_RELEASE})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(trk_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

add_dependencies(check control_thread_test flowgraph_test gnss_block_test
                 gnuradio_block_test trk_test)



################################################################################
# System testing
################################################################################
if(ENABLE_SYSTEM_TESTING)
    set(HOST_SYSTEM "Unknown")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(HOST_SYSTEM "GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCH_}")
    endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(HOST_SYSTEM "MacOS")
    endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_definitions(-DHOST_SYSTEM="${HOST_SYSTEM}")
    add_executable(ttff
        ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/ttff_gps_l1.cc )
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(ttff gtest-${gtest_RELEASE})
    else(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(ttff gtest)
    endif(NOT ${GTEST_DIR_LOCAL})
    target_link_libraries(ttff
        ${Boost_LIBRARIES}
        ${GFlags_LIBS}
        ${GLOG_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${GNURADIO_RUNTIME_LIBRARIES}
        ${GNURADIO_BLOCKS_LIBRARIES}
        ${GNURADIO_FILTER_LIBRARIES}
        ${GNURADIO_ANALOG_LIBRARIES}
        ${VOLK_GNSSSDR_LIBRARIES}
        gnss_sp_libs
        gnss_rx
        gnss_system_parameters
    )
    add_custom_command(TARGET ttff POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ttff>
        ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:ttff>
    )

    if(ENABLE_SYSTEM_TESTING_EXTRA)
        add_executable(obs_gps_l1_system_test ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/obs_gps_l1_system_test.cc)
        if(NOT ${GTEST_DIR_LOCAL})
            add_dependencies(obs_gps_l1_system_test gtest-${gtest_RELEASE} )
        else(NOT ${GTEST_DIR_LOCAL})
            add_dependencies(obs_gps_l1_system_test gtest)
        endif(NOT ${GTEST_DIR_LOCAL})
        include_directories(${GPSTK_INCLUDE_DIRS})
        target_link_libraries(obs_gps_l1_system_test ${GFlags_LIBS}
                                                     ${GLOG_LIBRARIES}
                                                     ${GTEST_LIBRARIES}
                                                     gnss_sp_libs
                                                     gnss_rx
                                                     ${gpstk_libs})
        add_custom_command(TARGET obs_gps_l1_system_test POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:obs_gps_l1_system_test>
                               ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:obs_gps_l1_system_test>
        )
    endif(ENABLE_SYSTEM_TESTING_EXTRA)
endif(ENABLE_SYSTEM_TESTING)

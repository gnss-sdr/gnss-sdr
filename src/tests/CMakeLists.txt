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
if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
    set(GTEST_COMPILER "")
    set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Toolchains/oe-sdk_cross.cmake")
endif(EXISTS $ENV{OECORE_TARGET_SYSROOT})

if(NOT ${GTEST_DIR_LOCAL})
     # if GTEST_DIR is not defined, we download and build it
     if(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         GIT_REPOSITORY https://github.com/google/googletest
         GIT_TAG release-${GNSSSDR_GTEST_LOCAL_VERSION}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         CMAKE_ARGS ${GTEST_COMPILER} -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF ${TOOLCHAIN_ARG}
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         INSTALL_COMMAND ""
      )
     else(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         GIT_REPOSITORY https://github.com/google/googletest
         GIT_TAG release-${GNSSSDR_GTEST_LOCAL_VERSION}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
         CMAKE_ARGS ${GTEST_COMPILER} -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF ${TOOLCHAIN_ARG}
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
                          ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
         INSTALL_COMMAND ""
      )
     endif(CMAKE_VERSION VERSION_LESS 3.2)

     # Set up variables
     # Set recently downloaded and build Googletest root folder
     set(GTEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest")
     # Source code
     ExternalProject_Get_Property(gtest-${GNSSSDR_GTEST_LOCAL_VERSION} source_dir)
     set(GTEST_INCLUDE_DIR ${source_dir}/googletest/include)
     set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
     # Library
     ExternalProject_Get_Property(gtest-${GNSSSDR_GTEST_LOCAL_VERSION} binary_dir)
     if(OS_IS_MACOSX)
         if(CMAKE_GENERATOR STREQUAL Xcode)
             set(binary_dir "${binary_dir}/Debug")
         endif(CMAKE_GENERATOR STREQUAL Xcode)
     endif(OS_IS_MACOSX)
     set(GTEST_LIBRARY_PATH "${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX};${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}")
     set(GTEST_LIBRARY gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
     set(GTEST_LIBRARIES
          ${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
          ${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
     )
     set(GTEST_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest")
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
    if(OS_IS_MACOSX)
       set(CLANG_FLAGS "-stdlib=libc++ -lc++")
    endif(OS_IS_MACOSX)
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

if(OPENCL_FOUND)
    add_definitions(-DOPENCL_BLOCKS_TEST=1)
endif(OPENCL_FOUND)

if(ENABLE_CUDA)
    add_definitions(-DCUDA_BLOCKS_TEST=1)
endif(ENABLE_CUDA)

if(ENABLE_FPGA)
    add_definitions(-DFPGA_BLOCKS_TEST=1)
endif(ENABLE_FPGA)

find_package(Gnuplot)
if(GNUPLOT_FOUND)
    add_definitions(-DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}")
endif(GNUPLOT_FOUND)

if(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13" )
     add_definitions( -DGR_GREATER_38=1 )
endif(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13" )


################################################################################
# Optional generator
################################################################################
if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)
   if(ENABLE_FPGA)
      set(CROSS_INSTALL_DIR "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
      if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
          set(CROSS_INSTALL_DIR "${CROSS_INSTALL_DIR} -DBOOST_ROOT=$ENV{OECORE_TARGET_SYSROOT}/usr")
      endif(EXISTS $ENV{OECORE_TARGET_SYSROOT})
   else(ENABLE_FPGA)
      set(CROSS_INSTALL_DIR "")
   endif(ENABLE_FPGA)
   ExternalProject_Add(
      gnss-sim
      GIT_REPOSITORY https://bitbucket.org/jarribas/gnss-simulator
      GIT_TAG ${GNSSSDR_GNSS_SIM_LOCAL_VERSION}
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gnss-sim
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim
      CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} ${CROSS_INSTALL_DIR}
      UPDATE_COMMAND ""
      PATCH_COMMAND ""
      INSTALL_COMMAND ""
   )
   if(ENABLE_INSTALL_TESTS)
      install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim/gnss_sim DESTINATION bin)
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/brdc3540.14n DESTINATION share/gnss-sim)
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/circle.csv DESTINATION share/gnss-sim)
      set(SW_GENERATOR_BIN ${CMAKE_INSTALL_PREFIX}/bin/gnss_sim)
      add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
      add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/brdc3540.14n")
      add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/circle.csv")
   else(ENABLE_INSTALL_TESTS)
      set(SW_GENERATOR_BIN ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim/gnss_sim)
      add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
      add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/brdc3540.14n")
      add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/circle.csv")
   endif(ENABLE_INSTALL_TESTS)

   ################################################################################
   # Local installation of GPSTk http://www.gpstk.org/
   ################################################################################
   find_package(GPSTK)
   if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
      message(STATUS "GPSTk v${GNSSSDR_GPSTK_LOCAL_VERSION} will be automatically donwloaded and built when doing 'make'.")
      if(NOT ENABLE_FPGA)
          if(CMAKE_VERSION VERSION_LESS 3.2)
           ExternalProject_Add(
             gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
             GIT_TAG v${GNSSSDR_GPSTK_LOCAL_VERSION}
             SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF
             UPDATE_COMMAND ""
             PATCH_COMMAND ""
           )
          else(CMAKE_VERSION VERSION_LESS 3.2)
           ExternalProject_Add(
             gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
             GIT_TAG v${GNSSSDR_GPSTK_LOCAL_VERSION}
             SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
             CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF
             BUILD_BYPRODUCTS ${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX}
             UPDATE_COMMAND ""
             PATCH_COMMAND ""
           )
          endif(CMAKE_VERSION VERSION_LESS 3.2)

          set(GPSTK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include CACHE PATH "Local GPSTK headers")
          add_library(gpstk UNKNOWN IMPORTED)
          set_property(TARGET gpstk PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
          add_dependencies(gpstk gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
          set(GPSTK_BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/bin/ )
          add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
          set(gpstk_libs gpstk)
          set(OWN_GPSTK True)
      else(NOT ENABLE_FPGA)
          message(STATUS "GPSTk has not been found, try to install it on target.")
          message(STATUS "Some extra tests requiring GPSTk will not be built.")
      endif(NOT ENABLE_FPGA)
   else(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
       set(gpstk_libs ${GPSTK_LIBRARIES})
       set(GPSTK_INCLUDE_DIRS ${GPSTK_INCLUDE_DIR})
       set(GPSTK_BINDIR ${GPSTK_LIBRARY}/../bin/ )
       add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
   endif(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
endif(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)


if(ENABLE_UNIT_TESTING_EXTRA)
   add_definitions(-DEXTRA_TESTS)
   if(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
      message(STATUS "Downloading some data files for testing...")
      file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/gps_l2c_m_prn7_5msps.dat ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat
           SHOW_PROGRESS
           EXPECTED_HASH MD5=a6fcbefe155137945d3c33c5ef7bd0f9 )
   endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
   if(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat)
      message(STATUS "Downloading some data files for testing...")
      file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat
           SHOW_PROGRESS
           EXPECTED_HASH MD5=ffb72fc63c116be58d5e5ccb1daaed3a )
   endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat)
   if(ENABLE_INSTALL_TESTS)
      install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat DESTINATION share/gnss-sdr/signal_samples)
      install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
   endif(ENABLE_INSTALL_TESTS)
endif(ENABLE_UNIT_TESTING_EXTRA)

if(ENABLE_INSTALL_TESTS)
   install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
   install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION share/gnss-sdr/signal_samples)
   install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION share/gnss-sdr/signal_samples)
   install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin DESTINATION share/gnss-sdr/signal_samples)
   add_definitions(-DTEST_PATH="${CMAKE_INSTALL_PREFIX}/share/gnss-sdr/")
else(ENABLE_INSTALL_TESTS)
   file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
   file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
   file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
   file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
   add_definitions(-DTEST_PATH="${CMAKE_SOURCE_DIR}/thirdparty/")
endif(ENABLE_INSTALL_TESTS)

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
     ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/observables/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/observables/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_generator/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/adapters
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${CMAKE_SOURCE_DIR}/src/tests/unit-tests/signal-processing-blocks/libs
     ${CMAKE_SOURCE_DIR}/src/tests/common-files
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${MATIO_INCLUDE_DIRS}
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
                                ${MATIO_LIBRARIES}
                                ${GNSS_SDR_TEST_OPTIONAL_LIBS}
    )
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(run_tests gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    endif(NOT ${GTEST_DIR_LOCAL})

    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/run_tests)
            file(REMOVE ${CMAKE_SOURCE_DIR}/install/run_tests)
        endif(EXISTS ${CMAKE_SOURCE_DIR}/install/run_tests)
        install(TARGETS run_tests RUNTIME DESTINATION bin COMPONENT "run_tests")
    else(ENABLE_INSTALL_TESTS)
        add_custom_command(TARGET run_tests POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:run_tests>
                           ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:run_tests>)
    endif(ENABLE_INSTALL_TESTS)
endif(ENABLE_UNIT_TESTING)

if(ENABLE_FPGA)
    add_executable(gps_l1_ca_dll_pll_tracking_test_fpga
                   ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                   ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc )
    target_link_libraries(gps_l1_ca_dll_pll_tracking_test_fpga
                             ${Boost_LIBRARIES}
                             ${GFlags_LIBS}
                             ${GLOG_LIBRARIES}
                             ${GTEST_LIBRARIES}
                             ${GNURADIO_RUNTIME_LIBRARIES}
                             ${GNURADIO_BLOCKS_LIBRARIES}
                             ${ARMADILLO_LIBRARIES}
                             ${VOLK_LIBRARIES}
                             ${MATIO_LIBRARIES}
                             channel_fsm
                             gnss_sp_libs
                             gnss_rx
                             gnss_system_parameters
                             signal_processing_testing_lib
                             )
    install(TARGETS gps_l1_ca_dll_pll_tracking_test_fpga
            RUNTIME DESTINATION bin
            COMPONENT "fpga-test" )
endif(ENABLE_FPGA)



################################################################################
# System testing
################################################################################
if(ENABLE_SYSTEM_TESTING)
    set(HOST_SYSTEM "Unknown")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(HOST_SYSTEM "GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCH_}")
        string(REPLACE "\n" "" HOST_SYSTEM "${HOST_SYSTEM}")
    endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(HOST_SYSTEM "MacOS")
    endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_definitions(-DHOST_SYSTEM="${HOST_SYSTEM}")
    set(TTFF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/ttff_gps_l1.cc)

    # Ensure that ttff is rebuilt if it was previously built and then removed
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
        execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${TTFF_SOURCES})
    endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)

    add_executable(ttff ${TTFF_SOURCES} )
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(ttff gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
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

    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
            file(REMOVE ${CMAKE_SOURCE_DIR}/install/ttff)
        endif(EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
        install(TARGETS ttff RUNTIME DESTINATION bin COMPONENT "ttff")
    else(ENABLE_INSTALL_TESTS)
        add_custom_command(TARGET ttff POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ttff>
                           ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:ttff> )
    endif(ENABLE_INSTALL_TESTS)

    if(ENABLE_SYSTEM_TESTING_EXTRA)
        set(POSITION_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/position_test.cc)
        # Ensure that position_test is rebuilt if it was previously built and then removed
        if(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
            execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${POSITION_TEST_SOURCES})
        endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
        add_executable(position_test ${POSITION_TEST_SOURCES})
        if(NOT ${GTEST_DIR_LOCAL})
             add_dependencies(position_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
        else(NOT ${GTEST_DIR_LOCAL})
             add_dependencies(position_test gtest)
        endif(NOT ${GTEST_DIR_LOCAL})
        target_link_libraries(position_test
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

        if(ENABLE_INSTALL_TESTS)
             if(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
                  file(REMOVE ${CMAKE_SOURCE_DIR}/install/position_test)
             endif(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
             install(TARGETS position_test RUNTIME DESTINATION bin COMPONENT "position_test")
        else(ENABLE_INSTALL_TESTS)
             add_custom_command(TARGET position_test POST_BUILD
                                COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:position_test>
                                ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:position_test> )
        endif(ENABLE_INSTALL_TESTS)

        if(GPSTK_FOUND OR OWN_GPSTK)
            set(OBS_GPS_L1_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/obs_gps_l1_system_test.cc)
            # Ensure that obs_gps_l1_system_test is rebuilt if it was previously built and then removed
            if(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
                execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${OBS_GPS_L1_TEST_SOURCES})
            endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
            add_executable(obs_gps_l1_system_test ${OBS_GPS_L1_TEST_SOURCES})

            set(OBS_SYSTEM_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/obs_system_test.cc)
            # Ensure that obs_system_test is rebuilt if it was previously built and then removed
            if(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
                execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${OBS_SYSTEM_TEST_SOURCES})
            endif(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
            add_executable(obs_system_test ${OBS_SYSTEM_TEST_SOURCES})

            if(NOT ${GTEST_DIR_LOCAL})
                add_dependencies(obs_gps_l1_system_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION} )
                add_dependencies(obs_system_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION} )
            else(NOT ${GTEST_DIR_LOCAL})
                add_dependencies(obs_gps_l1_system_test gtest)
                add_dependencies(obs_system_test gtest)
            endif(NOT ${GTEST_DIR_LOCAL})
            include_directories(${GPSTK_INCLUDE_DIRS} ${GPSTK_INCLUDE_DIRS}/gpstk)
            target_link_libraries(obs_gps_l1_system_test ${GFlags_LIBS}
                                                         ${GLOG_LIBRARIES}
                                                         ${GTEST_LIBRARIES}
                                                         gnss_sp_libs
                                                         gnss_rx
                                                         ${gpstk_libs})

            target_link_libraries(obs_system_test ${GFlags_LIBS}
                                                  ${GLOG_LIBRARIES}
                                                  ${GTEST_LIBRARIES}
                                                  gnss_sp_libs
                                                  gnss_rx
                                                  ${gpstk_libs})

            if(ENABLE_INSTALL_TESTS)
                if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
                    file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
                endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
                install(TARGETS obs_gps_l1_system_test RUNTIME DESTINATION bin COMPONENT "obs_gps_l1_system_test")

                if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
                    file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_system_test)
                endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
                install(TARGETS obs_system_test RUNTIME DESTINATION bin COMPONENT "obs_system_test")
            else(ENABLE_INSTALL_TESTS)
                add_custom_command(TARGET obs_gps_l1_system_test POST_BUILD
                                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:obs_gps_l1_system_test>
                                   ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:obs_gps_l1_system_test> )
                add_custom_command(TARGET obs_system_test POST_BUILD
                                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:obs_system_test>
                                   ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:obs_system_test> )

            endif(ENABLE_INSTALL_TESTS)
        endif(GPSTK_FOUND OR OWN_GPSTK)
    else(ENABLE_SYSTEM_TESTING_EXTRA)
        # Avoid working with old executables if they were switched ON and then OFF
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
           file(REMOVE ${CMAKE_SOURCE_DIR}/install/position_test)
        endif(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
           file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
        endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
           file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_system_test)
        endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
    endif(ENABLE_SYSTEM_TESTING_EXTRA)
else(ENABLE_SYSTEM_TESTING)
    # Avoid working with old executables if they were switched ON and then OFF
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
       file(REMOVE ${CMAKE_SOURCE_DIR}/install/ttff)
    endif(EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
        file(REMOVE ${CMAKE_SOURCE_DIR}/install/position_test)
    endif(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
       file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
    endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_gps_l1_system_test)
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
       file(REMOVE ${CMAKE_SOURCE_DIR}/install/obs_system_test)
    endif(EXISTS ${CMAKE_SOURCE_DIR}/install/obs_system_test)
endif(ENABLE_SYSTEM_TESTING)


#########################################################
#  Adding Tests to Ctest
#########################################################
set(CMAKE_CTEST_COMMAND ctest -V)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

add_executable(flowgraph_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                                   ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_flowgraph_test.cc )

target_link_libraries(flowgraph_test ${Boost_LIBRARIES}
                                     ${GFlags_LIBS}
                                     ${GLOG_LIBRARIES}
                                     ${GNURADIO_RUNTIME_LIBRARIES}
                                     ${GTEST_LIBRARIES}
                                     gnss_sp_libs
                                     gnss_rx
                                     gnss_system_parameters
                                     ${VOLK_GNSSSDR_LIBRARIES} )
add_test(flowgraph_test flowgraph_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(flowgraph_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(flowgraph_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})
set_property(TEST flowgraph_test PROPERTY TIMEOUT 30)

#########################################################

add_executable(gnss_block_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/fir_filter_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/pulse_blanking_filter_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/notch_filter_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/filter/notch_filter_lite_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/adapter/pass_through_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/adapter/adapter_test.cc
                               ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_block_factory_test.cc )
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
                                      ${VOLK_GNSSSDR_LIBRARIES} )
add_test(gnss_block_test gnss_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnss_block_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnss_block_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

#########################################################

add_executable(gnuradio_block_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                                   ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc )
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
                                          ${VOLK_GNSSSDR_LIBRARIES} )
add_test(gnuradio_block_test gnuradio_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnuradio_block_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnuradio_block_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

#########################################################

add_executable(matio_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                          ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/arithmetic/matio_test.cc )

target_link_libraries(matio_test ${Boost_LIBRARIES}
                                 ${GFlags_LIBS}
                                 ${GLOG_LIBRARIES}
                                 ${GTEST_LIBRARIES}
                                 ${MATIO_LIBRARIES}
                                 gnss_system_parameters )
add_test(matio_test matio_test)
if(NOT ${GTEST_DIR_LOCAL})
    if(MATIO_FOUND)
        add_dependencies(matio_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    else(MATIO_FOUND)
        add_dependencies(matio_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION} matio-${GNSSSDR_MATIO_LOCAL_VERSION})
    endif(MATIO_FOUND)
else(NOT ${GTEST_DIR_LOCAL})
   if(MATIO_FOUND)
        add_dependencies(matio_test gtest)
    else(MATIO_FOUND)
        add_dependencies(matio_test gtest matio-${GNSSSDR_MATIO_LOCAL_VERSION})
    endif(MATIO_FOUND)
endif(NOT ${GTEST_DIR_LOCAL})

#########################################################

add_executable(acq_test
      ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc )
target_link_libraries(acq_test ${Boost_LIBRARIES}
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
                               signal_processing_testing_lib
                               ${VOLK_GNSSSDR_LIBRARIES} )
add_test(acq_test acq_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(acq_test acq_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(acq_test acq_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

#########################################################

add_executable(trk_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc )

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
                               ${VOLK_GNSSSDR_LIBRARIES} )
add_test(trk_test trk_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(trk_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(trk_test gtest)
endif(NOT ${GTEST_DIR_LOCAL})

#########################################################

if(NOT ENABLE_PACKAGING)
    add_executable(control_thread_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
                                       ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_message_factory_test.cc
                                       ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_thread_test.cc )

    target_link_libraries(control_thread_test ${Boost_LIBRARIES}
                                              ${GFlags_LIBS}
                                              ${GLOG_LIBRARIES}
                                              ${GTEST_LIBRARIES}
                                              gnss_rx )

    add_test(control_thread_test control_thread_test)
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(control_thread_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    else(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(control_thread_test gtest)
    endif(NOT ${GTEST_DIR_LOCAL})
    set_property(TEST control_thread_test PROPERTY TIMEOUT 30)
endif(NOT ENABLE_PACKAGING)

#########################################################

if(ENABLE_PACKAGING)
    add_dependencies(check flowgraph_test gnss_block_test
                           gnuradio_block_test acq_test trk_test matio_test)
else(ENABLE_PACKAGING)
    add_dependencies(check control_thread_test flowgraph_test gnss_block_test
                           gnuradio_block_test acq_test trk_test matio_test)
endif(ENABLE_PACKAGING)
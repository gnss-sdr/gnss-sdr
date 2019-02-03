# Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


add_subdirectory(unit-tests/signal-processing-blocks/libs)
add_subdirectory(system-tests/libs)

################################################################################
# Google Test - https://github.com/google/googletest
################################################################################
if(EXISTS $ENV{GTEST_DIR})
    set(GTEST_DIR_LOCAL $ENV{GTEST_DIR})
endif()
if(GTEST_DIR)
    set(GTEST_DIR_LOCAL ${GTEST_DIR})
endif()
if(NOT GTEST_DIR_LOCAL)
    set(GTEST_DIR_LOCAL false)
else()
    set(GTEST_DIR_LOCAL true)
endif()

if(GTEST_INCLUDE_DIRS)
    set(GTEST_DIR_LOCAL true)
endif()

set(GTEST_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
set(TOOLCHAIN_ARG "")
if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
    set(GTEST_COMPILER "")
    set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Toolchains/oe-sdk_cross.cmake")
endif()

if(NOT ${GTEST_DIR_LOCAL})
    # if GTEST_DIR is not defined, we download and build it
    if(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
        gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG release-${GNSSSDR_GTEST_LOCAL_VERSION}
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        CMAKE_ARGS ${GTEST_COMPILER} -DINSTALL_GTEST=OFF -DBUILD_GMOCK=OFF ${TOOLCHAIN_ARG}
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        INSTALL_COMMAND ""
      )
    else()
      ExternalProject_Add(
        gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG release-${GNSSSDR_GTEST_LOCAL_VERSION}
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gtest/gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}
        CMAKE_ARGS ${GTEST_COMPILER} -DINSTALL_GTEST=OFF -DBUILD_GMOCK=OFF ${TOOLCHAIN_ARG}
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_STATIC_LIBRARY_SUFFIX}
            ${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX}
        INSTALL_COMMAND ""
      )
    endif()

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
            set(FINAL_D "d")
            set(ADD_DEBUG "Debug/")
        endif()
    endif()
    set(GTEST_LIBRARY_PATH "${binary_dir}/googletest/${ADD_DEBUG}${CMAKE_FIND_LIBRARY_PREFIXES}gtest${FINAL_D}${CMAKE_STATIC_LIBRARY_SUFFIX};${binary_dir}/googletest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${FINAL_D}${CMAKE_STATIC_LIBRARY_SUFFIX}")
    set(GTEST_LIBRARY gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    set(GTEST_LIBRARIES
        ${binary_dir}/googletest/${ADD_DEBUG}${CMAKE_FIND_LIBRARY_PREFIXES}gtest${FINAL_D}${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${binary_dir}/googletest/${ADD_DEBUG}${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${FINAL_D}${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${THREAD_LIBRARIES})
    set(GTEST_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../gtest-${GNSSSDR_GTEST_LOCAL_VERSION}/googletest")
else()
    if(GTEST_INCLUDE_DIRS)
        set(GTEST_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIRS} ${LIBGTEST_DEV_DIR})
        add_library(gtest ${LIBGTEST_DEV_DIR}/src/gtest-all.cc ${LIBGTEST_DEV_DIR}/src/gtest_main.cc)
        set(GTEST_LIBRARIES gtest)
    else()
        # If the variable GTEST_DIR is defined, we use the existing Googletest
        set(GTEST_DIR $ENV{GTEST_DIR})
        set(GTEST_INCLUDE_DIRECTORIES ${GTEST_DIR}/include ${GTEST_DIR} ${GTEST_DIR}/src)
        add_library(gtest ${GTEST_DIR}/src/gtest-all.cc ${GTEST_DIR}/src/gtest_main.cc)
        set(GTEST_LIBRARIES gtest)
    endif()
    target_link_libraries(gtest PUBLIC ${THREAD_LIBRARIES})
endif()



################################################################################
# Optional libraries
################################################################################
set(GNSS_SDR_TEST_OPTIONAL_LIBS "")
set(GNSS_SDR_TEST_OPTIONAL_HEADERS "")

if(ENABLE_CUDA)
    set(GNSS_SDR_TEST_OPTIONAL_HEADERS ${GNSS_SDR_TEST_OPTIONAL_HEADERS} ${CUDA_INCLUDE_DIRS})
    set(GNSS_SDR_TEST_OPTIONAL_LIBS ${GNSS_SDR_TEST_OPTIONAL_LIBS} ${CUDA_LIBRARIES})
endif()

if(ENABLE_GPERFTOOLS)
    if(GPERFTOOLS_FOUND)
        set(GNSS_SDR_TEST_OPTIONAL_LIBS "${GNSS_SDR_TEST_OPTIONAL_LIBS};${GPERFTOOLS_LIBRARIES}")
        set(GNSS_SDR_TEST_OPTIONAL_HEADERS "${GNSS_SDR_TEST_OPTIONAL_HEADERS};${GPERFTOOLS_INCLUDE_DIR}")
    endif()
endif()

if(Boost_VERSION LESS 105000)
    add_definitions(-DOLD_BOOST=1)
endif()

if(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13.4")
    add_definitions(-DGR_GREATER_38=1)
endif()

if(OPENSSL_FOUND)
    add_definitions(-DUSE_OPENSSL_FALLBACK=1)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(OS_IS_MACOSX)
        set(CLANG_FLAGS "-stdlib=libc++ -lc++")
    endif()
endif()

if(OPENCL_FOUND)
    add_definitions(-DOPENCL_BLOCKS_TEST=1)
endif()

if(ENABLE_CUDA)
    add_definitions(-DCUDA_BLOCKS_TEST=1)
endif()

if(ENABLE_FPGA)
    add_definitions(-DFPGA_BLOCKS_TEST=1)
endif()

find_package(Gnuplot)
if(GNUPLOT_FOUND)
    add_definitions(-DGNUPLOT_EXECUTABLE="${GNUPLOT_EXECUTABLE}")
endif()

if(ENABLE_UNIT_TESTING_MINIMAL)
    add_definitions(-DUNIT_TESTING_MINIMAL=1)
endif()


################################################################################
# Optional generator
################################################################################
option(ENABLE_GNSS_SIM_INSTALL "Enable the installation of gnss_sim on the fly" ON)
if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)
    if(ENABLE_FPGA)
      set(CROSS_INSTALL_DIR "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
      if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
          set(CROSS_INSTALL_DIR "${CROSS_INSTALL_DIR} -DBOOST_ROOT=$ENV{OECORE_TARGET_SYSROOT}/usr")
      endif()
    else()
      set(CROSS_INSTALL_DIR "")
    endif()
    find_package(GNSSSIMULATOR QUIET)
    if(GNSSSIMULATOR_FOUND OR NOT ENABLE_GNSS_SIM_INSTALL)
      add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
      add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/brdc3540.14n")
      add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_INSTALL_PREFIX}/share/gnss-sim/circle.csv")
    else()
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
        else()
            set(SW_GENERATOR_BIN ${CMAKE_CURRENT_BINARY_DIR}/../../gnss-sim/gnss_sim)
            add_definitions(-DSW_GENERATOR_BIN="${SW_GENERATOR_BIN}")
            add_definitions(-DDEFAULT_RINEX_NAV="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/brdc3540.14n")
            add_definitions(-DDEFAULT_POSITION_FILE="${CMAKE_CURRENT_BINARY_DIR}/../../../thirdparty/gnss-sim/circle.csv")
        endif()
    endif()

    ################################################################################
    # Local installation of GPSTk http://www.gpstk.org/
    ################################################################################
    find_package(GPSTK)
    if(NOT GPSTK_FOUND OR ENABLE_OWN_GPSTK)
      message(STATUS "GPSTk v${GNSSSDR_GPSTK_LOCAL_VERSION} will be automatically downloaded and built when doing 'make'.")
      if("${TOOLCHAIN_ARG}" STREQUAL "")
          set(TOOLCHAIN_ARG "-DCMAKE_CXX_FLAGS=\"-Wno-deprecated\"")
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
      endif()
      include(GNUInstallDirs)
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
      else()
          ExternalProject_Add(
            gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/SGL-UT/GPSTk
            GIT_TAG v${GNSSSDR_GPSTK_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}
            CMAKE_ARGS ${GTEST_COMPILER} ${TOOLCHAIN_ARG} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install -DBUILD_EXT=OFF -DBUILD_PYTHON=OFF
            BUILD_BYPRODUCTS ${CMAKE_SOURCE_DIR}/thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
          )
      endif()
      set(GPSTK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/include CACHE PATH "Local GPSTK headers")
      set(GPSTK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
      set(GPSTK_BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION}/install/bin/)
      add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
      set(OWN_GPSTK True)
      add_library(Gpstk::gpstk SHARED IMPORTED)
      add_dependencies(Gpstk::gpstk gpstk-${GNSSSDR_GPSTK_LOCAL_VERSION})
      file(MAKE_DIRECTORY ${GPSTK_INCLUDE_DIRS})
      set_target_properties(Gpstk::gpstk PROPERTIES
          IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
          IMPORTED_LOCATION "${GPSTK_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${GPSTK_INCLUDE_DIRS};${GPSTK_INCLUDE_DIRS}/gpstk"
          INTERFACE_LINK_LIBRARIES "${GPSTK_LIBRARY}"
      )
    else()
      set(GPSTK_BINDIR ${GPSTK_INCLUDE_DIR}/../bin/)
      add_definitions(-DGPSTK_BINDIR="${GPSTK_BINDIR}")
    endif()
endif()


if(ENABLE_UNIT_TESTING_EXTRA)
    add_definitions(-DEXTRA_TESTS)
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat)
      message(STATUS "Downloading some data files for testing...")
      file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/gps_l2c_m_prn7_5msps.dat ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=a6fcbefe155137945d3c33c5ef7bd0f9)
    endif()
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat)
      message(STATUS "Downloading some data files for testing...")
      file(DOWNLOAD https://sourceforge.net/projects/gnss-sdr/files/data/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat
            SHOW_PROGRESS
            EXPECTED_HASH MD5=ffb72fc63c116be58d5e5ccb1daaed3a)
    endif()
    if(ENABLE_INSTALL_TESTS)
      install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/gps_l2c_m_prn7_5msps.dat DESTINATION share/gnss-sdr/signal_samples)
      install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
    endif()
endif()

if(ENABLE_INSTALL_TESTS)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION share/gnss-sdr/signal_samples)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION share/gnss-sdr/signal_samples)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION share/gnss-sdr/signal_samples)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin DESTINATION share/gnss-sdr/signal_samples)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/data/rtklib_test/obs_test1.xml DESTINATION share/gnss-sdr/data/rtklib_test)
    install(FILES ${CMAKE_SOURCE_DIR}/src/tests/data/rtklib_test/eph_GPS_L1CA_test1.xml DESTINATION share/gnss-sdr/data/rtklib_test)
    add_definitions(-DTEST_PATH="${CMAKE_INSTALL_PREFIX}/share/gnss-sdr/")
else()
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/signal_samples)
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/data/rtklib_test/obs_test1.xml DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/data/rtklib_test)
    file(COPY ${CMAKE_SOURCE_DIR}/src/tests/data/rtklib_test/eph_GPS_L1CA_test1.xml DESTINATION ${CMAKE_SOURCE_DIR}/thirdparty/data/rtklib_test)
    add_definitions(-DTEST_PATH="${CMAKE_SOURCE_DIR}/thirdparty/")
endif()

set(LIST_INCLUDE_DIRS
    ${GTEST_INCLUDE_DIRECTORIES}
    ${CMAKE_SOURCE_DIR}/src/tests/common-files
    ${GNSS_SDR_TEST_OPTIONAL_HEADERS}
)

include_directories(${LIST_INCLUDE_DIRS})

################################################################################
# Unit testing
################################################################################
if(ENABLE_UNIT_TESTING)
    add_executable(run_tests ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cc)

    target_link_libraries(run_tests
        PUBLIC
            ${CLANG_FLAGS}
            Boost::filesystem
            Armadillo::armadillo
            Gflags::gflags
            Glog::glog
            Gnuradio::runtime
            Gnuradio::blocks
            Gnuradio::filter
            Gnuradio::analog
            Matio::matio
            Volk::volk
            Volkgnsssdr::volkgnsssdr
            ${GTEST_LIBRARIES}
            gnss_sp_libs
            gnss_rx
            gnss_system_parameters
            core_monitor_lib
            signal_source_adapters
            datatype_adapters
            input_filter_adapters
            resampler_adapters
            channel_adapters
            acq_adapters
            tracking_adapters
            tracking_lib
            telemetry_decoder_adapters
            obs_adapters
            signal_generator_adapters
            pvt_adapters
            signal_processing_testing_lib
            system_testing_lib
            ${GNSS_SDR_TEST_OPTIONAL_LIBS}
    )
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(run_tests gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    endif()
    if(ENABLE_UNIT_TESTING_EXTRA)
        target_link_libraries(run_tests PUBLIC Gpstk::gpstk)
    endif()
    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/run_tests)
            file(REMOVE ${CMAKE_SOURCE_DIR}/install/run_tests)
        endif()
        install(TARGETS run_tests RUNTIME DESTINATION bin COMPONENT "run_tests")
    else()
        add_custom_command(TARGET run_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:run_tests>
            ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:run_tests>)
    endif()
endif()

if(ENABLE_FPGA)
    add_executable(gps_l1_ca_dll_pll_tracking_test_fpga
        ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc
    )
    target_link_libraries(gps_l1_ca_dll_pll_tracking_test_fpga
        PUBLIC
            ${Boost_LIBRARIES}
            Gflags::gflags
            Glog::glog
            ${GTEST_LIBRARIES}
            Gnuradio::runtime
            Gnuradio::blocks
            Armadillo::armadillo
            Volk::volk
            Matio::matio
            channel_fsm
            gnss_sp_libs
            gnss_rx
            datatype_adapters
            gnss_system_parameters
            signal_processing_testing_lib
            tracking_adapters
    )
    install(TARGETS gps_l1_ca_dll_pll_tracking_test_fpga
        RUNTIME DESTINATION bin
        COMPONENT "fpga-test"
    )
endif()


################################################################################
# System testing
################################################################################
function(add_system_test executable)
    ## Please call this function with variables OPT_INCLUDES_ and OPT_LIBS_
    ## already defined.
    set(SYSTEM_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/system-tests/${executable}.cc)
    # Ensure that executable is rebuilt if it was previously built and then removed
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/install/${executable})
        execute_process(COMMAND ${CMAKE_COMMAND} -E touch ${SYSTEM_TEST_SOURCES})
    endif()
    add_executable(${executable} ${SYSTEM_TEST_SOURCES})

    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(${executable} gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    else()
        add_dependencies(${executable} gtest)
    endif()

    target_include_directories(${executable} PUBLIC ${OPT_INCLUDES_} ${CMAKE_SOURCES_DIR}/src/algorithms/libs)
    target_link_libraries(${executable} PUBLIC ${OPT_LIBS_} gnss_sp_libs)

    if(ENABLE_INSTALL_TESTS)
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/${executable})
            file(REMOVE ${CMAKE_SOURCE_DIR}/install/${executable})
        endif()
        install(TARGETS ${executable} RUNTIME DESTINATION bin COMPONENT "${executable}_test")
    else()
        add_custom_command(TARGET ${executable} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${executable}>
            ${CMAKE_SOURCE_DIR}/install/$<TARGET_FILE_NAME:${executable}>)
    endif()
endfunction()


if(ENABLE_SYSTEM_TESTING)
    set(HOST_SYSTEM "Unknown")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(HOST_SYSTEM "GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCH_}")
        string(REPLACE "\n" "" HOST_SYSTEM "${HOST_SYSTEM}")
        string(REPLACE "\"" "" HOST_SYSTEM "${HOST_SYSTEM}")
    endif()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(HOST_SYSTEM "MacOS")
    endif()
    add_definitions(-DHOST_SYSTEM="${HOST_SYSTEM}")

    #### TTFF
    set(OPT_LIBS_ ${Boost_LIBRARIES} ${THREAD_LIBRARIES} Gflags::gflags Glog::glog
                  Gnuradio::runtime ${GTEST_LIBRARIES}
                  Gnuradio::blocks Gnuradio::filter
                  Gnuradio::analog gnss_sp_libs
                  gnss_rx gnss_system_parameters)
    add_system_test(ttff)

    if(ENABLE_SYSTEM_TESTING_EXTRA)
        #### POSITION_TEST
        set(OPT_LIBS_ ${Boost_LIBRARIES} ${THREAD_LIBRARIES} Gflags::gflags Glog::glog
                      ${GTEST_LIBRARIES} Gnuradio::runtime
                      Gnuradio::blocks Gnuradio::filter
                      Gnuradio::analog Matio::matio Volkgnsssdr::volkgnsssdr
                      gnss_sp_libs gnss_rx gnss_system_parameters
                      system_testing_lib signal_processing_testing_lib)
        add_system_test(position_test)
    else()
        # Avoid working with old executables if they were switched ON and then OFF
        if(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
            file(REMOVE ${CMAKE_SOURCE_DIR}/install/position_test)
        endif()
    endif()
else()
    # Avoid working with old executables if they were switched ON and then OFF
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/ttff)
        file(REMOVE ${CMAKE_SOURCE_DIR}/install/ttff)
    endif()
    if(EXISTS ${CMAKE_SOURCE_DIR}/install/position_test)
        file(REMOVE ${CMAKE_SOURCE_DIR}/install/position_test)
    endif()
endif()


#########################################################
#  Adding Tests to Ctest
#########################################################
set(CMAKE_CTEST_COMMAND ctest -V)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

add_executable(flowgraph_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_flowgraph_test.cc)

target_link_libraries(flowgraph_test PUBLIC ${Boost_LIBRARIES}
    Gflags::gflags
    Glog::glog
    Gnuradio::runtime
    ${GTEST_LIBRARIES}
    gnss_sp_libs
    gnss_rx
    gnss_system_parameters
    signal_source_adapters
    signal_source_gr_blocks
    input_filter_adapters
    channel_adapters
    Volkgnsssdr::volkgnsssdr
)

target_include_directories(flowgraph_test PUBLIC ${CMAKE_SOURCE_DIR}/src/algorithms/libs)

add_test(flowgraph_test flowgraph_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(flowgraph_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else()
    add_dependencies(flowgraph_test gtest)
endif()
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
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/gnss_block_factory_test.cc
)
target_link_libraries(gnss_block_test PUBLIC ${Boost_LIBRARIES}
    Gflags::gflags
    Glog::glog
    ${GTEST_LIBRARIES}
    Gnuradio::runtime
    Gnuradio::blocks
    Gnuradio::filter
    Gnuradio::analog
    signal_source_adapters
    datatype_adapters
    input_filter_adapters
    channel_adapters
    gnss_sp_libs
    gnss_rx
    gnss_system_parameters
    Volkgnsssdr::volkgnsssdr
)
target_include_directories(gnss_block_test PUBLIC ${CMAKE_SOURCE_DIR}/src/algorithms/libs)
add_test(gnss_block_test gnss_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnss_block_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else()
    add_dependencies(gnss_block_test gtest)
endif()
set_property(TEST gnss_block_test PROPERTY TIMEOUT 60)

#########################################################

add_executable(gnuradio_block_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc
)
target_link_libraries(gnuradio_block_test ${Boost_LIBRARIES}
    Gflags::gflags
    Glog::glog
    ${GTEST_LIBRARIES}
    Gnuradio::runtime
    Gnuradio::blocks
    Gnuradio::filter
    Gnuradio::analog
    gnss_sp_libs
    gnss_rx
    gnss_system_parameters
    signal_source_gr_blocks
    Volkgnsssdr::volkgnsssdr
)
add_test(gnuradio_block_test gnuradio_block_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(gnuradio_block_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else()
    add_dependencies(gnuradio_block_test gtest)
endif()
set_property(TEST gnuradio_block_test PROPERTY TIMEOUT 30)

#########################################################

add_executable(matio_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/arithmetic/matio_test.cc
)
target_link_libraries(matio_test
    PUBLIC
        Gflags::gflags
        Glog::glog
        gnss_rx
        ${GTEST_LIBRARIES}
        gnss_system_parameters
        Matio::matio
)

add_test(matio_test matio_test)

set_property(TEST matio_test PROPERTY TIMEOUT 30)

#########################################################

add_executable(acq_test
    ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc)
target_link_libraries(acq_test
    PUBLIC
        ${Boost_LIBRARIES}
        Gflags::gflags
        Glog::glog
        ${GTEST_LIBRARIES}
        Gnuradio::runtime
        Gnuradio::blocks
        Gnuradio::filter
        Gnuradio::analog
        gnss_sp_libs
        gnss_rx
        gnss_system_parameters
        signal_generator_blocks
        acq_adapters
        signal_processing_testing_lib
        Volkgnsssdr::volkgnsssdr
)

add_test(acq_test acq_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(acq_test acq_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else()
    add_dependencies(acq_test acq_test gtest)
endif()
set_property(TEST acq_test PROPERTY TIMEOUT 30)

#########################################################

add_executable(trk_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/signal-processing-blocks/tracking/bayesian_estimation_test.cc
)

target_link_libraries(trk_test
    PUBLIC
        ${Boost_LIBRARIES}
        Gflags::gflags
        Glog::glog
        ${GTEST_LIBRARIES}
        Gnuradio::runtime
        Gnuradio::blocks
        Gnuradio::filter
        Gnuradio::analog
        gnss_sp_libs
        gnss_rx
        tracking_adapters
        gnss_system_parameters
        signal_generator_blocks
        Volkgnsssdr::volkgnsssdr
)
add_test(trk_test trk_test)
if(NOT ${GTEST_DIR_LOCAL})
    add_dependencies(trk_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
else()
    add_dependencies(trk_test gtest)
endif()
set_property(TEST trk_test PROPERTY TIMEOUT 30)


#########################################################

if(NOT ENABLE_PACKAGING)
    add_executable(control_thread_test ${CMAKE_CURRENT_SOURCE_DIR}/single_test_main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_message_factory_test.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/control-plane/control_thread_test.cc
    )

    target_link_libraries(control_thread_test ${Boost_LIBRARIES}
        Gflags::gflags
        Glog::glog
        signal_source_adapters
        gnss_sp_libs
        ${GTEST_LIBRARIES}
        gnss_rx
    )

    add_test(control_thread_test control_thread_test)
    if(NOT ${GTEST_DIR_LOCAL})
        add_dependencies(control_thread_test gtest-${GNSSSDR_GTEST_LOCAL_VERSION})
    else()
        add_dependencies(control_thread_test gtest)
    endif()
    set_property(TEST control_thread_test PROPERTY TIMEOUT 30)
endif()

#########################################################

if(ENABLE_PACKAGING)
    add_dependencies(check matio_test)
else()
    add_dependencies(check control_thread_test flowgraph_test gnss_block_test
        gnuradio_block_test acq_test trk_test matio_test)
endif()

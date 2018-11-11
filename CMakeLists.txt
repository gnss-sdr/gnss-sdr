# Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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

########################################################################
# Project setup
########################################################################
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(WARNING "In-tree build is bad practice. Try 'cd build && cmake ../' ")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

cmake_minimum_required(VERSION 2.8)
project(gnss-sdr CXX C)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
file(RELATIVE_PATH RELATIVE_CMAKE_CALL ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT CMAKE_PREFIX_PATH)
    if(DEFINED ENV{PYBOMBS_PREFIX})
        set(CMAKE_PREFIX_PATH $ENV{PYBOMBS_PREFIX})
    endif(DEFINED ENV{PYBOMBS_PREFIX})
    if(DEFINED ENV{SNAP})
        set(CMAKE_PREFIX_PATH $ENV{SNAP})
    endif(DEFINED ENV{SNAP})
endif(NOT CMAKE_PREFIX_PATH)



########################################################################
# Determine optional blocks/libraries to be built (default: not built)
# Enable them at the command line by doing 'cmake -DENABLE_XXX=ON ../'
########################################################################
# Support of optional RF front-ends
option(ENABLE_UHD "Enable the use of UHD (driver for all USRP devices)" ON)
option(ENABLE_OSMOSDR "Enable the use of OsmoSDR and other front-ends (RTL-based dongles, HackRF, bladeRF, etc.) as signal source" OFF)
option(ENABLE_FLEXIBAND "Enable the use of the signal source adater for the Teleorbit Flexiband GNURadio driver" OFF)
option(ENABLE_ARRAY "Enable the use of CTTC's antenna array front-end as signal source (experimental)" OFF)
option(ENABLE_GN3S "Enable the use of the GN3S dongle as signal source (experimental)" OFF)
option(ENABLE_PLUTOSDR "Enable the use of ADALM-PLUTO Evaluation Boards (Analog Devices Inc.), requires gr-iio" OFF)
option(ENABLE_FMCOMMS2 "Enable the use of FMCOMMS4-EBZ + ZedBoard hardware, requires gr-iio" OFF)
option(ENABLE_AD9361 "Enable the use of AD9361 directo to FPGA hardware, requires libiio" OFF)
option(ENABLE_RAW_UDP "Enable the use of high-optimized custom UDP packet sample source, requires libpcap" OFF)

# Performance analysis tools
option(ENABLE_GPERFTOOLS "Enable linking to Gperftools libraries (tcmalloc and profiler)" OFF)
option(ENABLE_GPROF "Enable the use of the GNU profiler tool 'gprof'" OFF)

# Acceleration
option(ENABLE_PROFILING "Enable execution of volk_gnsssdr_profile at the end of the building" OFF)
option(ENABLE_OPENCL "Enable building of processing blocks implemented with OpenCL (experimental)" OFF)
option(ENABLE_CUDA "Enable building of processing blocks implemented with CUDA (experimental, requires CUDA SDK)" OFF)
option(ENABLE_FPGA "Enable building of processing blocks implemented with FPGA (experimental, requires EZDMA driver)" OFF)

# Building and packaging options
option(ENABLE_GENERIC_ARCH "Builds a portable binary" OFF)
option(ENABLE_PACKAGING "Enable software packaging" OFF)
option(ENABLE_OWN_GLOG "Download glog and link it to gflags" OFF)
option(ENABLE_OWN_ARMADILLO "Download and build Armadillo locally" OFF)
option(ENABLE_LOG "Enable logging" ON)
if(ENABLE_PACKAGING)
    set(ENABLE_GENERIC_ARCH ON)
endif(ENABLE_PACKAGING)

# Testing
option(ENABLE_UNIT_TESTING "Build unit tests" ON)
option(ENABLE_UNIT_TESTING_MINIMAL "Build a minimal set of unit tests" OFF)
option(ENABLE_UNIT_TESTING_EXTRA "Download external files and build extra unit tests" OFF)
option(ENABLE_SYSTEM_TESTING "Build system tests" OFF)
option(ENABLE_SYSTEM_TESTING_EXTRA "Download external tools and build extra system tests" OFF)
if(ENABLE_SYSTEM_TESTING_EXTRA)
    set(ENABLE_SYSTEM_TESTING ON)
endif(ENABLE_SYSTEM_TESTING_EXTRA)
option(ENABLE_OWN_GPSTK "Force to download, build and link GPSTk for system tests, even if it is already installed" OFF)
option(ENABLE_INSTALL_TESTS "Install QA code system-wide" OFF)
if(ENABLE_FPGA)
    set(ENABLE_INSTALL_TESTS ON)
endif(ENABLE_FPGA)



###############################
# GNSS-SDR version information
###############################
set(THIS_IS_A_RELEASE OFF)   # only related to version name, no further implications.
if(NOT ${THIS_IS_A_RELEASE})
    # Get the current working branch
    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Get the latest abbreviated commit hash of the working branch
    execute_process(
        COMMAND git log -1 --format=%h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(NOT ${THIS_IS_A_RELEASE})

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_API_COMPAT    0)
if(${THIS_IS_A_RELEASE})
    set(VERSION_INFO_MINOR_VERSION 9)
else(${THIS_IS_A_RELEASE})
    set(VERSION_INFO_MINOR_VERSION 9.git-${GIT_BRANCH}-${GIT_COMMIT_HASH})
endif(${THIS_IS_A_RELEASE})

set(VERSION ${VERSION_INFO_MAJOR_VERSION}.${VERSION_INFO_API_COMPAT}.${VERSION_INFO_MINOR_VERSION})



########################################################################
# Environment setup
########################################################################
include(ExternalProject)
# Detect 64-bits machine
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
     set(ARCH_64BITS TRUE)
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

set(OS_IS_MACOSX "")
set(OS_IS_LINUX "")

# Detect Linux Distribution
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     set(OperatingSystem "Linux")
     set(OS_IS_LINUX TRUE)
     if(ARCH_64BITS)
         set(ARCH_ "(64 bits)")
     else(ARCH_64BITS)
         set(ARCH_ "(32 bits)")
     endif(ARCH_64BITS)
     if(EXISTS "/etc/lsb-release")
         execute_process(COMMAND cat /etc/lsb-release
                         COMMAND grep DISTRIB_ID
                         COMMAND awk -F= "{ print $2 }"
                         COMMAND tr "\n" " "
                         COMMAND sed "s/ //"
                         OUTPUT_VARIABLE LINUX_DISTRIBUTION
                         RESULT_VARIABLE LINUX_ID_RESULT
         )
         execute_process(COMMAND cat /etc/lsb-release
                         COMMAND grep DISTRIB_RELEASE
                         COMMAND awk -F= "{ print $2 }"
                         COMMAND tr "\n" " "
                         COMMAND sed "s/ //"
                         OUTPUT_VARIABLE LINUX_VER
                         RESULT_VARIABLE LINUX_VER_RESULT
         )
     endif(EXISTS "/etc/lsb-release")
     if(NOT LINUX_DISTRIBUTION)
         if(EXISTS "/etc/linuxmint/info")
             set(LINUX_DISTRIBUTION "LinuxMint")
             execute_process(COMMAND cat /etc/linuxmint/info
                             COMMAND grep -m1 RELEASE
                             COMMAND awk -F= "{ print $2 }"
                             COMMAND tr "\n" " "
                             COMMAND sed "s/ //"
                             OUTPUT_VARIABLE LINUX_VER
                             RESULT_VARIABLE LINUX_VER_RESULT
             )
         endif(EXISTS "/etc/linuxmint/info")
     endif(NOT LINUX_DISTRIBUTION)
     if(NOT LINUX_DISTRIBUTION)
         if(EXISTS "/etc/os-release")
             execute_process(COMMAND cat /etc/os-release
                             COMMAND grep -m1 NAME
                             COMMAND awk -F= "{ print $2 }"
                             COMMAND tr "\n" " "
                             COMMAND sed "s/ //"
                             OUTPUT_VARIABLE LINUX_DISTRIBUTION
                             RESULT_VARIABLE LINUX_ID_RESULT
             )
             execute_process(COMMAND cat /etc/os-release
                             COMMAND grep VERSION_ID
                             COMMAND awk -F= "{ print $2 }"
                             COMMAND tr "\n" " "
                             COMMAND sed "s/ //"
                             OUTPUT_VARIABLE LINUX_VER
                             RESULT_VARIABLE LINUX_VER_RESULT
             )
             if(${LINUX_DISTRIBUTION} MATCHES "Debian")
                 set(LINUX_DISTRIBUTION "Debian")
                 file(READ /etc/debian_version LINUX_VER)
             endif(${LINUX_DISTRIBUTION} MATCHES "Debian")
         endif(EXISTS "/etc/os-release")
     endif(NOT LINUX_DISTRIBUTION)
     if(NOT LINUX_DISTRIBUTION)
         if(EXISTS "/etc/redhat-release")
             set(LINUX_DISTRIBUTION "Red Hat")
             file (READ /etc/redhat-release LINUX_VER)
         endif(EXISTS "/etc/redhat-release")
     endif(NOT LINUX_DISTRIBUTION)
     if(NOT LINUX_DISTRIBUTION)
         if(EXISTS "/etc/debian_version")
             set(LINUX_DISTRIBUTION "Debian")
             file (READ /etc/debian_version LINUX_VER)
         endif(EXISTS "/etc/debian_version")
     endif(NOT LINUX_DISTRIBUTION)
     if(NOT LINUX_DISTRIBUTION)
         set(LINUX_DISTRIBUTION "Generic")
         set(LINUX_VER "Unknown")
     endif(NOT LINUX_DISTRIBUTION)
     message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on ${LINUX_DISTRIBUTION} GNU/Linux Release ${LINUX_VER} ${ARCH_}")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
if(NOT LINUX_DISTRIBUTION)
    set(LINUX_DISTRIBUTION "Unknown")
endif(NOT LINUX_DISTRIBUTION)

# Detect macOS / Mac OS X Version
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(OperatingSystem "Mac OS X")
     set(OS_IS_MACOSX TRUE)
     execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
     string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
     if(${DARWIN_VERSION} MATCHES "18")
         set(MACOS_MOJAVE TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS Mojave 10.14")
     endif(${DARWIN_VERSION} MATCHES "18")
     if(${DARWIN_VERSION} MATCHES "17")
         set(MACOS_HIGH_SIERRA TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS High Sierra 10.13")
     endif(${DARWIN_VERSION} MATCHES "17")
     if(${DARWIN_VERSION} MATCHES "16")
         set(MACOS_SIERRA TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS Sierra 10.12")
     endif(${DARWIN_VERSION} MATCHES "16")
     if(${DARWIN_VERSION} MATCHES "15")
         set(MACOSX_EL_CAPITAN TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.11 El Capitan")
     endif(${DARWIN_VERSION} MATCHES "15")
     if(${DARWIN_VERSION} MATCHES "14")
         set(MACOSX_YOSEMITE TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.10 Yosemite")
     endif(${DARWIN_VERSION} MATCHES "14")
     if(${DARWIN_VERSION} MATCHES "13")
         set(MACOSX_MAVERICKS TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION="com.apple.compilers.llvm.clang.1_0")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.9 Mavericks")
     endif(${DARWIN_VERSION} MATCHES "13")
     if(${DARWIN_VERSION} MATCHES "12")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.8 Mountain Lion")
     endif(${DARWIN_VERSION} MATCHES "12")
     if(${DARWIN_VERSION} MATCHES "11")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.7 Lion")
     endif(${DARWIN_VERSION} MATCHES "11")
     if(${DARWIN_VERSION} MATCHES "10")
         message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.6 Snow Leopard")
     endif(${DARWIN_VERSION} MATCHES "10")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# Define extra build types and select Release by default to get optimization flags
include(GnsssdrBuildTypes)
# Available options:
#  - None: nothing set
#  - Debug: -O2 -g
#  - Release: -O3
#  - RelWithDebInfo: -O3 -g
#  - MinSizeRel: -Os
#  - Coverage: -Wall -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage
#  - NoOptWithASM: -O0 -g -save-temps
#  - O2WithASM: -O2 -g -save-temps
#  - O3WithASM: -O3 -g -save-temps
#  - ASAN: -Wall -Wextra -g -O2 -fsanitize=address -fno-omit-frame-pointer
if(NOT CMAKE_BUILD_TYPE)
     if(ENABLE_GPERFTOOLS OR ENABLE_GPROF)
         set(CMAKE_BUILD_TYPE "RelWithDebInfo")
         message(STATUS "Build type not specified: defaulting to RelWithDebInfo.")
     else(ENABLE_GPERFTOOLS OR ENABLE_GPROF)
         set(CMAKE_BUILD_TYPE "Release")
         message(STATUS "Build type not specified: defaulting to Release.")
     endif(ENABLE_GPERFTOOLS OR ENABLE_GPROF)
else(NOT CMAKE_BUILD_TYPE)
     message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}.")
endif(NOT CMAKE_BUILD_TYPE)
GNSSSDR_CHECK_BUILD_TYPE(${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

# allow 'large' files in 32 bit builds
if(UNIX)
    add_definitions( -D_LARGEFILE_SOURCE
                     -D_FILE_OFFSET_BITS=64
                     -D_LARGE_FILES
                   )
endif(UNIX)

# Determine if we are using make or ninja
if(CMAKE_MAKE_PROGRAM MATCHES "make")
   set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "make")
endif(CMAKE_MAKE_PROGRAM MATCHES "make")
if(CMAKE_MAKE_PROGRAM MATCHES "ninja")
   set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "ninja")
endif(CMAKE_MAKE_PROGRAM MATCHES "ninja")
if(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)
   set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "${CMAKE_MAKE_PROGRAM}")
endif(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)



################################################################################
# Minimum required versions
################################################################################
set(GNSSSDR_CMAKE_MIN_VERSION "2.8.8")
set(GNSSSDR_GCC_MIN_VERSION "4.7.2")
set(GNSSSDR_CLANG_MIN_VERSION "3.4.0")
set(GNSSSDR_APPLECLANG_MIN_VERSION "500")
set(GNSSSDR_GNURADIO_MIN_VERSION "3.7.3")
set(GNSSSDR_BOOST_MIN_VERSION "1.45")
set(GNSSSDR_PYTHON_MIN_VERSION "2.7")
set(GNSSSDR_PYTHON3_MIN_VERSION "3.4")
set(GNSSSDR_MAKO_MIN_VERSION "0.4.2")
set(GNSSSDR_ARMADILLO_MIN_VERSION "5.300.0")
set(GNSSSDR_MATIO_MIN_VERSION "1.5.3")



################################################################################
# Versions to download and build (but not installed) if not found
################################################################################
set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.2.1")
set(GNSSSDR_GLOG_LOCAL_VERSION "0.3.5")
set(GNSSSDR_ARMADILLO_LOCAL_VERSION "9.200.x")
set(GNSSSDR_GTEST_LOCAL_VERSION "1.8.1")
set(GNSSSDR_GNSS_SIM_LOCAL_VERSION "master")
set(GNSSSDR_GPSTK_LOCAL_VERSION "2.10.6")
set(GNSSSDR_MATIO_LOCAL_VERSION "1.5.13")
set(GNSSSDR_PUGIXML_LOCAL_VERSION "1.9")


################################################################################
# Check cmake version
################################################################################
if(CMAKE_VERSION VERSION_LESS ${GNSSSDR_CMAKE_MIN_VERSION})
      message(STATUS "Your CMake version is too old and does not support some features required by GNSS-SDR. CMake version must be at least ${GNSSSDR_CMAKE_MIN_VERSION}. For more information check https://github.com/joakimkarlsson/bandit/issues/40")
      message(FATAL_ERROR "Fatal error: CMake >= ${GNSSSDR_CMAKE_MIN_VERSION} required.")
endif(CMAKE_VERSION VERSION_LESS ${GNSSSDR_CMAKE_MIN_VERSION})



################################################################################
# Check compiler version
################################################################################
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${GNSSSDR_GCC_MIN_VERSION})
          message(STATUS "Your GCC version is too old and does not support some C++ features required by GNSS-SDR. GCC version must be at least ${GNSSSDR_GCC_MIN_VERSION}")
          if(${LINUX_DISTRIBUTION} MATCHES "Ubuntu")
              if(${LINUX_VER} MATCHES "12.04")
                  message(STATUS "For instructions on how to upgrade GCC, check https://askubuntu.com/a/271561")
              endif(${LINUX_VER} MATCHES "12.04")
          endif(${LINUX_DISTRIBUTION} MATCHES "Ubuntu")
          message(FATAL_ERROR "Fatal error: GCC >= ${GNSSSDR_GCC_MIN_VERSION} required.")
      endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${GNSSSDR_GCC_MIN_VERSION})
endif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    execute_process(COMMAND
        ${CMAKE_CXX_COMPILER} -v
        RESULT_VARIABLE _res ERROR_VARIABLE _err
        ERROR_STRIP_TRAILING_WHITESPACE)
    if(${_res} STREQUAL "0")
        # output is in error stream
        string(REGEX MATCH "^Apple.*" IS_APPLE ${_err})
        if("${IS_APPLE}" STREQUAL "")
            set(MIN_VERSION ${GNSSSDR_CLANG_MIN_VERSION})
            set(APPLE_STR "")
            # retrieve the compiler's version from it
            string(REGEX MATCH "clang version [0-9.]+" CLANG_OTHER_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_OTHER_VERSION})
        else("${IS_APPLE}" STREQUAL "")
            set(MIN_VERSION ${GNSSSDR_APPLECLANG_MIN_VERSION})
            set(APPLE_STR "Apple ")
            # retrieve the compiler's version from it
            string(REGEX MATCH "(clang-[0-9.]+)" CLANG_APPLE_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_APPLE_VERSION})
        endif("${IS_APPLE}" STREQUAL "")
        if(${CLANG_VERSION} VERSION_LESS "${MIN_VERSION}")
            message(WARNING "\nThe compiler selected to build GNSS-SDR (${APPLE_STR}Clang version ${CLANG_VERSION} : ${CMAKE_CXX_COMPILER}) is older than that officially supported (${MIN_VERSION} minimum). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
        endif(${CLANG_VERSION} VERSION_LESS "${MIN_VERSION}")
    else(${_res} STREQUAL "0")
        message(WARNING "\nCannot determine the version of the compiler selected to build GNSS-SDR (${APPLE_STR}Clang : ${CMAKE_CXX_COMPILER}). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
    endif(${_res} STREQUAL "0")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")



################################################################################
# Check if the compiler defines the architecture as ARM
################################################################################
if(NOT OS_IS_MACOSX)
    if(CMAKE_CROSSCOMPILING)
        set(IS_ARM TRUE)
    else(CMAKE_CROSSCOMPILING)
        include(TestForARM)
    endif(CMAKE_CROSSCOMPILING)
endif(NOT OS_IS_MACOSX)



################################################################################
# Check for availability of SSE
################################################################################
if(NOT ENABLE_GENERIC_ARCH)
     include(TestForSSE)
endif(NOT ENABLE_GENERIC_ARCH)



################################################################################
# Find the POSIX thread (pthread) libraries
################################################################################
if(CMAKE_VERSION VERSION_LESS 3.1)
    find_package(Threads REQUIRED)
    link_libraries(${CMAKE_THREAD_LIBS_INIT})
else(CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    if(CMAKE_CROSSCOMPILING)
        set(THREADS_PREFER_PTHREAD_FLAG FALSE)
    else(CMAKE_CROSSCOMPILING)
        set(THREADS_PREFER_PTHREAD_FLAG TRUE)
    endif(CMAKE_CROSSCOMPILING)
    find_package(Threads REQUIRED)
    link_libraries(Threads::Threads)
endif(CMAKE_VERSION VERSION_LESS 3.1)



################################################################################
# Googletest - https://github.com/google/googletest
################################################################################
enable_testing()
if(ENABLE_UNIT_TESTING OR ENABLE_SYSTEM_TESTING)
    if(EXISTS $ENV{GTEST_DIR})
        set(GTEST_DIR $ENV{GTEST_DIR})
    endif(EXISTS $ENV{GTEST_DIR})
    if(GTEST_DIR)
        message(STATUS "Googletest root folder set at ${GTEST_DIR}")
        find_path(LIBGTEST_DEV_DIR NAMES src/gtest-all.cc PATHS ${GTEST_DIR})
        if(LIBGTEST_DEV_DIR)
            message (STATUS "Googletest has been found.")
        else(LIBGTEST_DEV_DIR)
            message (FATAL_ERROR " Googletest source code has not been found at ${GTEST_DIR}.")
        endif(LIBGTEST_DEV_DIR)
        find_path(GTEST_INCLUDE_DIRS NAMES gtest/gtest.h PATHS ${GTEST_DIR}/include)
    else(GTEST_DIR)
        find_path(LIBGTEST_DEV_DIR NAMES src/gtest-all.cc PATHS /usr/src/googletest/googletest /usr/src/gtest /usr/include/gtest /opt/local/src/gtest-1.7.0 )
        find_path(GTEST_INCLUDE_DIRS NAMES gtest/gtest.h PATHS /usr/include /opt/local/src/gtest-1.7.0/include)
        if(LIBGTEST_DEV_DIR)
            message (STATUS "Googletest package has been found.")
        else(LIBGTEST_DEV_DIR)
            message (STATUS " Googletest has not been found.")
            message (STATUS " Googletest v${GNSSSDR_GTEST_LOCAL_VERSION} will be downloaded and built automatically ")
            message (STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'. ")
        endif(LIBGTEST_DEV_DIR)
    endif(GTEST_DIR)
endif(ENABLE_UNIT_TESTING OR ENABLE_SYSTEM_TESTING)



################################################################################
# Boost - https://www.boost.org
################################################################################
if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") # Fedora 64-bit fix
endif(UNIX AND EXISTS "/usr/lib64")
set(Boost_ADDITIONAL_VERSIONS
    "1.45.0" "1.45" "1.46.0" "1.46"                 "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51"                 "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71" "1.72.0" "1.72" "1.73.0" "1.73" "1.74.0" "1.74"
    "1.75.0" "1.75" "1.76.0" "1.76" "1.77.0" "1.77" "1.78.0" "1.78" "1.79.0" "1.79"
)
set(Boost_USE_MULTITHREAD ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS date_time system filesystem thread serialization chrono REQUIRED)
if(NOT Boost_FOUND)
     message(FATAL_ERROR "Fatal error: Boost (version >=${GNSSSDR_BOOST_MIN_VERSION}) required.")
endif(NOT Boost_FOUND)



################################################################################
# GNU Radio - https://gnuradio.org
################################################################################
set(GR_REQUIRED_COMPONENTS RUNTIME ANALOG BLOCKS FFT FILTER PMT)
find_package(Gnuradio)
if(PC_GNURADIO_RUNTIME_VERSION)
   if(PC_GNURADIO_RUNTIME_VERSION VERSION_LESS ${GNSSSDR_GNURADIO_MIN_VERSION})
       set(GNURADIO_RUNTIME_FOUND)
       message(STATUS "The GNU Radio version installed in your system is too old.")
   endif(PC_GNURADIO_RUNTIME_VERSION VERSION_LESS ${GNSSSDR_GNURADIO_MIN_VERSION})
endif(PC_GNURADIO_RUNTIME_VERSION)
if(NOT GNURADIO_RUNTIME_FOUND)
   message(STATUS "CMake cannot find GNU Radio >= ${GNSSSDR_GNURADIO_MIN_VERSION}")
   if(OS_IS_LINUX)
       message("Go to https://github.com/gnuradio/pybombs")
       message("and follow the instructions to install GNU Radio in your system.")
       message(FATAL_ERROR "GNU Radio ${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
   endif(OS_IS_LINUX)
   if(OS_IS_MACOSX)
       message("You can install it easily via Macports:")
       message("  sudo port install gnuradio ")
       message("Alternatively, you can use homebrew:")
       message("  brew install gnuradio" )
       message(FATAL_ERROR "GNU Radio ${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
   endif(OS_IS_MACOSX)
endif(NOT GNURADIO_RUNTIME_FOUND)

if(NOT GNURADIO_ANALOG_FOUND)
    message(FATAL_ERROR "*** The gnuradio-analog library v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_BLOCKS_FOUND)
    message(FATAL_ERROR "*** The gnuradio-blocks library v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_FILTER_FOUND)
    message(FATAL_ERROR "*** The gnuradio-filter library v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_FFT_FOUND)
    message(FATAL_ERROR "*** The gnuradio-fft library v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_PMT_FOUND)
    message(FATAL_ERROR "*** The gnuradio-pmt library v${GNSSSDR_GNURADIO_MIN_VERSION} or later is required to build gnss-sdr")
endif()



################################################################################
# VOLK - Vector-Optimized Library of Kernels
################################################################################
find_package(Volk)
if(NOT VOLK_FOUND)
    message(FATAL_ERROR "*** VOLK is required to build gnss-sdr")
endif()



################################################################################
# Log4cpp - http://log4cpp.sourceforge.net/
################################################################################
find_package(Log4cpp)
if(NOT LOG4CPP_FOUND)
    message(FATAL_ERROR "*** Log4cpp is required to build gnss-sdr")
endif()



################################################################################
# volk_gnsssdr module - GNSS-SDR's own VOLK library
################################################################################
find_package(VolkGnssSdr)

if(NOT VOLK_GNSSSDR_FOUND)
    message(STATUS " volk_gnsssdr will be built along with gnss-sdr when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'")
    ###############################
    # Find Python required modules
    ###############################
    include(SetupPython) # sets PYTHON_EXECUTABLE and search for required modules

    if(NOT PYTHON_MIN_VER_FOUND)
        message(FATAL_ERROR "Python ${GNSSSDR_PYTHON_MIN_VERSION} or greater required to build VOLK_GNSSSDR")
    endif(NOT PYTHON_MIN_VER_FOUND)

    if(${PYTHON3})
        set(PYTHON_NAME "python3")
    else(${PYTHON3})
        set(PYTHON_NAME "python")
    endif(${PYTHON3})

    #  Mako
    if(NOT MAKO_FOUND)
        message(STATUS "Mako template library not found. See http://www.makotemplates.org/ ")
        message(STATUS " You can try to install it by typing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo yum install ${PYTHON_NAME}-mako")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(STATUS " sudo zypper install ${PYTHON_NAME}-Mako")
        else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo apt-get install ${PYTHON_NAME}-mako")
        endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(FATAL_ERROR "Mako templates required to build VOLK_GNSSSDR")
    endif(NOT MAKO_FOUND)

    # Six
    if(NOT SIX_FOUND)
        message(STATUS "python-six not found. See https://pythonhosted.org/six/ ")
        message(STATUS " You can try to install it by typing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo yum install ${PYTHON_NAME}-six")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(STATUS " sudo zypper install ${PYTHON_NAME}-six")
        else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo apt-get install ${PYTHON_NAME}-six")
        endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(FATAL_ERROR "six - python 2 and 3 compatibility library required to build VOLK_GNSSSDR")
    endif(NOT SIX_FOUND)

    set(READ_ENVIRO "")
    if(ENABLE_PACKAGING)
        if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
            set(STRIP_VOLK_GNSSSDR_PROFILE "-DENABLE_STRIP=ON -DCMAKE_VERBOSE_MAKEFILE=ON")
        endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
        if(NOT DEFINED ENV{PROTECT_PASSWORDS})
            set(READ_ENVIRO ${CMAKE_COMMAND} -E environment)
        endif(NOT DEFINED ENV{PROTECT_PASSWORDS})
    endif(ENABLE_PACKAGING)

    set(VOLK_GNSSSDR_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
    if(PYTHON_EXECUTABLE)
        set(USE_THIS_PYTHON "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}")
    endif(PYTHON_EXECUTABLE)
    if(OS_IS_MACOSX)
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(VOLK_GNSSSDR_BUILD_COMMAND "xcodebuild" "-configuration" "Debug" "-target")
        endif(CMAKE_GENERATOR STREQUAL Xcode)
    endif(OS_IS_MACOSX)

    if(CMAKE_CROSSCOMPILING)
        set(VOLK_GNSSSDR_COMPILER "")
    else(CMAKE_CROSSCOMPILING)
        set(VOLK_GNSSSDR_COMPILER -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    endif(CMAKE_CROSSCOMPILING)
    set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_COMPILER}
                                -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
                                -DENABLE_STATIC_LIBS=ON
                                -DENABLE_PROFILING=${ENABLE_PROFILING}
                                -DCMAKE_INCLUDE_PATH=${Boost_INCLUDE_DIR}
                                -DENABLE_ORC=OFF
                                ${STRIP_VOLK_GNSSSDR_PROFILE}
                                ${USE_THIS_PYTHON} )
    if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
        set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                                    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake
                                    -DCROSSCOMPILE_MULTILIB=TRUE )
    endif(EXISTS $ENV{OECORE_TARGET_SYSROOT})
    if(CMAKE_VERSION VERSION_LESS 3.2)
       ExternalProject_Add(volk_gnsssdr_module
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build
         CMAKE_ARGS ${READ_ENVIRO} ${VOLK_GNSSSDR_CMAKE_ARGS}
         DOWNLOAD_COMMAND ""
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         BUILD_COMMAND ${READ_ENVIRO} ${VOLK_GNSSSDR_BUILD_COMMAND} volk_gnsssdr_profile
         INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
         )
    else(CMAKE_VERSION VERSION_LESS 3.2)
       ExternalProject_Add(volk_gnsssdr_module
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build
         CMAKE_ARGS ${READ_ENVIRO} ${VOLK_GNSSSDR_CMAKE_ARGS}
         DOWNLOAD_COMMAND ""
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         BUILD_COMMAND ${READ_ENVIRO} ${VOLK_GNSSSDR_BUILD_COMMAND} volk_gnsssdr_profile
         BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}volk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}
                          ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
         INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
         )
    endif(CMAKE_VERSION VERSION_LESS 3.2)
    find_package(ORC)
    if(NOT ORC_FOUND)
         set(ORC_LIBRARIES "")
         set(ORC_INCLUDE_DIRS "")
    endif(NOT ORC_FOUND)

    add_library(volk_gnsssdr UNKNOWN IMPORTED)
    set_property(TARGET volk_gnsssdr PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/lib/libvolk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(VOLK_GNSSSDR_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build/include/;${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/include;${ORC_INCLUDE_DIRS}")
    set(VOLK_GNSSSDR_LIBRARIES volk_gnsssdr ${ORC_LIBRARIES})

    if(CMAKE_VERSION VERSION_LESS 3.2)
      add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile)
    else(CMAKE_VERSION VERSION_LESS 3.2)
      add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile
        BYPRODUCTS ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile)
    endif(CMAKE_VERSION VERSION_LESS 3.2)

    add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr-config-info
        ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr-config-info)
endif(NOT VOLK_GNSSSDR_FOUND)



################################################################################
# gflags - https://github.com/gflags/gflags
################################################################################
set(LOCAL_GFLAGS false)
find_package(GFlags)
if (NOT GFlags_FOUND)
     message (STATUS " gflags library has not been found.")
     message (STATUS " gflags v${GNSSSDR_GFLAGS_LOCAL_VERSION} will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")

     if(CMAKE_VERSION VERSION_LESS 3.2)
          ExternalProject_Add(
          gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          GIT_REPOSITORY git://github.com/gflags/gflags.git
          GIT_TAG v${GNSSSDR_GFLAGS_LOCAL_VERSION}
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          CMAKE_ARGS -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON -DBUILD_gflags_nothreads_LIB=OFF -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )
     else(CMAKE_VERSION VERSION_LESS 3.2)
     ExternalProject_Add(
          gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          GIT_REPOSITORY git://github.com/gflags/gflags.git
          GIT_TAG v${GNSSSDR_GFLAGS_LOCAL_VERSION}
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
          CMAKE_ARGS -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON -DBUILD_gflags_nothreads_LIB=OFF -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
          BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )
     endif(CMAKE_VERSION VERSION_LESS 3.2)

     set(GFlags_INCLUDE_DIRS
          ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include CACHE PATH "Local Gflags headers"
     )

     add_library(gflags UNKNOWN IMPORTED)
     set_property(TARGET gflags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX})
     add_dependencies(gflags gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
     set(GFlags_LIBS gflags)
     file(GLOB GFlags_SHARED_LIBS "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_SHARED_LIBRARY_SUFFIX}*")
     set(GFlags_LIBRARY gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
     set(GFlags_LIBRARY_PATH ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib )
     link_directories(${GFlags_LIBRARY_PATH})
     set(GFlags_lib ${GFlags_LIBS} CACHE FILEPATH "Local Gflags library")
     set(GFlags_LIBRARY_PATH ${GFlags_LIBS})
     set(LOCAL_GFLAGS true CACHE STRING "GFlags downloaded and built automatically" FORCE)
endif(NOT GFlags_FOUND)



################################################################################
# glog - https://github.com/google/glog
################################################################################
if(NOT ${ENABLE_OWN_GLOG})
     find_package(GLOG)
     if(GLOG_INCLUDE_DIRS)
        set(GLOG_FOUND ON)
     endif(GLOG_INCLUDE_DIRS)
endif(NOT ${ENABLE_OWN_GLOG})
set(glog_RELEASE ${GNSSSDR_GLOG_LOCAL_VERSION})
if (NOT GLOG_FOUND OR ${LOCAL_GFLAGS})
     message (STATUS " glog library has not been found")
     if(NOT GFlags_FOUND)
          message(STATUS " or it is likely not linked to gflags.")
     endif(NOT GFlags_FOUND)
     message (STATUS " glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")
     if(NOT ${LOCAL_GFLAGS})
         add_library(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} UNKNOWN IMPORTED)
         set_property(TARGET gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} PROPERTY IMPORTED_LOCATION "${GFlags_LIBS}")
     endif(NOT ${LOCAL_GFLAGS})
     set(TARGET_GFLAGS gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
     if(${LOCAL_GFLAGS})
         set(GFLAGS_LIBRARIES_TO_LINK ${GFlags_SHARED_LIBS})
         set(GFLAGS_LIBRARY_DIR_TO_LINK ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib)
     else(${LOCAL_GFLAGS})
         set(GFLAGS_LIBRARIES_TO_LINK ${GFlags_LIBS})
         set(GFLAGS_LIBRARY_DIR_TO_LINK ${GFlags_LIBRARY_DIRS})
     endif(${LOCAL_GFLAGS})

     if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
         file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
"#!/bin/sh
export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
export LDFLAGS=-L${GFLAGS_LIBRARY_DIR_TO_LINK}
export LIBS=\"${GFLAGS_LIBRARIES_TO_LINK} -lc++\"
export CXXFLAGS=\"-stdlib=libc++\"
export CC=clang
export CXX=clang++
cd ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/
aclocal
automake --add-missing
autoreconf -vfi
cd ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure")

      else(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
             file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
"#!/bin/sh
export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
export LDFLAGS=-L${GFLAGS_LIBRARY_DIR_TO_LINK}
export LIBS=${GFLAGS_LIBRARIES_TO_LINK}
cd ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/
aclocal
automake --add-missing
autoreconf -vfi
cd ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure")

     endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

     file(COPY ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
          DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
          FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                           GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

     set(GLOG_CONFIGURE ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure_with_gflags)

     # Ensure that aclocal and libtool are present
     if(OS_IS_LINUX)
         if(EXISTS "/usr/bin/libtoolize")
             if(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
                 # Everything ok, we can move on
             else(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
                 message(" aclocal has not been found.")
                 message(" You can try to install it by typing:")
                 if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                      message(" sudo yum groupinstall 'Development Tools'")
                 elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                      message(" sudo zypper install automake")
                 else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                      message(" sudo apt-get install automake")
                 endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                 message(FATAL_ERROR "aclocal is required to build glog from source")
             endif(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
         else(EXISTS "/usr/bin/libtoolize")
             message(" libtool has not been found.")
             message(" You can try to install it by typing:")
             if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                 message(" sudo yum groupinstall 'Development Tools'")
             elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                 message(" sudo zypper install libtoool")
             else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                 message(" sudo apt-get install libtool")
             endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(FATAL_ERROR "libtool is required to build glog from source")
         endif(EXISTS "/usr/bin/libtoolize")
     endif(OS_IS_LINUX)

     if(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         DEPENDS ${TARGET_GFLAGS}
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         GIT_REPOSITORY https://github.com/google/glog/
         GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
         BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}"
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         INSTALL_COMMAND ""
      )
     else(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         DEPENDS ${TARGET_GFLAGS}
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         GIT_REPOSITORY https://github.com/google/glog/
         GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
         CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
         BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}"
         BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
         UPDATE_COMMAND ""
         PATCH_COMMAND ""
         INSTALL_COMMAND ""
      )
     endif(CMAKE_VERSION VERSION_LESS 3.2)

     # Set up variables
     set(GLOG_INCLUDE_DIRS
          ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src/
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
     )
     set(GLOG_LIBRARIES
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
     )
     set(LOCAL_GLOG true CACHE STRING "Glog downloaded and built automatically" FORCE)
else(NOT GLOG_FOUND OR ${LOCAL_GFLAGS})
     add_library(glog-${GNSSSDR_GLOG_LOCAL_VERSION} UNKNOWN IMPORTED)
     set_property(TARGET glog-${GNSSSDR_GLOG_LOCAL_VERSION} PROPERTY IMPORTED_LOCATION "${GLOG_LIBRARIES}")
endif(NOT GLOG_FOUND OR ${LOCAL_GFLAGS})

if(NOT ENABLE_LOG)
     message(STATUS "Logging is not enabled")
     add_definitions(-DGOOGLE_STRIP_LOG=1)
endif(NOT ENABLE_LOG)



#############################################################################
# Check that BLAS (Basic Linear Algebra Subprograms) is found in the system
# See http://www.netlib.org/blas/
#############################################################################
find_library(BLAS blas)
if(NOT BLAS)
    message(" The BLAS library has not been found.")
    message(" You can try to install it by typing:")
    if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(" sudo yum install blas-devel")
    else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(" sudo apt-get install libblas-dev")
    endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
    message(FATAL_ERROR "BLAS is required to build gnss-sdr")
endif(NOT BLAS)



#############################################################################
# Check that LAPACK (Linear Algebra PACKage) is found in the system
# See http://www.netlib.org/lapack/
#############################################################################
find_library(LAPACK lapack)
if(NOT LAPACK)
    message(" The LAPACK library has not been found.")
    message(" You can try to install it by typing:")
    if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(" sudo yum install lapack-devel")
    elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
        message(" sudo zypper install lapack-devel")
    else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(" sudo apt-get install liblapack-dev")
    endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
    message(FATAL_ERROR "LAPACK is required to build gnss-sdr")
endif(NOT LAPACK)



################################################################################
# Armadillo - http://arma.sourceforge.net/
################################################################################
find_package(Armadillo)
if(ARMADILLO_FOUND)
    if(${ARMADILLO_VERSION_STRING} VERSION_LESS ${GNSSSDR_ARMADILLO_MIN_VERSION})
        set(ARMADILLO_FOUND false)
        set(ENABLE_OWN_ARMADILLO true)
    endif(${ARMADILLO_VERSION_STRING} VERSION_LESS ${GNSSSDR_ARMADILLO_MIN_VERSION})
endif(ARMADILLO_FOUND)

if(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)
     message(STATUS " Armadillo has not been found.")
     message(STATUS " Armadillo ${GNSSSDR_ARMADILLO_LOCAL_VERSION} will be downloaded and built automatically")
     message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'. ")
     set(armadillo_BRANCH ${GNSSSDR_ARMADILLO_LOCAL_VERSION})
     set(armadillo_RELEASE ${armadillo_BRANCH})

     #############################################
     # Check if GFORTRAN is found in the system
     #############################################
     if(NOT OS_IS_MACOSX)
         find_package(GFORTRAN)
         if(NOT GFORTRAN)
             message(STATUS "The gfortran library has not been found.")
             message(STATUS " You can try to install it by typing:")
             if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                 message(STATUS " sudo yum install gcc-fortran")
             elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                 message(STATUS " sudo zypper install gcc-fortran")
             else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                 message(STATUS " sudo apt-get install gfortran")
             endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(FATAL_ERROR "gfortran is required to build gnss-sdr")
         endif(NOT GFORTRAN)
     endif(NOT OS_IS_MACOSX)

     #############################################
     # Download and build Armadillo
     #############################################
     if(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         armadillo-${armadillo_RELEASE}
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
         GIT_REPOSITORY https://gitlab.com/conradsnicta/armadillo-code.git
         GIT_TAG ${armadillo_BRANCH}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
         CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DBUILD_SHARED_LIBS=OFF -DCMAKE_CXX_FLAGS=-std=c++11
         BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
         UPDATE_COMMAND ""
         INSTALL_COMMAND ""
      )
     else(CMAKE_VERSION VERSION_LESS 3.2)
      ExternalProject_Add(
         armadillo-${armadillo_RELEASE}
         PREFIX ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
         GIT_REPOSITORY https://gitlab.com/conradsnicta/armadillo-code.git
         GIT_TAG ${armadillo_BRANCH}
         SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}
         BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
         CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DBUILD_SHARED_LIBS=OFF -DCMAKE_CXX_FLAGS=-std=c++11
         BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
         BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
         UPDATE_COMMAND ""
         INSTALL_COMMAND ""
      )
     endif(CMAKE_VERSION VERSION_LESS 3.2)

     # Set up variables
     ExternalProject_Get_Property(armadillo-${armadillo_RELEASE} binary_dir)
     set(ARMADILLO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}/include )
     if(NOT GFORTRAN)
         set(GFORTRAN "")
     endif(NOT GFORTRAN)
     set(ARMADILLO_LIBRARIES ${BLAS} ${LAPACK} ${GFORTRAN} ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX})
     set(LOCAL_ARMADILLO true CACHE STRING "Armadillo downloaded and built automatically" FORCE)
     set(ARMADILLO_VERSION_STRING ${armadillo_RELEASE})
else(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)
     set(armadillo_RELEASE ${ARMADILLO_VERSION_STRING})
     add_library(armadillo-${armadillo_RELEASE} UNKNOWN IMPORTED)
     set_property(TARGET armadillo-${armadillo_RELEASE} PROPERTY IMPORTED_LOCATION "${ARMADILLO_LIBRARIES}")
endif(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)



################################################################################
# GnuTLS - https://www.gnutls.org/
################################################################################
find_package(GnuTLS)
find_library(GNUTLS_OPENSSL_LIBRARY NAMES gnutls-openssl libgnutls-openssl.so.27
                                    HINTS /usr/lib
                                          /usr/lib64
                                          /usr/local/lib
                                          /usr/local/lib64
                                          /opt/local/lib
                                          /usr/lib/x86_64-linux-gnu
                                          /usr/lib/aarch64-linux-gnu
                                          /usr/lib/arm-linux-gnueabihf
                                          /usr/lib/arm-linux-gnueabi
                                          /usr/lib/i386-linux-gnu
                                          /usr/lib/alpha-linux-gnu
                                          /usr/lib/hppa-linux-gnu
                                          /usr/lib/i386-gnu
                                          /usr/lib/i686-gnu
                                          /usr/lib/i686-linux-gnu
                                          /usr/lib/x86_64-kfreebsd-gnu
                                          /usr/lib/i686-kfreebsd-gnu
                                          /usr/lib/m68k-linux-gnu
                                          /usr/lib/mips-linux-gnu
                                          /usr/lib/mips64el-linux-gnuabi64
                                          /usr/lib/mipsel-linux-gnu
                                          /usr/lib/powerpc-linux-gnu
                                          /usr/lib/powerpc-linux-gnuspe
                                          /usr/lib/powerpc64-linux-gnu
                                          /usr/lib/powerpc64le-linux-gnu
                                          /usr/lib/s390x-linux-gnu
                                          /usr/lib/sparc64-linux-gnu
                                          /usr/lib/x86_64-linux-gnux32
                                          /usr/lib/sh4-linux-gnu
             )

if(NOT GNUTLS_OPENSSL_LIBRARY)
     message(STATUS "Looking for OpenSSL instead...")
     find_package(OpenSSL)
     if(OPENSSL_FOUND)
        set(GNUTLS_INCLUDE_DIR ${OPENSSL_INCLUDE_DIR})
        set(GNUTLS_LIBRARIES "")
        set(GNUTLS_OPENSSL_LIBRARY ${OPENSSL_SSL_LIBRARY})
     else(OPENSSL_FOUND)
         message(" The GnuTLS library with openssl compatibility enabled has not been found.")
         message(" You can try to install the required libraries by typing:")
         if(OS_IS_LINUX)
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(" sudo yum install openssl-devel")
            else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(" sudo apt-get install libgnutls-openssl-dev")
            endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
         endif(OS_IS_LINUX)
         if(OS_IS_MACOSX)
            message(" sudo port install gnutls")
         endif(OS_IS_MACOSX)
            message(FATAL_ERROR "GnuTLS libraries with openssl compatibility are required to build gnss-sdr")
     endif(OPENSSL_FOUND)
endif(NOT GNUTLS_OPENSSL_LIBRARY)



########################################################################
# Matio - https://github.com/tbeu/matio
########################################################################
find_package(MATIO)
if(NOT MATIO_FOUND OR MATIO_VERSION_STRING VERSION_LESS ${GNSSSDR_MATIO_MIN_VERSION})
    if(MATIO_FOUND)
         message(STATUS " Matio installed version (${MATIO_VERSION_STRING}) is too old (>= ${GNSSSDR_MATIO_MIN_VERSION} is required).")
    endif(MATIO_FOUND)
    message(STATUS " Matio v${GNSSSDR_MATIO_LOCAL_VERSION} will be downloaded and built automatically")
    message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'. ")
    find_package(ZLIB)
    if(ZLIB_FOUND)
        get_filename_component(ZLIB_BASE_DIR ${ZLIB_INCLUDE_DIRS} DIRECTORY)
        if(OS_IS_LINUX)
            if(NOT EXISTS "/usr/bin/libtoolize")
                message(" libtool has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install libtoool")
                else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo apt-get install libtool")
                endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(FATAL_ERROR "libtool is required to build matio from source")
            endif(NOT EXISTS "/usr/bin/libtoolize")
            if(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
                message(STATUS "Automake found.")
            else(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
                message(" aclocal has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install automake")
                else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo apt-get install automake")
                endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(FATAL_ERROR "aclocal is required to build matio from source")
             endif(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
        endif(OS_IS_LINUX)
        find_package(HDF5)
        if(HDF5_FOUND)
            list(GET HDF5_LIBRARIES 0 HDF5_FIRST_DIR)
            get_filename_component(HDF5_BASE_DIR2 ${HDF5_FIRST_DIR} DIRECTORY)
            get_filename_component(HDF5_BASE_DIR ${HDF5_BASE_DIR2} DIRECTORY)
            if(OS_IS_MACOSX)
                if(EXISTS /opt/local/include/hdf5.h)
                    set(HDF5_BASE_DIR /opt/local)
                endif(EXISTS /opt/local/include/hdf5.h)
                if(EXISTS /usr/local/include/hdf5.h)
                    set(HDF5_BASE_DIR /usr/local)
                endif(EXISTS /usr/local/include/hdf5.h)
            endif(OS_IS_MACOSX)
            if(CMAKE_VERSION VERSION_LESS 3.2)
              ExternalProject_Add(
                 matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                 PREFIX ${CMAKE_CURRENT_BINARY_DIR}/matio
                 GIT_REPOSITORY https://github.com/tbeu/matio
                 GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
                 SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                 UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/autogen.sh
                 CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/configure --with-hdf5=${HDF5_BASE_DIR} --with-zlib=${ZLIB_BASE_DIR} --with-default-file-ver=7.3 --enable-mat73=yes --prefix=<INSTALL_DIR>
                 BUILD_COMMAND make
              )
            else(CMAKE_VERSION VERSION_LESS 3.2)
              ExternalProject_Add(
                 matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                 PREFIX ${CMAKE_CURRENT_BINARY_DIR}/matio
                 GIT_REPOSITORY https://github.com/tbeu/matio
                 GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
                 SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                 UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/autogen.sh
                 CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/configure --with-hdf5=${HDF5_BASE_DIR} --with-zlib=${ZLIB_BASE_DIR} --with-default-file-ver=7.3 --enable-mat73=yes --prefix=<INSTALL_DIR>
                 BUILD_COMMAND make
                 BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_STATIC_LIBRARY_SUFFIX}
              )
            endif(CMAKE_VERSION VERSION_LESS 3.2)
            set(MATIO_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_STATIC_LIBRARY_SUFFIX} ${HDF5_LIBRARIES} ${ZLIB_LIBRARIES} )
            set(MATIO_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/matio/include )
            set(MATIO_LOCAL true)
        else(HDF5_FOUND)
            message(STATUS " The hdf5 library has not been found in your system.")
            message(STATUS " Please try to install it by doing:")
            if(OS_IS_MACOSX)
                message(STATUS "  $ sudo port install hdf5")
                message(STATUS " or")
                message(STATUS "  $ brew install hdf5")
            endif(OS_IS_MACOSX)
            if(OS_IS_LINUX)
                message(STATUS "  $ sudo apt-get install libhdf5-dev")
            endif(OS_IS_LINUX)
            message(FATAL_ERROR "*** The hdf5 library is required to build gnss-sdr")
        endif(HDF5_FOUND)
    else(ZLIB_FOUND)
        message(FATAL_ERROR "*** The zlib library is required to build gnss-sdr")
    endif(ZLIB_FOUND)
endif(NOT MATIO_FOUND OR MATIO_VERSION_STRING VERSION_LESS ${GNSSSDR_MATIO_MIN_VERSION})


################################################################################
# PugiXML - https://pugixml.org/
################################################################################
find_package(PugiXML QUIET)
if(PugiXML_FOUND)
    message(STATUS "PugiXML has been found. Reading of GSA Galileo almanac XML files enabled.")
else(PugiXML_FOUND)
    message(STATUS " PugiXML v${GNSSSDR_PUGIXML_LOCAL_VERSION} will be downloaded and built automatically when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set(PUGIXML_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
    set(TOOLCHAIN_ARG "")
    if(EXISTS $ENV{OECORE_TARGET_SYSROOT})
        set(PUGIXML_COMPILER "")
        set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake")
    endif(EXISTS $ENV{OECORE_TARGET_SYSROOT})
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(
            pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/pugixml
            GIT_REPOSITORY https://github.com/zeux/pugixml
            GIT_TAG v${GNSSSDR_PUGIXML_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            CMAKE_ARGS ${PUGIXML_COMPILER} ${TOOLCHAIN_ARG}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
    else(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(
            pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/pugixml
            GIT_REPOSITORY https://github.com/zeux/pugixml
            GIT_TAG v${GNSSSDR_PUGIXML_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            CMAKE_ARGS ${PUGIXML_COMPILER} ${TOOLCHAIN_ARG}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
            INSTALL_COMMAND ""
        )
    endif(CMAKE_VERSION VERSION_LESS 3.2)
    set(PUGIXML_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(PUGIXML_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/src )
    set(PUGIXML_LOCAL true)
endif(PugiXML_FOUND)


################################################################################
# USRP Hardware Driver (UHD) - OPTIONAL
################################################################################
if(ENABLE_UHD)
    find_package(UHD)
    if(NOT UHD_FOUND)
        set(ENABLE_UHD OFF)
        message(STATUS " The USRP Hardware Driver (UHD) signal source will not be built,")
        message(STATUS " so all USRP-based front-ends will not be usable.")
        message(STATUS " Please check https://files.ettus.com/manual/")
    else(NOT UHD_FOUND)
        set(GR_REQUIRED_COMPONENTS UHD)
        find_package(Gnuradio)
    endif(NOT UHD_FOUND)
endif(ENABLE_UHD)


################################################################################
# Doxygen - http://www.stack.nl/~dimitri/doxygen/index.html (OPTIONAL, used if found)
################################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
     message(STATUS "Doxygen found.")
     message(STATUS "You can build the documentation with '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'." )
     message(STATUS "When done, point your browser to ${CMAKE_BINARY_DIR}/html/index.html")
     set(HAVE_DOT ${DOXYGEN_DOT_FOUND})
     file(TO_NATIVE_PATH ${CMAKE_SOURCE_DIR} top_srcdir)
     file(TO_NATIVE_PATH ${CMAKE_BINARY_DIR} top_builddir)
     find_package(LATEX)
     if (PDFLATEX_COMPILER)
          set(GENERATE_PDF_DOCUMENTATION "YES")
          set(GNSSSDR_USE_MATHJAX "NO")
     else(PDFLATEX_COMPILER)
          set(GENERATE_PDF_DOCUMENTATION "NO")
          set(GNSSSDR_USE_MATHJAX "YES")
     endif(PDFLATEX_COMPILER)
     configure_file(${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile.in
          ${CMAKE_BINARY_DIR}/docs/doxygen/Doxyfile
          @ONLY
     )
     add_custom_target(doc
          ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/docs/doxygen/Doxyfile
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
          COMMENT "Generating API documentation with Doxygen." VERBATIM
     )
     if(LATEX_COMPILER)
          message(STATUS "'${CMAKE_MAKE_PROGRAM_PRETTY_NAME} pdfmanual' will generate a manual at ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf")
          add_custom_target(pdfmanual
               COMMAND ${CMAKE_MAKE_PROGRAM}
               COMMAND ${CMAKE_COMMAND} -E copy refman.pdf ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
               COMMAND ${CMAKE_MAKE_PROGRAM} clean
               DEPENDS doc
               WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/docs/latex
               COMMENT "Generating PDF manual with Doxygen." VERBATIM
          )
     endif(LATEX_COMPILER)
     message(STATUS "'${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc-clean' will clean the documentation.")
     add_custom_target(doc-clean
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/docs/html
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/docs/latex
          COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
          COMMENT "Cleaning documentation." VERBATIM
     )
else(DOXYGEN_FOUND)
     message(STATUS " Doxygen has not been found in your system.")
     message(STATUS " You can get nice code documentation by using it!")
     message(STATUS " Get it from http://www.stack.nl/~dimitri/doxygen/index.html")
     if(OS_IS_LINUX)
         if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(STATUS " or simply by doing 'sudo yum install doxygen-latex'.")
         else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(STATUS " or simply by doing 'sudo apt-get install doxygen-latex'.")
         endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
     endif(OS_IS_LINUX)
     if(OS_IS_MACOSX)
         message(STATUS " or simply by doing 'sudo port install doxygen +latex'.")
     endif(OS_IS_MACOSX)
endif(DOXYGEN_FOUND)



###############################################################################
# OpenCL (OPTIONAL)
###############################################################################
if(ENABLE_OPENCL)
    find_package(OpenCL)
    if($ENV{DISABLE_OPENCL})
        set(DISABLE_OPENCL TRUE)
    endif($ENV{DISABLE_OPENCL})
    if(DISABLE_OPENCL)
        set(OPENCL_FOUND FALSE)
    else(DISABLE_OPENCL)
        if(OPENCL_FOUND)
            message(STATUS "OpenCL has been found and will be used by some processing blocks")
            message(STATUS "You can disable OpenCL use by doing 'cmake -DENABLE_OPENCL=OFF ../' ")
        endif(OPENCL_FOUND)
    endif(DISABLE_OPENCL)
    if(ENABLE_GENERIC_ARCH)
        set(OPENCL_FOUND FALSE)
        message(STATUS "ENABLE_GENERIC_ARCH is set to ON so the use of OpenCL has been disabled.")
    endif(ENABLE_GENERIC_ARCH)
    if(NOT OPENCL_FOUND)
        message(STATUS "Processing blocks using OpenCL will not be built.")
    endif(NOT OPENCL_FOUND)
else(ENABLE_OPENCL)
    set(OPENCL_FOUND FALSE)
endif(ENABLE_OPENCL)



###############################################################################
# CUDA (OPTIONAL)
###############################################################################
if($ENV{CUDA_GPU_ACCEL})
    message(STATUS "CUDA_GPU_ACCEL environment variable found." )
    set(ENABLE_CUDA ON)
endif($ENV{CUDA_GPU_ACCEL})

if(ENABLE_CUDA)
    FIND_PACKAGE(CUDA REQUIRED)
    message(STATUS "NVIDIA CUDA GPU Acceleration will be enabled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_CUDA=OFF ../'" )
else(ENABLE_CUDA)
    message(STATUS "NVIDIA CUDA GPU Acceleration will be not enabled." )
    message(STATUS "Enable it with 'cmake -DENABLE_CUDA=ON ../' to add support for GPU-based acceleration using CUDA." )
endif(ENABLE_CUDA)

###############################################################################
# CUSTOM UDP PACKET SOURCE (OPTIONAL)
###############################################################################
if(ENABLE_RAW_UDP)
    message(STATUS "High-optimized custom UDP ip packet source will be enabled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_RAW_UDP=OFF ../'" )
else(ENABLE_RAW_UDP)
    message(STATUS "High-optimized custom UDP ip packet source will be enabled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_RAW_UDP=OFF ../'" )
endif(ENABLE_RAW_UDP)


###############################################################################
# FPGA (OPTIONAL)
###############################################################################
if(ENABLE_FPGA)
    message(STATUS "FPGA Acceleration will be enabled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_FPGA=OFF ../'" )
else(ENABLE_FPGA)
    message(STATUS "Fpga Acceleration will be not enabled." )
    message(STATUS "Enable it with 'cmake -DENABLE_FPGA=ON ../' to add support for GPU-based acceleration using the FPGA." )
endif(ENABLE_FPGA)



################################################################################
# Setup of optional drivers
################################################################################
if($ENV{GN3S_DRIVER})
    message(STATUS "GN3S_DRIVER environment variable found." )
    set(ENABLE_GN3S ON)
endif($ENV{GN3S_DRIVER})
if(GN3S_DRIVER)
    set(ENABLE_GN3S ON)
endif(GN3S_DRIVER)
if(ENABLE_GN3S)
     message(STATUS "The GN3S driver will be compiled.")
     message(STATUS "You can disable it with 'cmake -DENABLE_GN3S=OFF ../'" )
else(ENABLE_GN3S)
     message(STATUS "The (optional and experimental) GN3S driver is not enabled." )
     message(STATUS "Enable it with 'cmake -DENABLE_GN3S=ON ../' to add support for the GN3S dongle." )
endif(ENABLE_GN3S)

if($ENV{RAW_ARRAY_DRIVER})
    message(STATUS "RAW_ARRAY_DRIVER environment variable found." )
    set(ENABLE_ARRAY ON)
endif($ENV{RAW_ARRAY_DRIVER})

if(RAW_ARRAY_DRIVER)
    set(ENABLE_ARRAY ON)
endif(RAW_ARRAY_DRIVER)

if(ENABLE_ARRAY)
    message(STATUS "CTTC's Antenna Array front-end driver will be compiled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_ARRAY=OFF ../'" )
    # copy firmware to install folder
    # Build project gr-dbfcttc
else(ENABLE_ARRAY)
    message(STATUS "The (optional) CTTC's Antenna Array front-end driver is not enabled." )
    message(STATUS "Enable it with 'cmake -DENABLE_ARRAY=ON ../' to add support for the CTTC experimental array front-end." )
endif(ENABLE_ARRAY)

if($ENV{RTLSDR_DRIVER})
    message(STATUS "RTLSDR_DRIVER environment variable found." )
    set(ENABLE_OSMOSDR ON)
endif($ENV{RTLSDR_DRIVER})

if(ENABLE_OSMOSDR)
    find_package(GrOsmoSDR)
    if(GROSMOSDR_FOUND)
        message(STATUS "The driver for OsmoSDR and other front-ends (HackRF, bladeRF, Realtek's RTL2832U-based dongles, etc.) will be compiled." )
        message(STATUS "You can disable it with 'cmake -DENABLE_OSMOSDR=OFF ../'" )
    else(GROSMOSDR_FOUND)
        if(ENABLE_PACKAGING)
           message(WARNING "gr-osmosdr has not been found. Source blocks depending on it will NOT be built.")
        else(ENABLE_PACKAGING)
           message(FATAL_ERROR "gr-osmosdr required to build gnss-sdr with the optional OSMOSDR driver")
        endif(ENABLE_PACKAGING)
    endif(GROSMOSDR_FOUND)
else(ENABLE_OSMOSDR)
    message(STATUS "The (optional) driver for OsmoSDR and related front-ends is not enabled." )
    message(STATUS "Enable it with 'cmake -DENABLE_OSMOSDR=ON ../' to add support for OsmoSDR and other front-ends (HackRF, bladeRF, Realtek's RTL2832U-based USB dongles, etc.)" )
endif(ENABLE_OSMOSDR)

if($ENV{FLEXIBAND_DRIVER})
    message(STATUS "FLEXIBAND_DRIVER environment variable found." )
    set(ENABLE_FLEXIBAND ON)
endif($ENV{FLEXIBAND_DRIVER})

if(FLEXIBAND_DRIVER)
    set(ENABLE_FLEXIBAND ON)
endif(FLEXIBAND_DRIVER)

if(ENABLE_FLEXIBAND)
    message(STATUS "The Teleorbit Flexiband front-end source will be compiled." )
    message(STATUS "You can disable it with 'cmake -DENABLE_FLEXIBAND=OFF ../'" )
else(ENABLE_FLEXIBAND)
    message(STATUS "The (optional) Teleorbit Flexiband front-end driver adapter is not enabled." )
    message(STATUS "Enable it with 'cmake -DENABLE_FLEXIBAND=ON ../' to add support for the Teleorbit Flexiband front-end." )
endif(ENABLE_FLEXIBAND)



################################################################################
# GPerftools - https://github.com/gperftools/gperftools (OPTIONAL)
################################################################################
if(ENABLE_GPERFTOOLS)
    find_package(Gperftools)
    if ( NOT GPERFTOOLS_FOUND )
        message(STATUS "Although ENABLE_GPERFTOOLS has been set to ON, GPerftools has not been found.")
        message(STATUS "Binaries will be compiled without 'tcmalloc' and 'profiler' libraries.")
        message(STATUS "You can install GPerftools from https://github.com/gperftools/gperftools")
    else( NOT GPERFTOOLS_FOUND )
        message(STATUS "GPerftools libraries found." )
        message(STATUS "Binaries will be compiled with 'tcmalloc' and 'profiler' libraries.")
    endif( NOT GPERFTOOLS_FOUND )
endif(ENABLE_GPERFTOOLS)
if(ENABLE_GPERFTOOLS)
    # Set GPerftools related flags if it is available
    # See https://github.com/gperftools/gperftools/blob/master/README
    if(GPERFTOOLS_FOUND)
        if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")
        endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -fno-builtin")
        endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    endif(GPERFTOOLS_FOUND)
endif(ENABLE_GPERFTOOLS)



########################################################################
# GNU gprof (OPTIONAL) - https://sourceware.org/binutils/docs/gprof/
########################################################################
if(ENABLE_GPROF)
    #if(CMAKE_COMPILER_IS_GNUCXX)
        set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -pg")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
    #endif(CMAKE_COMPILER_IS_GNUCXX)
endif(ENABLE_GPROF)


########################################################################
# Set compiler flags
########################################################################
# Support of C++17 is still not possible due to pm_remez.h (solved in GNU Radio 3.8)
# Enable C++14 support in GCC >= 6.1.1
# Fallback to C++11 when using GCC < 6.1.1
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
        set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
    else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
        # if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
        # else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")
            # set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++17")
        # endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")
    endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
    set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wall -Wextra")  #Add warning flags: For "-Wall" see https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)

# Support of C++17 is still not possible due to pm_remez.h (solved in GNU Radio 3.8)
# Enable C++14 support in Clang >= 3.5.0 or AppleClang >= 600
# Fallback to C++11 if older version
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(OS_IS_MACOSX)
        # See https://trac.macports.org/wiki/XcodeVersionInfo for Apple Clang version equivalences
        if(CLANG_VERSION VERSION_LESS "600")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
        else(CLANG_VERSION VERSION_LESS "600")
            # if(CLANG_VERSION VERSION_LESS "900")
                set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
            # else(CLANG_VERSION VERSION_LESS "900")
                # set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++17")
            # endif(CLANG_VERSION VERSION_LESS "900")
        endif(CLANG_VERSION VERSION_LESS "600")
    else(OS_IS_MACOSX)
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
        else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
            # if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0.0")
                set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
            # else(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0.0")
                # set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++17")
            # endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0.0")
        endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
    endif(OS_IS_MACOSX)

    if(OS_IS_MACOSX)
        set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -stdlib=libc++")
    endif(OS_IS_MACOSX)
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

if(NOT (CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32) AND NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    if(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
        set(CMAKE_C_STANDARD 11)
        set(CMAKE_CXX_STANDARD 14)
    endif(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
endif(NOT (CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32) AND NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))

# Processor-architecture related flags
# See https://gcc.gnu.org/onlinedocs/gcc/x86-Options.html
if (NOT ARCH_COMPILER_FLAGS)
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
        if(OS_IS_MACOSX)
            set(ARCH_COMPILER_FLAGS "-march=corei7 -mfpmath=sse")
        else(OS_IS_MACOSX)
            if(NOT ${ENABLE_GENERIC_ARCH})
                if(IS_ARM) # ARM-specific options (https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html)
                    if(NOT CMAKE_CROSSCOMPILING)
                        if(ARM_VERSION STREQUAL "arm")
                            # Unknown arm version - try our best to detect
                            set(ARCH_COMPILER_FLAGS "-mcpu=native")
                        else(ARM_VERSION STREQUAL "arm")
                            set(ARCH_COMPILER_FLAGS "-march=${ARM_VERSION}")
	                    endif(ARM_VERSION STREQUAL "arm")
	                endif(NOT CMAKE_CROSSCOMPILING)
                else(IS_ARM)
                    set(ARCH_COMPILER_FLAGS "-march=native -mfpmath=sse")
	            endif(IS_ARM)
            endif(NOT ${ENABLE_GENERIC_ARCH})
        endif(OS_IS_MACOSX)
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
endif (NOT ARCH_COMPILER_FLAGS)
set(MY_CXX_FLAGS "${MY_CXX_FLAGS} ${ARCH_COMPILER_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
    # https://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)



########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)



########################################################################
# Add subdirectories (in order of deps)
########################################################################
add_subdirectory(src)

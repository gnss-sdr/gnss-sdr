# Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
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

################################################################################
# Project setup
################################################################################
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree build, it is bad practice.\nTry 'cd build && cmake ..' instead.")
endif()

cmake_minimum_required(VERSION 2.8.12...3.15)
project(gnss-sdr CXX C)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)



################################################################################
# Determine optional blocks/libraries to be built (default: not built)
# Enable them at the command line by doing 'cmake -DENABLE_XXX=ON ..'
################################################################################
include(FeatureSummary)

# Support of optional RF front-ends
option(ENABLE_UHD "Enable the use of UHD (driver for all USRP devices)" ON)

option(ENABLE_OSMOSDR "Enable the use of OsmoSDR and other front-ends (RTL-based dongles, HackRF, bladeRF, etc.) as signal source" OFF)

option(ENABLE_FMCOMMS2 "Enable the use of FMCOMMS4-EBZ + ZedBoard hardware, requires gr-iio" OFF)

option(ENABLE_PLUTOSDR "Enable the use of ADALM-PLUTO Evaluation Boards (Analog Devices Inc.), requires gr-iio" OFF)

option(ENABLE_AD9361 "Enable the use of AD9361 direct to FPGA hardware, requires libiio" OFF)

option(ENABLE_RAW_UDP "Enable the use of high-optimized custom UDP packet sample source, requires libpcap" OFF)

option(ENABLE_FLEXIBAND "Enable the use of the signal source adater for the Teleorbit Flexiband GNU Radio driver" OFF)

option(ENABLE_GN3S "Enable the use of the GN3S dongle as signal source (experimental)" OFF)

option(ENABLE_ARRAY "Enable the use of CTTC's antenna array front-end as signal source (experimental)" OFF)

# Performance analysis tools
option(ENABLE_GPERFTOOLS "Enable linking to Gperftools libraries (tcmalloc and profiler)" OFF)

option(ENABLE_GPROF "Enable the use of the GNU profiler tool 'gprof'" OFF)

# Code correctness
option(ENABLE_CLANG_TIDY "Enable the use of clang-tidy when compiling" OFF)

# Acceleration
option(ENABLE_PROFILING "Enable execution of volk_gnsssdr_profile at the end of the building" OFF)

option(ENABLE_OPENCL "Enable building of processing blocks implemented with OpenCL (experimental)" OFF)

option(ENABLE_CUDA "Enable building of processing blocks implemented with CUDA (experimental, requires CUDA SDK)" OFF)

option(ENABLE_FPGA "Enable building of processing blocks implemented with FPGA" OFF)

# Building and packaging options
option(ENABLE_GENERIC_ARCH "Builds a portable binary" OFF)

option(ENABLE_PACKAGING "Enable software packaging" OFF)

option(ENABLE_OWN_GLOG "Download glog and link it to gflags" OFF)

option(ENABLE_OWN_ARMADILLO "Download and build Armadillo locally" OFF)

option(ENABLE_LOG "Enable logging" ON)

if(ENABLE_PACKAGING)
    set(ENABLE_GENERIC_ARCH ON)
endif()

# Testing
option(ENABLE_UNIT_TESTING "Build unit tests" ON)

option(ENABLE_UNIT_TESTING_MINIMAL "Build a minimal set of unit tests" OFF)

option(ENABLE_UNIT_TESTING_EXTRA "Download external files and build extra unit tests" OFF)

option(ENABLE_SYSTEM_TESTING "Build system tests" OFF)

option(ENABLE_SYSTEM_TESTING_EXTRA "Download external tools and build extra system tests" OFF)

option(ENABLE_GNSS_SIM_INSTALL "Enable the installation of gnss_sim on the fly" ON)

if(NOT (ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA))
    set(ENABLE_GNSS_SIM_INSTALL OFF)
endif()

if(ENABLE_SYSTEM_TESTING_EXTRA)
    set(ENABLE_SYSTEM_TESTING ON)
endif()

option(ENABLE_OWN_GPSTK "Force to download, build and link GPSTk for system tests, even if it is already installed" OFF)

option(ENABLE_INSTALL_TESTS "Install QA code system-wide" OFF)
if(ENABLE_FPGA)
    set(ENABLE_INSTALL_TESTS ON)
endif()



################################################################################
# GNSS-SDR version information
################################################################################
set(THIS_IS_A_RELEASE ON)   # only related to version name, no further implications.
if(NOT ${THIS_IS_A_RELEASE})
    find_package(Git)
    set_package_properties(Git PROPERTIES
        URL "https://git-scm.com"
        DESCRIPTION "A free and open source distributed version control system (found: v${GIT_VERSION_STRING})"
        PURPOSE "Manage version control, get MINOR_VERSION name for version number."
        TYPE REQUIRED
    )
    if(GIT_FOUND)
        # was this info set in the CMake commandline?
        if(NOT GIT_BRANCH)
            # no: try to find it
            execute_process(
                COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_BRANCH
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
        endif(NOT GIT_BRANCH)
        # was this info set in the CMake commandline?
        if(NOT GIT_COMMIT_HASH)
            # Get the latest abbreviated commit hash of the working branch
            execute_process(
                COMMAND ${GIT_EXECUTABLE} log -1 --format=%h
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
        endif(NOT GIT_COMMIT_HASH)
    endif()
endif()

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_API_COMPAT 0)
if(${THIS_IS_A_RELEASE})
    set(VERSION_INFO_MINOR_VERSION 11)
else()
    set(VERSION_INFO_MINOR_VERSION 11.git-${GIT_BRANCH}-${GIT_COMMIT_HASH})
endif()

set(VERSION ${VERSION_INFO_MAJOR_VERSION}.${VERSION_INFO_API_COMPAT}.${VERSION_INFO_MINOR_VERSION})



################################################################################
# Environment setup
################################################################################
include(ExternalProject)

# Detect 64-bits machine
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH_64BITS TRUE)
endif()

# Set prefix path for PyBOMBS and Snaps, if defined in environment variables
if(NOT CMAKE_PREFIX_PATH)
    if(DEFINED ENV{PYBOMBS_PREFIX})
        set(CMAKE_PREFIX_PATH $ENV{PYBOMBS_PREFIX})
    endif()
    if(DEFINED ENV{SNAP})
        set(CMAKE_PREFIX_PATH $ENV{SNAP})
    endif()
endif()

# Detect Linux Distribution
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(OperatingSystem "Linux")
    set(OS_IS_LINUX TRUE)
    if(ARCH_64BITS)
        set(ARCH_ "(64 bits)")
    else()
        set(ARCH_ "(32 bits)")
    endif()
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
    endif()
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
        endif()
    endif()
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
            endif()
        endif()
    endif()
    if(NOT LINUX_DISTRIBUTION)
        if(EXISTS "/etc/redhat-release")
            set(LINUX_DISTRIBUTION "Red Hat")
            file(READ /etc/redhat-release LINUX_VER)
        endif()
    endif()
    if(NOT LINUX_DISTRIBUTION)
        if(EXISTS "/etc/debian_version")
            set(LINUX_DISTRIBUTION "Debian")
            file(READ /etc/debian_version LINUX_VER)
        endif()
    endif()
    if(NOT LINUX_DISTRIBUTION)
        set(LINUX_DISTRIBUTION "Generic")
        set(LINUX_VER "Unknown")
    endif()
    message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on ${LINUX_DISTRIBUTION} GNU/Linux Release ${LINUX_VER} ${ARCH_}")
endif()
if(NOT LINUX_DISTRIBUTION)
    set(LINUX_DISTRIBUTION "Unknown")
endif()

# Detect macOS / Mac OS X Version
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(OperatingSystem "Mac OS X")
    set(OS_IS_MACOSX TRUE)
    execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
    string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
    if(${DARWIN_VERSION} MATCHES "19")
        set(MACOS_CATALINA TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS Catalina 10.15")
    endif()
    if(${DARWIN_VERSION} MATCHES "18")
        set(MACOS_MOJAVE TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS Mojave 10.14")
    endif()
    if(${DARWIN_VERSION} MATCHES "17")
        set(MACOS_HIGH_SIERRA TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS High Sierra 10.13")
    endif()
    if(${DARWIN_VERSION} MATCHES "16")
        set(MACOS_SIERRA TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on macOS Sierra 10.12")
    endif()
    if(${DARWIN_VERSION} MATCHES "15")
        set(MACOSX_EL_CAPITAN TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.11 El Capitan")
    endif()
    if(${DARWIN_VERSION} MATCHES "14")
        set(MACOSX_YOSEMITE TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.10 Yosemite")
    endif()
    if(${DARWIN_VERSION} MATCHES "13")
        set(MACOSX_MAVERICKS TRUE)
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
        set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION="com.apple.compilers.llvm.clang.1_0")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.9 Mavericks")
    endif()
    if(${DARWIN_VERSION} MATCHES "12")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.8 Mountain Lion")
    endif()
    if(${DARWIN_VERSION} MATCHES "11")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.7 Lion")
    endif()
    if(${DARWIN_VERSION} MATCHES "10")
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on Mac OS X 10.6 Snow Leopard")
    endif()
endif()

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
    else()
        set(CMAKE_BUILD_TYPE "Release")
        message(STATUS "Build type not specified: defaulting to Release.")
    endif()
else()
    message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}.")
endif()
gnsssdr_check_build_type(${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

# allow 'large' files in 32 bit builds
if(UNIX)
    add_definitions(-D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES)
endif()

# Determine if we are using make or ninja
if(CMAKE_MAKE_PROGRAM MATCHES "make")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "make")
endif()
if(CMAKE_MAKE_PROGRAM MATCHES "ninja")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "ninja")
endif()
if(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "${CMAKE_MAKE_PROGRAM}")
endif()



################################################################################
# Minimum required versions
################################################################################
set(GNSSSDR_GCC_MIN_VERSION "4.7.2")
set(GNSSSDR_CLANG_MIN_VERSION "3.4.0")
set(GNSSSDR_APPLECLANG_MIN_VERSION "500")
set(GNSSSDR_GNURADIO_MIN_VERSION "3.7.3")
set(GNSSSDR_BOOST_MIN_VERSION "1.53")
set(GNSSSDR_PYTHON_MIN_VERSION "2.7")
set(GNSSSDR_PYTHON3_MIN_VERSION "3.4")
set(GNSSSDR_MAKO_MIN_VERSION "0.4.2")
set(GNSSSDR_ARMADILLO_MIN_VERSION "5.300.0")
set(GNSSSDR_MATIO_MIN_VERSION "1.5.3")
set(GNSSSDR_PROTOBUF_MIN_VERSION "3.0.0")



################################################################################
# Versions to download and build (but not installed) if not found
################################################################################
set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.2.2")
set(GNSSSDR_GLOG_LOCAL_VERSION "0.4.0")
set(GNSSSDR_ARMADILLO_LOCAL_VERSION "9.600.x")
set(GNSSSDR_GTEST_LOCAL_VERSION "1.8.1")
set(GNSSSDR_GNSS_SIM_LOCAL_VERSION "master")
set(GNSSSDR_GPSTK_LOCAL_VERSION "2.10.6")
set(GNSSSDR_MATIO_LOCAL_VERSION "1.5.17")
set(GNSSSDR_PUGIXML_LOCAL_VERSION "1.9")
set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "3.9.0")

if(CMAKE_VERSION VERSION_LESS "3.0.2") # Fix for CentOS 7
    set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.2.1")
endif()



################################################################################
# Check compiler version
################################################################################
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${GNSSSDR_GCC_MIN_VERSION})
        message(STATUS "Your GCC version is too old and does not support some C++ features required by GNSS-SDR. GCC version must be at least ${GNSSSDR_GCC_MIN_VERSION}")
        message(FATAL_ERROR "Fatal error: GCC >= ${GNSSSDR_GCC_MIN_VERSION} required.")
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    execute_process(COMMAND
        ${CMAKE_CXX_COMPILER} -v
        RESULT_VARIABLE _res ERROR_VARIABLE _err
        ERROR_STRIP_TRAILING_WHITESPACE
    )
    if(${_res} STREQUAL "0")
        # output is in error stream
        string(REGEX MATCH "^Apple.*" IS_APPLE ${_err})
        if("${IS_APPLE}" STREQUAL "")
            set(MIN_VERSION ${GNSSSDR_CLANG_MIN_VERSION})
            set(APPLE_STR "")
            # retrieve the compiler's version from it
            string(REGEX MATCH "clang version [0-9.]+" CLANG_OTHER_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_OTHER_VERSION})
        else()
            set(MIN_VERSION ${GNSSSDR_APPLECLANG_MIN_VERSION})
            set(APPLE_STR "Apple ")
            # retrieve the compiler's version from it
            string(REGEX MATCH "(clang-[0-9.]+)" CLANG_APPLE_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_APPLE_VERSION})
        endif()
        if(${CLANG_VERSION} VERSION_LESS "${MIN_VERSION}")
            message(WARNING "\nThe compiler selected to build GNSS-SDR (${APPLE_STR}Clang version ${CLANG_VERSION} : ${CMAKE_CXX_COMPILER}) is older than that officially supported (${MIN_VERSION} minimum). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
        endif()
    else()
        message(WARNING "\nCannot determine the version of the compiler selected to build GNSS-SDR (${APPLE_STR}Clang : ${CMAKE_CXX_COMPILER}). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
    endif()
endif()



################################################################################
# Set minimal C and C++ standards
################################################################################
if(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_EXTENSIONS OFF)
else()
    add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,C>:-std=gnu11>")
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
            add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
        else()
            add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
        endif()
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
            if(CLANG_VERSION VERSION_LESS "600")
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
            else()
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
            endif()
        else()
            if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
            else()
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
            endif()
        endif()
    endif()
endif()

# Visibility
# See https://gcc.gnu.org/wiki/Visibility
if(POLICY CMP0063)
    cmake_policy(SET CMP0063 NEW)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_C_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
else()
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        add_definitions(-fvisibility=hidden)
    endif()
endif()



################################################################################
# Check if the compiler defines the architecture as ARM
################################################################################
if(NOT OS_IS_MACOSX)
    if(CMAKE_CROSSCOMPILING)
        set(IS_ARM TRUE)
        if(NOT CMAKE_NO_SYSTEM_FROM_IMPORTED)
            set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)
        endif()
    else()
        include(TestForARM)
    endif()
endif()



################################################################################
# Find the POSIX thread (pthread) libraries
################################################################################
if(CMAKE_VERSION VERSION_LESS 3.1)
    # Workaround for CMake < 3.1
    find_package(Threads REQUIRED)
    add_library(Threads::Threads SHARED IMPORTED)
    set_property(TARGET Threads::Threads PROPERTY INTERFACE_LINK_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")
    set_property(TARGET Threads::Threads PROPERTY IMPORTED_LINK_INTERFACE_LANGUAGES "CXX")
    include(GNUInstallDirs)
    # Fix bug in Debian 8.11
    if(${LINUX_DISTRIBUTION} MATCHES "Debian")
        if(${LINUX_VER} VERSION_LESS 8.12)
            if(ARCH_64BITS)
                set(FIX_PTHREADS_LOCATION "x86_64-linux-gnu/")
            endif()
        endif()
    endif()
    set_property(TARGET Threads::Threads PROPERTY IMPORTED_LOCATION /usr/${CMAKE_INSTALL_LIBDIR}/${FIX_PTHREADS_LOCATION}${CMAKE_FIND_LIBRARY_PREFIXES}pthread${CMAKE_SHARED_LIBRARY_SUFFIX})
else()
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    if(CMAKE_CROSSCOMPILING)
        set(THREADS_PREFER_PTHREAD_FLAG FALSE)
    else()
        set(THREADS_PREFER_PTHREAD_FLAG TRUE)
    endif()
    find_package(Threads REQUIRED)
endif()
set_package_properties(Threads PROPERTIES
    URL "https://computing.llnl.gov/tutorials/pthreads/"
    DESCRIPTION "Implements the POSIX Threads execution model"
    PURPOSE "Used to implement parallelism."
    TYPE REQUIRED
)



################################################################################
# Googletest - https://github.com/google/googletest
################################################################################
enable_testing()
if(ENABLE_UNIT_TESTING OR ENABLE_SYSTEM_TESTING)
    if(NOT GTEST_DIR)
        if(DEFINED ENV{GTEST_DIR})
            set(GTEST_DIR $ENV{GTEST_DIR})
            message(STATUS "Googletest root folder set at ${GTEST_DIR}")
        endif()
    endif()
endif()

find_package(GOOGLETEST)
set_package_properties(GOOGLETEST PROPERTIES
    PURPOSE "Used for Unit and System Tests."
    TYPE REQUIRED
)
if(NOT GOOGLETEST_FOUND)
    set_package_properties(GOOGLETEST PROPERTIES
        PURPOSE "Googletest v${GNSSSDR_GTEST_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# GNU Radio - https://gnuradio.org
################################################################################
set(GR_REQUIRED_COMPONENTS RUNTIME PMT BLOCKS FFT FILTER ANALOG)

find_package(UHD)
set_package_properties(UHD PROPERTIES
    PURPOSE "Used for communication with front-ends of the USRP family."
    TYPE OPTIONAL
)
if(ENABLE_UHD)
    if(NOT UHD_FOUND)
        set(ENABLE_UHD OFF)
    else()
        set(GR_REQUIRED_COMPONENTS ${GR_REQUIRED_COMPONENTS} UHD)
    endif()
endif()

find_package(GNURADIO)
set_package_properties(GNURADIO PROPERTIES
    PURPOSE "Implements flowgraph scheduler, provides some processing blocks and classes to create new ones."
    TYPE REQUIRED
)



################################################################################
# Log4cpp - http://log4cpp.sourceforge.net/
################################################################################
find_package(LOG4CPP)
set_package_properties(LOG4CPP PROPERTIES
    PURPOSE "Required by GNU Radio."
    TYPE REQUIRED
)
if(NOT LOG4CPP_FOUND)
    message(FATAL_ERROR "*** Log4cpp is required to build gnss-sdr")
endif()



################################################################################
# Detect availability of std::filesystem and set C++ standard accordingly
################################################################################
set(FILESYSTEM_FOUND FALSE)
if(NOT (GNURADIO_VERSION VERSION_LESS 3.8) AND LOG4CPP_READY_FOR_CXX17)
    # Check if we have std::filesystem
    if(NOT (CMAKE_VERSION VERSION_LESS 3.8))
        if((NOT ENABLE_UNIT_TESTING_EXTRA) AND (NOT ENABLE_SYSTEM_TESTING_EXTRA) AND (NOT ENABLE_FPGA)) # Workaround for GPSTk
            find_package(FILESYSTEM COMPONENTS Final Experimental)
            set_package_properties(FILESYSTEM PROPERTIES
                URL "https://en.cppreference.com/w/cpp/filesystem"
                DESCRIPTION "Provides facilities for performing operations on file systems and their components"
                PURPOSE "Work with paths, regular files, and directories."
                TYPE OPTIONAL
            )
            if(${FILESYSTEM_FOUND})
                if(CMAKE_VERSION VERSION_LESS 3.12)
                    set(CMAKE_CXX_STANDARD 17)
                else()
                    set(CMAKE_CXX_STANDARD 20)
                endif()
                set(CMAKE_CXX_STANDARD_REQUIRED ON)
            endif()
        endif()
    endif()
endif()



################################################################################
# Boost - https://www.boost.org
################################################################################
if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") # Fedora 64-bit fix
endif()
set(Boost_ADDITIONAL_VERSIONS
    "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71" "1.72.0" "1.72" "1.73.0" "1.73" "1.74.0" "1.74"
    "1.75.0" "1.75" "1.76.0" "1.76" "1.77.0" "1.77" "1.78.0" "1.78" "1.79.0" "1.79"
    "1.80.0" "1.80" "1.81.0" "1.81" "1.82.0" "1.82" "1.83.0" "1.83" "1.84.0" "1.84"
)
set(Boost_USE_MULTITHREAD ON)
set(Boost_USE_STATIC_LIBS OFF)
set(BOOST_COMPONENTS atomic chrono date_time serialization system thread)
if(NOT ${FILESYSTEM_FOUND})
    set(BOOST_COMPONENTS ${BOOST_COMPONENTS} filesystem)
endif()
find_package(Boost ${GNSSSDR_BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Fatal error: Boost (version >=${GNSSSDR_BOOST_MIN_VERSION}) required.")
endif()

set_package_properties(Boost PROPERTIES
    URL "https://www.boost.org"
    PURPOSE "Used widely across the source code."
    TYPE REQUIRED
)

if(CMAKE_VERSION VERSION_LESS 3.14)
    set(Boost_VERSION_STRING "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
endif()
if(POLICY CMP0093)
    cmake_policy(SET CMP0093 NEW)  # FindBoost reports Boost_VERSION in x.y.z format.
endif()
set_package_properties(Boost PROPERTIES
    DESCRIPTION "Portable C++ source libraries (found: v${Boost_VERSION_STRING})"
)

if(CMAKE_VERSION VERSION_LESS 3.5)
    if(NOT TARGET Boost::boost)
        add_library(Boost::boost SHARED IMPORTED) # Trick for CMake 2.8.12
        set_target_properties(Boost::boost PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            IMPORTED_LOCATION ${Boost_DATE_TIME_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::date_time)
        add_library(Boost::date_time SHARED IMPORTED)
        set_target_properties(Boost::date_time PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_DATE_TIME_LIBRARIES}
            IMPORTED_LOCATION ${Boost_DATE_TIME_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::system)
        add_library(Boost::system SHARED IMPORTED)
        set_target_properties(Boost::system PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_SYSTEM_LIBRARIES}
            IMPORTED_LOCATION ${Boost_SYSTEM_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::thread)
        add_library(Boost::thread SHARED IMPORTED)
        set_target_properties(Boost::thread PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_THREAD_LIBRARIES}
            IMPORTED_LOCATION ${Boost_THREAD_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::serialization)
        add_library(Boost::serialization SHARED IMPORTED)
        set_target_properties(Boost::serialization PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_SERIALIZATION_LIBRARIES}
            IMPORTED_LOCATION ${Boost_SERIALIZATION_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::chrono)
        add_library(Boost::chrono SHARED IMPORTED)
        set_target_properties(Boost::chrono PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_CHRONO_LIBRARIES}
            IMPORTED_LOCATION ${Boost_CHRONO_LIBRARIES}
        )
    endif()
    if(NOT TARGET Boost::atomic)
        add_library(Boost::atomic SHARED IMPORTED)
        set_target_properties(Boost::atomic PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            INTERFACE_LINK_LIBRARIES ${Boost_ATOMIC_LIBRARIES}
            IMPORTED_LOCATION ${Boost_ATOMIC_LIBRARIES}
        )
    endif()
    if(NOT ${FILESYSTEM_FOUND})
        if(NOT TARGET Boost::filesystem)
            add_library(Boost::filesystem SHARED IMPORTED)
            set_target_properties(Boost::filesystem PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
                INTERFACE_LINK_LIBRARIES ${Boost_FILESYSTEM_LIBRARIES}
                IMPORTED_LOCATION ${Boost_FILESYSTEM_LIBRARIES}
            )
         endif()
    endif()
endif()

# Fix for Boost Asio < 1.70 when using Clang in macOS
if(Boost_VERSION_STRING VERSION_LESS 1.70.0)
    # Check if we have std::string_view
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <string_view>
        int main()
        { std::string_view sv; }"
        has_string_view
    )
endif()



################################################################################
# VOLK - Vector-Optimized Library of Kernels
################################################################################
find_package(VOLK)
if(NOT VOLK_FOUND)
    message(FATAL_ERROR "*** VOLK is required to build gnss-sdr")
endif()
set_package_properties(VOLK PROPERTIES
    PURPOSE "Provides an abstraction of optimized math routines targeting several SIMD processors."
    TYPE REQUIRED
)



################################################################################
# volk_gnsssdr module - GNSS-SDR's own VOLK library
################################################################################
find_package(VOLKGNSSSDR)
set_package_properties(VOLKGNSSSDR PROPERTIES
    PURPOSE "Accelerates math routines targeting several SIMD processors."
    TYPE REQUIRED
)
if(NOT VOLKGNSSSDR_FOUND)
    message(STATUS " volk_gnsssdr will be built along with gnss-sdr when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    ###############################
    # Find Python required modules
    ###############################
    include(SetupPython) # sets PYTHON_EXECUTABLE and search for required modules

    if(NOT PYTHON_MIN_VER_FOUND)
        message(FATAL_ERROR "Python ${GNSSSDR_PYTHON_MIN_VERSION} or greater required to build VOLK_GNSSSDR")
    endif()

    if(${PYTHON3})
        set(PYTHON_NAME "python3")
    else()
        set(PYTHON_NAME "python")
    endif()

    #  Mako
    if(NOT MAKO_FOUND)
        message(STATUS "Mako template library not found. See http://www.makotemplates.org/")
        message(STATUS " You can try to install it by typing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo yum install ${PYTHON_NAME}-mako")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(STATUS " sudo zypper install ${PYTHON_NAME}-Mako")
        else()
            message(STATUS " sudo apt-get install ${PYTHON_NAME}-mako")
        endif()
        message(FATAL_ERROR "Mako templates required to build VOLK_GNSSSDR")
    endif()

    # Six
    if(NOT SIX_FOUND)
        message(STATUS "python-six not found. See https://pythonhosted.org/six/")
        message(STATUS " You can try to install it by typing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " sudo yum install ${PYTHON_NAME}-six")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(STATUS " sudo zypper install ${PYTHON_NAME}-six")
        else()
            message(STATUS " sudo apt-get install ${PYTHON_NAME}-six")
        endif()
        message(FATAL_ERROR "six - python 2 and 3 compatibility library required to build VOLK_GNSSSDR")
    endif()
    if(CMAKE_VERSION VERSION_GREATER 3.12)
        set_package_properties(Python3 PROPERTIES
            URL "https://www.python.org/"
            PURPOSE "Required to build volk_gnsssdr."
            TYPE REQUIRED
        )
        if(Python3_FOUND)
            set_package_properties(Python3 PROPERTIES
                DESCRIPTION "An interpreted, high-level, general-purpose programming language (found: v${Python3_VERSION})"
            )
        else()
            set_package_properties(Python3 PROPERTIES
                DESCRIPTION "An interpreted, high-level, general-purpose programming language"
                PURPOSE "Another Python version will be used."
            )
        endif()
        if(Python2_FOUND)
            set_package_properties(Python2 PROPERTIES
                URL "https://www.python.org/"
                DESCRIPTION "An interpreted, high-level, general-purpose programming language (found: v${Python2_VERSION})"
                PURPOSE "Required to build volk_gnsssdr."
                TYPE REQUIRED
            )
        endif()
    endif()

    if(PYTHONINTERP_FOUND)
        set_package_properties(PythonInterp PROPERTIES
            URL "https://www.python.org/"
            DESCRIPTION "An interpreted, high-level, general-purpose programming language (found: v${PYTHON_VERSION_STRING})"
            PURPOSE "Required to build volk_gnsssdr."
            TYPE REQUIRED
        )
    endif()

    if(ENABLE_PACKAGING)
        if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
            set(STRIP_VOLK_GNSSSDR_PROFILE "-DENABLE_STRIP=ON -DCMAKE_VERBOSE_MAKEFILE=ON")
        endif()
    endif()

    set(VOLK_GNSSSDR_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
    if(PYTHON_EXECUTABLE)
        set(USE_THIS_PYTHON "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}")
    endif()
    if(OS_IS_MACOSX)
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(VOLK_GNSSSDR_BUILD_COMMAND "xcodebuild" "-configuration" "Debug" "-target")
        endif()
    endif()

    if(CMAKE_CROSSCOMPILING)
        set(VOLK_GNSSSDR_COMPILER "")
    else()
        set(VOLK_GNSSSDR_COMPILER -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    endif()
    set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_COMPILER}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
        -DENABLE_STATIC_LIBS=ON
        -DENABLE_PROFILING=${ENABLE_PROFILING}
        -DCMAKE_INCLUDE_PATH=${Boost_INCLUDE_DIR}
        -DENABLE_ORC=OFF
        ${STRIP_VOLK_GNSSSDR_PROFILE}
        ${USE_THIS_PYTHON}
    )
    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
            -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake
            -DCROSSCOMPILE_MULTILIB=TRUE
        )
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(volk_gnsssdr_module
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build
            CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
            DOWNLOAD_COMMAND ""
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_COMMAND ${VOLK_GNSSSDR_BUILD_COMMAND} volk_gnsssdr_profile
            INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
        )
    else()
        ExternalProject_Add(volk_gnsssdr_module
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build
            CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
            DOWNLOAD_COMMAND ""
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_COMMAND ${VOLK_GNSSSDR_BUILD_COMMAND} volk_gnsssdr_profile
            BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/lib/${CMAKE_FIND_LIBRARY_PREFIXES}volk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}
                ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
            INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install
        )
    endif()
    find_package(ORC)
    set_package_properties(ORC PROPERTIES
        PURPOSE "Used by volk_gnsssdr."
        TYPE OPTIONAL
    )

    if(NOT ORC_FOUND)
        set(ORC_LIBRARIES "")
        set(ORC_INCLUDE_DIRS "")
    endif()

    add_library(volk_gnsssdr UNKNOWN IMPORTED)
    set_property(TARGET volk_gnsssdr PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/lib/libvolk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(VOLK_GNSSSDR_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build/include/;${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/include;${ORC_INCLUDE_DIRS}")
    set(VOLK_GNSSSDR_LIBRARIES volk_gnsssdr ${ORC_LIBRARIES})

    if(NOT TARGET Volkgnsssdr::volkgnsssdr)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/build/include)
        add_library(Volkgnsssdr::volkgnsssdr STATIC IMPORTED)
        add_dependencies(Volkgnsssdr::volkgnsssdr volk_gnsssdr_module)
        set_target_properties(Volkgnsssdr::volkgnsssdr PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/lib/libvolk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}"
            INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
            INTERFACE_INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${VOLK_GNSSSDR_LIBRARIES}"
        )
    endif()

    if(CMAKE_VERSION VERSION_LESS 3.2)
        add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile
        )
    else()
        add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile
            BYPRODUCTS ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr_profile
        )
    endif()

    add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr-config-info
            ${CMAKE_SOURCE_DIR}/install/volk_gnsssdr-config-info
    )

    set_package_properties(VOLKGNSSSDR PROPERTIES
        PURPOSE "volk_gnsssdr will be built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# gflags - https://github.com/gflags/gflags
################################################################################
set(LOCAL_GFLAGS false)
find_package(GFLAGS)
set_package_properties(GFLAGS PROPERTIES
    PURPOSE "Used for commandline flags management."
    TYPE REQUIRED
)
if(NOT GFLAGS_FOUND)
    message(STATUS " gflags library has not been found.")
    message(STATUS " gflags v${GNSSSDR_GFLAGS_LOCAL_VERSION} will be downloaded and built automatically")
    message(STATUS " when doing 'make'.")

    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
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
    else()
        ExternalProject_Add(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
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
    endif()

    set(GFlags_INCLUDE_DIRS
        ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include CACHE PATH "Local Gflags headers"
    )

    file(GLOB GFlags_SHARED_LIBS
        "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_SHARED_LIBRARY_SUFFIX}*"
    )

    if(NOT TARGET Gflags::gflags)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include)
        add_library(Gflags::gflags STATIC IMPORTED)
        add_dependencies(Gflags::gflags gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
        set_target_properties(Gflags::gflags PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}"
            INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include"
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include"
            INTERFACE_LINK_LIBRARIES "${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()

    set(LOCAL_GFLAGS true CACHE STRING "GFlags downloaded and built automatically" FORCE)
    set_package_properties(GFLAGS PROPERTIES
        PURPOSE "Gflags v${GNSSSDR_GFLAGS_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# glog - https://github.com/google/glog
################################################################################
if(NOT ${ENABLE_OWN_GLOG})
    find_package(GLOG)
    if(GLOG_INCLUDE_DIRS)
        set(GLOG_FOUND ON)
    endif()
endif()
set_package_properties(GLOG PROPERTIES
    PURPOSE "Used for runtime internal logging."
    TYPE REQUIRED
)
if(NOT GLOG_FOUND OR ${LOCAL_GFLAGS})
    message(STATUS " glog library has not been found")
    if(NOT GFLAGS_FOUND)
        message(STATUS " or it is likely not linked to gflags.")
    endif()
    message(STATUS " glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded and built automatically")
    message(STATUS " when doing 'make'.")
    if(NOT ${LOCAL_GFLAGS})
        add_library(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} UNKNOWN IMPORTED)
        set_property(TARGET gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} PROPERTY IMPORTED_LOCATION "${GFlags_LIBS}")
    endif()
    set(TARGET_GFLAGS gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
    if(${LOCAL_GFLAGS})
        set(GFLAGS_LIBRARIES_TO_LINK ${GFlags_SHARED_LIBS})
        set(GFLAGS_LIBRARY_DIR_TO_LINK ${CMAKE_CURRENT_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib)
    else()
        set(GFLAGS_LIBRARIES_TO_LINK ${GFlags_LIBS})
        set(GFLAGS_LIBRARY_DIR_TO_LINK ${GFlags_LIBRARY_DIRS})
    endif()
    if(OS_IS_MACOSX)
        set(GFLAGS_LIBRARIES_TO_LINK "${GFLAGS_LIBRARIES_TO_LINK} -lc++")
        set(GLOG_EXPORT_CXX_LIBRARIES "export CXXFLAGS=\"-stdlib=libc++\"")
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(GLOG_EXPORT_C_COMPILER "export CC=clang")
        set(GLOG_EXPORT_CXX_COMPILER "export CXX=clang++")
    endif()

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
"#!/bin/sh
export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
export LDFLAGS=-L${GFLAGS_LIBRARY_DIR_TO_LINK}
export LIBS=\"${GFLAGS_LIBRARIES_TO_LINK}\"
${GLOG_EXPORT_CXX_LIBRARIES}
${GLOG_EXPORT_C_COMPILER}
${GLOG_EXPORT_CXX_COMPILER}
cd ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/
aclocal
automake --add-missing
autoreconf -vfi
cd ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure"
    )

    file(COPY ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
        FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
            GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    )

    set(GLOG_CONFIGURE ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure_with_gflags)

    # Ensure that aclocal and libtool are present
    if(OS_IS_LINUX)
        if(EXISTS "/usr/bin/libtoolize")
            if(EXISTS "/usr/bin/aclocal" OR EXISTS "/usr/bin/aclocal-1.16" OR EXISTS "/usr/bin/aclocal-1.15" OR EXISTS "/usr/bin/aclocal-1.14" OR EXISTS "/usr/bin/aclocal-1.13" OR EXISTS "/usr/bin/aclocal-1.11"  OR EXISTS "/usr/bin/aclocal-1.10")
                # Everything ok, we can move on
            else()
                message(" aclocal has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install automake")
                else()
                    message(" sudo apt-get install automake")
                endif()
                message(FATAL_ERROR "aclocal is required to build glog from source")
            endif()
        else()
            message(" libtool has not been found.")
            message(" You can try to install it by typing:")
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(" sudo yum groupinstall 'Development Tools'")
            elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                message(" sudo zypper install libtoool")
            else()
                message(" sudo apt-get install libtool")
            endif()
            message(FATAL_ERROR "libtool is required to build glog from source")
        endif()
    endif()

    set(GLOG_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
    if(GLOG_MAKE_PROGRAM MATCHES "ninja")
        find_program(GLOG_MAKE_EXECUTABLE make
            HINTS
                /usr/bin/
                /usr/local/bin/
        )
        if(NOT GLOG_MAKE_EXECUTABLE)
            message(FATAL_ERROR "make is required to build Glog from source.")
        endif()
        set(GLOG_MAKE_PROGRAM ${GLOG_MAKE_EXECUTABLE})
    endif()

    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            DEPENDS ${TARGET_GFLAGS}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/google/glog/
            GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
            BUILD_COMMAND "${GLOG_MAKE_PROGRAM}"
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
    else()
        ExternalProject_Add(glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            DEPENDS ${TARGET_GFLAGS}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/google/glog/
            GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
            CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
            BUILD_COMMAND "${GLOG_MAKE_PROGRAM}"
            BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
    endif()
    add_dependencies(glog-${GNSSSDR_GLOG_LOCAL_VERSION} Gflags::gflags)

    # Set up variables
    set(GLOG_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src/
        ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
    )
    set(GLOG_LIBRARIES
        ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
    )

    # Create Glog::glog target
    if(NOT TARGET Glog::glog)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src)
        add_library(Glog::glog STATIC IMPORTED)
        add_dependencies(Glog::glog glog-${GNSSSDR_GLOG_LOCAL_VERSION})
        set_target_properties(Glog::glog PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}"
            INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${CMAKE_CURRENT_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()

    set(LOCAL_GLOG true CACHE STRING "Glog downloaded and built automatically" FORCE)

    set_package_properties(GLOG PROPERTIES
        PURPOSE "Glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded and built when doing 'make'."
    )
endif()

if(NOT ENABLE_LOG)
    message(STATUS "Logging is not enabled")
endif()



################################################################################
# Check that BLAS (Basic Linear Algebra Subprograms) is found in the system
# See http://www.netlib.org/blas/
################################################################################
if(OS_IS_MACOSX)
    # Avoid using the implementation that comes with the Accelerate framework
    include(AvoidAccelerate)
else()
    if(NOT BLA_VENDOR)
        set(BLA_VENDOR "Generic")
    endif()
    find_package(BLAS)
    set_package_properties(BLAS PROPERTIES
        URL "http://www.netlib.org/blas/"
        DESCRIPTION "Basic Linear Algebra Subprograms"
        PURPOSE "Used for matrix algebra computations."
        TYPE REQUIRED
    )
endif()
if(NOT BLAS_FOUND)
    message(" The BLAS library has not been found.")
    message(" You can try to install it by typing:")
    if(OS_IS_MACOSX)
        message(" 'sudo port install lapack' if you are using Macports, or")
        message(" 'brew install lapack' if you are using Homebrew.")
    else()
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo yum install blas-devel")
        else()
            message(" sudo apt-get install libblas-dev")
        endif()
    endif()
    message(FATAL_ERROR "BLAS is required to build gnss-sdr")
endif()



################################################################################
# Check that LAPACK (Linear Algebra PACKage) is found in the system
# See http://www.netlib.org/lapack/
################################################################################
if(NOT OS_IS_MACOSX)
    find_package(LAPACK)
    set_package_properties(LAPACK PROPERTIES
        URL "http://www.netlib.org/lapack/"
        DESCRIPTION "Linear Algebra PACKage"
        PURPOSE "Used for matrix algebra computations."
        TYPE REQUIRED
    )
endif()
if(NOT LAPACK_FOUND)
    message(" The LAPACK library has not been found.")
    message(" You can try to install it by typing:")
    if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(" sudo yum install lapack-devel")
    elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
        message(" sudo zypper install lapack-devel")
    else()
        message(" sudo apt-get install liblapack-dev")
    endif()
    message(FATAL_ERROR "LAPACK is required to build gnss-sdr")
endif()



################################################################################
# Armadillo - http://arma.sourceforge.net/
################################################################################
find_package(Armadillo)
set_package_properties(Armadillo PROPERTIES
    URL "http://arma.sourceforge.net/"
    PURPOSE "Used for matrix computations."
    TYPE REQUIRED
)
if(ARMADILLO_FOUND)
    set_package_properties(Armadillo PROPERTIES
        DESCRIPTION "C++ library for linear algebra and scientific computing (found: v${ARMADILLO_VERSION_STRING})"
    )
    if(${ARMADILLO_VERSION_STRING} VERSION_LESS ${GNSSSDR_ARMADILLO_MIN_VERSION})
        set(ARMADILLO_FOUND false)
        set(ENABLE_OWN_ARMADILLO true)
    else()
        add_library(Armadillo::armadillo SHARED IMPORTED)
        set_target_properties(Armadillo::armadillo PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${ARMADILLO_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${ARMADILLO_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${ARMADILLO_LIBRARIES}"
        )
    endif()
endif()

if(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)
    message(STATUS " Armadillo has not been found.")
    message(STATUS " Armadillo ${GNSSSDR_ARMADILLO_LOCAL_VERSION} will be downloaded and built automatically")
    message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set(armadillo_BRANCH ${GNSSSDR_ARMADILLO_LOCAL_VERSION})
    set(armadillo_RELEASE ${armadillo_BRANCH})

    #############################################
    # Check if GFORTRAN is found in the system
    #############################################
    if(NOT OS_IS_MACOSX)
        find_package(GFORTRAN)
        set_package_properties(GFORTRAN PROPERTIES
            PURPOSE "Required by Armadillo."
            TYPE REQUIRED
        )
        if(NOT GFORTRAN)
            message(STATUS "The gfortran library has not been found.")
            message(STATUS " You can try to install it by typing:")
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(STATUS " sudo yum install gcc-fortran")
            elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                message(STATUS " sudo zypper install gcc-fortran")
            else()
                message(STATUS " sudo apt-get install gfortran")
            endif()
            message(FATAL_ERROR "gfortran is required to build gnss-sdr")
        endif()
    endif()

    #############################################
    # Download and build Armadillo
    #############################################
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(armadillo-${armadillo_RELEASE}
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
    else()
        ExternalProject_Add(armadillo-${armadillo_RELEASE}
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
    endif()

    # Create imported target
    ExternalProject_Get_Property(armadillo-${armadillo_RELEASE} binary_dir)
    if(NOT GFORTRAN)
        set(GFORTRAN "")
    endif()
    set(ARMADILLO_LIBRARIES ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${GFORTRAN} ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(LOCAL_ARMADILLO true CACHE STRING "Armadillo downloaded and built automatically" FORCE)
    set(ARMADILLO_VERSION_STRING ${armadillo_RELEASE})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}/include)
    add_library(Armadillo::armadillo STATIC IMPORTED)
    add_dependencies(Armadillo::armadillo armadillo-${armadillo_RELEASE})
    set_target_properties(Armadillo::armadillo PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}/include"
        INTERFACE_LINK_LIBRARIES "${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )
    set_package_properties(Armadillo PROPERTIES
        DESCRIPTION "C++ library for linear algebra and scientific computing"
        PURPOSE "Armadillo ${GNSSSDR_ARMADILLO_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# GnuTLS - https://www.gnutls.org/
################################################################################
find_package(GnuTLS)
set_package_properties(GnuTLS PROPERTIES
    URL "https://www.gnutls.org/"
    PURPOSE "Used for the SUPL protocol implementation."
    TYPE REQUIRED
)
if(GnuTLS_FOUND AND GNUTLS_VERSION_STRING)
    set_package_properties(GnuTLS PROPERTIES
        DESCRIPTION "Transport Layer Security Library (found: v${GNUTLS_VERSION_STRING})"
    )
else()
    set_package_properties(GnuTLS PROPERTIES
        DESCRIPTION "Transport Layer Security Library"
    )
endif()
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
    if(GnuTLS_FOUND)
        message(STATUS " But it was not built with openssl compatibility.")
    endif()
    message(STATUS " Looking for OpenSSL instead...")
    if(OS_IS_MACOSX)
        set(OPENSSL_ROOT_DIR /usr/local/opt/openssl) # Trick for Homebrew
    endif()
    find_package(OpenSSL)
    set_package_properties(OpenSSL PROPERTIES
        URL "https://www.openssl.org"
        DESCRIPTION "Cryptography and SSL/TLS Toolkit (found: v${OPENSSL_VERSION})"
        PURPOSE "Used for the SUPL protocol implementation."
        TYPE REQUIRED
    )
    if(OPENSSL_FOUND)
        set_package_properties(GnuTLS PROPERTIES
            PURPOSE "Not found, but OpenSSL can replace it."
        )
        set(GNUTLS_INCLUDE_DIR ${OPENSSL_INCLUDE_DIR})
        set(GNUTLS_LIBRARIES "")
        set(GNUTLS_OPENSSL_LIBRARY ${OPENSSL_SSL_LIBRARY})
    else()
        message(" The GnuTLS library with openssl compatibility enabled has not been found.")
        message(" You can try to install the required libraries by typing:")
        if(OS_IS_LINUX)
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(" sudo yum install openssl-devel")
            else()
                message(" sudo apt-get install libgnutls28-dev")
            endif()
        endif()
        if(OS_IS_MACOSX)
            message(" 'sudo port install gnutls', if you are using Macports, or")
            message(" 'brew install openssl', if you are using Homebrew.")
        endif()
        message(FATAL_ERROR "GnuTLS libraries with openssl compatibility are required to build gnss-sdr")
    endif()
endif()



################################################################################
# Matio - https://github.com/tbeu/matio
################################################################################
find_package(MATIO)
set_package_properties(MATIO PROPERTIES
    PURPOSE "Used to store processing block's results in MAT files readable from MATLAB/Octave."
    TYPE REQUIRED
)
if(NOT MATIO_FOUND OR MATIO_VERSION_STRING VERSION_LESS ${GNSSSDR_MATIO_MIN_VERSION})
    if(MATIO_FOUND)
        message(STATUS " Matio installed version (${MATIO_VERSION_STRING}) is too old (>= ${GNSSSDR_MATIO_MIN_VERSION} is required).")
    endif()
    message(STATUS " Matio v${GNSSSDR_MATIO_LOCAL_VERSION} will be downloaded and built automatically")
    message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    find_package(ZLIB)
    set_package_properties(ZLIB PROPERTIES
        URL "https://www.zlib.net/"
        PURPOSE "Required to build Matio."
        TYPE REQUIRED
    )
    if(ZLIB_FOUND AND ZLIB_VERSION_STRING)
        set_package_properties(ZLIB PROPERTIES
            DESCRIPTION "A Massively Spiffy Yet Delicately Unobtrusive Compression Library (found: v${ZLIB_VERSION_STRING})"
        )
    else()
        set_package_properties(ZLIB PROPERTIES
            DESCRIPTION "A Massively Spiffy Yet Delicately Unobtrusive Compression Library"
        )
    endif()
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
                else()
                    message(" sudo apt-get install libtool")
                endif()
                message(FATAL_ERROR "libtool is required to build matio from source")
            endif()
            if(EXISTS "/usr/bin/aclocal" OR
                EXISTS "/usr/bin/aclocal-1.16" OR
                EXISTS "/usr/bin/aclocal-1.15" OR
                EXISTS "/usr/bin/aclocal-1.14" OR
                EXISTS "/usr/bin/aclocal-1.13" OR
                EXISTS "/usr/bin/aclocal-1.11" OR
                EXISTS "/usr/bin/aclocal-1.10")
                message(STATUS "Automake found.")
            else()
                message(" aclocal has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install automake")
                else()
                    message(" sudo apt-get install automake")
                endif()
                message(FATAL_ERROR "aclocal is required to build matio from source")
            endif()
        endif()
        find_package(HDF5)
        set_package_properties(HDF5 PROPERTIES
            URL "https://support.hdfgroup.org/HDF5/"
            PURPOSE "Required to build Matio."
            TYPE REQUIRED
        )
        if(HDF5_FOUND AND HDF5_VERSION)
            set_package_properties(HDF5 PROPERTIES
                DESCRIPTION "A versatile data model, a portable file format and a software library (found: v${HDF5_VERSION})"
            )
        else()
            set_package_properties(HDF5 PROPERTIES
                DESCRIPTION "A versatile data model, a portable file format and a software library"
            )
        endif()
        if(HDF5_FOUND)
            list(GET HDF5_LIBRARIES 0 HDF5_FIRST_DIR)
            get_filename_component(HDF5_BASE_DIR2 ${HDF5_FIRST_DIR} DIRECTORY)
            get_filename_component(HDF5_BASE_DIR ${HDF5_BASE_DIR2} DIRECTORY)
            if(OS_IS_MACOSX)
                if(EXISTS /opt/local/include/hdf5.h)
                    set(HDF5_BASE_DIR /opt/local)
                endif()
                if(EXISTS /usr/local/include/hdf5.h)
                    set(HDF5_BASE_DIR /usr/local)
                endif()
            endif()

            set(MATIO_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
            if(MATIO_MAKE_PROGRAM MATCHES "ninja")
                find_program(MATIO_MAKE_EXECUTABLE make
                    HINTS
                        /usr/bin/
                        /usr/local/bin/
                )
                if(NOT MATIO_MAKE_EXECUTABLE)
                    message(FATAL_ERROR "make is required to build Matio from source.")
                endif()
                set(MATIO_MAKE_PROGRAM ${MATIO_MAKE_EXECUTABLE})
            endif()

            if(CMAKE_VERSION VERSION_LESS 3.2)
                ExternalProject_Add(matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/matio
                    GIT_REPOSITORY https://github.com/tbeu/matio
                    GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                    UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/autogen.sh
                    CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/configure --with-hdf5=${HDF5_BASE_DIR} --with-zlib=${ZLIB_BASE_DIR} --with-default-file-ver=7.3 --enable-mat73=yes --prefix=<INSTALL_DIR>
                    BUILD_COMMAND ${MATIO_MAKE_PROGRAM}
                )
            else()
                ExternalProject_Add(matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/matio
                    GIT_REPOSITORY https://github.com/tbeu/matio
                    GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
                    UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/autogen.sh
                    CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/configure --with-hdf5=${HDF5_BASE_DIR} --with-zlib=${ZLIB_BASE_DIR} --with-default-file-ver=7.3 --enable-mat73=yes --prefix=<INSTALL_DIR>
                    BUILD_COMMAND ${MATIO_MAKE_PROGRAM}
                    BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_SHARED_LIBRARY_SUFFIX}
                )
            endif()
            set(MATIO_LOCAL true)
            if(NOT TARGET Matio::matio)
                file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/matio/include)
                file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/matio/lib)
                add_library(Matio::matio SHARED IMPORTED)
                add_dependencies(Matio::matio matio-${GNSSSDR_MATIO_LOCAL_VERSION})
                set_target_properties(Matio::matio PROPERTIES
                    IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_SHARED_LIBRARY_SUFFIX}"
                    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR}/matio/include
                    INTERFACE_LINK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_SHARED_LIBRARY_SUFFIX}
                )
            endif()
        else()
            message(STATUS " The hdf5 library has not been found in your system.")
            message(STATUS " Please try to install it by doing:")
            if(OS_IS_MACOSX)
                message(STATUS "  $ sudo port install hdf5")
                message(STATUS " or")
                message(STATUS "  $ brew install hdf5")
            endif()
            if(OS_IS_LINUX)
                message(STATUS "  $ sudo apt-get install libhdf5-dev")
            endif()
            message(FATAL_ERROR "*** The hdf5 library is required to build gnss-sdr")
        endif()
    else()
        message(FATAL_ERROR "*** The zlib library is required to build gnss-sdr")
    endif()
    set_package_properties(MATIO PROPERTIES
        PURPOSE "Matio v${GNSSSDR_MATIO_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# PugiXML - https://pugixml.org/
################################################################################
find_package(PUGIXML)
set_package_properties(PUGIXML PROPERTIES
    PURPOSE "Used to handle Galileo almanac XML files published by the European GNSS Service Centre."
    TYPE REQUIRED
)
if(NOT PUGIXML_FOUND)
    message(STATUS " PugiXML v${GNSSSDR_PUGIXML_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set(PUGIXML_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})
    set(TOOLCHAIN_ARG "")
    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(PUGIXML_COMPILER "")
        set(TOOLCHAIN_ARG "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake")
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/zeux/pugixml
            GIT_TAG v${GNSSSDR_PUGIXML_LOCAL_VERSION}
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            CMAKE_ARGS ${PUGIXML_COMPILER} ${TOOLCHAIN_ARG}
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            INSTALL_COMMAND ""
        )
    else()
        ExternalProject_Add(pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
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
    endif()

    if(NOT TARGET Pugixml::pugixml)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/src)
        add_library(Pugixml::pugixml STATIC IMPORTED)
        add_dependencies(Pugixml::pugixml pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION})
        set_target_properties(Pugixml::pugixml PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}"
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/src"
            INTERFACE_LINK_LIBRARIES "${CMAKE_CURRENT_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()

    set_package_properties(PUGIXML PROPERTIES
        PURPOSE "PugiXML v${GNSSSDR_PUGIXML_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# Protocol Buffers https://github.com/protocolbuffers/protobuf
################################################################################
find_package(Protobuf)
set_package_properties(Protobuf PROPERTIES
    URL "https://developers.google.com/protocol-buffers/"
    PURPOSE "Used to serialize output data in a way that can be read by other applications."
    TYPE REQUIRED
)
if(NOT Protobuf_VERSION)
    set(Protobuf_VERSION "0.0.0")
endif()
if(CMAKE_VERSION VERSION_LESS 3.6)
    if(PROTOBUF_FOUND)
        set(Protobuf_FOUND ${PROTOBUF_FOUND})
    endif()
endif()
if(Protobuf_FOUND AND CMAKE_VERSION VERSION_LESS 3.9)
    if(PROTOBUF_INCLUDE_DIR)
        set(Protobuf_INCLUDE_DIR ${PROTOBUF_INCLUDE_DIR})
    endif()
    if(PROTOBUF_LIBRARY)
        set(Protobuf_LIBRARY ${PROTOBUF_LIBRARY})
    endif()
    if(PROTOBUF_PROTOC_EXECUTABLE)
        set(Protobuf_PROTOC_EXECUTABLE ${PROTOBUF_PROTOC_EXECUTABLE})
    endif()
    if(PROTOBUF_PROTOC_LIBRARY)
        set(Protobuf_PROTOC_LIBRARY ${PROTOBUF_PROTOC_EXECUTABLE})
    endif()
    add_library(protobuf::libprotobuf SHARED IMPORTED)
    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${Protobuf_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${Protobuf_LIBRARY}"
    )
    add_executable(protobuf::protoc IMPORTED)
    set_target_properties(protobuf::protoc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${Protobuf_PROTOC_EXECUTABLE}"
        INTERFACE_LINK_LIBRARIES "${Protobuf_PROTOC_LIBRARY}"
    )
    if(${Protobuf_VERSION} VERSION_EQUAL "0.0.0")
        set(_PROTOBUF_COMMON_HEADER ${Protobuf_INCLUDE_DIR}/google/protobuf/stubs/common.h)
        set(Protobuf_VERSION "")
        set(Protobuf_LIB_VERSION "")
        file(STRINGS ${_PROTOBUF_COMMON_HEADER} _PROTOBUF_COMMON_H_CONTENTS REGEX "#define[ \t]+GOOGLE_PROTOBUF_VERSION[ \t]+")
        if(_PROTOBUF_COMMON_H_CONTENTS MATCHES "#define[ \t]+GOOGLE_PROTOBUF_VERSION[ \t]+([0-9]+)")
            set(Protobuf_LIB_VERSION "${CMAKE_MATCH_1}")
        endif()
        unset(_PROTOBUF_COMMON_H_CONTENTS)
        math(EXPR _PROTOBUF_MAJOR_VERSION "${Protobuf_LIB_VERSION} / 1000000")
        math(EXPR _PROTOBUF_MINOR_VERSION "${Protobuf_LIB_VERSION} / 1000 % 1000")
        math(EXPR _PROTOBUF_SUBMINOR_VERSION "${Protobuf_LIB_VERSION} % 1000")
        set(Protobuf_VERSION "${_PROTOBUF_MAJOR_VERSION}.${_PROTOBUF_MINOR_VERSION}.${_PROTOBUF_SUBMINOR_VERSION}")
    endif()
endif()

if(Protobuf_FOUND)
    set_package_properties(Protobuf PROPERTIES
        DESCRIPTION "Structured data serialization mechanism (found: v${Protobuf_VERSION})"
    )
else()
    set_package_properties(Protobuf PROPERTIES
        DESCRIPTION "Structured data serialization mechanism"
    )
endif()

if(Protobuf_FOUND AND CMAKE_CROSSCOMPILING)
    find_program(PROTOC_EXECUTABLE protoc)
    if(NOT PROTOC_EXECUTABLE)
        find_program(PROTOC_EXECUTABLE protoc
            HINTS
                /usr/bin/
                /usr/local/bin/
        )
    endif()
    if(PROTOC_EXECUTABLE)
        set_target_properties(protobuf::protoc PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION ${PROTOC_EXECUTABLE}
        )
        set(Protobuf_PROTOC_EXECUTABLE ${PROTOC_EXECUTABLE})
    else()
        message(FATAL_ERROR "Please install the Protocol Buffers compiler v${Protobuf_VERSION} in the host machine")
    endif()
endif()

if((NOT Protobuf_FOUND) OR (NOT Protobuf_PROTOC_EXECUTABLE) OR (${Protobuf_VERSION} VERSION_LESS ${GNSSSDR_PROTOBUF_MIN_VERSION}))
    unset(Protobuf_PROTOC_EXECUTABLE)
    if(CMAKE_CROSSCOMPILING)
        if(NOT Protobuf_FOUND)
            ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                PREFIX ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                CONFIGURE_COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} --host=$ENV{OECORE_TARGET_ARCH} --with-protoc=${PROTOC_EXECUTABLE}"
                BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
                INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} install
                BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
                    ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
            )
        endif()
    else()
        if(OS_IS_LINUX)
            if(NOT EXISTS "/usr/bin/libtoolize")
                message(" libtool has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install libtoool")
                else()
                    message(" sudo apt-get install libtool")
                endif()
                message(FATAL_ERROR "libtool is required to build Protocol Buffers from source")
            endif()
            if(EXISTS "/usr/bin/aclocal" OR
                EXISTS "/usr/bin/aclocal-1.16" OR
                EXISTS "/usr/bin/aclocal-1.15" OR
                EXISTS "/usr/bin/aclocal-1.14" OR
                EXISTS "/usr/bin/aclocal-1.13" OR
                EXISTS "/usr/bin/aclocal-1.11" OR
                EXISTS "/usr/bin/aclocal-1.10")
                message(STATUS "Automake found.")
            else()
                message(" aclocal has not been found.")
                message(" You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(" sudo yum groupinstall 'Development Tools'")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(" sudo zypper install automake")
                else()
                    message(" sudo apt-get install automake")
                endif()
                message(FATAL_ERROR "aclocal is required to build Protocol Buffers from source")
            endif()
        endif()

        set(PROTOBUF_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
        if(PROTOBUF_MAKE_PROGRAM MATCHES "ninja")
            find_program(MAKE_EXECUTABLE make
                HINTS
                    /usr/bin/
                    /usr/local/bin/
            )
            if(NOT MAKE_EXECUTABLE)
                message(FATAL_ERROR "make is required to build Protocol Buffers from source.")
            endif()
            set(PROTOBUF_MAKE_PROGRAM ${MAKE_EXECUTABLE})
        endif()
        if(CMAKE_VERSION VERSION_LESS 3.2)
            ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                PREFIX ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BUILD_COMMAND ${PROTOBUF_MAKE_PROGRAM}
                INSTALL_COMMAND ${PROTOBUF_MAKE_PROGRAM} install
            )
        else()
            ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                PREFIX ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                UPDATE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BUILD_COMMAND ${PROTOBUF_MAKE_PROGRAM}
                INSTALL_COMMAND ${PROTOBUF_MAKE_PROGRAM} install
                BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
                    ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
            )
        endif()

        if(NOT TARGET protobuf::protoc)
            add_executable(protobuf::protoc IMPORTED)
            add_dependencies(protobuf::protoc protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
        endif()
        unset(Protobuf_PROTOC_EXECUTABLE)
        set(PROTOBUF_PROTOC_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc")
        set_target_properties(protobuf::protoc PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc"
            INTERFACE_LINK_LIBRARIES "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protoc${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include)
    if(NOT TARGET protobuf::libprotobuf)
        add_library(protobuf::libprotobuf STATIC IMPORTED)
        add_dependencies(protobuf::libprotobuf protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
    endif()
    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include"
        INTERFACE_LINK_LIBRARIES "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )

    if(${Protobuf_VERSION} VERSION_LESS ${GNSSSDR_PROTOBUF_MIN_VERSION})
        if(NOT (${Protobuf_VERSION} EQUAL "0.0.0"))
            set_package_properties(Protobuf PROPERTIES
                PURPOSE "Protocol Buffers found (v${Protobuf_VERSION}) is too old (> v${GNSSSDR_PROTOBUF_MIN_VERSION} needed)."
            )
        endif()
    endif()
    set_package_properties(Protobuf PROPERTIES
        PURPOSE "Protocol Buffers v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} will be downloaded and built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()
if(${Protobuf_VERSION} VERSION_EQUAL "0.0.0")
    unset(Protobuf_VERSION)
endif()



################################################################################
# Doxygen - http://www.doxygen.nl (OPTIONAL, used if found)
################################################################################
find_package(Doxygen)
set_package_properties(Doxygen PROPERTIES
    URL "http://www.doxygen.nl"
    PURPOSE "Used to generate code documentation by doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'"
    TYPE OPTIONAL
)
if(DOXYGEN_FOUND AND DOXYGEN_VERSION)
    set_package_properties(Doxygen PROPERTIES
        DESCRIPTION "Generates documentation from annotated C++ sources (found: v${DOXYGEN_VERSION})"
    )
else()
    set_package_properties(Doxygen PROPERTIES
        DESCRIPTION "Generates documentation from annotated C++ sources"
    )
endif()
find_package(LATEX)
set_package_properties(LATEX PROPERTIES
    URL "https://www.latex-project.org"
    DESCRIPTION "High-quality typesetting system"
    PURPOSE "Used to generate a PDF manual by doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} pdfmanual'"
    TYPE OPTIONAL
)
if(DOXYGEN_FOUND)
    message(STATUS "Doxygen found.")
    message(STATUS " You can build the documentation with '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'.")
    message(STATUS " When done, point your browser to ${CMAKE_BINARY_DIR}/html/index.html")
    set(HAVE_DOT ${DOXYGEN_DOT_FOUND})
    file(TO_NATIVE_PATH ${CMAKE_SOURCE_DIR} top_srcdir)
    file(TO_NATIVE_PATH ${CMAKE_BINARY_DIR} top_builddir)
    if(PDFLATEX_COMPILER)
        set(GENERATE_PDF_DOCUMENTATION "YES")
        set(GNSSSDR_USE_MATHJAX "NO")
    else()
        set(GENERATE_PDF_DOCUMENTATION "NO")
        set(GNSSSDR_USE_MATHJAX "YES")
    endif()
    configure_file(${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile.in
        ${CMAKE_BINARY_DIR}/docs/doxygen/Doxyfile
        @ONLY
    )
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/docs/doxygen/Doxyfile
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating source code documentation with Doxygen." VERBATIM
    )
    if(PDFLATEX_COMPILER)
        message(STATUS " '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} pdfmanual' will generate a manual at ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf")
        add_custom_target(pdfmanual
            COMMAND ${CMAKE_MAKE_PROGRAM}
            COMMAND ${CMAKE_COMMAND} -E copy refman.pdf ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
            COMMAND ${CMAKE_MAKE_PROGRAM} clean
            DEPENDS doc
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/docs/latex
            COMMENT "Generating PDF manual with Doxygen." VERBATIM
        )
    endif()
    message(STATUS " '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc-clean' will clean the documentation.")
    add_custom_target(doc-clean
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/docs/html
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/docs/latex
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
        COMMENT "Cleaning documentation." VERBATIM
    )
else()
    message(STATUS " Doxygen has not been found in your system.")
    message(STATUS " You can get nice code documentation by using it!")
    message(STATUS " Get it from http://www.doxygen.nl/download.html")
    if(OS_IS_LINUX)
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " or simply by doing 'sudo yum install doxygen-latex'.")
        else()
            message(STATUS " or simply by doing 'sudo apt-get install doxygen-latex'.")
        endif()
    endif()
    if(OS_IS_MACOSX)
        message(STATUS " or simply by doing 'sudo port install doxygen +latex'.")
    endif()
endif()



################################################################################
# OpenCL (OPTIONAL)
################################################################################
find_package(OPENCL QUIET)
set_package_properties(OPENCL PROPERTIES
    PURPOSE "Used in some processing block implementations."
    TYPE OPTIONAL
)
if(NOT OPENCL_FOUND)
    set(ENABLE_OPENCL OFF)
endif()
if(ENABLE_OPENCL)
    if(DEFINED ENV{DISABLE_OPENCL})
        set(DISABLE_OPENCL TRUE)
    endif()
    if(DISABLE_OPENCL)
        set(ENABLE_OPENCL OFF)
    else()
        if(OPENCL_FOUND)
            message(STATUS "OpenCL has been found and will be used by some processing blocks")
            message(STATUS " You can disable OpenCL use by doing 'cmake -DENABLE_OPENCL=OFF ..'")
        endif()
    endif()
    if(ENABLE_GENERIC_ARCH)
        set(ENABLE_OPENCL OFF)
        message(STATUS "ENABLE_GENERIC_ARCH is set to ON so the use of OpenCL has been disabled.")
    endif()
    if(NOT OPENCL_FOUND)
        message(STATUS "Processing blocks using OpenCL will not be built.")
    endif()
endif()



################################################################################
# CUDA (OPTIONAL)
################################################################################
if(DEFINED ENV{CUDA_GPU_ACCEL})
    message(STATUS "CUDA_GPU_ACCEL environment variable found.")
    set(ENABLE_CUDA ON)
endif()

if(ENABLE_CUDA)
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        include(CheckLanguage)
        check_language(CUDA)
        if(CMAKE_CUDA_COMPILER)
            enable_language(CUDA)
            set(CUDA_FOUND TRUE)
            if(NOT DEFINED CMAKE_CUDA_STANDARD)
                set(CMAKE_CUDA_STANDARD 11)
                set(CMAKE_CUDA_STANDARD_REQUIRED ON)
            endif()
        else()
            set(ENABLE_CUDA OFF)
        endif()
    else()
        find_package(CUDA REQUIRED)
        set_package_properties(CUDA PROPERTIES
            URL "https://developer.nvidia.com/cuda-downloads"
            DESCRIPTION "Library for parallel programming in Nvidia GPUs"
            PURPOSE "Used in some processing block implementations."
            TYPE REQUIRED
        )
        if(NOT CUDA_FOUND)
            set(ENABLE_CUDA OFF)
        endif()
    endif()
endif()
if(ENABLE_CUDA)
    message(STATUS "NVIDIA CUDA GPU Acceleration will be enabled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_CUDA=OFF ..'")
else()
    message(STATUS "NVIDIA CUDA GPU Acceleration will be not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_CUDA=ON ..' to add support for GPU-based acceleration using CUDA.")
endif()



################################################################################
# CUSTOM UDP PACKET SOURCE (OPTIONAL)
################################################################################
find_package(PCAP)
set_package_properties(PCAP PROPERTIES
    PURPOSE "Used for the custom UDP IP packet source."
    TYPE OPTIONAL
)
if(PCAP_FOUND)
    set(ENABLE_RAW_UDP ON)
endif()
if(ENABLE_RAW_UDP)
    message(STATUS "Highly-optimized custom UDP IP packet source is enabled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_RAW_UDP=OFF ..'")
    if(NOT PCAP_FOUND)
        message(FATAL_ERROR "PCAP required to compile custom UDP packet sample source (with ENABLE_RAW_UDP=ON)")
    endif()
endif()



################################################################################
# FPGA (OPTIONAL)
################################################################################
if(ENABLE_FPGA)
    message(STATUS "FPGA Acceleration will be enabled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_FPGA=OFF ..'")
else()
    message(STATUS "Fpga Acceleration will be not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_FPGA=ON ..' to add support for GPU-based acceleration using the FPGA.")
endif()



################################################################################
# Setup of optional drivers
################################################################################

##########################################
# gr-osmosdr - OPTIONAL
# https://github.com/osmocom/gr-osmosdr
##########################################
if(DEFINED ENV{RTLSDR_DRIVER})
    message(STATUS "RTLSDR_DRIVER environment variable found.")
    set(ENABLE_OSMOSDR ON)
endif()

find_package(GROSMOSDR)
set_package_properties(GROSMOSDR PROPERTIES
    PURPOSE "Used for communication with OsmoSDR and other front-ends (HackRF, bladeRF, Realtek's RTL2832U-based dongles, etc.)."
    TYPE OPTIONAL
)
if(ENABLE_OSMOSDR)
    if(GROSMOSDR_FOUND)
        message(STATUS "The driver for OsmoSDR and other front-ends (HackRF, bladeRF, Realtek's RTL2832U-based dongles, etc.) will be compiled.")
        message(STATUS " You can disable it with 'cmake -DENABLE_OSMOSDR=OFF ..'")
    else()
        if(ENABLE_PACKAGING)
            message(WARNING "gr-osmosdr has not been found. Source blocks depending on it will NOT be built.")
        else()
            message(FATAL_ERROR "gr-osmosdr required to build gnss-sdr with the optional OSMOSDR driver")
        endif()
        set(ENABLE_OSMOSDR OFF)
    endif()
else()
    message(STATUS "The (optional) driver for OsmoSDR and related front-ends is not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_OSMOSDR=ON ..' to add support for OsmoSDR and other front-ends (HackRF, bladeRF, Realtek's RTL2832U-based USB dongles, etc.)")
endif()


##############################################
# gr-iio - OPTIONAL
# IIO blocks for GNU Radio
# https://github.com/analogdevicesinc/gr-iio
##############################################
find_package(GRIIO)
set_package_properties(GRIIO PROPERTIES
    PURPOSE "Used for communication with PlutoSDR and FMCOMMS devices."
    TYPE OPTIONAL
)


#####################################################################
# libiio - OPTIONAL
# A library for interfacing with local and remote Linux IIO devices
# https://github.com/analogdevicesinc/libiio
#####################################################################
find_package(LIBIIO)
set_package_properties(LIBIIO PROPERTIES
    PURPOSE "Used for communication with the AD9361 chipset."
    TYPE OPTIONAL
)


##############################################
# TELEORBIT FLEXIBAND FRONTEND - OPTIONAL
##############################################
if(DEFINED ENV{FLEXIBAND_DRIVER})
    message(STATUS "FLEXIBAND_DRIVER environment variable found.")
    set(ENABLE_FLEXIBAND ON)
endif()

if(FLEXIBAND_DRIVER)
    set(ENABLE_FLEXIBAND ON)
endif()

if(ENABLE_FLEXIBAND)
    message(STATUS "The Teleorbit Flexiband front-end source will be compiled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_FLEXIBAND=OFF ..'")
else()
    message(STATUS "The (optional) Teleorbit Flexiband front-end driver adapter is not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_FLEXIBAND=ON ..' to add support for the Teleorbit Flexiband front-end.")
endif()
find_package(TELEORBIT)
set_package_properties(TELEORBIT PROPERTIES
    PURPOSE "Used for communication with the Flexiband front-end."
    TYPE OPTIONAL
)
if(ENABLE_FLEXIBAND)
    if(NOT TELEORBIT_FOUND)
        message(FATAL_ERROR "Teleorbit Flexiband GNU Radio driver required to build gnss-sdr with the optional FLEXIBAND adapter")
    endif()
endif()


######################
# GN3S - OPTIONAL
######################
if(DEFINED ENV{GN3S_DRIVER})
    message(STATUS "GN3S_DRIVER environment variable found.")
    set(ENABLE_GN3S ON)
endif()
if(GN3S_DRIVER)
    set(ENABLE_GN3S ON)
endif()
find_package(GRGN3S QUIET)
set_package_properties(GRGN3S PROPERTIES
    URL "https://github.com/gnss-sdr/gr-gn3s"
    DESCRIPTION "The GN3S v2 front-end GNU Radio block."
    PURPOSE "Used for communication with the GN3S v2 front-end."
    TYPE OPTIONAL
)
if(ENABLE_GN3S)
    message(STATUS "The GN3S driver will be compiled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_GN3S=OFF ..'")
else()
    message(STATUS "The (optional and experimental) GN3S driver is not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_GN3S=ON ..' to add support for the GN3S dongle.")
endif()


#######################################################
# CTTC's digital array beamformer prototype - OPTIONAL
#######################################################
if(DEFINED ENV{RAW_ARRAY_DRIVER})
    message(STATUS "RAW_ARRAY_DRIVER environment variable found.")
    set(ENABLE_ARRAY ON)
endif()

if(RAW_ARRAY_DRIVER)
    set(ENABLE_ARRAY ON)
endif()

find_package(GRDBFCTTC QUIET)
set_package_properties(GRDBFCTTC PROPERTIES
    URL "https://github.com/gnss-sdr/gr-dbfcttcs"
    DESCRIPTION "CTTC's array prototype GNU Radio block."
    PURPOSE "Used for communication with CTTC's antenna array."
    TYPE OPTIONAL
)
if(ENABLE_ARRAY)
    message(STATUS "CTTC's Antenna Array front-end driver will be compiled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_ARRAY=OFF ..'")
else()
    message(STATUS "The (optional) CTTC's Antenna Array front-end driver is not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_ARRAY=ON ..' to add support for the CTTC experimental array front-end.")
endif()



################################################################################
# GPerftools - https://github.com/gperftools/gperftools - OPTIONAL)
################################################################################
find_package(GPERFTOOLS)
set_package_properties(GPERFTOOLS PROPERTIES
    PURPOSE "Used for performance analysis."
    TYPE OPTIONAL
)
if(ENABLE_GPERFTOOLS)
    if(NOT GPERFTOOLS_FOUND)
        message(STATUS "Although ENABLE_GPERFTOOLS has been set to ON, GPerftools has not been found.")
        message(STATUS " Binaries will be compiled without 'tcmalloc' and 'profiler' libraries.")
        message(STATUS " You can install GPerftools from https://github.com/gperftools/gperftools")
    else()
        message(STATUS "GPerftools libraries found.")
        message(STATUS " Binaries will be compiled with 'tcmalloc' and 'profiler' libraries.")
    endif()
    # Set GPerftools related flags if it is available
    # See https://github.com/gperftools/gperftools/blob/master/README
    if(GPERFTOOLS_FOUND)
        if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
            add_compile_options(-fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free)
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            add_compile_options(-fno-builtin)
        endif()
    endif()
endif()



################################################################################
# GNU gprof (OPTIONAL) - https://sourceware.org/binutils/docs/gprof/
################################################################################
if(ENABLE_GPROF)
    add_compile_options(-pg)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
endif()



################################################################################
# Set compiler flags
################################################################################
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    # Add warning flags
    # For "-Wall" see https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
    set(cxx_warning_flags -Wall -Wextra)
    if(NOT (CMAKE_VERSION VERSION_LESS "3.3"))
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${cxx_warning_flags}>")
    else()
        add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:${cxx_warning_flags}>")
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
        add_compile_options(-Wno-missing-field-initializers)
    endif()
    if(CMAKE_CROSSCOMPILING)
        add_compile_options(-Wno-psabi)
    endif()
endif()

# Processor-architecture related flags
# See https://gcc.gnu.org/onlinedocs/gcc/x86-Options.html
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    if(OS_IS_MACOSX)
        add_compile_options(-march=corei7 -mfpmath=sse)
    else()
        if(NOT ENABLE_GENERIC_ARCH)
            if(IS_ARM)
                # ARM-specific options
                # See https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
                if(NOT CMAKE_CROSSCOMPILING)
                    if(ARM_VERSION STREQUAL "arm")
                        # Unknown arm version - try our best to detect
                        add_compile_options(-mcpu=native)
                    else()
                        add_compile_options(-march=${ARM_VERSION})
                    endif()
                endif()
            else()
                add_compile_options(-march=native -mfpmath=sse)
            endif()
        endif()
    endif()
endif()



################################################################################
# clang-tidy https://clang.llvm.org/extra/clang-tidy/index.html
################################################################################
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(NOT (CMAKE_VERSION VERSION_LESS "3.6"))
        find_program(
            CLANG_TIDY_EXE
            NAMES "clang-tidy"
            DOC "Path to clang-tidy executable"
        )
        if(NOT CLANG_TIDY_EXE)
            message(STATUS "clang-tidy not found.")
        else()
            message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
            if(ENABLE_CLANG_TIDY)
                message(STATUS " clang-tidy will be used in compilation. You can disable it with 'cmake -DENABLE_CLANG_TIDY=OFF ..'")
            else()
                message(STATUS " You can enable clang-tidy usage in compilation with 'cmake -DENABLE_CLANG_TIDY=ON ..'")
            endif()
            set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-fix")
            set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
        endif()
    endif()
endif()



################################################################################
# Create uninstall target
################################################################################
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)



################################################################################
# Add subdirectories
################################################################################
add_subdirectory(src)



################################################################################
# Print summary
################################################################################
add_feature_info(ENABLE_UHD ENABLE_UHD "Enables UHD_Signal_Source for using RF front-ends from the USRP family. Requires gr-uhd.")
add_feature_info(ENABLE_OSMOSDR ENABLE_OSMOSDR "Enables Osmosdr_Signal_Source and RtlTcp_Signal_Source for using RF front-ends compatible with the OsmoSDR driver. Requires gr-osmosdr.")
add_feature_info(ENABLE_FMCOMMS2 ENABLE_FMCOMMS2 "Enables Fmcomms2_Signal_Source for FMCOMMS2/3/4 devices. Requires gr-iio.")
add_feature_info(ENABLE_PLUTOSDR ENABLE_PLUTOSDR "Enables Plutosdr_Signal_Source for using ADALM-PLUTO boards. Requires gr-iio.")
add_feature_info(ENABLE_AD9361 ENABLE_AD9361 "Enables Ad9361_Fpga_Signal_Source for devices with the AD9361 chipset. Requires libiio.")
add_feature_info(ENABLE_RAW_UDP ENABLE_RAW_UDP "Enables Custom_UDP_Signal_Source for custom UDP packet sample source. Requires libpcap.")
add_feature_info(ENABLE_FLEXIBAND ENABLE_FLEXIBAND "Enables Flexiband_Signal_Source for using Teleorbit's Flexiband RF front-end. Requires gr-teleorbit.")
add_feature_info(ENABLE_GN3S ENABLE_GN3S "Enables Gn3s_Signal_Source for using the GN3S v2 dongle. Requires gr-gn3s.")
add_feature_info(ENABLE_ARRAY ENABLE_ARRAY "Enables Raw_Array_Signal_Source and Array_Signal_Conditioner for using CTTC's antenna array. Requires gr-dbfcttc.")
add_feature_info(ENABLE_GPERFTOOLS ENABLE_GPERFTOOLS "Enables performance analysis. Requires Gperftools.")
add_feature_info(ENABLE_GPROF ENABLE_GPROF "Enables performance analysis with 'gprof'.")
add_feature_info(ENABLE_CLANG_TIDY ENABLE_CLANG_TIDY "Runs clang-tidy along with the compiler. Requires Clang.")
add_feature_info(ENABLE_PROFILING ENABLE_PROFILING "Runs volk_gnsssdr_profile at the end of the building.")
add_feature_info(ENABLE_OPENCL ENABLE_OPENCL "Enables GPS_L1_CA_PCPS_OpenCl_Acquisition (experimental). Requires OpenCL.")
add_feature_info(ENABLE_CUDA ENABLE_CUDA "Enables GPS_L1_CA_DLL_PLL_Tracking_GPU (experimental). Requires CUDA.")
add_feature_info(ENABLE_FPGA ENABLE_FPGA "Enables building of processing blocks for FPGA off-loading.")
add_feature_info(ENABLE_GENERIC_ARCH ENABLE_GENERIC_ARCH "When disabled, flags such as '-march=native' are passed to the compiler.")
add_feature_info(ENABLE_PACKAGING ENABLE_PACKAGING "Enables software packaging.")
add_feature_info(ENABLE_OWN_GLOG ENABLE_OWN_GLOG "Forces the downloading and building of Google glog.")
add_feature_info(ENABLE_OWN_ARMADILLO ENABLE_OWN_ARMADILLO "Forces the downloading and building of Armadillo.")
add_feature_info(ENABLE_LOG ENABLE_LOG "Enables runtime internal logging with Google glog.")
add_feature_info(ENABLE_UNIT_TESTING ENABLE_UNIT_TESTING "Enables building of Unit Tests.")
add_feature_info(ENABLE_UNIT_TESTING_MINIMAL ENABLE_UNIT_TESTING_MINIMAL "Enables building a minimal set of Unit Tests.")
add_feature_info(ENABLE_UNIT_TESTING_EXTRA ENABLE_UNIT_TESTING_EXTRA "Enables building of Extra Unit Tests and downloading of external data files.")
add_feature_info(ENABLE_SYSTEM_TESTING ENABLE_SYSTEM_TESTING "Enables building of System Tests.")
add_feature_info(ENABLE_SYSTEM_TESTING_EXTRA ENABLE_SYSTEM_TESTING_EXTRA "Enables building of Extra System Tests and downloading of external tools.")
add_feature_info(ENABLE_OWN_GPSTK ENABLE_OWN_GPSTK "Forces the downloading and building of GPSTk for system tests.")
add_feature_info(ENABLE_GNSS_SIM_INSTALL ENABLE_GNSS_SIM_INSTALL "Enables downloading and building of gnss-sim.")
add_feature_info(ENABLE_INSTALL_TESTS ENABLE_INSTALL_TESTS "Install test binaries when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} install'.")

message(STATUS "")
message(STATUS "***************************************")
message(STATUS "*           SUMMARY REPORT            *")
message(STATUS "***************************************")
message(STATUS "")
if(CMAKE_VERSION VERSION_LESS 3.4)
    feature_summary(WHAT ALL)
    feature_summary(FILENAME ${CMAKE_CURRENT_BINARY_DIR}/features.log WHAT ALL)
else()
    feature_summary(WHAT
        REQUIRED_PACKAGES_FOUND
        REQUIRED_PACKAGES_NOT_FOUND
        OPTIONAL_PACKAGES_FOUND
        OPTIONAL_PACKAGES_NOT_FOUND
        ENABLED_FEATURES
        DISABLED_FEATURES
    )
    feature_summary(FILENAME ${CMAKE_CURRENT_BINARY_DIR}/features.log WHAT
        REQUIRED_PACKAGES_FOUND
        REQUIRED_PACKAGES_NOT_FOUND
        OPTIONAL_PACKAGES_FOUND
        OPTIONAL_PACKAGES_NOT_FOUND
        ENABLED_FEATURES
        DISABLED_FEATURES
    )
endif()
message(STATUS "GNSS-SDR v${VERSION} is ready to be built.")

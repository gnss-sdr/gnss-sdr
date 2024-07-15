# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

################################################################################
# Project setup
################################################################################
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree build, it is bad practice.\nTry 'cd build && cmake ..' instead.")
endif()

# Select the release build type by default to get optimization flags.
# This has to come before project() which otherwise initializes it.
# Build type can still be overridden by setting -DCMAKE_BUILD_TYPE=
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "")

cmake_minimum_required(VERSION 2.8.12...3.29)
project(gnss-sdr CXX C)

set(GNSSSDR_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # Allows to be a sub-project
set(GNSSSDR_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND CMAKE_MODULE_PATH ${GNSSSDR_SOURCE_DIR}/cmake/Modules)

################################################################################
# Determine optional blocks/libraries to be built (default: not built)
# Enable them at the command line by doing 'cmake -DENABLE_XXX=ON ..'
################################################################################
include(FeatureSummary)

# Support of optional RF front-ends
option(ENABLE_UHD "Enable the use of UHD (driver for all USRP devices)" ON)

option(ENABLE_OSMOSDR "Enable the use of OsmoSDR and other front-ends (RTL-based dongles, HackRF, bladeRF, etc.) as signal source" OFF)

option(ENABLE_LIMESDR "Enable the use of LimeSDR and Custom LimeSDR as signal source" OFF)

option(ENABLE_FMCOMMS2 "Enable the use of FMCOMMS4-EBZ + ZedBoard hardware, requires gr-iio" OFF)

option(ENABLE_PLUTOSDR "Enable the use of ADALM-PLUTO Evaluation Boards (Analog Devices Inc.), requires gr-iio" OFF)

option(ENABLE_AD936X_SDR "Enable the use of AD936X front-ends using libiio, requires libiio" OFF)

option(ENABLE_AD9361 "Enable the use of AD9361 direct to FPGA hardware, requires libiio" OFF)

option(ENABLE_RAW_UDP "Enable the use of high-optimized custom UDP packet sample source, requires libpcap" OFF)

option(ENABLE_FLEXIBAND "Enable the use of the signal source adater for the Teleorbit Flexiband GNU Radio driver" OFF)

option(ENABLE_ARRAY "Enable the use of CTTC's antenna array front-end as signal source (experimental)" OFF)

option(ENABLE_ZMQ "Enable GNU Radio ZeroMQ Messaging, requires gr-zeromq" ON)

# Performance analysis tools
option(ENABLE_GPERFTOOLS "Enable linking to Gperftools libraries (tcmalloc and profiler)" OFF)

option(ENABLE_GPROF "Enable the use of the GNU profiler tool 'gprof'" OFF)

# Code correctness
option(ENABLE_CLANG_TIDY "Enable the use of clang-tidy when compiling" OFF)

# Acceleration
option(ENABLE_PROFILING "Enable execution of volk_gnsssdr_profile at the end of the building" OFF)

option(ENABLE_OPENCL "Enable building of processing blocks implemented with OpenCL (experimental)" OFF)

option(ENABLE_CUDA "Enable building of processing blocks implemented with CUDA (experimental, requires CUDA SDK)" OFF)

option(ENABLE_FPGA "Enable building of processing blocks implementing FPGA offloading" OFF)

# Building and packaging options
option(ENABLE_PACKAGING "Enable software packaging" OFF)

option(ENABLE_OWN_GLOG "Download glog and link it to gflags" OFF)

option(ENABLE_GLOG_AND_GFLAGS "Force using Google glog and Gflags instead of Abseil" OFF)

option(ENABLE_OWN_ABSEIL "Forces downloading and building of Abseil" OFF)
if(CMAKE_VERSION VERSION_LESS 3.24)
    set(ENABLE_OWN_ABSEIL OFF)
endif()
if(ENABLE_OWN_ABSEIL)
    set(ENABLE_OWN_GLOG OFF)
    set(ENABLE_GLOG_AND_GFLAGS OFF)
endif()

option(ENABLE_OWN_ARMADILLO "Download and build Armadillo locally" OFF)

option(ENABLE_LOG "Enable internal logging" ON)

option(ENABLE_ARMA_NO_DEBUG OFF)

option(ENABLE_STRIP "Create stripped binaries without debugging symbols (in Release build mode only)" OFF)

option(Boost_USE_STATIC_LIBS "Use Boost static libs" OFF)

option(ENABLE_GNUTLS "Forces linking against GnuTLS" OFF)

if(ENABLE_PACKAGING)
    set(ENABLE_ARMA_NO_DEBUG ON)
    set(CMAKE_VERBOSE_MAKEFILE ON)
    set(ENABLE_STRIP OFF)
endif()

# Testing
option(ENABLE_UNIT_TESTING "Build unit tests" ON)

option(ENABLE_UNIT_TESTING_MINIMAL "Build a minimal set of unit tests" OFF)

option(ENABLE_UNIT_TESTING_EXTRA "Download external files and build extra unit tests" OFF)

option(ENABLE_SYSTEM_TESTING "Build system tests" OFF)

option(ENABLE_SYSTEM_TESTING_EXTRA "Download external tools and build extra system tests" OFF)

option(ENABLE_GNSS_SIM_INSTALL "Enable the installation of gnss_sim on the fly" ON)

option(ENABLE_CPUFEATURES "Make use of the cpu_features library" ON)

if(NOT (ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA))
    set(ENABLE_GNSS_SIM_INSTALL OFF)
endif()

if(ENABLE_SYSTEM_TESTING_EXTRA)
    set(ENABLE_SYSTEM_TESTING ON)
endif()

option(ENABLE_OWN_GNSSTK "Force to download, build and link gnsstk for system tests, even if it is already installed" OFF)

if(NOT ENABLE_OWN_GNSSTK)
    option(ENABLE_OWN_GPSTK "Force to download, build and link gnsstk for system tests, even if it is already installed" OFF)
    if(ENABLE_OWN_GPSTK)
        unset(Gnsstk:gnsstk CACHE)
        unset(GNSSTK_FOUND CACHE)
        message(STATUS "WARNING: Option ENABLE_OWN_GPSTK is deprecated, please use ENABLE_OWN_GNSSTK")
        set(ENABLE_OWN_GNSSTK ON)
    endif()
endif()

option(ENABLE_INSTALL_TESTS "Install QA code system-wide" OFF)
if(ENABLE_FPGA)
    set(ENABLE_INSTALL_TESTS ON)
endif()

option(ENABLE_BENCHMARKS "Build code snippets benchmarks" OFF)
if(CMAKE_VERSION VERSION_LESS 3.16.3)
    set(ENABLE_BENCHMARKS OFF)
endif()

option(ENABLE_EXTERNAL_MATHJAX "Use MathJax from an external CDN in HTML docs" ON)

option(ENABLE_ORC "Use (if available) the Optimized Inner Loop Runtime Compiler (ORC)" OFF)

################################################################################
# GNSS-SDR version information
################################################################################
set(THIS_IS_A_RELEASE OFF)   # only related to version name, no further implications.
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
                WORKING_DIRECTORY ${GNSSSDR_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_BRANCH
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
        endif()
        # was this info set in the CMake commandline?
        if(NOT GIT_COMMIT_HASH)
            # Get the latest abbreviated commit hash of the working branch
            execute_process(
                COMMAND ${GIT_EXECUTABLE} log -1 --format=%h
                WORKING_DIRECTORY ${GNSSSDR_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
        endif()
    endif()
endif()

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_API_COMPAT 0)
if(${THIS_IS_A_RELEASE})
    set(VERSION_INFO_MINOR_VERSION 19)
else()
    set(VERSION_INFO_MINOR_VERSION 19.git-${GIT_BRANCH}-${GIT_COMMIT_HASH})
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
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
    include(DetectLinuxDistro)
    if(CMAKE_CROSSCOMPILING)
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be cross-compiled on ${LINUX_DISTRIBUTION} ${LINUX_VER} (${CMAKE_HOST_SYSTEM_PROCESSOR}) for ${CMAKE_SYSTEM_PROCESSOR}")
    else()
        message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCHITECTURE_STRING}")
    endif()
endif()

# Detect macOS / Mac OS X Version
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include(DetectMacOSVersion)
    message(STATUS "Configuring GNSS-SDR v${VERSION} to be built on ${MACOS_DISTRIBUTION}")
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
gnsssdr_check_build_type(${CMAKE_BUILD_TYPE})
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}.")

if(NOT (${CMAKE_BUILD_TYPE} STREQUAL "Release"))
    set(ENABLE_STRIP OFF)
endif()

# Enable optimization options in GCC for Release and RelWithDebInfo build types
if((${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU") AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    if(NOT (${LINUX_DISTRIBUTION} MATCHES "Fedora") AND NOT (${LINUX_DISTRIBUTION} MATCHES "Gentoo"))
        # flag -O3 enables tree vectorization
        # See https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
        set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
        set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
        set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
    endif()
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        # Fix for Debug and None modes in macOS
        # without this, get get a runtime error
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag(-fstandalone-debug HAVE_STANDALONE_DEBUG)
        check_cxx_compiler_flag(-Og HAVE_OG_FLAG)
        if(HAVE_STANDALONE_DEBUG AND HAVE_OG_FLAG)
            set(CMAKE_CXX_FLAGS_DEBUG "-Og -g -fstandalone-debug")
            set(CMAKE_C_FLAGS_DEBUG "-Og -g -fstandalone-debug")
            if(CMAKE_BUILD_TYPE STREQUAL "None")
                add_compile_options(-Og -fstandalone-debug)
            endif()
        endif()
    endif()
endif()

# allow 'large' files in 32 bit builds
if(UNIX)
    if(CMAKE_VERSION VERSION_GREATER 3.12.0)
        add_compile_definitions(_LARGEFILE_SOURCE _FILE_OFFSET_BITS=64 _LARGE_FILES)
    else()
        add_definitions(-D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES)
    endif()
endif()

# If this is an out-of-tree build, do not pollute the original source directory
if(${GNSSSDR_BINARY_DIR} MATCHES ${GNSSSDR_SOURCE_DIR})
    set(LOCAL_INSTALL_BASE_DIR ${GNSSSDR_SOURCE_DIR})
else()
    set(LOCAL_INSTALL_BASE_DIR ${GNSSSDR_BINARY_DIR})
endif()

# Determine if CMake scripts make use of target_sources()
if(CMAKE_VERSION VERSION_GREATER 3.13)
    set(USE_CMAKE_TARGET_SOURCES ON)
endif()

# Determine if we are using make or ninja
if(CMAKE_MAKE_PROGRAM MATCHES "make")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "make")
endif()
if(CMAKE_MAKE_PROGRAM MATCHES "ninja")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "ninja")
endif()
if(CMAKE_MAKE_PROGRAM MATCHES "xcodebuild")
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "xcodebuild")
endif()
if(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)
    set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "${CMAKE_MAKE_PROGRAM}")
endif()

if(CMAKE_VERSION VERSION_LESS 3.12)
    if(POLICY CMP0057)  # required by FindDoxygen.cmake module
        cmake_policy(SET CMP0057 NEW)  # Support if() IN_LIST operator, added in CMake 3.3
    endif()
endif()



################################################################################
# Minimum required versions
################################################################################
set(GNSSSDR_APPLECLANG_MIN_VERSION "500")
set(GNSSSDR_ARMADILLO_MIN_VERSION "5.300.0")
set(GNSSSDR_BOOST_MIN_VERSION "1.53")
set(GNSSSDR_CLANG_MIN_VERSION "3.4.0")
set(GNSSSDR_GCC_MIN_VERSION "4.7.2")
set(GNSSSDR_GFLAGS_MIN_VERSION "2.1.2")
set(GNSSSDR_GNURADIO_MIN_VERSION "3.7.3")
set(GNSSSDR_MAKO_MIN_VERSION "0.4.2")
set(GNSSSDR_MATIO_MIN_VERSION "1.5.3")
set(GNSSSDR_PROTOBUF_MIN_VERSION "3.0.0")
set(GNSSSDR_PYTHON_MIN_VERSION "2.7")
set(GNSSSDR_PYTHON3_MIN_VERSION "3.4")
set(GNSSSDR_ABSEIL_MIN_VERSION "20240116")



################################################################################
# Versions to download and build (but not to install system-wide) if not found
################################################################################
set(GNSSSDR_ARMADILLO_LOCAL_VERSION "12.8.x")
set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.2.2")
set(GNSSSDR_GLOG_LOCAL_VERSION "0.7.1")
set(GNSSSDR_MATIO_LOCAL_VERSION "1.5.27")
set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "27.1")
set(GNSSSDR_PUGIXML_LOCAL_VERSION "1.14")
set(GNSSSDR_GTEST_LOCAL_VERSION "1.14.0")
set(GNSSSDR_GNSS_SIM_LOCAL_VERSION "origin/master")
set(GNSSSDR_GNSSTK_LOCAL_VERSION "14.3.0")
set(GNSSSDR_BENCHMARK_LOCAL_VERSION "1.8.4")
set(GNSSSDR_MATHJAX_EXTERNAL_VERSION "2.7.7")
set(GNSSSDR_ABSL_LOCAL_VERSION "origin/master") # live at head (see https://abseil.io/about/releases)

# Downgrade versions if requirements are not met
if(CMAKE_VERSION VERSION_LESS "3.22")
    set(GNSSSDR_GLOG_LOCAL_VERSION "0.6.0")
endif()
if(CMAKE_VERSION VERSION_LESS "3.16")
    set(GNSSSDR_GLOG_LOCAL_VERSION "0.5.0")
endif()

if(CMAKE_VERSION VERSION_LESS "3.3")
    set(GNSSSDR_GLOG_LOCAL_VERSION "0.4.0")
endif()

if(CMAKE_VERSION VERSION_LESS "3.0.2")
    set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.2.1")  # Fix for CentOS 7
    set(GNSSSDR_GLOG_LOCAL_VERSION "0.3.4")    # Fix for Ubuntu 14.04
endif()

if(CMAKE_VERSION VERSION_LESS "3.5")
    set(GNSSSDR_PUGIXML_LOCAL_VERSION "1.13")
endif()
if(CMAKE_VERSION VERSION_LESS "3.4")
    set(GNSSSDR_PUGIXML_LOCAL_VERSION "1.10")
endif()

if(CMAKE_CROSSCOMPILING OR CMAKE_VERSION VERSION_LESS "3.13" OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0.0)) OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")) OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11")))
    set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "21.12")
endif()

if(CMAKE_VERSION VERSION_LESS "3.13" OR
  (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.3.1) OR
  (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0.0))
    set(GNSSSDR_GTEST_LOCAL_VERSION "1.13.0")
endif()

if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0) OR
  (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0) OR
  CMAKE_VERSION VERSION_LESS 3.5)
    set(GNSSSDR_GTEST_LOCAL_VERSION "1.10.x")
endif()

if(CMAKE_VERSION VERSION_LESS "3.17")
    set(GNSSSDR_GNSSTK_LOCAL_VERSION "13.7.0")
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

# Determine if we use lambdas
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
        set(DO_NOT_USE_LAMBDAS ON)
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        if(CLANG_VERSION VERSION_LESS "600")
            set(DO_NOT_USE_LAMBDAS ON)
        endif()
    else()
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
            set(DO_NOT_USE_LAMBDAS ON)
        endif()
    endif()
endif()

# Determine if we try to use generic lambdas
if(NOT DO_NOT_USE_LAMBDAS)
    if(CMAKE_VERSION VERSION_GREATER 3.1 AND NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND
        (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)))
        set(USE_GENERIC_LAMBDAS ON)
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
    set(CMAKE_C_STANDARD 11)   # set variable just for reporting
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
            add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
            set(CMAKE_CXX_STANDARD 11)   # set variable just for reporting
        else()
            add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
            set(CMAKE_CXX_STANDARD 14)   # set variable just for reporting
        endif()
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
            if(CLANG_VERSION VERSION_LESS "600")
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
                set(CMAKE_CXX_STANDARD 11)
            else()
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
                set(CMAKE_CXX_STANDARD 14)
            endif()
        else()
            if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++11>")
                set(CMAKE_CXX_STANDARD 11)
            else()
                add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:-std=c++14>")
                set(CMAKE_CXX_STANDARD 14)
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
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(^arm)|(^aarch64)")
    set(IS_ARM TRUE)
endif()
if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
    if(CMAKE_CROSSCOMPILING)
        if(NOT CMAKE_NO_SYSTEM_FROM_IMPORTED)
            set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)
        endif()
    endif()
endif()



################################################################################
# pkg-config - Helper tool used when compiling applications and libraries.
################################################################################
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
set(FPHSA_NAME_MISMATCHED ON)
find_package(PkgConfig)



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
        PURPOSE "Googletest v${GNSSSDR_GTEST_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
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
# GNU Radio - https://www.gnuradio.org
################################################################################
list(APPEND GR_REQUIRED_COMPONENTS RUNTIME PMT BLOCKS FFT FILTER ANALOG)
find_package(UHD)
set_package_properties(UHD PROPERTIES
    PURPOSE "Used for communication with front-ends of the USRP family."
    TYPE OPTIONAL
)
if(ENABLE_UHD)
    if(NOT UHD_FOUND)
        set(ENABLE_UHD OFF)
    else()
        list(APPEND GR_REQUIRED_COMPONENTS UHD)
    endif()
endif()

find_package(ZEROMQ)
set_package_properties(ZEROMQ PROPERTIES
    PURPOSE "Used by the ZMQ_Signal_Source."
    TYPE OPTIONAL
)
if(ENABLE_ZMQ)
    if(NOT ZEROMQ_FOUND)
        set(ENABLE_ZMQ OFF)
    else()
        list(APPEND GR_REQUIRED_COMPONENTS ZEROMQ)
    endif()
endif()

find_package(GNURADIO)
set_package_properties(GNURADIO PROPERTIES
    PURPOSE "Implements flowgraph scheduler, provides some processing blocks and classes to create new ones."
    TYPE REQUIRED
)

if(NOT (GNURADIO_VERSION VERSION_LESS "3.8"))
    set(GNURADIO_IS_38_OR_GREATER ON)
endif()



################################################################################
# Detect availability of std::filesystem and set C++ standard accordingly
################################################################################
set(FILESYSTEM_FOUND FALSE)
if(NOT ENABLE_OWN_GNSSTK)
    unset(Gnsstk::gnsstk CACHE)
    unset(GNSSTK_FOUND CACHE)
    unset(GNSSTK_OLDER_THAN_8 CACHE)
    unset(GNSSTK_OLDER_THAN_9 CACHE)
    unset(GNSSTK_OLDER_THAN_13 CACHE)
    find_package(GNSSTK)
    set_package_properties(GNSSTK PROPERTIES
        PURPOSE "Used in some Extra Tests."
    )
else()
    unset(Gnsstk::gnsstk CACHE)
    unset(GNSSTK_FOUND CACHE)
    unset(GNSSTK_OLDER_THAN_8 CACHE)
    unset(GNSSTK_OLDER_THAN_9 CACHE)
    unset(GNSSTK_OLDER_THAN_13 CACHE)
endif()
if(NOT (GNURADIO_VERSION VERSION_LESS 3.8) AND (LOG4CPP_READY_FOR_CXX17 OR GNURADIO_USES_SPDLOG))
    # Check if we have std::filesystem
    if(NOT (CMAKE_VERSION VERSION_LESS 3.8))
        if(NOT GNSSTK_FOUND OR NOT (GNSSTK_FOUND AND GNSSTK_OLDER_THAN_8))  # Fix for GNSSTk < 8.0.0
            find_package(FILESYSTEM COMPONENTS Final Experimental)
            set_package_properties(FILESYSTEM PROPERTIES
                URL "https://en.cppreference.com/w/cpp/filesystem"
                DESCRIPTION "Provides facilities for performing operations on file systems and their components"
                PURPOSE "Work with paths, regular files, and directories."
                TYPE OPTIONAL
            )
        endif()
        if(FILESYSTEM_FOUND)
            set(CMAKE_CXX_STANDARD 17)
            if(CMAKE_VERSION VERSION_GREATER 3.13)
                if(((NOT UHD_FOUND) OR (UHD_FOUND AND ("${UHD_VERSION}" VERSION_GREATER 4.2.99))) AND (GNURADIO_VERSION VERSION_GREATER 3.10.3.99))
                    set(CMAKE_CXX_STANDARD 20)
                #    if(CMAKE_VERSION VERSION_GREATER 3.20.99)
                #         if(((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11.0.0")) OR
                #             ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND NOT (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "12.0")))
                #             set(CMAKE_CXX_STANDARD 23)
                #         endif()
                #    endif()
                endif()
            endif()
            set(CMAKE_CXX_STANDARD_REQUIRED ON)
        endif()
    endif()
endif()
if((NOT PMT_USES_BOOST_ANY) AND (CMAKE_CXX_STANDARD VERSION_LESS 17))
    message(FATAL_ERROR "GNU Radio v${GNURADIO_VERSION} requires C++17. Please update your environment.")
endif()



################################################################################
# Boost - https://www.boost.org
################################################################################
if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") # Fedora 64-bit fix
endif()
# Boost_ADDITIONAL_VERSIONS is only used internally by cmake to know the
# formation of newer versions. No need to increase, not used anymore since newer
# Boost provides its own CMake configuration files.
set(Boost_ADDITIONAL_VERSIONS
    "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
    "1.70.0" "1.70" "1.71.0" "1.71"
)
set(Boost_USE_MULTITHREAD ON)
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

# Define targets if CMake < 3.5
if(CMAKE_VERSION VERSION_LESS 3.5)
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

# Define Boost::headers target if CMake < 3.15
if(CMAKE_VERSION VERSION_LESS 3.15)
    if(NOT TARGET Boost::headers)
        if(CMAKE_VERSION VERSION_LESS 3.0)
            add_library(Boost::headers SHARED IMPORTED) # Trick for CMake 2.8.12
            set_target_properties(Boost::headers PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
                IMPORTED_LOCATION ${Boost_DATE_TIME_LIBRARIES}
            )
        else()
            add_library(Boost::headers INTERFACE IMPORTED)
            set_target_properties(Boost::headers PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR}
            )
        endif()
    endif()
endif()

# Provide package descriptions if Boost >= 1.71.00
if(Boost_VERSION_STRING VERSION_GREATER 1.70.99)
    set_package_properties(boost_headers PROPERTIES
        URL "https://www.boost.org/"
        DESCRIPTION "Header files of Boost libraries"
        PURPOSE "Used widely across the source code."
        TYPE REQUIRED
    )
    set_package_properties(boost_atomic PROPERTIES
        URL "https://www.boost.org/doc/libs/release/doc/html/atomic.html"
        DESCRIPTION "Provides atomic data types and operations on those types"
        PURPOSE "Required by Boost Thread."
        TYPE REQUIRED
    )
    set_package_properties(boost_chrono PROPERTIES
        URL "https://www.boost.org/doc/libs/release/doc/html/chrono.html"
        DESCRIPTION "Useful time utilities"
        PURPOSE "Required by Boost Thread."
        TYPE REQUIRED
    )
    set_package_properties(boost_date_time PROPERTIES
        URL "https://www.boost.org/doc/libs/release/doc/html/date_time.html"
        DESCRIPTION "A set of date-time libraries"
        PURPOSE "Required by Boost Thread."
        TYPE REQUIRED
    )
    set_package_properties(boost_serialization PROPERTIES
        URL "https://www.boost.org/doc/libs/release/libs/serialization/doc/index.html"
        DESCRIPTION "Reversible deconstruction of C++ data structures to sequences of bytes"
        PURPOSE "Used for serializing data."
        TYPE REQUIRED
    )
    set_package_properties(boost_system PROPERTIES
        URL "https://www.boost.org/doc/libs/release/libs/system/doc/html/system.html"
        DESCRIPTION "Extensible error reporting library"
        PURPOSE "Used for error reporting."
        TYPE REQUIRED
    )
    set_package_properties(boost_thread PROPERTIES
        URL "https://www.boost.org/doc/libs/release/doc/html/thread.html"
        DESCRIPTION "Portable C++ multi-threading"
        PURPOSE "Used by GNU Radio multi-threading system."
        TYPE REQUIRED
    )
    if(NOT ${FILESYSTEM_FOUND})
        set_package_properties(boost_filesystem PROPERTIES
            URL "https://www.boost.org/doc/libs/release/libs/filesystem/doc/index.htm"
            DESCRIPTION "Portable facilities to manipulate paths and files"
            PURPOSE "Used for output file handling."
            TYPE REQUIRED
        )
    endif()
endif()

if(Boost_VERSION_STRING VERSION_LESS 1.58.0)
    set(USE_OLD_BOOST_MATH_COMMON_FACTOR ON)
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.65.99)
    set(USE_BOOST_ASIO_IO_CONTEXT ON)
endif()

if(Boost_VERSION_STRING VERSION_LESS 1.73)
    # Disable concepts to address https://github.com/boostorg/asio/issues/312
    if(((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.9) OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (CMAKE_VERSION VERSION_GREATER 3.11))
        target_compile_definitions(Boost::headers
            INTERFACE
                -DBOOST_ASIO_DISABLE_CONCEPTS
        )
    endif()
endif()

# Workaround for https://github.com/boostorg/format/issues/67
if((Boost_VERSION_STRING VERSION_GREATER 1.71) AND (Boost_VERSION_STRING VERSION_LESS 1.73))
    if(CMAKE_CXX_STANDARD VERSION_GREATER 17)
        set(CMAKE_CXX_STANDARD 17)
    endif()
endif()

# Workaround for macOS Sonoma
if((CMAKE_CXX_STANDARD EQUAL 17) AND ((${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND ("${DARWIN_VERSION}" VERSION_GREATER "22.99")))
    add_definitions(-D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION=1)
endif()

# Fix for Boost Asio < 1.70 when using Clang in macOS
if(Boost_VERSION_STRING VERSION_LESS 1.70.0)
    # Check if we have std::string_view
    unset(has_string_view CACHE)
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <string_view>
        int main()
        { std::string_view sv; }"
        has_string_view
    )
endif()

# Fox for CentOS 7 with cmake3 and gcc8
if(Boost_VERSION_STRING VERSION_LESS "1.60")
    set(USE_GENERIC_LAMBDAS OFF)
endif()

# Fix for Boost >= 1.73
if(Boost_VERSION_STRING VERSION_GREATER 1.72.99)
    set(USE_BOOST_BIND_PLACEHOLDERS ON)
endif()



################################################################################
# Detect availability of std::span
################################################################################
unset(has_span CACHE)
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
    #include <span>
    int main()
    { std::span<float> s; }"
    has_span
)



################################################################################
# Detect availability of std::rotl
################################################################################
unset(has_rotl CACHE)
if(CMAKE_CXX_STANDARD VERSION_GREATER 17)
    check_cxx_source_compiles("
        #include <bit>
        #include <cstdint>
        int main()
        {
            std::uint8_t i = 0b00011101;
            auto k = std::rotl(i,0);
        }"
        has_rotl
    )
endif()



################################################################################
# Detect availability of std::put_time (Workaround for gcc < 5.0)
################################################################################
check_cxx_source_compiles("
    #include <iomanip>
    int main()
    { std::put_time(nullptr, \"\"); }"
    has_put_time
)



################################################################################
# Detect availability of std::plus without class specifier
################################################################################
unset(has_std_plus_void CACHE)
if(CMAKE_CXX_STANDARD VERSION_GREATER 11)
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <functional>
        int main()
        { [](float a=1, float b=0){return std::plus<>();}; };"
        has_std_plus_void
    )
endif()



################################################################################
# Detect availability of std::transform_reduce
################################################################################
unset(has_transform_reduce CACHE)
unset(has_transform_reduce_with_execution_policy CACHE)
if(CMAKE_CXX_STANDARD VERSION_GREATER 14)
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <functional>
        #include <numeric>
        #include <vector>
        int main()
        {
        std::vector<float> a(5);
        std::vector<float> b(5);
        auto c = std::transform_reduce(cbegin(a), cend(a), cbegin(b), 0, std::plus<>{}, std::multiplies<>{}); };"
        has_transform_reduce
    )
    check_cxx_source_compiles("
        #include <execution>
        #include <functional>
        #include <numeric>
        #include <vector>
        int main()
        {
        std::vector<float> a(5);
        std::vector<float> b(5);
        auto c = std::transform_reduce(std::execution::par, cbegin(a), cend(a), cbegin(b), 0, std::plus<>{}, std::multiplies<>{}); };"
        has_transform_reduce_with_execution_policy
    )
endif()



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
        message(STATUS "Mako template library not found. See https://www.makotemplates.org/")
        message(STATUS " You can try to install it by typing:")
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                message(STATUS " sudo yum install ${PYTHON_NAME}-mako")
            elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                message(STATUS " sudo zypper install ${PYTHON_NAME}-Mako")
            else()
                message(STATUS " sudo apt-get install ${PYTHON_NAME}-mako")
            endif()
        endif()
        message(FATAL_ERROR "Mako templates required to build VOLK_GNSSSDR")
    endif()

    if(PYTHON_NAME STREQUAL "python")
        # Six
        if(NOT SIX_FOUND)
            message(STATUS "python-six not found. See https://pythonhosted.org/six/")
            if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
                message(STATUS " You can try to install it by typing:")
                if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
                    message(STATUS " sudo yum install ${PYTHON_NAME}-six")
                elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                    message(STATUS " sudo zypper install ${PYTHON_NAME}-six")
                else()
                    message(STATUS " sudo apt-get install ${PYTHON_NAME}-six")
                endif()
            endif()
            message(FATAL_ERROR "six - python 2 and 3 compatibility library required to build VOLK_GNSSSDR")
        endif()
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

    if(CMAKE_VERSION VERSION_LESS 3.27 AND PYTHONINTERP_FOUND)
        set_package_properties(PythonInterp PROPERTIES
            URL "https://www.python.org/"
            DESCRIPTION "An interpreted, high-level, general-purpose programming language (found: v${PYTHON_VERSION_STRING})"
            PURPOSE "Required to build volk_gnsssdr."
            TYPE REQUIRED
        )
    endif()

    set(STRIP_VOLK_GNSSSDR_PROFILE "")
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
        set(STRIP_VOLK_GNSSSDR_PROFILE -DENABLE_STRIP=${ENABLE_STRIP})
        if(ENABLE_PACKAGING)
            set(STRIP_VOLK_GNSSSDR_PROFILE ${STRIP_VOLK_GNSSSDR_PROFILE} -DCMAKE_VERBOSE_MAKEFILE=ON)
        endif()
    endif()

    set(VOLK_GNSSSDR_BUILD_COMMAND "${CMAKE_MAKE_PROGRAM}")
    if(PYTHON_EXECUTABLE)
        set(USE_THIS_PYTHON "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}")
    endif()

    if(CMAKE_GENERATOR STREQUAL Xcode)
        set(VOLK_GNSSSDR_BUILD_COMMAND "xcodebuild"
            "-configuration" $<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:Debug>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        )
    endif()

    if(CMAKE_TOOLCHAIN_FILE)
        set(VOLK_GNSSSDR_COMPILER "")
    else()
        set(VOLK_GNSSSDR_COMPILER -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    endif()

    if(ENABLE_ORC)
        find_package(ORC)
        set_package_properties(ORC PROPERTIES
            PURPOSE "Used by volk_gnsssdr."
            TYPE OPTIONAL
            )
        if(ORC_FOUND)
            set(ORC_ENABLED ON)
        else()
            set(ORC_ENABLED OFF)
            set(ENABLE_ORC OFF)
        endif()
    else()
        set(ORC_ENABLED OFF)
        set(ENABLE_ORC OFF)
    endif()

    set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_COMPILER}
        -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install
        -DENABLE_STATIC_LIBS=ON
        -DENABLE_PROFILING=${ENABLE_PROFILING}
        -DENABLE_ORC=${ORC_ENABLED}
        ${STRIP_VOLK_GNSSSDR_PROFILE}
        ${USE_THIS_PYTHON}
    )
    if(CMAKE_C_FLAGS)    # Required by some packaging systems
        set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS} -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS})
    endif()
    if(CMAKE_CXX_FLAGS)  # Required by some packaging systems
        set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS})
    endif()

    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
            -DCROSSCOMPILE_MULTILIB=TRUE
            -DBOOST_ROOT=$ENV{OECORE_TARGET_SYSROOT}/usr
        )
        if(NOT CMAKE_TOOLCHAIN_FILE)
            set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                -DCMAKE_TOOLCHAIN_FILE=${GNSSSDR_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake
            )
        else()
            set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
            )
        endif()
    else()
        if(CMAKE_TOOLCHAIN_FILE)
            set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
            )
        endif()
    endif()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^cortex")
        set(CMAKE_SYSTEM_PROCESSOR arm-${CMAKE_SYSTEM_PROCESSOR})
    endif()
    include(GNUInstallDirs)
    set(SUPPORTED_CPU_FEATURES_ARCH FALSE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^mips" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "(^aarch64)|(^arm64)|(^ARM64)" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "^arm" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(x86_64)|(AMD64|amd64)|(^i.86$)" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "^(powerpc|ppc)" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "^(s390x)" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "^riscv" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "^loongarch")
        set(SUPPORTED_CPU_FEATURES_ARCH TRUE)
    endif()
    if(${CMAKE_INSTALL_LIBDIR} MATCHES lib64)
        set(VOLK_GNSSSDR_LIB_SUFFIX 64)
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(volk_gnsssdr_module
            PREFIX ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module
            SOURCE_DIR ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/build
            CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:ASAN>
                -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
            DOWNLOAD_COMMAND ""
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_COMMAND ${VOLK_GNSSSDR_BUILD_COMMAND}
            INSTALL_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install
        )
    else()
        if(SUPPORTED_CPU_FEATURES_ARCH)
            set(VOLK_GNSSSDR_BUILD_BYPRODUCTS
                ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/lib${VOLK_GNSSSDR_LIB_SUFFIX}/${CMAKE_FIND_LIBRARY_PREFIXES}volk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}
                ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
            )
            if(ENABLE_CPUFEATURES)
                find_package(CPUFEATURES)
                set_package_properties(CPUFEATURES PROPERTIES
                    URL "https://github.com/google/cpu_features"
                    PURPOSE "Used by the volk_gnsssdr library."
                    TYPE REQUIRED
                )
            endif()
            if(CPUFEATURES_FOUND)
                message(STATUS "Found CpuFeatures: (found version ${CPUFEATURES_VERSION})")
                set_package_properties(CPUFEATURES PROPERTIES
                    DESCRIPTION "A cross platform C99 library to get CPU features at runtime (version: ${CPUFEATURES_VERSION})"
                )
                if((CMAKE_SYSTEM_PROCESSOR MATCHES "(^s390x)|(^riscv)|(^loongarch)") AND (CPUFEATURES_VERSION VERSION_LESS "0.8.0")) # detect cpu_features without s390x / riscv support
                    set(ENABLE_CPUFEATURES OFF)
                endif()
            else()
                set_package_properties(CPUFEATURES PROPERTIES
                    DESCRIPTION "A cross platform C99 library to get CPU features at runtime"
                )
                if((DEFINED VOLK_VERSION AND VOLK_VERSION VERSION_GREATER "2.3") OR (CMAKE_VERSION VERSION_LESS "3.13")) # avoid clash with volk's cpufeatures.
                    set(ENABLE_CPUFEATURES OFF)
                else()
                    set_package_properties(CPUFEATURES PROPERTIES
                        PURPOSE "CpuFeatures will be built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
                    )
                    set(VOLK_GNSSSDR_BUILD_BYPRODUCTS
                        ${VOLK_GNSSSDR_BUILD_BYPRODUCTS}
                        ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}cpu_features${CMAKE_STATIC_LIBRARY_SUFFIX}
                    )
                endif()
            endif()
            set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                    -DVOLK_CPU_FEATURES=${ENABLE_CPUFEATURES}
            )
            ExternalProject_Add(volk_gnsssdr_module
                PREFIX ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module
                SOURCE_DIR ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/build
                CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                    -DCMAKE_BUILD_TYPE=$<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:ASAN>
                    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
                DOWNLOAD_COMMAND ""
                UPDATE_COMMAND ""
                PATCH_COMMAND ""
                BUILD_COMMAND ${VOLK_GNSSSDR_BUILD_COMMAND}
                BUILD_BYPRODUCTS ${VOLK_GNSSSDR_BUILD_BYPRODUCTS}
                INSTALL_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install
            )
        else()
            set(ENABLE_CPUFEATURES OFF)
            set(VOLK_GNSSSDR_CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                -DVOLK_CPU_FEATURES=${ENABLE_CPUFEATURES}
            )
            ExternalProject_Add(volk_gnsssdr_module
                PREFIX ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module
                SOURCE_DIR ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/build
                CMAKE_ARGS ${VOLK_GNSSSDR_CMAKE_ARGS}
                    -DCMAKE_BUILD_TYPE=$<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:ASAN>
                    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
                DOWNLOAD_COMMAND ""
                UPDATE_COMMAND ""
                PATCH_COMMAND ""
                BUILD_COMMAND ${VOLK_GNSSSDR_BUILD_COMMAND}
                BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/lib${VOLK_GNSSSDR_LIB_SUFFIX}/${CMAKE_FIND_LIBRARY_PREFIXES}volk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}
                    ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                    ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr-config-info
                INSTALL_DIR ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install
            )
        endif()
    endif()

    if(NOT ORC_LIBRARIES_STATIC)
        set(ORC_LIBRARIES_STATIC "")
        set(ORC_INCLUDE_DIRS "")
    endif()

    add_library(volk_gnsssdr UNKNOWN IMPORTED)
    set_property(TARGET volk_gnsssdr PROPERTY IMPORTED_LOCATION ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/lib${VOLK_GNSSSDR_LIB_SUFFIX}/libvolk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(VOLK_GNSSSDR_INCLUDE_DIRS "${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/build/include/;${GNSSSDR_SOURCE_DIR}/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/include;${ORC_INCLUDE_DIRS}")
    set(VOLK_GNSSSDR_LIBRARIES volk_gnsssdr ${ORC_LIBRARIES_STATIC})
    if(CPUFEATURES_FOUND)
        set(VOLK_GNSSSDR_LIBRARIES ${VOLK_GNSSSDR_LIBRARIES} CpuFeature::cpu_features)
    endif()

    if(NOT TARGET Volkgnsssdr::volkgnsssdr)
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/build/include)
        add_library(Volkgnsssdr::volkgnsssdr STATIC IMPORTED)
        add_dependencies(Volkgnsssdr::volkgnsssdr volk_gnsssdr_module)
        set_target_properties(Volkgnsssdr::volkgnsssdr PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/lib${VOLK_GNSSSDR_LIB_SUFFIX}/libvolk_gnsssdr${CMAKE_STATIC_LIBRARY_SUFFIX}"
            INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
            INTERFACE_INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${VOLK_GNSSSDR_LIBRARIES}"
        )
        if(CMAKE_VERSION VERSION_GREATER 3.0 AND SUPPORTED_CPU_FEATURES_ARCH)
            if(NOT CPUFEATURES_FOUND AND ENABLE_CPUFEATURES)
                set_target_properties(Volkgnsssdr::volkgnsssdr PROPERTIES
                    INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}cpu_features${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
            endif()
        endif()
    endif()

    if(CMAKE_VERSION VERSION_LESS 3.2)
        add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${LOCAL_INSTALL_BASE_DIR}/install/volk_gnsssdr_profile
        )
    else()
        add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr_profile
                ${LOCAL_INSTALL_BASE_DIR}/install/volk_gnsssdr_profile
            BYPRODUCTS ${LOCAL_INSTALL_BASE_DIR}/install/volk_gnsssdr_profile
        )
    endif()

    add_custom_command(TARGET volk_gnsssdr_module POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${GNSSSDR_BINARY_DIR}/volk_gnsssdr_module/install/bin/volk_gnsssdr-config-info
            ${LOCAL_INSTALL_BASE_DIR}/install/volk_gnsssdr-config-info
    )

    set_package_properties(VOLKGNSSSDR PROPERTIES
        PURPOSE "volk_gnsssdr will be built when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
else()
    set(ENABLE_ORC OFF)
endif()



################################################################################
# Abseil C++ - https://abseil.io/docs/cpp/
################################################################################
if(NOT CMAKE_VERSION VERSION_LESS 3.24
    AND NOT CMAKE_CXX_STANDARD VERSION_LESS 17
    AND NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.3.1)
    AND NOT (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0.0)
    AND NOT (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
    AND NOT ENABLE_OWN_GLOG
    AND NOT ENABLE_GLOG_AND_GFLAGS)
    # See https://github.com/google/oss-policies-info/blob/main/foundational-cxx-support-matrix.md

    if(ENABLE_OWN_ABSEIL)
        include(FetchContent)
        set(ABSEIL_BUILD_COMMAND ${CMAKE_COMMAND}
            "--build" "${GNSSSDR_BINARY_DIR}/abseil-cpp"
            "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            "--target" "install"
        )
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(ABSEIL_BUILD_COMMAND "xcodebuild" "-configuration" $<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:Debug>)
        endif()
        if(CMAKE_TOOLCHAIN_FILE)
            set(ABSEIL_TOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
        endif()
        set(ABSL_PROPAGATE_CXX_STD ON)
        FetchContent_Declare(
            absl
            GIT_REPOSITORY https://github.com/abseil/abseil-cpp
            GIT_TAG ${GNSSSDR_ABSL_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/abseil-cpp
            CMAKE_ARGS -DABSL_PROPAGATE_CXX_STD=ON -ABSL_BUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/abseil-cpp ${ABSEIL_TOOLCHAIN_FILE}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/abseil-cpp
            BUILD_COMMAND ${ABSEIL_BUILD_COMMAND}
            OVERRIDE_FIND_PACKAGE # Requires CMake 3.24
        )
        FetchContent_MakeAvailable(absl)
        set(absl_FOUND TRUE)
        set(ENABLE_GLOG_AND_GFLAGS OFF)
    else()
        find_package(absl)
        set_package_properties(absl PROPERTIES
            URL "https://github.com/abseil/abseil-cpp"
            PURPOSE "Making use of Abseil's log and flags libraries."
            TYPE OPTIONAL
        )
        if(absl_FOUND)
            set_package_properties(absl PROPERTIES
                DESCRIPTION "A collection of C++ library code designed to augment the C++ standard library (found: v${absl_VERSION})"
            )
        else()
            set_package_properties(absl PROPERTIES
                DESCRIPTION "A collection of C++ library code designed to augment the C++ standard library"
            )
        endif()
        if("${absl_VERSION}" VERSION_LESS ${GNSSSDR_ABSEIL_MIN_VERSION})
            unset(absl_FOUND CACHE)
            set(absl_FOUND FALSE)
            set(ENABLE_GLOG_AND_GFLAGS ON)
            set_package_properties(absl PROPERTIES
                DESCRIPTION "A collection of C++ library code designed to augment the C++ standard library (found: v${absl_VERSION}, but it is too old and it will not be used)"
            )
        endif()
    endif()
endif()


if(NOT absl_FOUND)
    set(ENABLE_GLOG_AND_GFLAGS ON)
    ################################################################################
    # gflags - https://github.com/gflags/gflags
    ################################################################################
    set(LOCAL_GFLAGS FALSE)
    if(ENABLE_OWN_GLOG)
        unset(Glog::glog CACHE)
        unset(GLOG_FOUND CACHE)
        unset(Gflags::gflags CACHE)
        unset(GLAGS_FOUND CACHE)
        set(GFLAGS_GREATER_20 TRUE)
    else()
        unset(Glog::glog CACHE)
        unset(GLOG_FOUND CACHE)
        find_package(GLOG)
        if(GLOG_FOUND)
            unset(GFLAGS_GREATER_20 CACHE)
            find_package(GFLAGS)
        endif()
    endif()
    set_package_properties(GFLAGS PROPERTIES
        PURPOSE "Used for commandline flags management."
        TYPE REQUIRED
    )
    if(NOT GFLAGS_FOUND)
        set(ENABLE_OWN_GLOG ON)
        if(GFLAGS_VERSION)
            message(STATUS " A version of the gflags library equal or higher than v${GNSSSDR_GFLAGS_MIN_VERSION} has not been found.")
        else()
            message(STATUS " The gflags library has not been found.")
        endif()
        message(STATUS " gflags v${GNSSSDR_GFLAGS_LOCAL_VERSION} will be downloaded, built, and statically linked automatically")
        message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
        set(GFLAGS_BUILD_COMMAND ${CMAKE_COMMAND}
            "--build" "${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}"
            "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        )
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(GFLAGS_BUILD_COMMAND "xcodebuild" "-configuration" $<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:Debug>)
        endif()
        if(CMAKE_TOOLCHAIN_FILE)
            set(GFLAGS_TOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
        endif()

        if(CMAKE_VERSION VERSION_LESS 3.2)
            ExternalProject_Add(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                PREFIX ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/gflags/gflags.git
                GIT_TAG v${GNSSSDR_GFLAGS_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gflags/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF
                    -DBUILD_STATIC_LIBS=ON
                    -DBUILD_gflags_LIB=ON
                    -DBUILD_gflags_nothreads_LIB=ON
                    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                    ${GFLAGS_TOOLCHAIN_FILE}
                    -DGFLAGS_NAMESPACE=google
                    -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                BUILD_COMMAND ${GFLAGS_BUILD_COMMAND}
                UPDATE_COMMAND ""
                PATCH_COMMAND ""
                INSTALL_COMMAND ""
            )
        else()
            set(GFLAGS_BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX})
            if((CMAKE_BUILD_TYPE STREQUAL Debug) OR (CMAKE_BUILD_TYPE STREQUAL NoOptWithASM) OR
                (CMAKE_BUILD_TYPE STREQUAL Coverage) OR (CMAKE_BUILD_TYPE STREQUAL ASAN))  # Workaround for Ninja generator
                set(GFLAGS_BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags_debug${CMAKE_STATIC_LIBRARY_SUFFIX})
            endif()
            if((CMAKE_VERSION VERSION_GREATER 3.12.0) AND NOT (CMAKE_GENERATOR STREQUAL Xcode))
                set(PARALLEL_BUILD "--parallel 2")
            endif()
            ExternalProject_Add(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                PREFIX ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/gflags/gflags.git
                GIT_TAG v${GNSSSDR_GFLAGS_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gflags/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF
                    -DBUILD_STATIC_LIBS=ON
                    -DBUILD_gflags_LIB=ON
                    -DBUILD_gflags_nothreads_LIB=ON
                    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                    ${GFLAGS_TOOLCHAIN_FILE}
                    -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                BUILD_COMMAND "${GFLAGS_BUILD_COMMAND} ${PARALLEL_BUILD}"
                BUILD_BYPRODUCTS ${GFLAGS_BUILD_BYPRODUCTS}
                UPDATE_COMMAND ""
                PATCH_COMMAND ""
                INSTALL_COMMAND ""
            )
        endif()
        # Note: -DBUILD_gflags_nothreads_LIB=ON is required as a workaround to a bug in gflags 2.2.2. This is fixed in gflags master branch

        set(GFlags_INCLUDE_DIRS
            ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/include CACHE PATH "Local Gflags headers"
        )

        if(CMAKE_VERSION VERSION_LESS "3.0.2")
            set(GFlags_LIBS
                ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
        endif()

        if(NOT TARGET Gflags::gflags)
            file(MAKE_DIRECTORY ${GFlags_INCLUDE_DIRS})
            add_library(Gflags::gflags STATIC IMPORTED)
            add_dependencies(Gflags::gflags gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
            set_target_properties(Gflags::gflags PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
                MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
                MAP_IMPORTED_CONFIG_COVERAGE Debug
                MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
                MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
                MAP_IMPORTED_CONFIG_ASAN Debug
                IMPORTED_LOCATION_NONE ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags_debug${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_INCLUDE_DIRECTORIES ${GFlags_INCLUDE_DIRS}
                INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gflags$<$<CONFIG:Debug>:_debug>${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
            if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
                if(MSVC)
                    set(MSVC_POSTFIX _static)
                endif()
                set_target_properties(Gflags::gflags PROPERTIES
                    IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${MSVC_POSTFIX}_debug${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/Release/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${MSVC_POSTFIX}${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${MSVC_POSTFIX}${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${MSVC_POSTFIX}${CMAKE_STATIC_LIBRARY_SUFFIX}
                    INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}gflags${MSVC_POSTFIX}$<$<CONFIG:Debug>:_debug>${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
            endif()
        endif()

        if(MSVC)
            target_link_libraries(Gflags::gflags INTERFACE shlwapi.lib)
        endif()

        set(LOCAL_GFLAGS TRUE CACHE STRING "GFlags downloaded, built, and statically linked automatically" FORCE)
        set_package_properties(GFLAGS PROPERTIES
            PURPOSE "Gflags v${GNSSSDR_GFLAGS_LOCAL_VERSION} and Glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
        )
        if(CMAKE_VERSION VERSION_LESS 3.2)
            set_property(TARGET Gflags::gflags APPEND PROPERTY
                INTERFACE_COMPILE_DEFINITIONS GFLAGS_OLD_NAMESPACE=1
            )
        endif()
    endif()



    ################################################################################
    # glog - https://github.com/google/glog
    ################################################################################
    set_package_properties(GLOG PROPERTIES
        PURPOSE "Used for runtime internal logging."
        TYPE REQUIRED
    )
    if(NOT GLOG_FOUND OR ${LOCAL_GFLAGS})
        message(STATUS " glog library has not been found")
        if(NOT GFLAGS_FOUND)
            message(STATUS " or it is likely not linked to gflags.")
        endif()
        message(STATUS " glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded, built, and statically linked automatically")
        message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
        find_package(LIBUNWIND)
        set_package_properties(LIBUNWIND PROPERTIES
            PURPOSE "Needed by glog."
            TYPE OPTIONAL
        )
        if(NOT ${LOCAL_GFLAGS})
            if(NOT TARGET gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
                add_library(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} UNKNOWN IMPORTED)
            endif()
            set_property(TARGET gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION} PROPERTY IMPORTED_LOCATION "${GFlags_LIBS}")
            string(REPLACE /include "" GFLAGS_PREFIX_PATH ${GFlags_INCLUDE_DIRS})
        else()
            set(GFLAGS_PREFIX_PATH ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
        endif()
        set(TARGET_GFLAGS gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION})
        set(GLOG_MAKE_PROGRAM ${CMAKE_COMMAND}
            "--build" "${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}"
            "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        )
        if(CMAKE_GENERATOR STREQUAL Xcode)
            set(GLOG_MAKE_PROGRAM "xcodebuild" "-configuration"
                $<$<CONFIG:None>:None>$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:NoOptWithASM>$<$<CONFIG:Coverage>:Coverage>$<$<CONFIG:O2WithASM>:O2WithASM>$<$<CONFIG:O3WithASM>:O3WithASM>$<$<CONFIG:ASAN>:Debug>
            )
        endif()
        if(CMAKE_TOOLCHAIN_FILE)
            set(GLOG_TOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
        endif()

        if(CMAKE_VERSION VERSION_LESS 3.3)
            if(CMAKE_VERSION VERSION_LESS 3.0)
                set(GLOG_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
                set(GFLAGS_LIBRARIES_TO_LINK ${GFlags_LIBS})
                if(${LOCAL_GFLAGS})
                    set(GFLAGS_LIBRARY_DIR_TO_LINK ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}/lib)
                else()
                    set(GFLAGS_LIBRARY_DIR_TO_LINK ${GFlags_LIBRARY_DIRS})
                endif()
                if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                    set(GFLAGS_LIBRARIES_TO_LINK "${GFLAGS_LIBRARIES_TO_LINK} -lc++")
                    set(GLOG_EXPORT_CXX_LIBRARIES "export CXXFLAGS=\"-stdlib=libc++\"")
                endif()
                if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
                    set(GLOG_EXPORT_C_COMPILER "export CC=clang")
                    set(GLOG_EXPORT_CXX_COMPILER "export CXX=clang++")
                endif()
                if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
                    set(GLOG_EXPORT_C_COMPILER "export CC=gcc")
                    set(GLOG_EXPORT_CXX_COMPILER "export CXX=g++")
                endif()
                file(WRITE ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
    "#!/bin/sh
    export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
    export LDFLAGS=-L${GFLAGS_LIBRARY_DIR_TO_LINK}
    export LIBS=\"${GFLAGS_LIBRARIES_TO_LINK}\"
    ${GLOG_EXPORT_CXX_LIBRARIES}
    ${GLOG_EXPORT_C_COMPILER}
    ${GLOG_EXPORT_CXX_COMPILER}
    cd ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/
    aclocal
    automake --add-missing
    autoreconf -vfi
    cd ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
    ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure --enable-shared=no"
                )

                file(COPY ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/tmp/configure_with_gflags
                    DESTINATION ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                        GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
                )

                set(GLOG_CONFIGURE ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/configure_with_gflags)

                # Ensure that aclocal and libtool are present
                if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
                    if(EXISTS "/usr/bin/libtoolize")
                        if(EXISTS "/usr/bin/aclocal" OR
                            EXISTS "/usr/bin/aclocal-1.16" OR
                            EXISTS "/usr/bin/aclocal-1.15" OR
                            EXISTS "/usr/bin/aclocal-1.14" OR
                            EXISTS "/usr/bin/aclocal-1.13" OR
                            EXISTS "/usr/bin/aclocal-1.11" OR
                            EXISTS "/usr/bin/aclocal-1.10")
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

                if(GLOG_MAKE_PROGRAM MATCHES "ninja")
                    find_program(GLOG_MAKE_EXECUTABLE make
                        PATHS
                            /usr/bin
                            /usr/local/bin
                    )
                    if(NOT GLOG_MAKE_EXECUTABLE)
                        message(FATAL_ERROR "make is required to build Glog from source.")
                    endif()
                    set(GLOG_MAKE_PROGRAM ${GLOG_MAKE_EXECUTABLE})
                endif()
                ExternalProject_Add(glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    DEPENDS ${TARGET_GFLAGS}
                    PREFIX ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/google/glog/
                    GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
                    BUILD_COMMAND "${GLOG_MAKE_PROGRAM}"
                    UPDATE_COMMAND ""
                    PATCH_COMMAND ""
                    INSTALL_COMMAND ""
                )
                set(GLOG_LIBRARIES
                    ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
                set(GLOG_INCLUDE_DIRS
                    ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
                    ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
                )
            else()  # CMake > 3.0 but < 3.3
                ExternalProject_Add(glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    DEPENDS ${TARGET_GFLAGS}
                    PREFIX ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/google/glog/
                    GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
                        -DCMAKE_PREFIX_PATH=${GFLAGS_PREFIX_PATH}
                        ${GLOG_TOOLCHAIN_FILE}
                        -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                    BUILD_COMMAND ${GLOG_MAKE_PROGRAM}
                    UPDATE_COMMAND ""
                    PATCH_COMMAND ""
                    INSTALL_COMMAND ""
                )
                set(GLOG_INCLUDE_DIRS
                    ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
                    ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                    ${GFlags_INCLUDE_DIRS}
                )
            endif()
        else()  # CMake > 3.3
            set(GLOG_BUILD_BYPRODUCTS
                ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
            if((CMAKE_BUILD_TYPE STREQUAL Debug) OR (CMAKE_BUILD_TYPE STREQUAL NoOptWithASM) OR
                (CMAKE_BUILD_TYPE STREQUAL Coverage) OR (CMAKE_BUILD_TYPE STREQUAL ASAN))  # Workaround for Ninja generator
                set(GLOG_BUILD_BYPRODUCTS
                    ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glogd${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
            endif()
            if((CMAKE_VERSION VERSION_GREATER 3.12.0) AND NOT (CMAKE_GENERATOR STREQUAL Xcode) AND NOT CMAKE_CROSSCOMPILING)
                set(PARALLEL_BUILD "--parallel 2")
            endif()
            if(GNSSSDR_GLOG_LOCAL_VERSION VERSION_GREATER 0.5.0)
                set(GLOG_GTEST -DWITH_GTEST=FALSE)
            endif()
            if(NOT (CMAKE_VERSION VERSION_LESS "3.22"))
                set(GNSSSDR_GLOG_LOCAL_GFLAGS -DCMAKE_REQUIRED_INCLUDES=${GFlags_INCLUDE_DIRS})
            endif()
            ExternalProject_Add(glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                DEPENDS ${TARGET_GFLAGS}
                PREFIX ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/google/glog/
                GIT_TAG v${GNSSSDR_GLOG_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                    -DCMAKE_PREFIX_PATH=${GFLAGS_PREFIX_PATH}
                    ${GLOG_TOOLCHAIN_FILE}
                    -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                    -DBUILD_SHARED_LIBS=OFF
                    ${GLOG_GTEST}
                    -DBUILD_TESTING=OFF
                    "${GNSSSDR_GLOG_LOCAL_GFLAGS}"
                BUILD_COMMAND "${GLOG_MAKE_PROGRAM} ${PARALLEL_BUILD}"
                BUILD_BYPRODUCTS ${GLOG_BUILD_BYPRODUCTS}
                UPDATE_COMMAND ""
                PATCH_COMMAND ""
                INSTALL_COMMAND ""
            )
            set(GLOG_INCLUDE_DIRS
                ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src
                ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}
                ${GFlags_INCLUDE_DIRS}
            )
        endif()

        add_dependencies(glog-${GNSSSDR_GLOG_LOCAL_VERSION} Gflags::gflags)

        # Create Glog::glog target
        if(NOT TARGET Glog::glog)
            file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/thirdparty/glog/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/src)
            file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION})
            add_library(Glog::glog STATIC IMPORTED)
            add_dependencies(Glog::glog glog-${GNSSSDR_GLOG_LOCAL_VERSION})
            if(CMAKE_VERSION VERSION_LESS 3.0)
                set_target_properties(Glog::glog PROPERTIES
                    IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                    IMPORTED_LOCATION "${GLOG_LIBRARIES}"
                    INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
                    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
                    INTERFACE_LINK_LIBRARIES "${GLOG_LIBRARIES}"
                )
            else()
                set_target_properties(Glog::glog PROPERTIES
                    IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                    IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
                    MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
                    MAP_IMPORTED_CONFIG_COVERAGE Debug
                    MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
                    MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
                    MAP_IMPORTED_CONFIG_ASAN Debug
                    IMPORTED_LOCATION_NONE ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glogd${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                    IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
                    INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}glog$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}
                )
                if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
                    set_target_properties(Glog::glog PROPERTIES
                        IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}glogd${CMAKE_STATIC_LIBRARY_SUFFIX}
                        IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/Release/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                        IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                        IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}glog${CMAKE_STATIC_LIBRARY_SUFFIX}
                        INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/glog-${GNSSSDR_GLOG_LOCAL_VERSION}/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}glog$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}
                    )
                endif()
            endif()
        endif()
        if(NOT (CMAKE_VERSION VERSION_LESS "3.22"))
            set_target_properties(Glog::glog PROPERTIES
                INTERFACE_COMPILE_DEFINITIONS "GLOG_USE_GLOG_EXPORT;GLOG_USE_GFLAGS"
                INTERFACE_COMPILE_FEATURES "cxx_std_14")
        endif()

        if(LIBUNWIND_FOUND)
            target_link_libraries(Glog::glog INTERFACE Libunwind::libunwind)
        endif()
        set(LOCAL_GLOG TRUE CACHE STRING "Glog downloaded, built, and statically linked automatically" FORCE)

        set_package_properties(GLOG PROPERTIES
            PURPOSE "Glog v${GNSSSDR_GLOG_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
        )
    endif()
endif()



################################################################################
# Check that BLAS (Basic Linear Algebra Subprograms) is found in the system
# See https://www.netlib.org/blas/
################################################################################
if((${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND ("${DARWIN_VERSION}" VERSION_LESS "23"))
    # Avoid using the implementation that comes with the Accelerate framework
    include(AvoidAccelerate)
else()
    if(NOT BLA_VENDOR)
        set(BLA_VENDOR "Generic")
    endif()
    if(NOT CMAKE_CROSSCOMPILING AND NOT DEFINED BLA_PREFER_PKGCONFIG)
        set(BLA_PREFER_PKGCONFIG ON)  # Required by riscv64 arch
    endif()
    find_package(BLAS)
    set_package_properties(BLAS PROPERTIES
        URL "https://www.netlib.org/blas/"
        DESCRIPTION "Basic Linear Algebra Subprograms"
        PURPOSE "Used for matrix algebra computations."
        TYPE REQUIRED
    )
endif()
if(NOT BLAS_FOUND)
    message(" The BLAS library has not been found.")
    message(" You can try to install it by typing:")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
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
if(NOT TARGET BLAS::BLAS)
    add_library(BLAS::BLAS SHARED IMPORTED)
    set_target_properties(BLAS::BLAS PROPERTIES
        IMPORTED_LOCATION ${BLAS_LIBRARIES}
        INTERFACE_LINK_LIBRARIES ${BLAS_LIBRARIES}
    )
endif()



################################################################################
# Check that LAPACK (Linear Algebra PACKage) is found in the system
# See https://www.netlib.org/lapack/
################################################################################
if(NOT((${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND ("${DARWIN_VERSION}" VERSION_LESS "23")))
    find_package(LAPACK)
    set_package_properties(LAPACK PROPERTIES
        URL "https://www.netlib.org/lapack/"
        DESCRIPTION "Linear Algebra PACKage"
        PURPOSE "Used for matrix algebra computations."
        TYPE REQUIRED
    )
endif()
if(NOT LAPACK_FOUND)
    message(" The LAPACK library has not been found.")
    if(LINUX_DISTRIBUTION)
        message(" You can try to install it by typing:")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo yum install lapack-devel")
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(" sudo zypper install lapack-devel")
        else()
            message(" sudo apt-get install liblapack-dev")
        endif()
    endif()
    message(FATAL_ERROR "LAPACK is required to build gnss-sdr")
endif()
if(NOT TARGET LAPACK::LAPACK)
    add_library(LAPACK::LAPACK SHARED IMPORTED)
    set_target_properties(LAPACK::LAPACK PROPERTIES
        IMPORTED_LOCATION ${LAPACK_LIBRARIES}
        INTERFACE_LINK_LIBRARIES ${LAPACK_LIBRARIES}
    )
endif()



################################################################################
# Armadillo - https://arma.sourceforge.net/
################################################################################
if(ENABLE_OWN_ARMADILLO)
    unset(Armadillo::armadillo CACHE)
    unset(ARMADILLO_FOUND CACHE)
else()
    unset(Armadillo::armadillo CACHE)
    unset(ARMADILLO_FOUND CACHE)
    find_package(Armadillo)
endif()

set_package_properties(Armadillo PROPERTIES
    URL "https://arma.sourceforge.net/"
    PURPOSE "Used for matrix computations."
    TYPE REQUIRED
)
if(ARMADILLO_FOUND)
    set_package_properties(Armadillo PROPERTIES
        DESCRIPTION "C++ library for linear algebra and scientific computing (found: v${ARMADILLO_VERSION_STRING})"
    )
    if(${ARMADILLO_VERSION_STRING} VERSION_LESS ${GNSSSDR_ARMADILLO_MIN_VERSION})
        unset(Armadillo::armadillo CACHE)
        unset(ARMADILLO_FOUND CACHE)
        set(ENABLE_OWN_ARMADILLO ON)
        message(STATUS " Armadillo >= v${GNSSSDR_ARMADILLO_MIN_VERSION} has not been found.")
    else()
        if(NOT ENABLE_OWN_ARMADILLO)
            add_library(Armadillo::armadillo SHARED IMPORTED)
            set_target_properties(Armadillo::armadillo PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${ARMADILLO_LIBRARIES}"
                INTERFACE_INCLUDE_DIRECTORIES "${ARMADILLO_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${ARMADILLO_LIBRARIES}"
            )
        endif()
    endif()
else()
    message(STATUS " Armadillo has not been found.")
endif()

if(NOT ARMADILLO_FOUND OR ENABLE_OWN_ARMADILLO)
    message(STATUS " Armadillo ${GNSSSDR_ARMADILLO_LOCAL_VERSION} will be downloaded, built, and statically linked automatically")
    message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set(armadillo_BRANCH ${GNSSSDR_ARMADILLO_LOCAL_VERSION})
    set(armadillo_RELEASE ${armadillo_BRANCH})

    #############################################
    # Check if GFORTRAN is found in the system
    #############################################
    if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND NOT MSVC)
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
    set(ARMADILLO_BUILD_COMMAND ${CMAKE_COMMAND}
        "--build" "${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
        "--target" install
    )
    if(CMAKE_TOOLCHAIN_FILE)
        set(ARMADILLO_TOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
    else()
        set(ARMADILLO_TOOLCHAIN_FILE "")
    endif()
    set(ARMADILLO_CXX_VERSION "")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 4.8.3)
        set(ARMADILLO_CXX_VERSION -DCMAKE_CXX_FLAGS=-std=c++11)
    endif()
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(armadillo-${armadillo_RELEASE}
            PREFIX ${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
            GIT_REPOSITORY https://gitlab.com/conradsnicta/armadillo-code.git
            GIT_TAG ${armadillo_BRANCH}
            UPDATE_COMMAND ""
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
            CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_SMOKE_TEST=OFF
                -DALLOW_BLAS_LAPACK_MACOS=ON
                ${ARMADILLO_CXX_VERSION}
                ${ARMADILLO_TOOLCHAIN_FILE}
            BUILD_COMMAND ${ARMADILLO_BUILD_COMMAND}
            INSTALL_COMMAND ""
        )
    else()
        ExternalProject_Add(armadillo-${armadillo_RELEASE}
            PREFIX ${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
            GIT_REPOSITORY https://gitlab.com/conradsnicta/armadillo-code.git
            GIT_TAG ${armadillo_BRANCH}
            UPDATE_COMMAND ""
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
            CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_SMOKE_TEST=OFF
                -DALLOW_BLAS_LAPACK_MACOS=ON
                ${ARMADILLO_CXX_VERSION}
                ${ARMADILLO_TOOLCHAIN_FILE}
            BUILD_COMMAND ${ARMADILLO_BUILD_COMMAND}
            BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/armadillo-${armadillo_RELEASE}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
            INSTALL_COMMAND ""
        )
    endif()

    # Create imported target
    ExternalProject_Get_Property(armadillo-${armadillo_RELEASE} binary_dir)
    if(NOT GFORTRAN)
        set(GFORTRAN "")
    endif()
    set(ARMADILLO_STATIC_LIBRARY ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(LOCAL_ARMADILLO TRUE CACHE STRING "Armadillo downloaded, built, and statically linked automatically" FORCE)
    set(ARMADILLO_VERSION_STRING ${armadillo_RELEASE})
    file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}/include)
    add_library(Armadillo::armadillo STATIC IMPORTED)
    add_dependencies(Armadillo::armadillo armadillo-${armadillo_RELEASE})
    set_target_properties(Armadillo::armadillo PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
        MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
        MAP_IMPORTED_CONFIG_COVERAGE Debug
        MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
        MAP_IMPORTED_CONFIG_ASAN Debug
        IMPORTED_LOCATION_NONE ${ARMADILLO_STATIC_LIBRARY}
        IMPORTED_LOCATION_DEBUG ${ARMADILLO_STATIC_LIBRARY}
        IMPORTED_LOCATION_RELEASE ${ARMADILLO_STATIC_LIBRARY}
        IMPORTED_LOCATION_RELWITHDEBINFO ${ARMADILLO_STATIC_LIBRARY}
        IMPORTED_LOCATION_MINSIZEREL ${ARMADILLO_STATIC_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/thirdparty/armadillo/armadillo-${armadillo_RELEASE}/include
        INTERFACE_LINK_LIBRARIES "${BLAS_LIBRARIES};${LAPACK_LIBRARIES};${GFORTRAN};${ARMADILLO_STATIC_LIBRARY}"
    )
    if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
        set_target_properties(Armadillo::armadillo PROPERTIES
            IMPORTED_LOCATION_DEBUG ${binary_dir}/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELEASE ${binary_dir}/Release/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_RELWITHDEBINFO ${binary_dir}/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
            IMPORTED_LOCATION_MINSIZEREL ${binary_dir}/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}
            INTERFACE_LINK_LIBRARIES "${BLAS_LIBRARIES};${LAPACK_LIBRARIES};${GFORTRAN};${binary_dir}/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}armadillo${CMAKE_STATIC_LIBRARY_SUFFIX}"
        )
    endif()
    set_package_properties(Armadillo PROPERTIES
        DESCRIPTION "C++ library for linear algebra and scientific computing"
        PURPOSE "Armadillo ${GNSSSDR_ARMADILLO_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
    # Fix for macOS
    find_library(ARPACK_LIBRARY
        NAMES arpack
        PATHS ${CMAKE_SYSTEM_LIBRARY_PATH} /opt/local/lib /usr/lib64 /usr/lib /usr/local/lib64 /usr/local/lib /opt/local/lib64
    )
    if(ARPACK_LIBRARY)
        target_link_libraries(Armadillo::armadillo INTERFACE ${ARPACK_LIBRARY})
    endif()
    find_library(FLEXIBLAS_LIBRARY
        NAMES flexiblas
        PATHS ${CMAKE_SYSTEM_LIBRARY_PATH} /opt/local/lib /lib64 /lib /usr/lib64 /usr/lib /usr/local/lib64 /usr/local/lib /opt/local/lib64
    )
    if(FLEXIBLAS_LIBRARY)
        target_link_libraries(Armadillo::armadillo INTERFACE ${FLEXIBLAS_LIBRARY})
    endif()
    find_library(SUPERLU_LIBRARY
        NAMES superlu
        PATHS ${CMAKE_SYSTEM_LIBRARY_PATH} /opt/local/lib /usr/lib64 /usr/lib /usr/local/lib64 /usr/local/lib /opt/local/lib64
    )
    if(SUPERLU_LIBRARY)
        target_link_libraries(Armadillo::armadillo INTERFACE ${SUPERLU_LIBRARY})
    endif()
endif()



################################################################################
# OpenSSL https://www.openssl.org/ or GnuTLS - https://www.gnutls.org/
################################################################################
include(GnssSdrCrypto)



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
    message(STATUS " Matio v${GNSSSDR_MATIO_LOCAL_VERSION} will be downloaded, built, and statically linked automatically")
    message(STATUS " when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set_package_properties(MATIO PROPERTIES
        PURPOSE "Matio v${GNSSSDR_MATIO_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
    # Find ZLIB library https://github.com/madler/zlib
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
    if(NOT ZLIB_FOUND)
        message(FATAL_ERROR "*** The zlib library is required to build Matio from source.")
    endif()

    # Find HDF5 library
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
    if(NOT HDF5_FOUND)
        message(STATUS " The hdf5 library has not been found in your system.")
        message(STATUS " Please try to install it by doing:")
        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            message(STATUS "  $ sudo port install hdf5")
            message(STATUS " or")
            message(STATUS "  $ brew install hdf5")
        endif()
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
            if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat" OR ${LINUX_DISTRIBUTION} MATCHES "CentOS")
                message(STATUS " sudo yum install hdf5-devel")
            elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
                message(STATUS " sudo zypper install hdf5-devel")
            else()
                message(STATUS " sudo apt-get install libhdf5-dev")
            endif()
        endif()
        message(FATAL_ERROR "*** The hdf5 library is required to build Matio from source.")
    endif()

    if(CMAKE_VERSION VERSION_LESS 3.7 OR "${HDF5_VERSION}" VERSION_LESS "1.8.13")
        get_filename_component(ZLIB_BASE_DIR ${ZLIB_INCLUDE_DIRS} DIRECTORY)
        if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
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
                message(FATAL_ERROR "libtool is required to build matio from source.")
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
                message(FATAL_ERROR "aclocal is required to build matio from source.")
            endif()
        endif()

        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            if((NOT EXISTS /usr/local/bin/glibtoolize AND NOT EXISTS /opt/local/bin/glibtoolize) OR
                (NOT EXISTS /usr/local/bin/aclocal AND NOT EXISTS /opt/local/bin/aclocal))
                message(" libtool/automake tools have not been found.")
                message(" You can try to install them by typing:")
                message(" 'sudo port install libtool automake', if you use Macports, or 'brew install libtool automake', if you use Homebrew")
                message(FATAL_ERROR "libtool/automake tools are required to build matio from source")
            endif()
            if(CMAKE_GENERATOR STREQUAL Xcode)
                if(EXISTS /opt/local/bin/glibtoolize OR EXISTS /opt/local/bin/aclocal)
                    if(NOT EXISTS /usr/local/bin/glibtoolize OR NOT EXISTS /usr/local/bin/aclocal)
                        message(" WARNING: libtool/atomake binaries cannot be found by Xcode. Please do:")
                        message("sudo ln -s /opt/local/bin/glibtoolize /usr/local/bin/")
                        message("sudo ln -s /opt/local/bin/aclocal /usr/local/bin/")
                        message("sudo ln -s /opt/local/bin/autom4te /usr/local/bin/")
                        message("sudo ln -s /opt/local/bin/automake /usr/local/bin/")
                        message("sudo ln -s /opt/local/bin/autoconf /usr/local/bin/")
                        message("sudo ln -s /opt/local/bin/autoreconf /usr/local/bin/")  # not needed by Matio, but by Protocol Buffers
                        message(FATAL_ERROR "libtool/automake tools cannot be found by Xcode")
                    endif()
                endif()
            endif()
        endif()

        list(GET HDF5_LIBRARIES 0 HDF5_FIRST_DIR)
        get_filename_component(HDF5_BASE_DIR2 ${HDF5_FIRST_DIR} DIRECTORY)
        get_filename_component(HDF5_BASE_DIR ${HDF5_BASE_DIR2} DIRECTORY)
        if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            if(EXISTS /opt/local/include/hdf5.h)
                set(HDF5_BASE_DIR /opt/local)
            endif()
            if(EXISTS /usr/local/include/hdf5.h)
                set(HDF5_BASE_DIR /usr/local)
            endif()
        endif()
        set(MATIO_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
        if(MATIO_MAKE_PROGRAM MATCHES "ninja" OR CMAKE_GENERATOR STREQUAL Xcode)
            find_program(MATIO_MAKE_EXECUTABLE make
                PATHS
                /usr/bin
                /usr/local/bin
            )
            if(NOT MATIO_MAKE_EXECUTABLE)
                message(FATAL_ERROR "make is required to build Matio from source.")
            endif()
            set(MATIO_MAKE_PROGRAM ${MATIO_MAKE_EXECUTABLE})
        endif()
        set(MATIO_STATIC_LIB ${GNSSSDR_BINARY_DIR}/matio/lib/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_STATIC_LIBRARY_SUFFIX})
        ExternalProject_Add(matio-${GNSSSDR_MATIO_LOCAL_VERSION}
            PREFIX ${GNSSSDR_BINARY_DIR}/matio
            GIT_REPOSITORY https://github.com/tbeu/matio
            GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
            UPDATE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/autogen.sh
            CONFIGURE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}/configure --with-hdf5=${HDF5_BASE_DIR} --with-zlib=${ZLIB_BASE_DIR} --with-default-file-ver=7.3 --enable-mat73=yes --prefix=<INSTALL_DIR>
            BUILD_COMMAND ${MATIO_MAKE_PROGRAM}
        )
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/matio/lib)
    else()  # CMake >= 3.7
        include(GNUInstallDirs)
        set(MATIO_STATIC_LIB ${GNSSSDR_BINARY_DIR}/matio/${CMAKE_INSTALL_LIBDIR}/${CMAKE_FIND_LIBRARY_PREFIXES}matio${CMAKE_STATIC_LIBRARY_SUFFIX})
        if("${HDF5_VERSION}" VERSION_GREATER "1.8.18" AND "${HDF5_VERSION}" VERSION_LESS "1.10.4")
            # weird workaround, but it works in all tested distros (Ubuntu, Debian, Fedora, CentOS, OpenSUSE)
            set(EXTRA_MATIO_BUILD_FLAGS -DMATIO_WITH_ZLIB=OFF -DHAVE_ZLIB=1)
        endif()
        ExternalProject_Add(matio-${GNSSSDR_MATIO_LOCAL_VERSION}
            PREFIX ${GNSSSDR_BINARY_DIR}/matio
            GIT_REPOSITORY https://github.com/tbeu/matio
            GIT_TAG v${GNSSSDR_MATIO_LOCAL_VERSION}
            UPDATE_COMMAND ""
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/matio/matio-${GNSSSDR_MATIO_LOCAL_VERSION}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/matio
            CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
                -DMATIO_SHARED=OFF
                "${EXTRA_MATIO_BUILD_FLAGS}"
                -DHDF5_USE_STATIC_LIBRARIES=OFF
                -DMATIO_DEFAULT_FILE_VERSION:STRING=7.3
                -DMATIO_MAT73=ON
                -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/matio
            BUILD_COMMAND ${CMAKE_COMMAND}
                "--build" "${GNSSSDR_BINARY_DIR}/matio"
                "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            BUILD_BYPRODUCTS ${MATIO_STATIC_LIB}
        )
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/matio/${CMAKE_INSTALL_LIBDIR})
    endif()

    if(NOT TARGET Matio::matio)
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/matio/include)
        add_library(Matio::matio STATIC IMPORTED)
        add_dependencies(Matio::matio matio-${GNSSSDR_MATIO_LOCAL_VERSION})
        set_target_properties(Matio::matio PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION ${MATIO_STATIC_LIB}
            INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/matio/include
            INTERFACE_LINK_LIBRARIES "${MATIO_STATIC_LIB};${HDF5_LIBRARIES};${ZLIB_LIBRARIES}"
        )
    endif()
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
    message(STATUS " PugiXML v${GNSSSDR_PUGIXML_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'.")
    set(PUGIXML_COMPILER -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})
    set(PUGIXML_CMAKE_FLAGS "")
    if(DEFINED ENV{OECORE_TARGET_SYSROOT})
        set(PUGIXML_COMPILER "")
        if(NOT CMAKE_TOOLCHAIN_FILE)
            set(PUGIXML_CMAKE_FLAGS "-DCMAKE_TOOLCHAIN_FILE=${GNSSSDR_SOURCE_DIR}/cmake/Toolchains/oe-sdk_cross.cmake")
        else()
            set(PUGIXML_CMAKE_FLAGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
        endif()
    else()
        if(CMAKE_TOOLCHAIN_FILE)
            set(PUGIXML_CMAKE_FLAGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
        endif()
    endif()
    set(PUGIXML_BUILD_COMMAND ${CMAKE_COMMAND}
        "--build" "${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}"
        "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
    )
    if(CMAKE_VERSION VERSION_LESS 3.2)
        ExternalProject_Add(pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/zeux/pugixml
            GIT_TAG v${GNSSSDR_PUGIXML_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            CMAKE_ARGS ${PUGIXML_COMPILER}
                ${PUGIXML_CMAKE_FLAGS}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_COMMAND ${PUGIXML_BUILD_COMMAND}
            INSTALL_COMMAND ""
        )
    else()
        if(CMAKE_VERSION VERSION_GREATER 3.12.0)
            set(PARALLEL_BUILD "--parallel 2")
        endif()
        ExternalProject_Add(pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            PREFIX ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            GIT_REPOSITORY https://github.com/zeux/pugixml
            GIT_TAG v${GNSSSDR_PUGIXML_LOCAL_VERSION}
            SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            BINARY_DIR ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}
            CMAKE_ARGS ${PUGIXML_COMPILER}
                ${PUGIXML_CMAKE_FLAGS}
                -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            UPDATE_COMMAND ""
            PATCH_COMMAND ""
            BUILD_COMMAND "${PUGIXML_BUILD_COMMAND} ${PARALLEL_BUILD}"
            BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
            INSTALL_COMMAND ""
        )
    endif()

    set(PUGIXML_LIBRARIES ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX})

    if(NOT TARGET Pugixml::pugixml)
        file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/src)
        add_library(Pugixml::pugixml STATIC IMPORTED)
        add_dependencies(Pugixml::pugixml pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION})
        set_target_properties(Pugixml::pugixml PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
            MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
            MAP_IMPORTED_CONFIG_COVERAGE Debug
            MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_ASAN Debug
            IMPORTED_LOCATION_NONE ${PUGIXML_LIBRARIES}
            IMPORTED_LOCATION_DEBUG ${PUGIXML_LIBRARIES}
            IMPORTED_LOCATION_RELEASE ${PUGIXML_LIBRARIES}
            IMPORTED_LOCATION_RELWITHDEBINFO ${PUGIXML_LIBRARIES}
            IMPORTED_LOCATION_MINSIZEREL ${PUGIXML_LIBRARIES}
            INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/thirdparty/pugixml/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/src
            INTERFACE_LINK_LIBRARIES ${PUGIXML_LIBRARIES}
        )
        if((CMAKE_GENERATOR STREQUAL Xcode) OR MSVC)
            set_target_properties(Pugixml::pugixml PROPERTIES
                IMPORTED_LOCATION_DEBUG ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/Debug/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELEASE ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/Release/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_RELWITHDEBINFO ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/RelWithDebInfo/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
                IMPORTED_LOCATION_MINSIZEREL ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/MinSizeRel/${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
                INTERFACE_LINK_LIBRARIES ${GNSSSDR_BINARY_DIR}/pugixml-${GNSSSDR_PUGIXML_LOCAL_VERSION}/$<$<CONFIG:Debug>:Debug/>$<$<CONFIG:Release>:Release/>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo/>$<$<CONFIG:MinSizeRel>:MinSizeRel/>${CMAKE_FIND_LIBRARY_PREFIXES}pugixml${CMAKE_STATIC_LIBRARY_SUFFIX}
            )
        endif()
    endif()

    set_package_properties(PUGIXML PROPERTIES
        PURPOSE "PugiXML v${GNSSSDR_PUGIXML_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()



################################################################################
# Protocol Buffers https://github.com/protocolbuffers/protobuf
################################################################################
if(((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")) OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")) OR
  ((CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11")))
    if(CMAKE_VERSION VERSION_LESS 3.19)
        find_package(Protobuf)
    else()
        find_package(Protobuf 3.0...21.12)
    endif()
else()
    find_package(Protobuf)

    if((CMAKE_BUILD_TYPE STREQUAL "Debug") AND Protobuf_FOUND AND absl_FOUND)
        # This Regular Expression is used to convert the version string provided by `find_package(Protobuf)` into the
        # appropriate binary version string. So, for instance, "4.25.3" becomes "25.3.0".
        string(REGEX REPLACE "^[0-9]+\.([0-9]+\.[0-9]+)$" "\\1.0" PROTOBUF_LIBRARY_VERSION "${Protobuf_VERSION}")
        if((PROTOBUF_LIBRARY_VERSION VERSION_GREATER_EQUAL "22") AND (PROTOBUF_LIBRARY_VERSION VERSION_LESS "26"))
            pkg_check_modules(protobuf REQUIRED IMPORTED_TARGET protobuf=${PROTOBUF_LIBRARY_VERSION})
            target_link_libraries(protobuf::libprotobuf INTERFACE PkgConfig::protobuf)
        endif()
    endif()
endif()
set_package_properties(Protobuf PROPERTIES
    URL "https://protobuf.dev/"
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
            PATHS
                /usr/bin
                /usr/local/bin
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
                PREFIX ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                UPDATE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                CONFIGURE_COMMAND "${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} --host=$ENV{OECORE_TARGET_ARCH} --with-protoc=${PROTOC_EXECUTABLE}"
                BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
                INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} DESTDIR= install
                BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
                    ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
            )
            set(PROTOBUF_PROTOC_EXECUTABLE "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc")
        endif()
    else()
        if(CMAKE_VERSION VERSION_GREATER "3.13")
            if(absl_FOUND)
                if(absl_VERSION)
                    if(${absl_VERSION} VERSION_LESS "20230117")
                        unset(absl_FOUND CACHE)
                        set(absl_FOUND OFF)
                    endif()
                else()
                    unset(absl_FOUND CACHE)
                    set(absl_FOUND OFF)
                endif()
            endif()
            if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0"))
                unset(absl_FOUND CACHE)
                set(absl_FOUND OFF)
            endif()
            if(absl_FOUND)
                set_package_properties(absl PROPERTIES
                    DESCRIPTION "An open-source collection of C++ code designed to augment the C++ standard library (found: v${absl_VERSION})"
                )
            else()
                if(absl_VERSION)
                    set_package_properties(absl PROPERTIES
                        DESCRIPTION "An open-source collection of C++ code designed to augment the C++ standard library (found: v${absl_VERSION})"
                    )
                else()
                    set_package_properties(absl PROPERTIES
                        DESCRIPTION "An open-source collection of C++ code designed to augment the C++ standard library"
                    )
                endif()
                if(NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0"))
                    message(STATUS "The Abseil library (https://github.com/abseil/abseil-cpp) >= v20230117 is required to be installed before building Protocol Buffers >22.x on the fly.")
                endif()
                message(STATUS " Instead, Protocol Buffers v21.12 will be built, which does not require Abseil.")
                set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "21.12")
            endif()
        endif()
        if(CMAKE_VERSION VERSION_LESS "3.13" OR NOT absl_FOUND)
            if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
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
            if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                if((NOT EXISTS /usr/local/bin/glibtoolize AND NOT EXISTS /opt/local/bin/glibtoolize) OR
                    (NOT EXISTS /usr/local/bin/aclocal AND NOT EXISTS /opt/local/bin/aclocal))
                    message(" libtool/automake tools have not been found.")
                    message(" You can try to install them by typing:")
                    message(" 'sudo port install libtool automake', if you use Macports, or 'brew install libtool automake', if you use Homebrew")
                    message(FATAL_ERROR "libtool/automake tools are required to build Protocol Buffers from source")
                endif()
                if(CMAKE_GENERATOR STREQUAL Xcode)
                    if(EXISTS /opt/local/bin/glibtoolize OR EXISTS /opt/local/bin/aclocal)
                        if(NOT EXISTS /usr/local/bin/glibtoolize OR NOT EXISTS /usr/local/bin/aclocal)
                            message(" WARNING: libtool/automake binaries cannot be found by Xcode. Please do:")
                            message("sudo ln -s /opt/local/bin/glibtoolize /usr/local/bin/")
                            message("sudo ln -s /opt/local/bin/aclocal /usr/local/bin/")
                            message("sudo ln -s /opt/local/bin/autom4te /usr/local/bin/")
                            message("sudo ln -s /opt/local/bin/automake /usr/local/bin/")
                            message("sudo ln -s /opt/local/bin/autoconf /usr/local/bin/")
                            message("sudo ln -s /opt/local/bin/autoreconf /usr/local/bin/")
                            message(FATAL_ERROR "libtool/automake tools cannot be found by Xcode")
                        endif()
                    endif()
                endif()
            endif()

            set(PROTOBUF_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
            if(PROTOBUF_MAKE_PROGRAM MATCHES "ninja" OR CMAKE_GENERATOR STREQUAL Xcode)
                find_program(MAKE_EXECUTABLE make
                    PATHS
                        /usr/bin
                        /usr/local/bin
                )
                if(NOT MAKE_EXECUTABLE)
                    message(FATAL_ERROR "make is required to build Protocol Buffers from source.")
                endif()
                set(PROTOBUF_MAKE_PROGRAM ${MAKE_EXECUTABLE})
            endif()
            if(CMAKE_VERSION VERSION_LESS 3.2)
                ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    PREFIX ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                    GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    UPDATE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                    CONFIGURE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    BUILD_COMMAND ${PROTOBUF_MAKE_PROGRAM}
                    INSTALL_COMMAND ${PROTOBUF_MAKE_PROGRAM} DESTDIR= install
                )
            else()
                if(CMAKE_MAKE_PROGRAM MATCHES "make")
                    include(ProcessorCount)
                    ProcessorCount(NUMBER_OF_PROCESSORS)
                    if(NUMBER_OF_PROCESSORS GREATER 1)
                        set(PROTOBUF_PARALLEL_BUILD "-j${NUMBER_OF_PROCESSORS}")
                    endif()
                endif()
                ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    PREFIX ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
                    GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    BINARY_DIR ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    UPDATE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/autogen.sh
                    CONFIGURE_COMMAND ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure --prefix=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                    BUILD_COMMAND ${PROTOBUF_MAKE_PROGRAM} ${PROTOBUF_PARALLEL_BUILD}
                    INSTALL_COMMAND ${PROTOBUF_MAKE_PROGRAM} DESTDIR= install
                    BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
                        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
                )
            endif()

            if(NOT TARGET protobuf::protoc)
                add_executable(protobuf::protoc IMPORTED)
            endif()
            add_dependencies(protobuf::protoc protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
            unset(Protobuf_PROTOC_EXECUTABLE)
            set(PROTOBUF_PROTOC_EXECUTABLE "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc")
            set_target_properties(protobuf::protoc PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc"
                INTERFACE_LINK_LIBRARIES "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protoc${CMAKE_STATIC_LIBRARY_SUFFIX}"
            )
            file(MAKE_DIRECTORY ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include)
            if(NOT TARGET protobuf::libprotobuf)
                add_library(protobuf::libprotobuf STATIC IMPORTED)
                add_dependencies(protobuf::libprotobuf protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
            endif()
            set_target_properties(protobuf::libprotobuf PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}"
                INTERFACE_INCLUDE_DIRECTORIES "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include"
                INTERFACE_LINK_LIBRARIES "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}"
            )
            if(CMAKE_VERSION VERSION_LESS "3.10")
                set(Protobuf_PROTOC_EXECUTABLE ${PROTOBUF_PROTOC_EXECUTABLE})
            endif()
        else()  # CMake >= 3.13 and Abseil found
            include(BuildProtobuf)
        endif()
    endif()
    if(${Protobuf_VERSION} VERSION_LESS ${GNSSSDR_PROTOBUF_MIN_VERSION})
        if(NOT (${Protobuf_VERSION} EQUAL "0.0.0"))
            set_package_properties(Protobuf PROPERTIES
                PURPOSE "Protocol Buffers found (v${Protobuf_VERSION}) is too old (> v${GNSSSDR_PROTOBUF_MIN_VERSION} needed)."
            )
        endif()
    endif()
    set_package_properties(Protobuf PROPERTIES
        PURPOSE "Protocol Buffers v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} will be downloaded, built, and statically linked when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME}'."
    )
endif()
if(${Protobuf_VERSION} VERSION_EQUAL "0.0.0")
    unset(Protobuf_VERSION)
endif()



################################################################################
# Doxygen - https://www.doxygen.nl (OPTIONAL, used if found)
################################################################################
if(CMAKE_VERSION VERSION_LESS 3.6.9)
    find_package(Doxygen)
else()
    find_package(Doxygen OPTIONAL_COMPONENTS dot)
endif()
set_package_properties(Doxygen PROPERTIES
    URL "https://www.doxygen.nl"
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
if(CMAKE_VERSION VERSION_LESS 3.2.3)
    find_package(LATEX)
else()
    find_package(LATEX COMPONENTS PDFLATEX)
endif()
set_package_properties(LATEX PROPERTIES
    URL "https://www.latex-project.org"
    DESCRIPTION "High-quality typesetting system"
    PURPOSE "Used to generate a PDF manual by doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} pdfmanual'."
    TYPE OPTIONAL
)
if(DEFINED MATHJAX2_ROOT)
    set(ENABLE_EXTERNAL_MATHJAX OFF)
endif()
if(ENABLE_EXTERNAL_MATHJAX AND NOT DEFINED MATHJAX2_USE_ROOT)
    set(MATHJAX2_USE_ROOT "https://cdnjs.cloudflare.com/ajax/libs/mathjax/${GNSSSDR_MATHJAX_EXTERNAL_VERSION}")
endif()
unset(MATHJAX2_FOUND CACHE)
find_package(MATHJAX2)
set_package_properties(MATHJAX2 PROPERTIES
    URL "https://www.mathjax.org"
    DESCRIPTION "Beautiful and accessible math in all browsers"
    PURPOSE "Used to generate equations in HTML docs when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'."
    TYPE OPTIONAL
)

if(DOXYGEN_FOUND)
    message(STATUS "Doxygen found.")
    message(STATUS " You can build the documentation with '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'.")
    message(STATUS " When done, point your browser to ${GNSSSDR_BINARY_DIR}/docs/html/index.html")
    if(CMAKE_VERSION VERSION_LESS 3.9.6)
        if(DOXYGEN_DOT_FOUND)
            set(HAVE_DOT "YES")
        endif()
        if(NOT TARGET Doxygen::doxygen)
            add_executable(Doxygen::doxygen IMPORTED GLOBAL)
            set_target_properties(Doxygen::doxygen PROPERTIES
                IMPORTED_LOCATION "${DOXYGEN_EXECUTABLE}"
            )
        endif()
    else()
        if(TARGET Doxygen::dot)
            set(HAVE_DOT "YES")
        else()
            set(HAVE_DOT "NO")
        endif()
    endif()
    file(TO_NATIVE_PATH ${GNSSSDR_SOURCE_DIR} top_srcdir)
    file(TO_NATIVE_PATH ${GNSSSDR_BINARY_DIR} top_builddir)
    set(PDFMANUAL_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
    if(PDFMANUAL_MAKE_PROGRAM MATCHES "ninja")
        find_program(PDFMANUAL_MAKE_EXECUTABLE make
            PATHS
                /usr/bin
                /usr/local/bin
        )
        if(NOT PDFMANUAL_MAKE_EXECUTABLE)
            if(PDFLATEX_COMPILER)
                message(STATUS "Warning: make is required to build the PDF manual, so target pdfmanual will not be generated.")
            endif()
            set(PDFLATEX_COMPILER FALSE)
        endif()
        set(PDFMANUAL_MAKE_PROGRAM ${PDFMANUAL_MAKE_EXECUTABLE})
    endif()
    if(PDFLATEX_COMPILER)
        set(GENERATE_PDF_DOCUMENTATION "YES")
    else()
        set(GENERATE_PDF_DOCUMENTATION "NO")
    endif()
    if(MATHJAX2_FOUND)
        set(GNSSSDR_USE_MATHJAX "YES")
    else()
        set(GNSSSDR_USE_MATHJAX "NO")
    endif()
    configure_file(${GNSSSDR_SOURCE_DIR}/docs/doxygen/Doxyfile.in
        ${GNSSSDR_BINARY_DIR}/docs/doxygen/Doxyfile
        @ONLY
    )
    add_custom_target(doc
        Doxygen::doxygen ${GNSSSDR_BINARY_DIR}/docs/doxygen/Doxyfile
        WORKING_DIRECTORY ${GNSSSDR_BINARY_DIR}
        COMMENT "Generating HTML documentation from source code with Doxygen..." VERBATIM
    )
    if(PDFLATEX_COMPILER)
        message(STATUS " '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} pdfmanual' will generate a manual at ${GNSSSDR_BINARY_DIR}/docs/GNSS-SDR_manual.pdf")
        add_custom_target(pdfmanual
            COMMAND ${PDFMANUAL_MAKE_PROGRAM}
            COMMAND ${CMAKE_COMMAND} -E copy refman.pdf ${GNSSSDR_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
            COMMAND ${PDFMANUAL_MAKE_PROGRAM} clean
            DEPENDS doc
            WORKING_DIRECTORY ${GNSSSDR_BINARY_DIR}/docs/latex
            COMMENT "Generating PDF manual with LaTeX and pdflatex..." VERBATIM
        )
    endif()
    message(STATUS " '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc-clean' will clean the documentation.")
    if(CMAKE_VERSION VERSION_LESS 3.17)
        add_custom_target(doc-clean
            COMMAND ${CMAKE_COMMAND} -E remove_directory ${GNSSSDR_BINARY_DIR}/docs/html
            COMMAND ${CMAKE_COMMAND} -E remove_directory ${GNSSSDR_BINARY_DIR}/docs/latex
            COMMAND ${CMAKE_COMMAND} -E remove ${GNSSSDR_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
            COMMENT "Cleaning documentation." VERBATIM
        )
    else()
        add_custom_target(doc-clean
            COMMAND ${CMAKE_COMMAND} -E rm -rf ${GNSSSDR_BINARY_DIR}/docs/html
            COMMAND ${CMAKE_COMMAND} -E rm -rf ${GNSSSDR_BINARY_DIR}/docs/latex
            COMMAND ${CMAKE_COMMAND} -E rm -f ${GNSSSDR_BINARY_DIR}/docs/GNSS-SDR_manual.pdf
            COMMENT "Cleaning documentation." VERBATIM
        )
    endif()
else()
    message(STATUS " Doxygen has not been found in your system.")
    message(STATUS " You can get nice code documentation by using it!")
    message(STATUS " Get it from https://www.doxygen.nl/download.html")
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(STATUS " or simply by doing 'sudo yum install doxygen-latex'.")
        else()
            message(STATUS " or simply by doing 'sudo apt-get install doxygen-latex'.")
        endif()
    endif()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        message(STATUS " or simply by doing 'sudo port install doxygen +docs', if you are using Macports,")
        message(STATUS " or 'brew cask install mactex', then restart Terminal and do 'brew install graphviz doxygen', if you use Homebrew.")
    endif()
    set(ENABLE_EXTERNAL_MATHJAX OFF)
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
    if(ENABLE_PACKAGING)
        set(ENABLE_OPENCL OFF)
        message(STATUS "ENABLE_PACKAGING is set to ON so the use of OpenCL has been disabled.")
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
    set(CMAKE_CUDA_STANDARD 14)
    set(CMAKE_CUDA_EXTENSIONS ON)
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        include(CheckLanguage)
        check_language(CUDA)
        if(CMAKE_CUDA_COMPILER)
            enable_language(CUDA)
            set(CUDA_FOUND TRUE)
        else()
            set(ENABLE_CUDA OFF)
        endif()
    else()
        find_package(CUDA REQUIRED)
        set_package_properties(CUDA PROPERTIES
            URL "https://developer.nvidia.com/cuda-downloads"
            PURPOSE "Used in some processing block implementations."
            TYPE REQUIRED
        )
        if(CUDA_FOUND)
            set_package_properties(CUDA PROPERTIES
                DESCRIPTION "Library for parallel programming in Nvidia GPUs (found: v${CUDA_VERSION_STRING})"
            )
            set(CMAKE_CUDA_COMPILER_ID "NVIDIA")  # only for reporting purposes
            set(CMAKE_CUDA_COMPILER_VERSION ${CUDA_VERSION_STRING})  # only for reporting purposes
        else()
            set_package_properties(CUDA PROPERTIES
                DESCRIPTION "Library for parallel programming in Nvidia GPUs"
            )
            set(ENABLE_CUDA OFF)
        endif()
    endif()
endif()
if(ENABLE_CUDA)
    message(STATUS "NVIDIA CUDA GPU Acceleration will be enabled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_CUDA=OFF ..'")
else()
    message(STATUS "NVIDIA CUDA GPU Acceleration will be not enabled.")
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
    message(STATUS "FPGA offloading will be enabled.")
    message(STATUS " You can disable it with 'cmake -DENABLE_FPGA=OFF ..'")
else()
    message(STATUS "FPGA offloading will be not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_FPGA=ON ..' to add support for FPGA offloading.")
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



##########################################
# gr-limesdr - OPTIONAL
# https://github.com/myriadrf/gr-limesdr
##########################################
find_package(GRLIMESDR)
set_package_properties(GRLIMESDR PROPERTIES
    PURPOSE "Used for communication with LimeSDR."
    TYPE OPTIONAL
)
if(ENABLE_LIMESDR)
    if(GRLIMESDR_FOUND)
        message(STATUS "The driver for LimeSDR will be compiled.")
        message(STATUS " You can disable it with 'cmake -DENABLE_LIMESDR=OFF ..'")
    else()
        if(ENABLE_PACKAGING)
            message(WARNING "gr-limesdr has not been found. Source blocks depending on it will NOT be built.")
        else()
            message(FATAL_ERROR "gr-limesdr required to build gnss-sdr with the optional LIMESDR driver")
        endif()
        set(ENABLE_LIMESDR OFF)
    endif()
else()
    message(STATUS "The (optional) driver for LimeSDR is not enabled.")
    message(STATUS " Enable it with 'cmake -DENABLE_LIMESDR=ON ..' to add support for LimeSDR.")
endif()



##############################################
# gr-iio - OPTIONAL
# IIO blocks for GNU Radio
# https://github.com/analogdevicesinc/gr-iio
##############################################
if(GNURADIO_IIO_FOUND)
    set(GRIIO_FOUND TRUE)
    set(GR_IIO_INCLUDE_HAS_GNURADIO TRUE)
    set(GNURADIO_API_IIO TRUE)
else()
    # Force to always search for gr-iio
    unset(GRIIO_FOUND CACHE)
    unset(IIO_INCLUDE_DIRS CACHE)
    unset(IIO_LIBRARIES CACHE)
    find_package(GRIIO)
    set_package_properties(GRIIO PROPERTIES
        PURPOSE "Used for communication with PlutoSDR and FMCOMMS devices."
        TYPE OPTIONAL
    )
endif()
find_package(LIBAD9361)
set_package_properties(LIBAD9361 PROPERTIES
    PURPOSE "Used for configuring devices with the AD9361 chipset."
    TYPE OPTIONAL
)
if(NOT LIBAD9361_FOUND)
    set(ENABLE_AD9361 OFF)
    set(ENABLE_AD936X_SDR OFF)
    set(ENABLE_FMCOMMS2 OFF)
endif()
if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
    if(NOT GRIIO_FOUND)
        message(STATUS "gnuradio-iio not found, its installation is required.")
        message(STATUS "Please build and install the following projects:")
        message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
        message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
        message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
        if(ENABLE_PACKAGING)
            set(ENABLE_PLUTOSDR OFF)
            set(ENABLE_FMCOMMS2 OFF)
        else()
            message(FATAL_ERROR "gnuradio-iio is required for building gnss-sdr with -DENABLE_PLUTOSDR=ON or -DENABLE_FMCOMMS2=ON.")
        endif()
    endif()
endif()



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
if(ENABLE_AD9361 OR ENABLE_FMCOMMS2)
    if(NOT LIBIIO_FOUND)
        message(STATUS "libiio not found, its installation is required.")
        message(STATUS "Please build and install the following projects:")
        message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
        message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
        message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
        if(ENABLE_PACKAGING)
            set(ENABLE_AD9361 OFF)
            set(ENABLE_AD936X_SDR OFF)
            set(ENABLE_FMCOMMS2 OFF)
        else()
            message(FATAL_ERROR "libiio is required for building gnss-sdr with -DENABLE_AD9361=ON.")
        endif()
    endif()
endif()



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
set(CXX_WARNING_FLAGS -Wall -Wextra)
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    # Add warning flags
    # For "-Wall" see https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
    if(NOT (CMAKE_VERSION VERSION_LESS "3.3"))
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${CXX_WARNING_FLAGS}>")
    else()
        add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:${CXX_WARNING_FLAGS}>")
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
        add_compile_options(-Wno-missing-field-initializers)
    endif()
    if(CMAKE_CROSSCOMPILING OR NOT ENABLE_PACKAGING)
        if(NOT ENABLE_CUDA)
            add_compile_options(-Wno-psabi)
        endif()
    endif()
    if(IS_ARM)
        if((CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "7.1.0") AND (CMAKE_VERSION VERSION_GREATER "3.1"))
            if(CMAKE_CXX_STANDARD VERSION_LESS 17)
                set(CXX_ALIGN_FLAG -faligned-new)
                if(NOT (CMAKE_VERSION VERSION_LESS "3.3"))
                    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${CXX_ALIGN_FLAG}>")
                else()
                    add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:${CXX_ALIGN_FLAG}>")
                endif()
            endif()
        endif()
    endif()
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(NOT (CMAKE_VERSION VERSION_LESS "3.3"))
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${CXX_WARNING_FLAGS}>")
    else()
        add_compile_options("$<$<STREQUAL:$<TARGET_PROPERTY:LINKER_LANGUAGE>,CXX>:${CXX_WARNING_FLAGS}>")
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
    ${GNSSSDR_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${GNSSSDR_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${GNSSSDR_BINARY_DIR}/cmake_uninstall.cmake
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
add_feature_info(ENABLE_LIMESDR ENABLE_LIMESDR "Enables Limesdr_Signal_Source. Requires gr-limesdr.")
add_feature_info(ENABLE_FMCOMMS2 ENABLE_FMCOMMS2 "Enables Fmcomms2_Signal_Source for FMCOMMS2/3/4 devices. Requires gr-iio and libad9361-dev.")
add_feature_info(ENABLE_PLUTOSDR ENABLE_PLUTOSDR "Enables Plutosdr_Signal_Source for using ADALM-PLUTO boards. Requires gr-iio.")
add_feature_info(ENABLE_AD9361 ENABLE_AD9361 "Enables Ad9361_Fpga_Signal_Source for devices with the AD9361 chipset. Requires libiio and libad9361-dev.")
add_feature_info(ENABLE_AD936X_SDR ENABLE_AD936X_SDR "Enables Ad936x_Iio_Signal_Source to access AD936X front-ends using libiio. Requires libiio and libad9361-dev.")
add_feature_info(ENABLE_RAW_UDP ENABLE_RAW_UDP "Enables Custom_UDP_Signal_Source for custom UDP packet sample source. Requires libpcap.")
add_feature_info(ENABLE_FLEXIBAND ENABLE_FLEXIBAND "Enables Flexiband_Signal_Source for using Teleorbit's Flexiband RF front-end. Requires gr-teleorbit.")
add_feature_info(ENABLE_ARRAY ENABLE_ARRAY "Enables Raw_Array_Signal_Source and Array_Signal_Conditioner for using CTTC's antenna array. Requires gr-dbfcttc.")
add_feature_info(ENABLE_ZMQ ENABLE_ZMQ "Enables ZMQ_Signal_Source for GNU Radio ZeroMQ messages. Requires gr-zeromq.")
add_feature_info(ENABLE_GPERFTOOLS ENABLE_GPERFTOOLS "Enables performance analysis. Requires Gperftools.")
add_feature_info(ENABLE_GPROF ENABLE_GPROF "Enables performance analysis with 'gprof'.")
add_feature_info(ENABLE_CLANG_TIDY ENABLE_CLANG_TIDY "Runs clang-tidy along with the compiler. Requires Clang.")
add_feature_info(ENABLE_PROFILING ENABLE_PROFILING "Runs volk_gnsssdr_profile at the end of the building.")
add_feature_info(ENABLE_OPENCL ENABLE_OPENCL "Enables GPS_L1_CA_PCPS_OpenCl_Acquisition (experimental). Requires OpenCL.")
add_feature_info(ENABLE_CUDA ENABLE_CUDA "Enables GPS_L1_CA_DLL_PLL_Tracking_GPU (experimental). Requires CUDA.")
add_feature_info(ENABLE_FPGA ENABLE_FPGA "Enables building of processing blocks for FPGA offloading.")
add_feature_info(ENABLE_ARMA_NO_DEBUG ENABLE_ARMA_NO_DEBUG "Enables passing the ARMA_NO_DEBUG macro to Armadillo, hence disabling bound checking.")
add_feature_info(ENABLE_PACKAGING ENABLE_PACKAGING "Enables software packaging.")
add_feature_info(ENABLE_OWN_GLOG ENABLE_OWN_GLOG "Forces the downloading and building of Google glog.")
add_feature_info(ENABLE_GLOG_AND_GFLAGS ENABLE_GLOG_AND_GFLAGS "Forces the usage of Google glog and Gflags instead of Abseil.")
add_feature_info(ENABLE_OWN_ABSEIL ENABLE_OWN_ABSEIL "Forces downloading and building Abseil. Supersedes ENABLE_OWN_GLOG.")
add_feature_info(ENABLE_OWN_ARMADILLO ENABLE_OWN_ARMADILLO "Forces the downloading and building of Armadillo.")
add_feature_info(ENABLE_GNUTLS ENABLE_GNUTLS "Forces linking against GnuTLS instead of OpenSSL.")
add_feature_info(ENABLE_LOG ENABLE_LOG "Enables runtime internal logging.")
add_feature_info(ENABLE_ORC ENABLE_ORC "Use the Optimized Inner Loop Runtime Compiler (ORC) for building volk_gnsssdr.")
add_feature_info(ENABLE_STRIP ENABLE_STRIP "Enables the generation of stripped binaries (without debugging symbols).")
add_feature_info(ENABLE_UNIT_TESTING ENABLE_UNIT_TESTING "Enables building of Unit Tests.")
add_feature_info(ENABLE_UNIT_TESTING_MINIMAL ENABLE_UNIT_TESTING_MINIMAL "Enables building a minimal set of Unit Tests.")
add_feature_info(ENABLE_UNIT_TESTING_EXTRA ENABLE_UNIT_TESTING_EXTRA "Enables building of Extra Unit Tests and downloading of external data files.")
add_feature_info(ENABLE_SYSTEM_TESTING ENABLE_SYSTEM_TESTING "Enables building of System Tests.")
add_feature_info(ENABLE_SYSTEM_TESTING_EXTRA ENABLE_SYSTEM_TESTING_EXTRA "Enables building of Extra System Tests and downloading of external tools.")
add_feature_info(ENABLE_OWN_GNSSTK ENABLE_OWN_GNSSTK "Forces the downloading and building of gnsstk for system tests.")
add_feature_info(ENABLE_GNSS_SIM_INSTALL ENABLE_GNSS_SIM_INSTALL "Enables downloading and building of gnss-sim.")
add_feature_info(ENABLE_INSTALL_TESTS ENABLE_INSTALL_TESTS "Install test binaries when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} install'.")
add_feature_info(ENABLE_BENCHMARKS ENABLE_BENCHMARKS "Enables building of code snippet benchmarks.")
add_feature_info(ENABLE_EXTERNAL_MATHJAX ENABLE_EXTERNAL_MATHJAX "Use MathJax from an external CDN in HTML docs when doing '${CMAKE_MAKE_PROGRAM_PRETTY_NAME} doc'.")
add_feature_info(ENABLE_CPUFEATURES ENABLE_CPUFEATURES "Make use of the cpu_features library.")
add_feature_info(Boost_USE_STATIC_LIBS Boost_USE_STATIC_LIBS "Use Boost static libraries.")

message(STATUS "")
message(STATUS "***************************************")
message(STATUS "*           SUMMARY REPORT            *")
message(STATUS "***************************************")
message(STATUS "")
if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross-compiling on ${LINUX_DISTRIBUTION} ${LINUX_VER} (${CMAKE_HOST_SYSTEM_PROCESSOR}) for ${CMAKE_SYSTEM_PROCESSOR}")
else()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
        message(STATUS "Building on GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCHITECTURE_STRING}")
    endif()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        message(STATUS "Building on ${MACOS_DISTRIBUTION}")
    endif()
endif()
message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "The CXX compiler identification is ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}. Standard: C++${CMAKE_CXX_STANDARD}.")
message(STATUS "The C compiler identification is ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}. Standard: C${CMAKE_C_STANDARD}.")
if(CUDA_FOUND)
    message(STATUS "The CUDA compiler identification is ${CMAKE_CUDA_COMPILER_ID} ${CMAKE_CUDA_COMPILER_VERSION}. Standard: C++${CMAKE_CUDA_STANDARD}.")
endif()
message(STATUS "")
file(REMOVE ${GNSSSDR_BINARY_DIR}/features.log)
file(WRITE ${GNSSSDR_BINARY_DIR}/features.log "**********************************\n")
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "* BUILDING CONFIGURATION SUMMARY *\n")
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "**********************************\n\n")
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "GNSS-SDR version: ${VERSION}\n")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux|kFreeBSD|GNU")
    if(CMAKE_CROSSCOMPILING)
        file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "Cross-compiling on ${LINUX_DISTRIBUTION} ${LINUX_VER} (${CMAKE_HOST_SYSTEM_PROCESSOR}) for ${CMAKE_SYSTEM_PROCESSOR}\n")
    else()
        file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "Building on GNU/Linux ${LINUX_DISTRIBUTION} ${LINUX_VER} ${ARCHITECTURE_STRING}\n")
    endif()
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "Building on ${MACOS_DISTRIBUTION}\n")
endif()
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "CMake version: ${CMAKE_VERSION}\n")
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "The CXX compiler identification is ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}. Standard: C++${CMAKE_CXX_STANDARD}.\n")
file(APPEND ${GNSSSDR_BINARY_DIR}/features.log "The C compiler identification is ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}. Standard: C${CMAKE_C_STANDARD}.\n\n")
if(CMAKE_VERSION VERSION_LESS 3.4)
    feature_summary(WHAT ALL)
    feature_summary(FILENAME ${GNSSSDR_BINARY_DIR}/features.log APPEND WHAT ALL)
else()
    feature_summary(WHAT
        REQUIRED_PACKAGES_FOUND
        REQUIRED_PACKAGES_NOT_FOUND
        OPTIONAL_PACKAGES_FOUND
        OPTIONAL_PACKAGES_NOT_FOUND
        ENABLED_FEATURES
        DISABLED_FEATURES
    )
    feature_summary(FILENAME ${GNSSSDR_BINARY_DIR}/features.log APPEND WHAT
        REQUIRED_PACKAGES_FOUND
        REQUIRED_PACKAGES_NOT_FOUND
        OPTIONAL_PACKAGES_FOUND
        OPTIONAL_PACKAGES_NOT_FOUND
        ENABLED_FEATURES
        DISABLED_FEATURES
    )
endif()
message(STATUS "GNSS-SDR v${VERSION} is ready to be built.")

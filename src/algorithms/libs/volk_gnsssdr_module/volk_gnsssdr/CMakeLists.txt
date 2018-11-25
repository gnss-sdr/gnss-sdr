#
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
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose build type: None Debug Release RelWithDebInfo MinSizeRel")
project(volk_gnsssdr)
enable_language(CXX)
enable_language(C)
enable_testing()

# Set compiler flags
set(GNSSSDR_CLANG_MIN_VERSION "3.4.0")
set(GNSSSDR_APPLECLANG_MIN_VERSION "500")
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
        else()
            set(MIN_VERSION ${GNSSSDR_APPLECLANG_MIN_VERSION})
            set(APPLE_STR "Apple ")
            # retrieve the compiler's version from it
            string(REGEX MATCH "(clang-[0-9.]+)" CLANG_APPLE_VERSION ${_err})
            string(REGEX MATCH "[0-9.]+" CLANG_VERSION ${CLANG_APPLE_VERSION})
        endif()
        if(${CLANG_VERSION} VERSION_LESS "${MIN_VERSION}")
            message(WARNING "\nThe compiler selected to build VOLK-GNSSSDR (${APPLE_STR}Clang version ${CLANG_VERSION} : ${CMAKE_CXX_COMPILER}) is older than that officially supported (${MIN_VERSION} minimum). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
        endif()
    else()
        message(WARNING "\nCannot determine the version of the compiler selected to build VOLK-GNSSSDR (${APPLE_STR}Clang : ${CMAKE_CXX_COMPILER}). This build may or not work. We highly recommend using Apple Clang version ${APPLECLANG_MIN_VERSION} or more recent, or Clang version ${CLANG_MIN_VERSION} or more recent.")
    endif()
endif()

# Enable C++17 support in GCC >= 8.0.0
# Enable C++14 support in 8.0.0 > GCC >= 6.1.1
# Fallback to C++11 when using GCC < 6.1.1

if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.1")
        set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
    else()
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
        else()
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++17")
        endif()
    endif()
    set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wall -Wextra")  #Add warning flags: For "-Wall" see http://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
endif()

# Enable C++17 support in Clang >= 6.0.0
# Enable C++14 support in 6.0.0 > Clang >= 3.5.0 or AppleClang >= 600
# Fallback to C++11 if older version
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
        # See https://trac.macports.org/wiki/XcodeVersionInfo for Apple Clang version equivalences
        if(CLANG_VERSION VERSION_LESS "600")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
        else()
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
        endif()
    else()
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.5.0")
            set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11")
        else()
            if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0.0")
                set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++14")
            else()
                set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++17")
            endif()
        endif()
    endif()
endif()

if(NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND NOT WIN32) AND NOT (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    if(NOT (CMAKE_VERSION VERSION_LESS "3.1"))
        set(CMAKE_C_STANDARD 11)
        set(CMAKE_CXX_STANDARD 11)
    endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

if(CMAKE_VERSION VERSION_GREATER "3.0")
    cmake_policy(SET CMP0042 NEW)
    if(CMAKE_VERSION VERSION_GREATER "3.9")
        cmake_policy(SET CMP0068 NEW)
    endif()
endif()

option(ENABLE_STRIP "Create a stripped volk_gnsssdr_profile binary (without shared libraries)" OFF)

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) #allows this to be a sub-project
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}) #allows this to be a sub-project
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules) #location for custom "Modules"

include(VolkBuildTypes)
#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif()
volk_check_build_type(${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}.")

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_MINOR_VERSION 0)
set(VERSION_INFO_MAINT_VERSION 9.git)
include(VolkVersion) #setup version info




########################################################################
# Environment setup
########################################################################
if(NOT DEFINED BOOST_ROOT AND NOT DEFINED ENV{BOOST_ROOT})
    set(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
endif()

if(NOT DEFINED CROSSCOMPILE_MULTILIB)
    set(CROSSCOMPILE_MULTILIB "")
endif()
set(CROSSCOMPILE_MULTILIB ${CROSSCOMPILE_MULTILIB} CACHE STRING "Define \"true\" if you have and want to use multiple C development libs installed for cross compile")

if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES) #enables math constants on all supported versions of MSVC
    add_compile_options(/W1) #reduce warnings
    add_compile_options(/wo4309)
    add_compile_options(/wd4752)
    add_compile_options(/wo4273)
    add_compile_options(/wo4838)
endif()

# allow 'large' files in 32 bit builds
if(UNIX)
    add_definitions(-D_LARGEFILE_SOURCE
        -D_FILE_OFFSET_BITS=64
        -D_LARGE_FILES
        -D_FORTIFY_SOURCE=2
    )
endif()


########################################################################
# Dependencies setup
########################################################################

# Python
include(VolkPython) #sets PYTHON_EXECUTABLE and PYTHON_DASH_B
volk_python_check_module("python >= 2.7" sys "sys.version.split()[0] >= '2.7'" PYTHON_MIN_VER_FOUND)
volk_python_check_module("mako >= 0.4.2" mako "mako.__version__ >= '0.4.2'" MAKO_FOUND)
volk_python_check_module("six - python 2 and 3 compatibility library" six "True" SIX_FOUND)


if(NOT PYTHON_MIN_VER_FOUND)
    message(FATAL_ERROR "Python 2.7 or greater required to build VOLK_GNSSSDR")
endif()

#  Mako
if(NOT MAKO_FOUND)
    message(FATAL_ERROR "Mako templates required to build VOLK_GNSSSDR")
endif()
# Six
if(NOT SIX_FOUND)
    message(FATAL_ERROR "six - python 2 and 3 compatibility library required to build VOLK")
endif()


# Boost
if(MSVC)
    if(NOT DEFINED BOOST_ALL_DYN_LINK)
        set(BOOST_ALL_DYN_LINK TRUE)
    endif()
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else()
        unset(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif()
endif()

include(VolkBoost)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "VOLK_GNSSSDR Requires boost to build")
endif()

# Orc
option(ENABLE_ORC "Enable Orc" True)
if(ENABLE_ORC)
  find_package(ORC)
else()
  message(STATUS "Disabling use of ORC")
endif()


########################################################################
# Setup doxygen
########################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(
        ${PROJECT_SOURCE_DIR}/Doxyfile.in
        ${PROJECT_BINARY_DIR}/Doxyfile
    @ONLY)

    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Generating documentation with Doxygen" VERBATIM
    )
endif()


########################################################################
# Setup the package config file
########################################################################
#set variables found in the pc.in file
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
set(libdir "\${exec_prefix}/lib${LIB_SUFFIX}")
set(includedir "\${prefix}/include")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/volk_gnsssdr.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr.pc
    @ONLY
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/volk_gnsssdr.pc
    DESTINATION lib${LIB_SUFFIX}/pkgconfig
    COMPONENT "volk_gnsssdr_devel"
)

########################################################################
# Install all headers in the include directories
########################################################################
set(VOLK_RUNTIME_DIR   bin)
set(VOLK_LIBRARY_DIR   lib${LIB_SUFFIX})
set(VOLK_INCLUDE_DIR   include)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/kernels/volk_gnsssdr
    DESTINATION include COMPONENT "volk_gnsssdr_devel"
    FILES_MATCHING PATTERN "*.h"
)

install(FILES
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_prefs.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_complex.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_common.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/saturation_arithmetic.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sse_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_neon_intrinsics.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_cpu.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_config_fixed.h
    ${PROJECT_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_typedefs.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_malloc.h
    ${PROJECT_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_sine_table.h
    DESTINATION include/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)


########################################################################
# On Apple only, set install name and use rpath correctly, if not already set
########################################################################
if(APPLE)
    if(NOT CMAKE_INSTALL_NAME_DIR)
        set(CMAKE_INSTALL_NAME_DIR
            ${CMAKE_INSTALL_PREFIX}/${VOLK_LIBRARY_DIR} CACHE
            PATH "Library Install Name Destination Directory" FORCE)
    endif()
    if(NOT CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH
            ${CMAKE_INSTALL_PREFIX}/${VOLK_LIBRARY_DIR} CACHE
            PATH "Library Install RPath" FORCE)
    endif()
    if(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
            BOOL "Do Build Using Library Install RPath" FORCE)
    endif()
endif()


########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

# Only add the target if there isn't one defined already
if(NOT TARGET uninstall)
    add_custom_target(uninstall
        ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    )
endif()




########################################################################
# Install our Cmake modules into $prefix/lib/cmake/volk_gnsssdr
# See "Package Configuration Files" on page:
#    http://www.cmake.org/Wiki/CMake/Tutorials/Packaging
########################################################################

configure_file(
  ${CMAKE_SOURCE_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake
@ONLY)

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake
@ONLY)


########################################################################
# Install cmake search routine for external use
########################################################################

if(NOT CMAKE_MODULES_DIR)
    set(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
endif()

install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/Modules/VolkGnsssdrConfigVersion.cmake
    DESTINATION ${CMAKE_MODULES_DIR}/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)

########################################################################
# Option to enable QA testing, on by default
########################################################################
option(ENABLE_TESTING "Enable QA testing" ON)
if(ENABLE_TESTING)
  message(STATUS "QA Testing is enabled.")
else()
  message(STATUS "QA Testing is disabled.")
endif()
message(STATUS "  Modify using: -DENABLE_TESTING=ON/OFF")


########################################################################
# Option to enable post-build profiling using volk_profile, off by default
########################################################################
option(ENABLE_PROFILING "Launch system profiler after build" OFF)
if(ENABLE_PROFILING)
  set(ENABLE_STATIC_LIBS ON)
  if(DEFINED VOLK_CONFIGPATH)
    get_filename_component(VOLK_CONFIGPATH ${VOLK_CONFIGPATH} ABSOLUTE)
    set(VOLK_CONFIGPATH "${VOLK_CONFIGPATH}/volk_gnsssdr")
    message(STATUS "System profiling is enabled, using path: ${VOLK_CONFIGPATH}")
  elseif(DEFINED ENV{VOLK_CONFIGPATH})
    set(VOLK_CONFIGPATH "$ENV{VOLK_CONFIGPATH}/volk_gnsssdr")
    message(STATUS "System profiling is enabled, using env path: $ENV{VOLK_CONFIGPATH}")
  else()
    message(STATUS "System profiling is enabled with default paths.")
    if(DEFINED ENV{HOME})
        set(VOLK_CONFIGPATH "$ENV{HOME}/.volk_gnsssdr")
    elseif(DEFINED ENV{APPDATA})
        set(VOLK_CONFIGPATH "$ENV{APPDATA}/.volk_gnsssdr")
    endif()
  endif()
else()
  message(STATUS "System profiling is disabled.")
endif()
message(STATUS "  Modify using: -DENABLE_PROFILING=ON/OFF")
########################################################################

########################################################################
# Setup the library
########################################################################
add_subdirectory(lib)

########################################################################
# And the utility apps
########################################################################
add_subdirectory(apps)
add_subdirectory(python/volk_gnsssdr_modtool)


########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")

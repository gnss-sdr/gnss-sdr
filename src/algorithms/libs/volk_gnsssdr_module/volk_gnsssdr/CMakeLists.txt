#
# Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.6)
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose build type: None Debug Release RelWithDebInfo MinSizeRel")
project(volk_gnsssdr)
enable_language(CXX)
enable_language(C)
enable_testing()
set(VERSION 0.1)
set(LIBVER 0.0.0)

option(ENABLE_STRIP "Create a stripped volk_gnsssdr_profile binary (without shared libraries)" OFF)

set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) #allows this to be a sub-project
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}) #allows this to be a sub-project
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake) #location for custom "Modules"
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

########################################################################
# Environment setup
########################################################################
IF(NOT DEFINED BOOST_ROOT)
    SET(BOOST_ROOT ${CMAKE_INSTALL_PREFIX})
ENDIF()

IF(NOT DEFINED CROSSCOMPILE_MULTILIB)
    SET(CROSSCOMPILE_MULTILIB "")
ENDIF()
SET(CROSSCOMPILE_MULTILIB ${CROSSCOMPILE_MULTILIB} CACHE STRING "Define \"true\" if you have and want to use multiple C development libs installed for cross compile")


########################################################################
# Dependencies setup
########################################################################
include(GrPython) #sets PYTHON_EXECUTABLE and PYTHON_DASH_B
VOLK_PYTHON_CHECK_MODULE("python >= 2.5" sys "sys.version.split()[0] >= '2.5'" PYTHON_MIN_VER_FOUND)
VOLK_PYTHON_CHECK_MODULE("Cheetah >= 2.0.0" Cheetah "Cheetah.Version >= '2.0.0'" CHEETAH_FOUND)

if(NOT PYTHON_MIN_VER_FOUND)
    message(FATAL_ERROR "Python 2.5 or greater required to build VOLK_GNSSSDR")
endif()

if(NOT CHEETAH_FOUND)
    message(FATAL_ERROR "Cheetah templates required to build VOLK_GNSSSDR")
endif()

if(MSVC)
    if (NOT DEFINED BOOST_ALL_DYN_LINK)
        set(BOOST_ALL_DYN_LINK TRUE)
    endif()
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else(BOOST_ALL_DYN_LINK)
        unset(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif(BOOST_ALL_DYN_LINK)
endif(MSVC)
include(VolkBoost)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "VOLK_GNSSSDR Requires boost to build")
endif()

option(ENABLE_ORC "Enable Orc" True)
if(ENABLE_ORC)
  find_package(ORC)
else(ENABLE_ORC)
  message(STATUS "Disabling use of ORC")
endif(ENABLE_ORC)

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
@ONLY)

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
    DIRECTORY ${CMAKE_SOURCE_DIR}/kernels/volk_gnsssdr
    DESTINATION include COMPONENT "volk_gnsssdr_devel"
    FILES_MATCHING PATTERN "*.h"
)

install(FILES
    ${CMAKE_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_prefs.h
    ${CMAKE_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_complex.h
    ${CMAKE_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_common.h
    ${CMAKE_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr.h
    ${CMAKE_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_cpu.h
    ${CMAKE_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_config_fixed.h
    ${CMAKE_BINARY_DIR}/include/volk_gnsssdr/volk_gnsssdr_typedefs.h
    ${CMAKE_SOURCE_DIR}/include/volk_gnsssdr/volk_gnsssdr_malloc.h
    DESTINATION include/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)

########################################################################
# Install cmake search routine for external use
########################################################################

if(NOT CMAKE_MODULES_DIR)
    set(CMAKE_MODULES_DIR lib${LIB_SUFFIX}/cmake)
endif(NOT CMAKE_MODULES_DIR)

install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/VolkConfig.cmake
    DESTINATION ${CMAKE_MODULES_DIR}/volk_gnsssdr
    COMPONENT "volk_gnsssdr_devel"
)

########################################################################
# On Apple only, set install name and use rpath correctly, if not already set
########################################################################
if(APPLE)
    if(NOT CMAKE_INSTALL_NAME_DIR)
        set(CMAKE_INSTALL_NAME_DIR
            ${CMAKE_INSTALL_PREFIX}/lib CACHE
            PATH "Library Install Name Destination Directory" FORCE)
    endif(NOT CMAKE_INSTALL_NAME_DIR)
    if(NOT CMAKE_INSTALL_RPATH)
        set(CMAKE_INSTALL_RPATH
            ${CMAKE_INSTALL_PREFIX}/lib CACHE
            PATH "Library Install RPath" FORCE)
    endif(NOT CMAKE_INSTALL_RPATH)
    if(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
        set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE
            BOOL "Do Build Using Library Install RPath" FORCE)
    endif(NOT CMAKE_BUILD_WITH_INSTALL_RPATH)
endif(APPLE)

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
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")

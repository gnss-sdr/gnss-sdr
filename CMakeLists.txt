# Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
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
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Prevented in-tree build. This is bad practice.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8)
project(gnss-sdr CXX)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH ${gnss-sdr_project_SOURCE_DIR}/cmake/Modules)

# Set the version information here
set(VERSION_INFO_MAJOR_VERSION 0)
#set(VERSION_INFO_API_COMPAT    1)
set(VERSION_INFO_MINOR_VERSION 1)


# Append -O2 optimization flag for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O2")
#SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O2")


########################################################################
# Environment setup
########################################################################


################################################################################
# Googletest - http://code.google.com/p/googletest/
################################################################################
set(GTEST_DIR $ENV{GTEST_DIR})
if(GTEST_DIR)
     message("GTEST root folder at ${GTEST_DIR}")
else()
     message (" Googletest has not been found because the variable ")
     message (" GTEST_DIR is not defined.")
     message (" Googletest will be downloaded and built automatically ")
     message (" when doing 'make'. ")
endif(GTEST_DIR)


################################################################################
# Boost - http://www.boost.org
################################################################################

if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND EXISTS "/usr/lib64")
set(Boost_ADDITIONAL_VERSIONS
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
)
find_package(Boost "1.45")

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost required to compile gnss-sdr")
endif()
set(Boost_USE_MULTITHREAD ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS date_time filesystem system thread REQUIRED) 

#if ( NOT BOOST_DATE_TIME_FOUND )
#     message(FATAL_ERROR "Boost date_time library not found " )
#endif()


################################################################################
# glog - http://code.google.com/p/google-glog/
################################################################################

find_package(GLOG REQUIRED)
if ( NOT GLOG_FOUND )
  message(FATAL_ERROR "google-glog library not found! Please check http://code.google.com/p/google-glog/")
else( NOT GLOG_FOUND )
  message ( "google-glog library found." )
endif( NOT GLOG_FOUND )
include_directories(${GLOG_INCLUDE_DIRS})

################################################################################
# gflags - http://code.google.com/p/gflags/
################################################################################

find_package(GFlags REQUIRED)
if ( NOT GFlags_FOUND )
  message(FATAL_ERROR "gflags library not found! Please check http://code.google.com/p/gflags/")
else( NOT GFlags_FOUND )
  message ( "gflags library found." )
endif( NOT GFlags_FOUND )
include_directories(${GFlags_INCLUDE_DIRS})

################################################################################
# GPerftools - http://code.google.com/p/gperftools/
################################################################################

set(GCC_GPERFTOOLS_FLAGS "")
find_package(Gperftools)
if ( NOT GPERFTOOLS_FOUND )
  message("The optional library GPerftools has not been found.")
else()
  message ( "GPerftools library found." )
  link_libraries(profiler tcmalloc)
endif( NOT GPERFTOOLS_FOUND )
list(APPEND CMAKE_CXX_FLAGS ${GCC_GPERFTOOLS_FLAGS})




################################################################################
# Doxygen - http://www.stack.nl/~dimitri/doxygen/index.html
################################################################################

find_package(Doxygen)
if(DOXYGEN_FOUND)
    message( "Doxygen found. You can build the documentation with 'make doc'." )
    configure_file(${CMAKE_SOURCE_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
         ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
         COMMENT "Generating API documentation with Doxygen." VERBATIM
    )
ENDIF(DOXYGEN_FOUND)



################################################################################
# GNURadio - http://gnuradio.org/redmine/projects/gnuradio/wiki
################################################################################

find_package(Gruel)
if(NOT GRUEL_FOUND)
    message(FATAL_ERROR "Gruel required to build gnss-sdr")
endif()


find_package(GnuradioCore)
if(NOT GNURADIO_CORE_FOUND)
    message(FATAL_ERROR "GnuRadio Core required to build gnss-sdr")
endif()


find_package(UHD)
if(NOT UHD_FOUND)
    message(FATAL_ERROR "UHD required to build gnss-sdr")
endif()

find_package(GnuradioUHD)
if(NOT GNURADIO_UHD_FOUND)
    message(FATAL_ERROR "Gnuradio UHD required to build gnss-sdr")
endif()

find_package(Volk)
if(NOT VOLK_FOUND)
    message(FATAL_ERROR "Volk required to build gnss-sdr")
endif()


################################################################################
# Armadillo - http://arma.sourceforge.net/
################################################################################

find_package(Armadillo)
if(NOT ARMADILLO_FOUND)
    message(FATAL_ERROR "Armadillo required to build gnss-sdr. Please check http://arma.sourceforge.net/")
endif()


################################################################################
# Setup of optional drivers
################################################################################
if( $ENV{GN3S_DRIVER} )
    message( "GN3S_DRIVER variable found." )
    # copy firmware to install folder
    # Build project gr-gn3s
else( $ENV{GN3S_DRIVER} )
    message( "GN3S_DRIVER is not defined." )
    message( "Define it with 'export GN3S_DRIVER=1' to add support for the GN3S dongle." )
endif($ENV{GN3S_DRIVER} )

if( $ENV{RTLSDR_DRIVER} )
    message( "RTLSDR_DRIVER variable found." )
    # find libosmosdr
    # find gr-osmosdr
endif($ENV{RTLSDR_DRIVER} )


########################################################################
# Setup the include paths
########################################################################
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${GRUEL_INCLUDE_DIRS}
    ${GNURADIO_CORE_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${GPERFTOOLS_INCLUDE_DIRS}
)


########################################################################
# Set compiler flags
########################################################################

# Add compiler flags related to SSE instructions
set(MY_CXX_FLAGS "-msse2 -msse3")
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -mfpmath=sse")
     # For SSE related flags, see http://gcc.gnu.org/onlinedocs/gcc-4.6.3/gcc/i386-and-x86_002d64-Options.html
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)

# Enable C++11 support in GCC
# For "-std=c++0x"  GCC's support for C++11 see http://gcc.gnu.org/projects/cxx0x.html
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)

# Add warning flags 
# For "-Wall" see http://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wall")


if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
    #http://gcc.gnu.org/wiki/Visibility
    add_definitions(-fvisibility=hidden)
endif()

# Set GPerftools related flags if it is available
# See http://gperftools.googlecode.com/svn/trunk/README
if(GPERFTOOLS_FOUND)
     if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
          set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")       
     endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
endif(GPERFTOOLS_FOUND)

list(APPEND CMAKE_CXX_FLAGS ${MY_CXX_FLAGS})

########################################################################
# Add subdirectories (in order of deps)
########################################################################
add_subdirectory(src)

#add_subdirectory(drivers)
#add_subdirectory(firmware)

 


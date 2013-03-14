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
    message(FATAL_ERROR "Prevented in-tree build. This is bad practice. Try 'cd build && cmake ../' ")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8)
project(gnss-sdr CXX C)

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
set(VERSION_INFO_API_COMPAT    0)
set(VERSION_INFO_MINOR_VERSION 1)
set(VERSION ${VERSION_INFO_MAJOR_VERSION}.${VERSION_INFO_API_COMPAT}.${VERSION_INFO_MINOR_VERSION})

# Append -O2 optimization flag for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O2")

option(INSTALL_GNURADIO "Install GNU Radio and all its dependencies" OFF)

file(RELATIVE_PATH RELATIVE_CMAKE_CALL ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

########################################################################
# Environment setup
########################################################################
include(ExternalProject)

set(OS_IS_MACOSX "")
set(OS_IS_LINUX "")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     set(OperatingSystem "Linux")
     set(OS_IS_LINUX TRUE)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(OperatingSystem "Mac OS X")
     set(OS_IS_MACOSX TRUE)
     set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
     set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")



################################################################################
# Googletest - http://code.google.com/p/googletest/
################################################################################
enable_testing()
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
# GNU Radio - http://gnuradio.org/redmine/projects/gnuradio/wiki
################################################################################

if(INSTALL_GNURADIO)
     if(OperatingSystem MATCHES "Linux")
          message("Trying to install GNU Radio and all its dependencies.")
          message("This process can take a long time.")
          set(GNURADIO_RELEASE "3.6.3")
          set(UHD_RELEASE "003_005_000")
          set(INSTALL_GNURADIO CACHE BOOL OFF FORCE)
          file(DOWNLOAD http://www.sbrac.org/files/build-gnuradio
               ${CMAKE_CURRENT_BINARY_DIR}/download/build-gnuradio
          )
          file(COPY ${CMAKE_CURRENT_BINARY_DIR}/download/build-gnuradio
               DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
               FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
          )
          execute_process(
               COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-gnuradio -l ${CMAKE_CURRENT_BINARY_DIR}/build-gnuradio.log
               WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
          )
          #  The flags -gt v${GNURADIO_RELEASE} -ut release_${UHD_RELEASE} do not work :-(     
     else(OperatingSystem MATCHES "Linux")
          message( "The script http://www.sbrac.org/files/build-gnuradio does not work in your operating system.")
          message( "Please visit http://gnuradio.org and find out how to install GNU Radio.")
          message( "Once installed, run 'cmake ${RELATIVE_CMAKE_CALL}' again.")
     endif(OperatingSystem MATCHES "Linux")
endif(INSTALL_GNURADIO)

find_package(GnuradioCore)
if(NOT GNURADIO_CORE_FOUND)
     if(OperatingSystem MATCHES "Linux")       
          message( "If you are using Ubuntu, Fedora, RedHat, Debian or Mint")
          message( "use 'cmake ${RELATIVE_CMAKE_CALL} -DINSTALL_GNURADIO=ON' to install GNU Radio and all its dependencies." )
     endif(OperatingSystem MATCHES "Linux")
     message(FATAL_ERROR "Please install GNU Radio and all its dependencies. Visit http://gnuradio.org")
endif(NOT GNURADIO_CORE_FOUND)

find_library(
     GNURADIO_CORE_LIBRARIES
     NAMES gnuradio-core
     PATHS usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

find_library(
     GR_FILTER_LIBRARIES
     NAMES gnuradio-filter
     PATHS usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)
find_library(
     GR_FFT_LIBRARIES
     NAMES gnuradio-fft
     PATHS usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)
find_library(
     GR_BLOCKS_LIBRARIES
     NAMES gnuradio-blocks
     PATHS usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)  

find_package(Gruel)
if(NOT GRUEL_FOUND)
    message(FATAL_ERROR "Gruel required to build gnss-sdr")
endif()
link_libraries(${GRUEL_LIBRARIES})
include_directories(${GRUEL_INCLUDE_DIRS})

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
find_library(
     VOLK_LIBRARIES
     NAMES volk
     PATHS usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

################################################################################
# Boost - http://www.boost.org
################################################################################

if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND EXISTS "/usr/lib64")
set(Boost_ADDITIONAL_VERSIONS
    "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46"                 "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51"                 "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
)
set(Boost_USE_MULTITHREAD ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS date_time system filesystem thread REQUIRED) 
if(NOT Boost_FOUND)
     message(FATAL_ERROR "Fatal error: Boost (version >=1.42.0) required.")
endif(NOT Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})





################################################################################
# gflags - http://code.google.com/p/gflags/
################################################################################

find_package(GFlags)

if (NOT GFlags_FOUND OR LOCAL_GFLAGS)
     message (" gflags library has not been found.")
     message (" gflags will be downloaded and built automatically ")
     message (" when doing 'make'. ")
     set(gflags_RELEASE 2.0 CACHE STRING "Local gflags release")
     set(gflags_MD5 "5fd4554d2ba84bf222a2fec0870d28df")
     ExternalProject_Add(
          gflags-${gflags_RELEASE}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
          URL http://gflags.googlecode.com/files/gflags-${gflags_RELEASE}.zip
          DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/gflags-${gflags_RELEASE}
          URL_MD5 ${gflags_MD5}
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${gflags_RELEASE}
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
          CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
          BUILD_COMMAND make
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )

     set(GFlags_INCLUDE_DIRS 
          ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/src CACHE PATH "Local Gflags headers"      
     )
     set(GFlags_LIBS 
          ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}gflags.a
     )
     file(GLOB GFlags_SHARED_LIBS "${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_SHARED_LIBRARY_SUFFIX}*")
     set(GFlags_LIBRARY gflags-${gflags_RELEASE})
     set(gflags gflags-${gflags_RELEASE})
     set(GFlags_LIBRARY_PATH  ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs )
     link_directories(${GFlags_LIBRARY_PATH})
     set(GFlags_lib ${GFlags_LIBS}  CACHE FILEPATH "Local Gflags library")
     set(GFlags_LIBRARY_PATH ${GFlags_LIBS})
     set(LOCAL_GFLAGS true CACHE STRING "GFlags downloaded and build automatically")
endif(NOT GFlags_FOUND OR LOCAL_GFLAGS)

include_directories(${GFlags_INCLUDE_DIRS})



################################################################################
# glog - http://code.google.com/p/google-glog/
################################################################################

find_package(GLOG)
if (NOT GLOG_FOUND OR LOCAL_GFLAGS)
     message (" glog library has not been found")
     if(LOCAL_GFLAGS)
          message(" or it is likely not linked to gflags.")
     endif(LOCAL_GFLAGS)
     message (" glog will be downloaded and built automatically ")
     message (" when doing 'make'. ")
     set(glog_RELEASE 0.3.3)
     set(glog_MD5 "a6fd2c22f8996846e34c763422717c18")
     if(LOCAL_GFLAGS) 
          file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
"#!/bin/sh
export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
export LDFLAGS=-L${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs
export LIBS=${GFlags_SHARED_LIBS}
../../thirdparty/glog/glog-${glog_RELEASE}/configure")
          file(COPY ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
               DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
               FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                                GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
          set(GLOG_CONFIGURE ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/configure_with_gflags)                      
     else(LOCAL_GFLAGS)
          set(GLOG_CONFIGURE ../../thirdparty/glog/glog-${glog_RELEASE}/configure)
     endif(LOCAL_GFLAGS)
    
     ExternalProject_Add(
          glog-${glog_RELEASE}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
          URL http://google-glog.googlecode.com/files/glog-${glog_RELEASE}.tar.gz
          DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/glog-${glog_RELEASE}
          URL_MD5 ${glog_MD5}
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${glog_RELEASE}
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
          CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR>
          BUILD_COMMAND make
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )
     # Set up variables
     set(GLOG_INCLUDE_DIRS 
          ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${glog_RELEASE}/src/ 
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/src
     )
     set(GLOG_LIBRARIES 
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog.a 
     )
endif(NOT GLOG_FOUND OR LOCAL_GFLAGS)
include_directories(${GLOG_INCLUDE_DIRS})



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
     message( "Doxygen found.")
     message(STATUS "You can build the documentation with 'make doc'." )
     message(STATUS "When done, point your browser to ${CMAKE_SOURCE_DIR}/html/index.html")
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
          ${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile
          @ONLY
     )
     add_custom_target(doc
          ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
          COMMENT "Generating API documentation with Doxygen." VERBATIM
     )
     if(LATEX_COMPILER)
          message(STATUS "'make pdfmanual' will generate a manual at ${CMAKE_SOURCE_DIR}/docs/GNSS-SDR_manual.pdf")         
          add_custom_target(pdfmanual
               COMMAND ${CMAKE_MAKE_PROGRAM}
               COMMAND ${CMAKE_COMMAND} -E copy refman.pdf ${CMAKE_SOURCE_DIR}/docs/GNSS-SDR_manual.pdf
               COMMAND ${CMAKE_MAKE_PROGRAM} clean
               DEPENDS doc
               WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/docs/latex
               COMMENT "Generating PDF manual with Doxygen." VERBATIM
          )
     endif(LATEX_COMPILER)
     message(STATUS "'make doc-clean' will clean the documentation.")
     add_custom_target(doc-clean
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/docs/html
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/docs/latex
          COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_SOURCE_DIR}/docs/GNSS-SDR_manual.pdf
          COMMENT "Cleaning documentation." VERBATIM
     )
else(DOXYGEN_FOUND)
     message( "Doxygen has not been found in your system.")
     message( "You can get nice code documentation by using it!")
     message( "Get it from http://www.stack.nl/~dimitri/doxygen/index.html")
endif(DOXYGEN_FOUND)



################################################################################
# Armadillo - http://arma.sourceforge.net/
################################################################################

find_package(Armadillo)
if(NOT ARMADILLO_FOUND)
     message (" Armadillo has not been found.")
     message (" Armadillo will be downloaded and built automatically ")
     message (" when doing 'make'. ")
     set(armadillo_RELEASE 3.6.2)
     set(armadillo_MD5 "0ccb90f5f40d936a59785eff10c362ad")
     if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
          set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_no.patch)
          file(WRITE ${ARMADILLO_PATCH_FILE} "")
     else(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
          set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_staticlib.patch)
          file(WRITE ${ARMADILLO_PATCH_FILE}
"43c43
< set(ARMA_USE_WRAPPER true )  # set this to false if you prefer to directly link with LAPACK and/or BLAS (eg. -llapack -lblas) instead of -larmadillo
---
> set(ARMA_USE_WRAPPER false)  # set this to false if you prefer to directly link with LAPACK and/or BLAS (eg. -llapack -lblas) instead of -larmadillo
274c274
< add_library( armadillo SHARED src/wrap_libs )
---
> add_library( armadillo STATIC src/wrap_libs )
")                  
     endif(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
     ExternalProject_Add(
          armadillo-${armadillo_RELEASE}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
          URL http://sourceforge.net/projects/arma/files/armadillo-${armadillo_RELEASE}.tar.gz
          DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}
          URL_MD5 ${armadillo_MD5}    
          PATCH_COMMAND patch -N <BINARY_DIR>/CMakeLists.txt ${ARMADILLO_PATCH_FILE} 
          CONFIGURE_COMMAND <BINARY_DIR>/configure --prefix=${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}     
          BUILD_IN_SOURCE 1
          BUILD_COMMAND make
          UPDATE_COMMAND ""
          INSTALL_COMMAND ""
     )
     # Set up variables
     ExternalProject_Get_Property(armadillo-${armadillo_RELEASE} binary_dir)
     set(ARMADILLO_INCLUDE_DIRS ${binary_dir}/include )     
     find_library(lapack NAMES lapack HINTS /usr/lib /usr/local/lib /usr/lib64)
     find_library(blas NAMES blas HINTS /usr/lib /usr/local/lib /usr/lib64)
     set(ARMADILLO_LIBRARIES ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo.a ${lapack} ${blas} gfortran)
     set(LOCAL_ARMADILLO true)
     # Save a copy at the thirdparty folder
     file(COPY ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
          DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo
     )
endif(NOT ARMADILLO_FOUND)


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
    ${GNURADIO_CORE_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${GPERFTOOLS_INCLUDE_DIRS}
)


########################################################################
# Set compiler flags
########################################################################

# Enable C++11 support in GCC
# For "-std=c++0x"  GCC's support for C++11 see http://gcc.gnu.org/projects/cxx0x.html
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)

# Add warning flags 
# For "-Wall" see http://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wall")

# Processor-architecture related flags
# See http://gcc.gnu.org/onlinedocs/gcc/i386-and-x86_002d64-Options.html#i386-and-x86_002d64-Options
# Change to '-mtune=generic' when building for distribution
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -march=native -mfpmath=sse")
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)

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

# Detect 64-bits machine
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
     #set(MY_CXX_FLAGS "${MY_CXX_FLAGS}  ...")
     # link_libraries(pthread) # Needed by Fedora 18
     message("64-bits architecture detected.") 
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

list(APPEND CMAKE_CXX_FLAGS ${MY_CXX_FLAGS})

#add_definitions( -DGNSS_SDR_VERSION="${VERSION}" )

########################################################################
# Add subdirectories (in order of deps)
########################################################################
add_subdirectory(src)

 

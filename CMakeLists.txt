# Copyright (C) 2010-2014 (see AUTHORS file for a list of contributors)
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

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# Set the version information here
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

set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_API_COMPAT    0)
set(VERSION_INFO_MINOR_VERSION 2.git-${GIT_BRANCH}-${GIT_COMMIT_HASH})
set(VERSION ${VERSION_INFO_MAJOR_VERSION}.${VERSION_INFO_API_COMPAT}.${VERSION_INFO_MINOR_VERSION})

file(RELATIVE_PATH RELATIVE_CMAKE_CALL ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

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
     message(STATUS "Configuring GNSS-SDR to be built on ${LINUX_DISTRIBUTION} GNU/Linux Release ${LINUX_VER} ${ARCH_}")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# Detect Mac OS X Version
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(OperatingSystem "Mac OS X")
     set(OS_IS_MACOSX TRUE)
     exec_program(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
     string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
     if(${DARWIN_VERSION} MATCHES "13")
         set(MACOSX_MAVERICKS TRUE)
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
         set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
         set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION="com.apple.compilers.llvm.clang.1_0")
         message(STATUS "Configuring GNSS-SDR to be built on Mac OS X 10.9 Mavericks")
     endif(${DARWIN_VERSION} MATCHES "13")
     if(${DARWIN_VERSION} MATCHES "12")
         message(STATUS "Configuring GNSS-SDR to be built on Mac OS X 10.8 Mountain Lion")
     endif(${DARWIN_VERSION} MATCHES "12")
     if(${DARWIN_VERSION} MATCHES "11")
         message(STATUS "Configuring GNSS-SDR to be built on Mac OS X 10.7 Lion")
     endif(${DARWIN_VERSION} MATCHES "11")
     if(${DARWIN_VERSION} MATCHES "10")
         message(STATUS "Configuring GNSS-SDR to be built on Mac OS X 10.6 Snow Leopard")
     endif(${DARWIN_VERSION} MATCHES "10")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to Release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

# Append -O2 optimization flag for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O2")

################################################################################
# Checkout compiler version
################################################################################

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
      if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.7)
          message(WARNING "Your GCC version is too old and could not support some C++11 features required by GNSS-SDR. GCC version must be at least 4.7")
      endif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.7)
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

################################################################################
# Googletest - http://code.google.com/p/googletest/
################################################################################
enable_testing()
if($ENV{GTEST_DIR})
     set(GTEST_DIR $ENV{GTEST_DIR})
endif($ENV{GTEST_DIR})
if(GTEST_DIR)
     message(STATUS "GTEST root folder at ${GTEST_DIR}")
else()
     message (STATUS " Googletest has not been found because the environment variable ")
     message (STATUS " GTEST_DIR is not defined.")
     message (STATUS " Googletest will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")
endif(GTEST_DIR)



################################################################################
# Boost - http://www.boost.org
################################################################################

if(UNIX AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") # Fedora 64-bit fix
endif(UNIX AND EXISTS "/usr/lib64")
if(GNURADIO_INSTALL_PREFIX)
    if(EXISTS "${GNURADIO_INSTALL_PREFIX}/lib/libboost_thread-mt.so")
        list(APPEND BOOST_LIBRARYDIR "${GNURADIO_INSTALL_PREFIX}/lib")     # Boost installed by PyBOMBS
        list(APPEND BOOST_INCLUDEDIR "${GNURADIO_INSTALL_PREFIX}/include") # Boost installed by PyBOMBS
    endif(EXISTS "${GNURADIO_INSTALL_PREFIX}/lib/libboost_thread-mt.so")
endif(GNURADIO_INSTALL_PREFIX)
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
find_package(Boost COMPONENTS date_time system filesystem thread serialization REQUIRED) 
if(NOT Boost_FOUND)
     message(FATAL_ERROR "Fatal error: Boost (version >=1.42.0) required.")
endif(NOT Boost_FOUND)


################################################################################
# GNU Radio - http://gnuradio.org/redmine/projects/gnuradio/wiki
################################################################################

find_package(Gnuradio)

if(NOT GNURADIO_RUNTIME_FOUND)
   message(STATUS "CMake cannot find GNU Radio >= 3.7")
   if(OS_IS_LINUX)
       message("Go to http://gnuradio.org/redmine/projects/pybombs/wiki")
       message("and follow the instructions to install GNU Radio in your system.")
       message(FATAL_ERROR "GNU Radio 3.7 or later is required to build gnss-sdr")
   endif(OS_IS_LINUX)
   if(OS_IS_MACOSX)
       message("You can install it easily via Macports.")
       message("Open a terminal and type:")
       message("sudo port install gnuradio ")
       message(FATAL_ERROR "GNU Radio 3.7 or later is required to build gnss-sdr")
   endif(OS_IS_MACOSX)
endif(NOT GNURADIO_RUNTIME_FOUND)

find_package(Volk)
find_package(UHD)

if(NOT GNURADIO_BLOCKS_FOUND)
    message(FATAL_ERROR "*** gnuradio-blocks 3.7 or later is required to build gnss-sdr")
endif()
if(NOT UHD_FOUND)
    message(FATAL_ERROR "*** UHD is required to build gnss-sdr. Please check http://code.ettus.com/redmine/ettus/projects/uhd/wiki")
endif()
if(NOT GNURADIO_UHD_FOUND)
    message(FATAL_ERROR "*** gnuradio-uhd 3.7 or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_FILTER_FOUND)
    message(FATAL_ERROR "*** gnuradio-filter 3.7 or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_FFT_FOUND)
    message(FATAL_ERROR "*** gnuradio-fft 3.7 or later is required to build gnss-sdr")
endif()
if(NOT VOLK_FOUND)
    message(FATAL_ERROR "*** VOLK is required to build gnss-sdr")
endif()
if(NOT GNURADIO_ANALOG_FOUND)
    message(FATAL_ERROR "*** gnuradio-analog 3.7 or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_FEC_FOUND)
    message(FATAL_ERROR "*** gnuradio-fec 3.7 or later is required to build gnss-sdr")
endif()
if(NOT GNURADIO_TRELLIS_FOUND)
    message(FATAL_ERROR "*** gnuradio-trellis 3.7 or later is required to build gnss-sdr")
endif()



################################################################################
# gflags - http://code.google.com/p/gflags/
################################################################################
set(LOCAL_GFLAGS false CACHE STRING "GFlags installed on the system")
set(GFlags_ROOT_DIR false)
find_package(GFlags)

if (NOT GFlags_FOUND OR LOCAL_GLOG)
     message (STATUS " gflags library has not been found.")
     message (STATUS " gflags will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")
     set(gflags_RELEASE 2.0 CACHE STRING "Local gflags release")
     set(gflags_MD5 "5fd4554d2ba84bf222a2fec0870d28df")
     if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
         file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/tmp/configure_osx
"#!/bin/sh
export CXXFLAGS=\"-stdlib=libc++\" 
export CC=clang 
export CXX=clang++
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${gflags_RELEASE}/configure")
         file(COPY ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/tmp/configure_osx
                 DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
                 FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                 GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
          set(CONF_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/configure_osx)
     else("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
          file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/tmp/configure_linux
"#!/bin/sh
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${gflags_RELEASE}/configure")
          file(COPY ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/tmp/configure_linux
                 DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
                 FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                 GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
          set(CONF_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/configure_linux)
     endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
     
     ExternalProject_Add(
          gflags-${gflags_RELEASE}
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
          URL http://gflags.googlecode.com/files/gflags-${gflags_RELEASE}.zip
          DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/gflags-${gflags_RELEASE}
          URL_MD5 ${gflags_MD5}
          SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gflags/gflags-${gflags_RELEASE}
          BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}
          CONFIGURE_COMMAND ${CONF_SCRIPT} --prefix=${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}  
          BUILD_COMMAND make
          UPDATE_COMMAND ""
          PATCH_COMMAND ""
          INSTALL_COMMAND ""
     )

     set(GFlags_INCLUDE_DIRS 
          ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/src CACHE PATH "Local Gflags headers"      
     )
     
     add_library(gflags UNKNOWN IMPORTED)
     set_property(TARGET gflags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}gflags.a)
     add_dependencies(gflags gflags-${gflags_RELEASE})
     set(GFlags_LIBS gflags)
     set(GFlags_ROOT_DIR ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE} CACHE PATH "GFlags root folder" FORCE)
     file(GLOB GFlags_SHARED_LIBS "${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}gflags${CMAKE_SHARED_LIBRARY_SUFFIX}*")
     set(GFlags_LIBRARY gflags-${gflags_RELEASE})
     set(GFlags_LIBRARY_PATH ${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs )
     link_directories(${GFlags_LIBRARY_PATH})
     set(GFlags_lib ${GFlags_LIBS} CACHE FILEPATH "Local Gflags library")
     set(GFlags_LIBRARY_PATH ${GFlags_LIBS})
     set(LOCAL_GFLAGS true CACHE STRING "GFlags downloaded and built automatically" FORCE)
endif(NOT GFlags_FOUND OR LOCAL_GLOG)



################################################################################
# glog - http://code.google.com/p/google-glog/
################################################################################

find_package(GLOG)
if (NOT GLOG_FOUND OR LOCAL_GFLAGS)
     message (STATUS " glog library has not been found")
     if(NOT GFlags_FOUND)
          message(STATUS " or it is likely not linked to gflags.")
     endif(NOT GFlags_FOUND)
     message (STATUS " glog will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")
     set(glog_RELEASE 0.3.3)
     set(glog_MD5 "a6fd2c22f8996846e34c763422717c18")
     if(LOCAL_GFLAGS)
         if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
             file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
"#!/bin/sh
export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
export LDFLAGS=-L${CMAKE_CURRENT_BINARY_DIR}/gflags-${gflags_RELEASE}/.libs
export LIBS=${GFlags_SHARED_LIBS}
export CXXFLAGS=\"-stdlib=libc++\" 
export CC=clang 
export CXX=clang++
../../thirdparty/glog/glog-${glog_RELEASE}/configure")
             file(COPY ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
               DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
               FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                                GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
         else("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
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
         endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")                           
     else(LOCAL_GFLAGS)
     file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
 "#!/bin/sh
 export CPPFLAGS=-I${GFlags_INCLUDE_DIRS}
 export LDFLAGS=-L${GFlags_LIBRARY_DIRS}
 export LIBS=${GFlags_LIBS}
 ../../thirdparty/glog/glog-${glog_RELEASE}/configure")
          file(COPY ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/tmp/configure_with_gflags
               DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
               FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
                                GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
     endif(LOCAL_GFLAGS)

     set(GLOG_CONFIGURE ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/configure_with_gflags)   
     if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
          ExternalProject_Add(
              glog-${glog_RELEASE}
              DEPENDS gflags-${gflags_RELEASE}
              PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
              SVN_REPOSITORY http://google-glog.googlecode.com/svn/trunk
              SVN_REVISION -r "142"
              DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/glog-${glog_RELEASE}
              SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${glog_RELEASE}
              BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}
              CONFIGURE_COMMAND ${GLOG_CONFIGURE} --prefix=<INSTALL_DIR> 
              BUILD_COMMAND make
              UPDATE_COMMAND ""
              PATCH_COMMAND ""
              INSTALL_COMMAND ""
          )
     else("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
          ExternalProject_Add(
              glog-${glog_RELEASE}
              DEPENDS gflags-${gflags_RELEASE}
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
     endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
     # Set up variables
     set(GLOG_INCLUDE_DIRS 
          ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glog/glog-${glog_RELEASE}/src/ 
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/src
     )
     set(GLOG_LIBRARIES 
          ${CMAKE_CURRENT_BINARY_DIR}/glog-${glog_RELEASE}/.libs/${CMAKE_FIND_LIBRARY_PREFIXES}glog.a 
     )
     set(LOCAL_GLOG true CACHE STRING "Glog downloaded and built automatically" FORCE)
endif(NOT GLOG_FOUND OR LOCAL_GFLAGS)




################################################################################
# GPerftools - http://code.google.com/p/gperftools/
################################################################################

set(GCC_GPERFTOOLS_FLAGS "")
find_package(Gperftools)
if ( NOT GPERFTOOLS_FOUND )
    message(STATUS "The optional library GPerftools has not been found.")
else( NOT GPERFTOOLS_FOUND )
    message (STATUS "GPerftools library found." )
    link_libraries(${GPERFTOOLS_PROFILER} ${GPERFTOOLS_TCMALLOC})
endif( NOT GPERFTOOLS_FOUND )
list(APPEND CMAKE_CXX_FLAGS ${GCC_GPERFTOOLS_FLAGS})




################################################################################
# Doxygen - http://www.stack.nl/~dimitri/doxygen/index.html
################################################################################

find_package(Doxygen)
if(DOXYGEN_FOUND)
     message(STATUS "Doxygen found.")
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
     message(STATUS " Doxygen has not been found in your system.")
     message(STATUS " You can get nice code documentation by using it!")
     message(STATUS " Get it from http://www.stack.nl/~dimitri/doxygen/index.html")
     if(OS_IS_LINUX)
         if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(" or simply by doing 'sudo yum install doxygen-latex'.") 
         else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
             message(" or simply by doing 'sudo apt-get install doxygen-latex'.") 
         endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
     endif(OS_IS_LINUX)
     if(OS_IS_MACOSX)
         message(STATUS " or simply by doing 'sudo port install doxygen +latex'.")
     endif(OS_IS_MACOSX)
endif(DOXYGEN_FOUND)



################################################################################
# Armadillo - http://arma.sourceforge.net/
################################################################################

if(OS_IS_LINUX)
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
    find_library(GFORTRAN NAMES gfortran
                 PATHS /usr/lib
                       /usr/local/lib
                       /usr/local/lib/i386
                       /usr/lib/gcc/x86_64-linux-gnu
                       /usr/lib/gcc/i686-linux-gnu
                       /usr/lib/gcc/i386-linux-gnu
                       /usr/lib/gcc/x86_64-linux-gnu/4.6 # Ubuntu 12.04
                       /usr/lib/gcc/i686-linux-gnu/4.6
                       /usr/lib/gcc/x86_64-linux-gnu/4.7
                       /usr/lib/gcc/i686-linux-gnu/4.7
                       /usr/lib/gcc/x86_64-linux-gnu/4.8
                       /usr/lib/gcc/i686-linux-gnu/4.8
                       /usr/lib/gcc/x86_64-redhat-linux/4.7.2 # Fedora 18
                       /usr/lib/gcc/i686-redhat-linux/4.7.2
                       /usr/lib/gcc/x86_64-redhat-linux/4.8.1 # Fedora 19
                       /usr/lib/gcc/i686-redhat-linux/4.8.1
                       /usr/lib/gcc/x86_64-redhat-linux/4.4.4 # CentOS 6
                       /usr/lib/gcc/i686-redhat-linux/4.4.4
                       /usr/lib/gcc/x86_64-redhat-linux/4.8.2
                       /usr/lib/gcc/i686-redhat-linux/4.8.2
                       /usr/lib/gcc/i586-suse-linux/4.8  # OpenSUSE 13.1
                       /usr/lib/gcc/x86_64-suse-linux/4.8
                       /usr/lib/gcc/i486-linux-gnu # Debian 7
                       /usr/lib/gcc/i486-linux-gnu/4.4
                       /usr/lib/gcc/i486-linux-gnu/4.6
                       /usr/lib/gcc/i486-linux-gnu/4.7
                       /usr/lib/gcc/i486-linux-gnu/4.8
                )
    if(NOT GFORTRAN)
        message(" The gfortran library has not been found.")
        message(" You can try to install it by typing:")    
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo yum install gcc-fortran") 
        elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
            message(" sudo zypper install gcc-fortran")
        else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo apt-get install gfortran") 
        endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
        message(FATAL_ERROR "gfortran is required to build gnss-sdr")
    endif(NOT GFORTRAN)
endif(OS_IS_LINUX)

find_package(Armadillo)
if(NOT ARMADILLO_FOUND)
     message (STATUS " Armadillo has not been found.")
     message (STATUS " Armadillo will be downloaded and built automatically ")
     message (STATUS " when doing 'make'. ")
     # Check if the patch command is available (does not come by default on Fedora or openSUSE)
     find_program(PATCH_EXECUTABLE NAME patch PATHS ENV PATH)     
     if(NOT PATCH_EXECUTABLE)
          message(" The patch command has not been found.")
          message(" You can try to install it by typing:")    
          if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
              message(" sudo yum install patch") 
          elseif(${LINUX_DISTRIBUTION} MATCHES "openSUSE")
              message(" sudo zypper install patch")
          else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
              message(" sudo apt-get install patch") 
          endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
          message(FATAL_ERROR "The patch command is required to download and build armadillo")
     endif(NOT PATCH_EXECUTABLE)
     set(OLD_GCC false)
     if(CMAKE_COMPILER_IS_GNUCC)
         execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
         if(GCC_VERSION VERSION_EQUAL 4.6 OR GCC_VERSION VERSION_LESS 4.6)
             set(OLD_GCC true)
         endif(GCC_VERSION VERSION_EQUAL 4.6 OR GCC_VERSION VERSION_LESS 4.6)
     endif(CMAKE_COMPILER_IS_GNUCC)
     
     if(OLD_GCC)
         set(armadillo_RELEASE 3.920.2)
         set(armadillo_MD5 "3d0396513e2802c08152f50e18b4a1cd")
         if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
             set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_no.patch)
             file(WRITE ${ARMADILLO_PATCH_FILE} "")
         else(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
             set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_staticlib.patch)
             file(WRITE ${ARMADILLO_PATCH_FILE}
"35c35
< set(ARMA_USE_WRAPPER true )
---
> set(ARMA_USE_WRAPPER false)
323c323
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
             BUILD_IN_SOURCE 1
             BUILD_COMMAND make
             UPDATE_COMMAND ""
             INSTALL_COMMAND ""
         )
     else(OLD_GCC)
         set(armadillo_RELEASE 4.300.3)  
         set(armadillo_MD5 "cdcd41961c3b06ac8cef9645c38bdf50") # cdcd41961c3b06ac8cef9645c38bdf50
         if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
             set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_no.patch)
             file(WRITE ${ARMADILLO_PATCH_FILE} "")
             set(ARMADILLO_PATCH_FILE2 ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_no2.patch)
             file(WRITE ${ARMADILLO_PATCH_FILE2} "")
         else(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
             set(ARMADILLO_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_staticlib.patch)
             set(ARMADILLO_PATCH_FILE2 ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}/armadillo_enable_lapack.patch)
             file(WRITE ${ARMADILLO_PATCH_FILE}
"30c30
< set(ARMA_USE_LAPACK    false)
---
> set(ARMA_USE_LAPACK true)
36c36
< set(ARMA_USE_WRAPPER   true )
---
> set(ARMA_USE_WRAPPER false)
312c312
< add_library( armadillo SHARED \${PROJECT_SOURCE_DIR}/src/wrapper.cpp )
---
> add_library( armadillo STATIC \${PROJECT_SOURCE_DIR}/src/wrapper.cpp )
")
             file(WRITE ${ARMADILLO_PATCH_FILE2}
"12c12
< // #define ARMA_USE_LAPACK
---
>    #define ARMA_USE_LAPACK 
") 
         endif(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}/armadillo-${armadillo_RELEASE}.tar.gz)
         ExternalProject_Add(
             armadillo-${armadillo_RELEASE}
             PREFIX ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
             URL http://sourceforge.net/projects/arma/files/armadillo-${armadillo_RELEASE}.tar.gz
             DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download/armadillo-${armadillo_RELEASE}
             URL_MD5 ${armadillo_MD5}    
             PATCH_COMMAND patch -N <BINARY_DIR>/CMakeLists.txt ${ARMADILLO_PATCH_FILE} && patch -N <BINARY_DIR>/include/armadillo_bits/config.hpp ${ARMADILLO_PATCH_FILE2}
             BUILD_IN_SOURCE 1
             BUILD_COMMAND make
             UPDATE_COMMAND "" 
             INSTALL_COMMAND ""
         )
     endif(OLD_GCC)
     # Set up variables
     ExternalProject_Get_Property(armadillo-${armadillo_RELEASE} binary_dir)
     set(ARMADILLO_INCLUDE_DIRS ${binary_dir}/include )     
     find_library(lapack NAMES lapack HINTS /usr/lib /usr/local/lib /usr/lib64)
     find_library(blas NAMES blas HINTS /usr/lib /usr/local/lib /usr/lib64)
     set(ARMADILLO_LIBRARIES ${lapack} ${blas} ${GFORTRAN} ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}armadillo.a)
     set(LOCAL_ARMADILLO true CACHE STRING "Armadillo downloaded and built automatically" FORCE)
     # Save a copy at the thirdparty folder
     file(COPY ${CMAKE_CURRENT_BINARY_DIR}/armadillo-${armadillo_RELEASE}
          DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/armadillo
     )
endif(NOT ARMADILLO_FOUND)



###############################################################################
# OpenCL
###############################################################################
find_package(OpenCL)
if($ENV{DISABLE_OPENCL})
     set(DISABLE_OPENCL TRUE)
endif($ENV{DISABLE_OPENCL})
if(DISABLE_OPENCL)
    set(OPENCL_FOUND FALSE)
else(DISABLE_OPENCL)
    if(OPENCL_FOUND)
         message(STATUS "OpenCL has been found and will be used by some processing blocks")
         message(STATUS "You can disable OpenCL use by doing 'cmake -DDISABLE_OPENCL=1 ../' ")
    endif(OPENCL_FOUND)
endif(DISABLE_OPENCL)
if(NOT OPENCL_FOUND)
    message(STATUS "Processing blocks using OpenCL will not be built.")
endif(NOT OPENCL_FOUND)



################################################################################
# OpenSSL - http://www.openssl.org
################################################################################
find_package(OpenSSL)
if(NOT OPENSSL_FOUND)
    message(" The OpenSSL library has not been found.")
    message(" You can try to install it by typing:")
    if(OS_IS_LINUX)      
        if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo yum install openssl-devel") 
        else(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
            message(" sudo apt-get install libssl-dev") 
        endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "Red Hat")
    endif(OS_IS_LINUX) 
    if(OS_IS_MACOSX)
        message(" sudo port install openssl")
    endif(OS_IS_MACOSX)
    message(FATAL_ERROR "OpenSSL libraries are required to build gnss-sdr")
endif(NOT OPENSSL_FOUND)


################################################################################
# Setup of optional drivers
################################################################################
if( $ENV{GN3S_DRIVER} )
    message(STATUS "GN3S_DRIVER variable found." )
    # copy firmware to install folder
    # Build project gr-gn3s
else( $ENV{GN3S_DRIVER} )
    if( GN3S_DRIVER )
       message(STATUS "GN3S driver will be compiled")
    else( GNSS_DRIVER )
       message(STATUS "GN3S_DRIVER is not defined." )
       message(STATUS "Define it with 'export GN3S_DRIVER=1' to add support for the GN3S dongle." )
    endif( GN3S_DRIVER )
endif($ENV{GN3S_DRIVER} )

if( $ENV{RAW_ARRAY_DRIVER} )
    message(STATUS "RAW_ARRAY_DRIVER variable found." )
    # copy firmware to install folder
    # Build project gr-dbfcttc
else( $ENV{RAW_ARRAY_DRIVER} )
    if( RAW_ARRAY_DRIVER )
       message(STATUS "RAW_ARRAY_DRIVER driver will be compiled")
    else( RAW_ARRAY_DRIVER )
       message(STATUS "RAW_ARRAY_DRIVER is not defined." )
       message(STATUS "Define it with 'export RAW_ARRAY_DRIVER=1' to add support for the CTTC experimental array front-end." )
    endif( RAW_ARRAY_DRIVER )
endif($ENV{RAW_ARRAY_DRIVER} )

if( $ENV{RTLSDR_DRIVER} )
    message(STATUS "RTLSDR_DRIVER variable found." )
    # find libosmosdr (done in src/algorithms/signal_sources/adapters)
    # find gr-osmosdr (done in src/algorithms/signal_sources/adapters)
endif($ENV{RTLSDR_DRIVER} )



########################################################################
# Set compiler flags
########################################################################

# Enable C++11 support in GCC
# For "-std=c++0x"  GCC's support for C++11 see http://gcc.gnu.org/projects/cxx0x.html
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++0x -Wall")  #Add warning flags: For "-Wall" see http://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -std=c++11 -stdlib=libc++ -Wno-c++11-narrowing") 
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -Wno-unused-private-field") 
    endif(CMAKE_BUILD_TYPE MATCHES "Release")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

# Processor-architecture related flags
# See http://gcc.gnu.org/onlinedocs/gcc/i386-and-x86_002d64-Options.html#i386-and-x86_002d64-Options
# Change to '-mtune=generic' when building for distribution
if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
     if(OS_IS_MACOSX)
         set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -march=corei7 -mfpmath=sse")
     else(OS_IS_MACOSX)
         set(MY_CXX_FLAGS "${MY_CXX_FLAGS} -march=native -mfpmath=sse")
     endif(OS_IS_MACOSX)
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

list(APPEND CMAKE_CXX_FLAGS ${MY_CXX_FLAGS})

if(OS_IS_LINUX)      
    if(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "openSUSE")
        link_libraries(pthread)
    endif(${LINUX_DISTRIBUTION} MATCHES "Fedora" OR ${LINUX_DISTRIBUTION} MATCHES "openSUSE")
endif(OS_IS_LINUX) 


########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)


########################################################################
# Add subdirectories (in order of deps)
########################################################################
add_subdirectory(src)



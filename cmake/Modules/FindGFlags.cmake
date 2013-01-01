# - Try to find GFlags
#
# The following variables are optionally searched for defaults
# GFlags_ROOT_DIR: Base directory where all GFlags components are found
#
# The following are set after configuration is done:
# GFlags_FOUND
# GFlags_INCLUDE_DIRS
# GFlags_LIBS
# GFlags_LIBRARY_DIRS
cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0011 OLD)

if (WIN32)
     FIND_PATH(GFlags_ROOT_DIR
     src/gflags.cc
     HINTS
     $ENV{GFLAGS_ROOT})
else (WIN32)
     FIND_PATH(GFlags_ROOT_DIR
     libgflags.dylib
     HINTS
     /usr/local/lib
)
endif (WIN32)

if (UNIX)
     FIND_PATH(GFlags_ROOT_DIR
     libgflags.so
     HINTS
     /usr/local/lib
     )
endif (UNIX)

IF(GFlags_ROOT_DIR)
     # We are testing only a couple of files in the include directories
     if (WIN32)
          FIND_PATH(GFlags_INCLUDE_DIRS
          gflags/gflags.h
          HINTS
          ${GFlags_ROOT_DIR}/src/windows
          )
     else (WIN32)
          FIND_PATH(GFlags_INCLUDE_DIRS
          gflags/gflags.h
          HINTS
          /usr/local/include
          ${GFlags_ROOT_DIR}/src
          )
     endif (WIN32)

     # Find the libraries
     SET(GFlags_LIBRARY_DIRS ${GFlags_ROOT_DIR})

     # TODO: This can use some per-component linking
     if(MSVC)
          SET(_gflags_libpath_suffixes /Release /Debug)
          FIND_LIBRARY(GFlags_lib_release
          NAMES libgflags
          HINTS
          ${GFlags_LIBRARY_DIRS}
          PATH_SUFFIXES ${_gflags_libpath_suffixes})
          FIND_LIBRARY(GFlags_lib_debug
          NAMES libgflags-debug
          HINTS
          ${GFlags_LIBRARY_DIRS}
          PATH_SUFFIXES ${_gflags_libpath_suffixes})
          SET(GFlags_lib optimized ${GFlags_lib_release} debug ${GFlags_lib_debug})
     else()
          FIND_LIBRARY(GFlags_lib gflags ${GFlags_LIBRARY_DIRS})
     endif()

     # set up include and link directory
     include_directories(${GFlags_INCLUDE_DIRS})
     link_directories(${GFlags_LIBRARY_DIRS})
     message("gflags library found at ${GFlags_lib}")
     SET(GFlags_LIBS ${GFlags_lib})
     SET(GFlags_FOUND true)
     MARK_AS_ADVANCED(GFlags_INCLUDE_DIRS)
ELSE(GFlags_ROOT_DIR)
     FIND_PATH(GFlags_ROOT_DIR src)
     MARK_AS_ADVANCED(GFlags_ROOT_DIR)
     MESSAGE(STATUS "Cannot find Root directory of gflags")
     SET(GFlags_FOUND false)
ENDIF(GFlags_ROOT_DIR)

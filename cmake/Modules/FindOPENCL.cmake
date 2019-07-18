# Copyright (C) 2011-2019 (see AUTHORS file for a list of contributors)
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
# This file taken from FindOpenCL project @ http://gitorious.com/findopencl
#
# - Try to find OpenCL
# This module tries to find an OpenCL implementation on your system. It supports
# AMD / ATI, Apple and NVIDIA implementations.
#
# Once done this will define
# OPENCL_FOUND - system has OpenCL
# OPENCL_INCLUDE_DIRS - the OpenCL include directory
# OPENCL_LIBRARIES - link these to use OpenCL
#
# WIN32 should work, but is untested

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

include(FindPackageHandleStandardArgs)

function(_FIND_OPENCL_VERSION)
    include(CheckSymbolExists)
    include(CMakePushCheckState)
    set(CMAKE_REQUIRED_QUIET ${OPENCL_FIND_QUIETLY})

    cmake_push_check_state()
    foreach(VERSION "2_2" "2_1" "2_0" "1_2" "1_1" "1_0")
        set(CMAKE_REQUIRED_INCLUDES "${OPENCL_INCLUDE_DIR}")

        if(APPLE)
            check_symbol_exists(
                CL_VERSION_${VERSION}
                "${OPENCL_INCLUDE_DIR}/Headers/cl.h"
                OPENCL_VERSION_${VERSION}
            )
        else()
            check_symbol_exists(
                CL_VERSION_${VERSION}
                "${OPENCL_INCLUDE_DIR}/CL/cl.h"
                OPENCL_VERSION_${VERSION}
            )
        endif()

        if(OPENCL_VERSION_${VERSION})
            string(REPLACE "_" "." VERSION "${VERSION}")
            set(OPENCL_VERSION_STRING ${VERSION} PARENT_SCOPE)
            string(REGEX MATCHALL "[0-9]+" version_components "${VERSION}")
            list(GET version_components 0 major_version)
            list(GET version_components 1 minor_version)
            set(OPENCL_VERSION_MAJOR ${major_version} PARENT_SCOPE)
            set(OPENCL_VERSION_MINOR ${minor_version} PARENT_SCOPE)
            break()
        endif()
    endforeach()
    cmake_pop_check_state()
endfunction()

find_path(OPENCL_INCLUDE_DIR
    NAMES
        CL/cl.h OpenCL/cl.h
    PATHS
        ENV "PROGRAMFILES(X86)"
        ENV AMDAPPSDKROOT
        ENV INTELOCLSDKROOT
        ENV NVSDKCOMPUTE_ROOT
        ENV CUDA_PATH
        ENV ATISTREAMSDKROOT
        ENV OCL_ROOT
        /usr/local/cuda/include
    PATH_SUFFIXES
        include
        OpenCL/common/inc
        "AMD APP/include"
)

find_path(_OPENCL_CPP_INCLUDE_DIRS
    NAMES
        CL/cl.hpp OpenCL/cl.hpp
    PATHS
        ENV "PROGRAMFILES(X86)"
        ENV AMDAPPSDKROOT
        ENV INTELOCLSDKROOT
        ENV NVSDKCOMPUTE_ROOT
        ENV CUDA_PATH
        ENV ATISTREAMSDKROOT
        ENV OCL_ROOT
        /usr/local/cuda/include
    PATH_SUFFIXES
       include
       OpenCL/common/inc
       "AMD APP/include"
)

set(OPENCL_INCLUDE_DIRS ${OPENCL_INCLUDE_DIR})
if(_OPENCL_CPP_INCLUDE_DIRS)
    set(OPENCL_HAS_CPP_BINDINGS TRUE)
    list(APPEND OPENCL_INCLUDE_DIRS ${_OPENCL_CPP_INCLUDE_DIRS})
    # This is often the same, so clean up
    list(REMOVE_DUPLICATES OPENCL_INCLUDE_DIRS)
endif()

_FIND_OPENCL_VERSION()

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        find_library(OPENCL_LIBRARY
            NAMES OpenCL
            PATHS
                ENV "PROGRAMFILES(X86)"
                ENV AMDAPPSDKROOT
                ENV INTELOCLSDKROOT
                ENV CUDA_PATH
                ENV NVSDKCOMPUTE_ROOT
                ENV ATISTREAMSDKROOT
                ENV OCL_ROOT
            PATH_SUFFIXES
                "AMD APP/lib/x86"
                lib/x86
                lib/Win32
                OpenCL/common/lib/Win32
        )
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
        find_library(OPENCL_LIBRARY
            NAMES OpenCL
            PATHS
                ENV "PROGRAMFILES(X86)"
                ENV AMDAPPSDKROOT
                ENV INTELOCLSDKROOT
                ENV CUDA_PATH
                ENV NVSDKCOMPUTE_ROOT
                ENV ATISTREAMSDKROOT
                ENV OCL_ROOT
            PATH_SUFFIXES
                "AMD APP/lib/x86_64"
                lib/x86_64
                lib/x64
                OpenCL/common/lib/x64
        )
    endif()
else()
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        find_library(OPENCL_LIBRARY
            NAMES OpenCL
            PATHS
                ENV AMDAPPSDKROOT
                ENV CUDA_PATH
                ENV LD_LIBRARY_PATH
            PATH_SUFFIXES
                lib/x86
                lib
        )
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
        find_library(OPENCL_LIBRARY
            NAMES OpenCL
            PATHS
                ENV AMDAPPSDKROOT
                ENV CUDA_PATH
                ENV LD_LIBRARY_PATH
            PATH_SUFFIXES
                lib/x86_64
                lib/x64
                lib
                lib64
        )
    endif()
endif()

set(OPENCL_LIBRARIES ${OPENCL_LIBRARY})

find_package_handle_standard_args(OPENCL DEFAULT_MSG OPENCL_LIBRARIES OPENCL_INCLUDE_DIRS)

mark_as_advanced(
  OPENCL_INCLUDE_DIRS
  OPENCL_LIBRARIES
)

set_package_properties(OPENCL PROPERTIES
    URL "https://www.khronos.org/opencl/"
)

if(OPENCL_FOUND AND OPENCL_VERSION_STRING)
    set_package_properties(OPENCL PROPERTIES
        DESCRIPTION "Library for parallel programming (found: v${OPENCL_VERSION_STRING})"
    )
else()
    set_package_properties(OPENCL PROPERTIES
        DESCRIPTION "Library for parallel programming"
    )
endif()

if(OPENCL_FOUND AND NOT TARGET OpenCL::OpenCL)
    if(OPENCL_LIBRARY MATCHES "/([^/]+)\\.framework$")
        add_library(OpenCL::OpenCL INTERFACE IMPORTED)
        set_target_properties(OpenCL::OpenCL PROPERTIES
            INTERFACE_LINK_LIBRARIES "${OPENCL_LIBRARY}"
        )
    else()
        add_library(OpenCL::OpenCL UNKNOWN IMPORTED)
        set_target_properties(OpenCL::OpenCL PROPERTIES
            IMPORTED_LOCATION "${OPENCL_LIBRARY}"
        )
    endif()
    set_target_properties(OpenCL::OpenCL PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OPENCL_INCLUDE_DIRS}"
    )
endif()

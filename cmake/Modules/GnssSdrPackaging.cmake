# Copyright (C) 2012-2014  (see AUTHORS file for a list of contributors)
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


#if(DEFINED __INCLUDED_GNSS_SDR_PACKAGE_CMAKE)
#    return()
#endif()
#set( __INCLUDED_GNSS_SDR_PACKAGE_CMAKE TRUE)


#set the cpack generator based on the platform type
if(CPACK_GENERATOR)
    #already set by user
elseif(APPLE)
    set(PACKAGE_GENERATOR "DragNDrop")
    set(PACKAGE_SOURCE_GENERATOR "TGZ;ZIP")
elseif(UNIX)
    if(${LINUX_DISTRIBUTION} MATCHES "Debian" OR ${LINUX_DISTRIBUTION} MATCHES "Ubuntu")
        set (PACKAGE_GENERATOR "DEB")
    endif(${LINUX_DISTRIBUTION} MATCHES "Debian" OR ${LINUX_DISTRIBUTION} MATCHES "Ubuntu")
    if(${LINUX_DISTRIBUTION} MATCHES "Red Hat" OR ${LINUX_DISTRIBUTION} MATCHES "Fedora")
        set (PACKAGE_GENERATOR "RPM")
    endif(${LINUX_DISTRIBUTION} MATCHES "Red Hat" OR ${LINUX_DISTRIBUTION} MATCHES "Fedora")
    set (PACKAGE_SOURCE_GENERATOR "TGZ;ZIP")
else()
    set(PACKAGE_GENERATOR "TGZ")
    set(PACKAGE_SOURCE_GENERATOR "TGZ")
endif()

# used package generators
set (CPACK_GENERATOR "${PACKAGE_GENERATOR}" CACHE STRING "List of binary package generators (CPack).")
set (CPACK_SOURCE_GENERATOR "${PACKAGE_SOURCE_GENERATOR}" CACHE STRING "List of source package generators (CPack).")




CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/cmake/Packaging/postinst.in" "${CMAKE_CURRENT_BINARY_DIR}/scripts/postinst" @ONLY IMMEDIATE)
FILE(COPY ${CMAKE_CURRENT_BINARY_DIR}/scripts/postinst DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/cmake/Packaging/prerm.in" "${CMAKE_CURRENT_BINARY_DIR}/scripts/prerm" @ONLY IMMEDIATE)
FILE(COPY ${CMAKE_CURRENT_BINARY_DIR}/scripts/prerm DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)




########################################################################
# Setup CPack
########################################################################
set(CPACK_PACKAGE_DESCRIPTION          "GNSS Software Defined Receiver")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY  "Global Navigation Satellite Systems Software Defined Receiver written in C++." )
set(CPACK_PACKAGE_VENDOR               "Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)")
set(CPACK_PACKAGE_NAME                 "gnss-sdr")
set(CPACK_PACKAGE_VERSION              "${VERSION}")
set(CPACK_PACKAGE_CONTACT              "Carles Fernandez-Prades <carles.fernandez@cttc.cat>")
set(CPACK_PACKAGE_ICON                 "${CMAKE_SOURCE_DIR}/docs/doxygen/images/gnss-sdr_logo_round.png")
set(CPACK_PACKAGE_VERSION_MAJOR        "${VERSION_INFO_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR        "${VERSION_INFO_API_COMPAT}")
set(CPACK_PACKAGE_VERSION_PATCH        "${VERSION_INFO_MINOR_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE        "${CMAKE_SOURCE_DIR}/COPYING")
set(CPACK_RESOURCE_FILE_README         "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_WELCOME        "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_SET_DESTDIR                  "OFF")
set(CPACK_STRIP_FILES                  "ON")

# Debian-specific settings
set(CPACK_DEBIAN_PACKAGE_SECTION       "science")
set(CPACK_DEBIAN_PACKAGE_PRIORITY      "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS       "cmake (>= 2.8), libstdc++6 (>= 4.7), libc6 (>= 2.13), libboost-dev (>= 1.45), libboost-date-time-dev (>= 1.45), libboost-system-dev (>= 1.45), libboost-filesystem-dev (>= 1.45), libboost-thread-dev (>= 1.45), libboost-serialization-dev (>= 1.45), libboost-program-options-dev (>= 1.45), libboost-test-dev (>= 1.45), libuhd-dev (>= 3.7), gnuradio-dev (>= 3.7.3), gr-osmosdr (>= 0.1.1), liblog4cpp5-dev (>= 1.0), libarmadillo-dev (>= 1:4.200.0), liblapack-dev (>= 3.4), libblas-dev (>= 1.2), gfortran (>= 1:4.7), libssl-dev (>= 1.0), libgflags-dev (>= 2.0), libgoogle-glog-dev (>= 0.3.3), libgtest-dev (>= 1.7)")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION   "GNSS Software Defined Receiver written in C++.
 Global Navigation Satellite Systems receiver defined by software.
 It performs all the signal processing from raw signal samples up to the
 computation of the Position-Velocity-Time solution, including
 code and phase observables. It is able to work with raw data files or, 
 if there is computational power enough, in real time with suitable 
 radio frequency front-ends. This software is mainly developed at CTTC 
 (Centre Tecnologic de Telecomunicacions de Catalunya, http://www.cttc.es) 
 with contributions from around the world. 
 More info at http://gnss-sdr.org")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_BINARY_DIR}/postinst;${CMAKE_CURRENT_BINARY_DIR}/prerm")



# system/architecture
if (APPLE)
    set (CPACK_PACKAGE_ARCHITECTURE darwin)
else(APPLE)
    string (TOLOWER "${CMAKE_SYSTEM_NAME}" CPACK_SYSTEM_NAME)
    if(CMAKE_CXX_FLAGS MATCHES "-m32")
        set (CPACK_PACKAGE_ARCHITECTURE i386)
    else(CMAKE_CXX_FLAGS MATCHES "-m32")
        execute_process (
               COMMAND dpkg --print-architecture
               RESULT_VARIABLE RV
               OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE
               )
        if(RV EQUAL 0)
            string (STRIP "${CPACK_PACKAGE_ARCHITECTURE}" CPACK_PACKAGE_ARCHITECTURE)
        else(RV EQUAL 0)
            execute_process (COMMAND uname -m OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE)
            if(CPACK_PACKAGE_ARCHITECTURE MATCHES "x86_64")
                set (CPACK_PACKAGE_ARCHITECTURE amd64)
            endif(CPACK_PACKAGE_ARCHITECTURE MATCHES "x86_64")
            if(CPACK_PACKAGE_ARCHITECTURE MATCHES "i386")
                set (CPACK_PACKAGE_ARCHITECTURE i386)
            endif(CPACK_PACKAGE_ARCHITECTURE MATCHES "i386")
        endif(RV EQUAL 0)
    endif(CMAKE_CXX_FLAGS MATCHES "-m32")
endif(APPLE)


if(NOT CPACK_PACKAGE_ARCHITECTURE)
# Code from https://qt.gitorious.org/qt/qtbase/source/src/corelib/global/qprocessordetection.h
set(archdetect_c_code "
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || defined(_M_ARM) || defined(__arm64__)
    #if defined(__arm64__)
        #error cmake_ARCH arm64
    #endif
   
    #if defined(__arm__) || defined(__TARGET_ARCH_ARM)
        #if defined(__ARM_ARCH_8__) \\
        || defined(__ARM_ARCH_8A__) \\
        || defined(__ARM_ARCH_8R__) \\
        || (defined(__TARGET_ARCH_ARM) && __TARGET_ARCH_ARM-0 >= 8)
            #error cmake_ARCH armv8   
        #elif defined(__ARM_ARCH_7__) \\
        || defined(__ARM_ARCH_7A__) \\
        || defined(__ARM_ARCH_7R__) \\
        || defined(__ARM_ARCH_7M__) \\
        || (defined(__TARGET_ARCH_ARM) && __TARGET_ARCH_ARM-0 >= 7)
            #error cmake_ARCH armv7
        #elif defined(__ARM_ARCH_6__) \\
        || defined(__ARM_ARCH_6J__) \\
        || defined(__ARM_ARCH_6T2__) \\
        || defined(__ARM_ARCH_6Z__) \\
        || defined(__ARM_ARCH_6K__) \\
        || defined(__ARM_ARCH_6ZK__) \\
        || defined(__ARM_ARCH_6M__) \\
        || (defined(__TARGET_ARCH_ARM) && __TARGET_ARCH_ARM-0 >= 6)
            #error cmake_ARCH armv6
        #elif defined(__ARM_ARCH_5TEJ__) \\
        || (defined(__TARGET_ARCH_ARM) && __TARGET_ARCH_ARM-0 >= 5)
            #error cmake_ARCH armv5
        #else
            #error cmake_ARCH arm
        #endif
#elif defined(__i386) || defined(__i386__) || defined(_M_IX86)
    #error cmake_ARCH i386
#elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
    #error cmake_ARCH x86_64
#elif defined(__ia64) || defined(__ia64__) || defined(_M_IA64)
    #error cmake_ARCH ia64
#elif defined(__ppc__) || defined(__ppc) || defined(__powerpc__) \\
    || defined(_ARCH_COM) || defined(_ARCH_PWR) || defined(_ARCH_PPC) \\
    || defined(_M_MPPC) || defined(_M_PPC)
    #if defined(__ppc64__) || defined(__powerpc64__) || defined(__64BIT__)
        #error cmake_ARCH ppc64
    #else
        #error cmake_ARCH ppc
    #endif
#endif
#error cmake_ARCH unknown
")



function(target_architecture output_var)
    if(APPLE AND CMAKE_OSX_ARCHITECTURES)
        # On OS X we use CMAKE_OSX_ARCHITECTURES *if* it was set
        # On OS X 10.6+ the default is x86_64 if the CPU supports it, i386 otherwise.
        foreach(osx_arch ${CMAKE_OSX_ARCHITECTURES})
            if("${osx_arch}" STREQUAL "i386")
                set(osx_arch_i386 TRUE)
            elseif("${osx_arch}" STREQUAL "x86_64")
                set(osx_arch_x86_64 TRUE)
            else()
                message(FATAL_ERROR "Invalid OS X arch name: ${osx_arch}")
            endif()
        endforeach()
        # Now add all the architectures in our normalized order
        if(osx_arch_i386)
            list(APPEND ARCH i386)
        endif()
        if(osx_arch_x86_64)
            list(APPEND ARCH x86_64)
        endif()
    else()
        file(WRITE "${CMAKE_BINARY_DIR}/arch.c" "${archdetect_c_code}")
        enable_language(C)
        # Detect the architecture in a rather creative way...
        # This compiles a small C program which is a series of ifdefs that selects a
        # particular #error preprocessor directive whose message string contains the
        # target architecture. The program will always fail to compile (both because
        # file is not a valid C program, and obviously because of the presence of the
        # #error preprocessor directives... but by exploiting the preprocessor in this
        # way, we can detect the correct target architecture even when cross-compiling,
        # since the program itself never needs to be run (only the compiler/preprocessor)
        try_run(
                run_result_unused
                compile_result_unused
                "${CMAKE_BINARY_DIR}"
                "${CMAKE_BINARY_DIR}/arch.c"
                COMPILE_OUTPUT_VARIABLE ARCH
                CMAKE_FLAGS CMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}
               )
        # Parse the architecture name from the compiler output
        string(REGEX MATCH "cmake_ARCH ([a-zA-Z0-9_]+)" ARCH "${ARCH}")
        # Get rid of the value marker leaving just the architecture name
        string(REPLACE "cmake_ARCH " "" ARCH "${ARCH}")
        # If we are compiling with an unknown architecture this variable should
        # already be set to "unknown" but in the case that it's empty (i.e. due
        # to a typo in the code), then set it to unknown
        if (NOT ARCH)
            set(ARCH unknown)
        endif()
    endif()
set(${output_var} "${ARCH}" PARENT_SCOPE)
endfunction()

# Set target architectures
target_architecture(CMAKE_TARGET_ARCHITECTURES)
endif(NOT CPACK_PACKAGE_ARCHITECTURE)

if(APPLE)
list(LENGTH CMAKE_TARGET_ARCHITECTURES cmake_target_arch_len)
    if(NOT "${cmake_target_arch_len}" STREQUAL "1")
        set(CMAKE_TARGET_ARCHITECTURE_UNIVERSAL TRUE)
        set(CMAKE_TARGET_ARCHITECTURE_CODE "universal")
    else(NOT "${cmake_target_arch_len}" STREQUAL "1")
        set(CMAKE_TARGET_ARCHITECTURE_UNIVERSAL FALSE)
        set(CMAKE_TARGET_ARCHITECTURE_CODE "${CMAKE_TARGET_ARCHITECTURES}")
    endif(NOT "${cmake_target_arch_len}" STREQUAL "1")
endif(APPLE)

# source package settings
#set (CPACK_SOURCE_TOPLEVEL_TAG "source")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_SOURCE_IGNORE_FILES "/\\\\.git/;\\\\.gitignore;/\\\\data/*;/\\\\install/*;/\\\\thirdparty/;/\\\\docs/html/;/\\\\docs/latex;\\\\.pdf;\\\\.project$;\\\\.DS_Store;\\\\.swp$;\\\\.#;/#;\\\\.*~;cscope\\\\.*;/[Bb]uild[.+-_a-zA-Z0-9]*/")

# default binary package settings
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY TRUE)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}")
if(CPACK_PACKAGE_ARCHITECTURE)
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_${CPACK_PACKAGE_ARCHITECTURE}")
endif(CPACK_PACKAGE_ARCHITECTURE)

if(CPACK_GENERATOR STREQUAL "DEB")
     configure_file("${CMAKE_SOURCE_DIR}/cmake/Packaging/fixup_deb_permissions.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/fixup_deb_permissions.sh" @ONLY IMMEDIATE)

     configure_file("${CMAKE_SOURCE_DIR}/cmake/Packaging/fixup_deb_permissions.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/scripts/fixup_deb_permissions.sh" @ONLY IMMEDIATE)
     
     file(COPY ${CMAKE_CURRENT_BINARY_DIR}/scripts/fixup_deb_permissions.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
          FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

     # Write license 
     file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/copyright"     
"Format: http://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: gnss-sdr
Source: git://github.com/gnss-sdr/gnss-sdr

Files: *
Copyright: Copyright 2014 ${CPACK_PACKAGE_CONTACT}
License: GPL-3+
 This program is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public
 License as published by the Free Software Foundation; either
 version 3 of the License, or (at your option) any later
 version.
 .
 This program is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied
 warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more
 details.
 .
 You should have received a copy of the GNU General Public
 License along with this package; if not, write to the Free
 Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 Boston, MA  02110-1301 USA
 .
 On Debian systems, the full text of the GNU General Public
 License version 3 can be found in the file
 `/usr/share/common-licenses/GPL-3'.
 
Files: src/core/libs/supl/*
Copyright: 2007 Tatu Mannisto
License: BSD
 
Files: src/core/libs/INIReader.*
Copyright: 2009 Brush Technologies
License: BSD-3-clause
 
Files: src/core/libs/ini.*
Copyright: 2009 Brush Technologies
License: BSD-3-clause")

     install(FILES "${CMAKE_CURRENT_BINARY_DIR}/copyright"        
             DESTINATION "share/doc/${CPACK_PACKAGE_NAME}")
             
     execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/docs/changelog
                     WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/changelog.gz")
                     
     execute_process(COMMAND gzip -9 -c ${CMAKE_SOURCE_DIR}/docs/changelog.Debian
                     WORKING_DIRECTORY ${CMAKE_BINARY_DIR} OUTPUT_FILE "${CMAKE_BINARY_DIR}/changelog.Debian.gz")
                             
     install(FILES "${CMAKE_BINARY_DIR}/changelog.gz" "${CMAKE_BINARY_DIR}/changelog.Debian.gz" DESTINATION "share/doc/${CPACK_PACKAGE_NAME}")
endif(CPACK_GENERATOR STREQUAL "DEB")

include(CPack)

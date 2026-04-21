# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(COMMAND setup_glog_gflags)
    return()
endif()

if(NOT COMMAND set_package_properties)
    include(FeatureSummary)
endif()

if(NOT COMMAND ExternalProject_Add)
    include(ExternalProject)
endif()

macro(setup_glog_gflags)
    if(NOT GNSSSDR_BINARY_DIR)
        if(PROJECT_BINARY_DIR)
            set(GNSSSDR_BINARY_DIR "${PROJECT_BINARY_DIR}")
        else()
            set(GNSSSDR_BINARY_DIR "${CMAKE_BINARY_DIR}")
        endif()
    endif()
    if(NOT GNSSSDR_GFLAGS_MIN_VERSION)
        set(GNSSSDR_GFLAGS_MIN_VERSION "2.1.2")
    endif()
    if(NOT GNSSSDR_GFLAGS_LOCAL_VERSION)
        set(GNSSSDR_GFLAGS_LOCAL_VERSION "2.3.0")
    endif()
    if(NOT GNSSSDR_GLOG_LOCAL_VERSION)
        set(GNSSSDR_GLOG_LOCAL_VERSION "0.7.1")
    endif()
    if(NOT CMAKE_MAKE_PROGRAM_PRETTY_NAME)
        if(CMAKE_MAKE_PROGRAM)
            set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "${CMAKE_MAKE_PROGRAM}")
        else()
            set(CMAKE_MAKE_PROGRAM_PRETTY_NAME "the build tool")
        endif()
    endif()
    if(NOT DEFINED ENABLE_OWN_GLOG)
        set(ENABLE_OWN_GLOG OFF)
    endif()
    if(NOT DEFINED LINUX_DISTRIBUTION)
        set(LINUX_DISTRIBUTION "")
    endif()
    if(NOT DEFINED GFLAGS_TOOLCHAIN_FILE)
        set(GFLAGS_TOOLCHAIN_FILE "")
    endif()
    if(NOT DEFINED GLOG_TOOLCHAIN_FILE)
        set(GLOG_TOOLCHAIN_FILE "")
    endif()
    if(NOT DEFINED GLOG_EXPORT_CXX_LIBRARIES)
        set(GLOG_EXPORT_CXX_LIBRARIES "")
    endif()
    if(NOT DEFINED GLOG_EXPORT_C_COMPILER)
        set(GLOG_EXPORT_C_COMPILER "")
    endif()
    if(NOT DEFINED GLOG_EXPORT_CXX_COMPILER)
        set(GLOG_EXPORT_CXX_COMPILER "")
    endif()
    if(NOT DEFINED GLOG_GTEST)
        set(GLOG_GTEST "")
    endif()
    if(NOT DEFINED GNSSSDR_GLOG_LOCAL_GFLAGS)
        set(GNSSSDR_GLOG_LOCAL_GFLAGS "")
    endif()
    if(NOT DEFINED PARALLEL_BUILD)
        set(PARALLEL_BUILD "")
    endif()

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
            if(CMAKE_VERSION VERSION_LESS 3.10)
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
                # Note: -DBUILD_gflags_nothreads_LIB=ON is required as a workaround to a bug in gflags 2.2.2. This is fixed in gflags master branch
            else()
                ExternalProject_Add(gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                PREFIX ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                GIT_REPOSITORY https://github.com/gflags/gflags.git
                GIT_TAG v${GNSSSDR_GFLAGS_LOCAL_VERSION}
                SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/gflags/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                BINARY_DIR ${GNSSSDR_BINARY_DIR}/gflags-${GNSSSDR_GFLAGS_LOCAL_VERSION}
                CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF
                    -DBUILD_STATIC_LIBS=ON
                    -DBUILD_gflags_LIB=ON
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
        endif()

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
                            if("${LINUX_DISTRIBUTION}" MATCHES "Fedora" OR "${LINUX_DISTRIBUTION}" MATCHES "Red Hat")
                                message(" sudo yum groupinstall 'Development Tools'")
                            elseif("${LINUX_DISTRIBUTION}" MATCHES "openSUSE")
                                message(" sudo zypper install automake")
                            else()
                                message(" sudo apt install automake")
                            endif()
                            message(FATAL_ERROR "aclocal is required to build glog from source")
                        endif()
                    else()
                        message(" libtool has not been found.")
                        message(" You can try to install it by typing:")
                        if("${LINUX_DISTRIBUTION}" MATCHES "Fedora" OR "${LINUX_DISTRIBUTION}" MATCHES "Red Hat")
                            message(" sudo yum groupinstall 'Development Tools'")
                        elseif("${LINUX_DISTRIBUTION}" MATCHES "openSUSE")
                            message(" sudo zypper install libtoool")
                        else()
                            message(" sudo apt install libtool")
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
endmacro()

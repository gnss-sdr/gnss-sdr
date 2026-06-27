# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2023-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Downloads and builds the protoc compiler and static libraries of Protocol
# Buffers (see https://protobuf.dev/). The legacy Autotools build path is used
# when the modern CMake build is not available. The modern path requires CMake
# >= 3.10 and the abseil-cpp >= 20230117 libraries (see
# https://github.com/abseil/abseil-cpp) already installed. Zlib is used if found.
#
# Creates protobuf::libprotobuf and protobuf::protoc imported targets.
#
# Optional inputs for external projects:
#   BUILD_PROTOBUF_BINARY_DIR  Base directory for the external build.
#   BUILD_PROTOBUF_VERSION     Protobuf version to download and build.
#   BUILD_PROTOBUF_FIND_ABSL   Find abseil-cpp when absl_FOUND is not set.
#   BUILD_PROTOBUF_HOST_PROTOC Host protoc executable for cross-compilation.
#   BUILD_PROTOBUF_CROSS_HOST  Autotools host triplet for cross-compilation.
#
# GNSS-SDR's variables supported:
#   GNSSSDR_BINARY_DIR
#   GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION

get_property(_build_protobuf_already_included GLOBAL PROPERTY
    BUILD_PROTOBUF_ALREADY_INCLUDED
)
if(_build_protobuf_already_included)
    return()
endif()
set_property(GLOBAL PROPERTY BUILD_PROTOBUF_ALREADY_INCLUDED TRUE)
unset(_build_protobuf_already_included)

if(NOT COMMAND set_package_properties)
    include(FeatureSummary)
endif()

if(NOT COMMAND ExternalProject_Add)
    include(ExternalProject)
endif()

macro(_build_protobuf_initialize)
    if(NOT DEFINED BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR)
        set(BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR OFF)
        if(DEFINED GNSSSDR_SOURCE_DIR)
            set(BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR ON)
        elseif("${PROJECT_NAME}" STREQUAL "gnss-sdr")
            set(BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR ON)
        endif()
    endif()

    if(NOT DEFINED BUILD_PROTOBUF_FIND_ABSL)
        if(BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR)
            set(BUILD_PROTOBUF_FIND_ABSL OFF)
        else()
            set(BUILD_PROTOBUF_FIND_ABSL ON)
        endif()
    endif()

    if(DEFINED BUILD_PROTOBUF_BINARY_DIR)
        set(GNSSSDR_BINARY_DIR "${BUILD_PROTOBUF_BINARY_DIR}")
    elseif(NOT GNSSSDR_BINARY_DIR)
        if(PROJECT_BINARY_DIR)
            set(GNSSSDR_BINARY_DIR "${PROJECT_BINARY_DIR}")
        else()
            set(GNSSSDR_BINARY_DIR "${CMAKE_BINARY_DIR}")
        endif()
    endif()

    if(DEFINED BUILD_PROTOBUF_VERSION)
        set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION
            "${BUILD_PROTOBUF_VERSION}"
        )
    endif()

    if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION)
        set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "22.0")
    endif()

    if(NOT DEFINED LINUX_DISTRIBUTION)
        set(LINUX_DISTRIBUTION "")
    endif()

    if(NOT DEFINED MACOS_PACKAGES_PREFIX)
        set(MACOS_PACKAGES_PREFIX "")
        if((NOT BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR) OR
                CMAKE_CROSSCOMPILING)
            set(_build_protobuf_package_system_name "${CMAKE_SYSTEM_NAME}")
            if(CMAKE_CROSSCOMPILING AND CMAKE_HOST_SYSTEM_NAME)
                set(_build_protobuf_package_system_name
                    "${CMAKE_HOST_SYSTEM_NAME}"
                )
            endif()
            if("${_build_protobuf_package_system_name}" MATCHES "Darwin")
                foreach(_build_protobuf_macos_packages_prefix
                        /opt/local
                        /opt/homebrew
                        /usr/local)
                    if(EXISTS ${_build_protobuf_macos_packages_prefix}/bin/glibtoolize AND
                            EXISTS ${_build_protobuf_macos_packages_prefix}/bin/aclocal)
                        set(MACOS_PACKAGES_PREFIX
                            "${_build_protobuf_macos_packages_prefix}"
                        )
                        break()
                    endif()
                endforeach()
                unset(_build_protobuf_macos_packages_prefix)
            endif()
            unset(_build_protobuf_package_system_name)
        endif()
    endif()
endmacro()

macro(_build_protobuf_warn_explicit_version_downgrade reason)
    if(DEFINED BUILD_PROTOBUF_VERSION AND
            NOT BUILD_PROTOBUF_VERSION_DOWNGRADE_WARNED AND
            NOT BUILD_PROTOBUF_VERSION VERSION_LESS "22.0")
        message(WARNING
            "BUILD_PROTOBUF_VERSION was set to ${BUILD_PROTOBUF_VERSION}, "
            "but that version cannot be built with the selected bundled "
            "Protocol Buffers fallback${reason}. Falling back to Protocol "
            "Buffers 21.12."
        )
        set(BUILD_PROTOBUF_VERSION_DOWNGRADE_WARNED TRUE)
    endif()
endmacro()

macro(_build_protobuf_check_absl_prefix_path)
    set(BUILD_PROTOBUF_ABSL_PREFIX_PATH_USABLE OFF)
    foreach(_build_protobuf_prefix ${CMAKE_PREFIX_PATH})
        foreach(_build_protobuf_absl_config_dir
                "${_build_protobuf_prefix}/lib/cmake/absl"
                "${_build_protobuf_prefix}/lib64/cmake/absl"
                "${_build_protobuf_prefix}/share/absl")
            if(EXISTS "${_build_protobuf_absl_config_dir}/abslConfig.cmake")
                set(BUILD_PROTOBUF_ABSL_PREFIX_PATH_USABLE ON)
                break()
            endif()
        endforeach()
        if(BUILD_PROTOBUF_ABSL_PREFIX_PATH_USABLE)
            break()
        endif()
    endforeach()
    unset(_build_protobuf_absl_config_dir)
    unset(_build_protobuf_prefix)
endmacro()

macro(_build_protobuf_check_absl_external_discoverable)
    set(BUILD_PROTOBUF_ABSL_EXTERNAL_DISCOVERABLE OFF)

    if(DEFINED absl_DIR AND
            NOT "${absl_DIR}" STREQUAL "" AND
            NOT "${absl_DIR}" MATCHES "-NOTFOUND$" AND
            EXISTS "${absl_DIR}/abslConfig.cmake")
        set(BUILD_PROTOBUF_ABSL_EXTERNAL_DISCOVERABLE ON)
    elseif(CMAKE_PREFIX_PATH)
        _build_protobuf_check_absl_prefix_path()
        if(BUILD_PROTOBUF_ABSL_PREFIX_PATH_USABLE)
            set(BUILD_PROTOBUF_ABSL_EXTERNAL_DISCOVERABLE ON)
        endif()
    endif()

    if(absl_FOUND AND NOT BUILD_PROTOBUF_ABSL_EXTERNAL_DISCOVERABLE)
        message(STATUS
            "Abseil is available only as in-tree CMake targets, or no "
            "absl package configuration path is known. The separate "
            "Protocol Buffers build cannot use parent-project targets."
        )
        unset(absl_FOUND CACHE)
        set(absl_FOUND OFF)
    endif()
endmacro()

macro(_build_protobuf_select_backend)
    set(BUILD_PROTOBUF_USE_LEGACY ON)

    if((NOT CMAKE_CROSSCOMPILING) AND
            (NOT CMAKE_VERSION VERSION_LESS "3.13"))
        if(BUILD_PROTOBUF_FIND_ABSL)
            if(NOT DEFINED absl_FOUND)
                find_package(absl QUIET)
            endif()
        endif()

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

        if(absl_FOUND)
            _build_protobuf_check_absl_external_discoverable()
        endif()

        if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND
                (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0"))
            unset(absl_FOUND CACHE)
            set(absl_FOUND OFF)
        endif()

        if(absl_FOUND)
            set(BUILD_PROTOBUF_USE_LEGACY OFF)
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
            if(NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND
                    (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8.0.0"))
                message(STATUS "The Abseil library (https://github.com/abseil/abseil-cpp) >= v20230117 is required to be installed before building Protocol Buffers >22.x on the fly.")
            endif()
            message(STATUS " Instead, Protocol Buffers v21.12 will be built, which does not require Abseil.")
            _build_protobuf_warn_explicit_version_downgrade(
                " because compatible Abseil is unavailable"
            )
            set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "21.12")
        endif()
    endif()

    if(BUILD_PROTOBUF_USE_LEGACY)
        if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "22.0")
            _build_protobuf_warn_explicit_version_downgrade(
                " because the legacy build path was selected"
            )
            set(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION "21.12")
        endif()
    endif()
endmacro()

macro(_build_protobuf_find_host_protoc)
    if(BUILD_PROTOBUF_HOST_PROTOC)
        set(PROTOC_EXECUTABLE "${BUILD_PROTOBUF_HOST_PROTOC}")
    else()
        find_program(PROTOC_EXECUTABLE
            NAMES protoc
            NO_CMAKE_FIND_ROOT_PATH
        )
        if(NOT PROTOC_EXECUTABLE)
            find_program(PROTOC_EXECUTABLE
                NAMES protoc
                PATHS
                    /usr/bin
                    /usr/local/bin
                NO_CMAKE_FIND_ROOT_PATH
            )
        endif()
    endif()

    if(NOT PROTOC_EXECUTABLE)
        message(FATAL_ERROR
            "Please install the Protocol Buffers compiler for the host "
            "machine, or set BUILD_PROTOBUF_HOST_PROTOC to its path."
        )
    endif()

    set(BUILD_PROTOBUF_HOST_PROTOC "${PROTOC_EXECUTABLE}")
endmacro()

macro(_build_protobuf_set_legacy_configure_command)
    set(PROTOBUF_CONFIGURE_COMMAND
        ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/configure
        --prefix=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    )

    if(CMAKE_CROSSCOMPILING)
        _build_protobuf_find_host_protoc()

        set(_build_protobuf_cross_host "")
        if(BUILD_PROTOBUF_CROSS_HOST)
            set(_build_protobuf_cross_host "${BUILD_PROTOBUF_CROSS_HOST}")
        elseif(DEFINED ENV{OECORE_TARGET_ARCH})
            set(_build_protobuf_cross_host "$ENV{OECORE_TARGET_ARCH}")
        elseif(CMAKE_C_COMPILER_TARGET)
            set(_build_protobuf_cross_host "${CMAKE_C_COMPILER_TARGET}")
        elseif(CMAKE_CXX_COMPILER_TARGET)
            set(_build_protobuf_cross_host "${CMAKE_CXX_COMPILER_TARGET}")
        endif()

        if(_build_protobuf_cross_host)
            list(APPEND PROTOBUF_CONFIGURE_COMMAND
                --host=${_build_protobuf_cross_host}
            )
        endif()
        list(APPEND PROTOBUF_CONFIGURE_COMMAND
            --with-protoc=${BUILD_PROTOBUF_HOST_PROTOC}
        )
        unset(_build_protobuf_cross_host)
    endif()
endmacro()

macro(_build_protobuf_check_legacy_tools)
    set(_build_protobuf_tool_system_name "${CMAKE_SYSTEM_NAME}")
    if(CMAKE_CROSSCOMPILING AND CMAKE_HOST_SYSTEM_NAME)
        set(_build_protobuf_tool_system_name "${CMAKE_HOST_SYSTEM_NAME}")
    endif()

    if("${_build_protobuf_tool_system_name}" MATCHES "Linux|kFreeBSD|GNU")
        if(NOT EXISTS "/usr/bin/libtoolize")
            message(" libtool has not been found.")
            message(" You can try to install it by typing:")
            if("${LINUX_DISTRIBUTION}" MATCHES "Fedora" OR
                    "${LINUX_DISTRIBUTION}" MATCHES "Red Hat")
                message(" sudo yum groupinstall 'Development Tools'")
            elseif("${LINUX_DISTRIBUTION}" MATCHES "openSUSE")
                message(" sudo zypper install libtoool")
            else()
                message(" sudo apt install libtool")
            endif()
            message(FATAL_ERROR
                "libtool is required to build Protocol Buffers from source"
            )
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
            if("${LINUX_DISTRIBUTION}" MATCHES "Fedora" OR
                    "${LINUX_DISTRIBUTION}" MATCHES "Red Hat")
                message(" sudo yum groupinstall 'Development Tools'")
            elseif("${LINUX_DISTRIBUTION}" MATCHES "openSUSE")
                message(" sudo zypper install automake")
            else()
                message(" sudo apt install automake")
            endif()
            message(FATAL_ERROR
                "aclocal is required to build Protocol Buffers from source"
            )
        endif()
    endif()

    if("${_build_protobuf_tool_system_name}" MATCHES "Darwin")
        if((NOT EXISTS /usr/local/bin/glibtoolize AND
                NOT EXISTS ${MACOS_PACKAGES_PREFIX}/bin/glibtoolize) OR
                (NOT EXISTS /usr/local/bin/aclocal AND
                NOT EXISTS ${MACOS_PACKAGES_PREFIX}/bin/aclocal))
            message(" libtool/automake tools have not been found.")
            message(" You can try to install them by typing:")
            message(" 'sudo port install libtool automake', if you use Macports, or 'brew install libtool automake', if you use Homebrew")
            message(FATAL_ERROR
                "libtool/automake tools are required to build Protocol Buffers from source"
            )
        endif()

        if(CMAKE_GENERATOR STREQUAL Xcode)
            if(EXISTS ${MACOS_PACKAGES_PREFIX}/bin/glibtoolize OR
                    EXISTS ${MACOS_PACKAGES_PREFIX}/bin/aclocal)
                if(NOT EXISTS /usr/local/bin/glibtoolize OR
                        NOT EXISTS /usr/local/bin/aclocal)
                    message(" WARNING: libtool/automake binaries cannot be found by Xcode. Please do:")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/glibtoolize /usr/local/bin/")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/aclocal /usr/local/bin/")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/autom4te /usr/local/bin/")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/automake /usr/local/bin/")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/autoconf /usr/local/bin/")
                    message("sudo ln -s ${MACOS_PACKAGES_PREFIX}/bin/autoreconf /usr/local/bin/")
                    message(FATAL_ERROR
                        "libtool/automake tools cannot be found by Xcode"
                    )
                endif()
            endif()
        endif()
    endif()

    unset(_build_protobuf_tool_system_name)
endmacro()

macro(_build_protobuf_add_legacy_external_project)
    _build_protobuf_set_legacy_configure_command()

    set(PROTOBUF_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM})
    if(PROTOBUF_MAKE_PROGRAM MATCHES "ninja" OR
            CMAKE_GENERATOR STREQUAL Xcode)
        find_program(MAKE_EXECUTABLE make
            PATHS
                /usr/bin
                /usr/local/bin
        )
        if(NOT MAKE_EXECUTABLE)
            message(FATAL_ERROR
                "make is required to build Protocol Buffers from source."
            )
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
            CONFIGURE_COMMAND ${PROTOBUF_CONFIGURE_COMMAND}
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
            CONFIGURE_COMMAND ${PROTOBUF_CONFIGURE_COMMAND}
            BUILD_COMMAND ${PROTOBUF_MAKE_PROGRAM} ${PROTOBUF_PARALLEL_BUILD}
            INSTALL_COMMAND ${PROTOBUF_MAKE_PROGRAM} DESTDIR= install
            BUILD_BYPRODUCTS ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
                ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc
        )
    endif()
endmacro()

macro(_build_protobuf_check_legacy_libatomic)
    set(GNSSSDR_PROTOBUF_LIBATOMIC "")
    if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") OR
            (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
        include(CheckCXXSourceCompiles)
        set(GNSSSDR_ATOMIC_TEST_SOURCE
"#include <stdint.h>

int main()
{
    uint64_t value = 0;
    __atomic_fetch_add(&value, 1, __ATOMIC_SEQ_CST);
    return (int)value;
}
")
        set(_GNSSSDR_CMAKE_REQUIRED_LIBRARIES_SAVED
            "${CMAKE_REQUIRED_LIBRARIES}"
        )
        set(CMAKE_REQUIRED_LIBRARIES "")
        check_cxx_source_compiles(
            "${GNSSSDR_ATOMIC_TEST_SOURCE}"
            GNSSSDR_64BIT_ATOMICS_WITHOUT_LIBATOMIC
        )
        if(NOT GNSSSDR_64BIT_ATOMICS_WITHOUT_LIBATOMIC)
            set(CMAKE_REQUIRED_LIBRARIES atomic)
            check_cxx_source_compiles(
                "${GNSSSDR_ATOMIC_TEST_SOURCE}"
                GNSSSDR_64BIT_ATOMICS_WITH_LIBATOMIC
            )
            if(GNSSSDR_64BIT_ATOMICS_WITH_LIBATOMIC)
                set(GNSSSDR_PROTOBUF_LIBATOMIC atomic)
            else()
                set(CMAKE_REQUIRED_LIBRARIES
                    "${_GNSSSDR_CMAKE_REQUIRED_LIBRARIES_SAVED}"
                )
                unset(_GNSSSDR_CMAKE_REQUIRED_LIBRARIES_SAVED)
                unset(GNSSSDR_ATOMIC_TEST_SOURCE)
                message(FATAL_ERROR
                    "64-bit atomic operations require libatomic, "
                    "but linking with -latomic failed."
                )
            endif()
        endif()
        set(CMAKE_REQUIRED_LIBRARIES
            "${_GNSSSDR_CMAKE_REQUIRED_LIBRARIES_SAVED}"
        )
        unset(_GNSSSDR_CMAKE_REQUIRED_LIBRARIES_SAVED)
        unset(GNSSSDR_ATOMIC_TEST_SOURCE)
    endif()
endmacro()

macro(_build_protobuf_create_legacy_targets)
    if(CMAKE_CROSSCOMPILING)
        _build_protobuf_find_host_protoc()
    else()
        set(BUILD_PROTOBUF_TARGET_PROTOC
            "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc${CMAKE_EXECUTABLE_SUFFIX}"
        )
    endif()

    if(NOT TARGET protobuf::protoc)
        add_executable(protobuf::protoc IMPORTED GLOBAL)
    endif()
    add_dependencies(
        protobuf::protoc
        protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    )
    unset(Protobuf_PROTOC_EXECUTABLE)
    if(CMAKE_CROSSCOMPILING)
        set(PROTOBUF_PROTOC_EXECUTABLE "${BUILD_PROTOBUF_HOST_PROTOC}")
    else()
        set(PROTOBUF_PROTOC_EXECUTABLE "${BUILD_PROTOBUF_TARGET_PROTOC}")
    endif()
    set_target_properties(protobuf::protoc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${PROTOBUF_PROTOC_EXECUTABLE}"
        INTERFACE_LINK_LIBRARIES "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protoc${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )

    file(MAKE_DIRECTORY
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include
    )

    if(NOT TARGET protobuf::libprotobuf)
        add_library(protobuf::libprotobuf STATIC IMPORTED GLOBAL)
    endif()
    add_dependencies(
        protobuf::libprotobuf
        protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
    )

    _build_protobuf_check_legacy_libatomic()

    set(GNSSSDR_PROTOBUF_STATIC_LIBRARY
        "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}"
    )
    set(GNSSSDR_PROTOBUF_INTERFACE_LINK_LIBRARIES
        "${GNSSSDR_PROTOBUF_STATIC_LIBRARY}"
    )
    if(GNSSSDR_PROTOBUF_LIBATOMIC)
        message(STATUS
            "Linking bundled static Protocol Buffers with libatomic"
        )
        list(APPEND GNSSSDR_PROTOBUF_INTERFACE_LINK_LIBRARIES
            "${GNSSSDR_PROTOBUF_LIBATOMIC}"
        )
    endif()

    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GNSSSDR_PROTOBUF_STATIC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES
            "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include"
        INTERFACE_LINK_LIBRARIES
            "${GNSSSDR_PROTOBUF_INTERFACE_LINK_LIBRARIES}"
    )

    if(CMAKE_VERSION VERSION_LESS "3.10")
        set(Protobuf_PROTOC_EXECUTABLE ${PROTOBUF_PROTOC_EXECUTABLE})
    endif()
endmacro()

macro(_build_protobuf_add_legacy)
    _build_protobuf_check_legacy_tools()
    _build_protobuf_add_legacy_external_project()
    _build_protobuf_create_legacy_targets()
endmacro()

macro(_build_protobuf_find_zlib)
    if(NOT ZLIB_FOUND)
        find_package(ZLIB)
        set_package_properties(ZLIB PROPERTIES
            URL "https://www.zlib.net/"
            PURPOSE "Enables gzip stream support in Protocol Buffers."
            TYPE RECOMMENDED
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
    endif()

    if(ZLIB_FOUND)
        set(ZLIB_LIBRARIES_ ${ZLIB_LIBRARIES})
        set(USE_ZLIB_ -Dprotobuf_WITH_ZLIB=ON)
    else()
        set(ZLIB_LIBRARIES_ "")
        set(USE_ZLIB_ -Dprotobuf_WITH_ZLIB=OFF)
    endif()
endmacro()

macro(_build_protobuf_set_modern_options)
    include(GNUInstallDirs)

    if(CMAKE_VERSION VERSION_LESS "3.10")
        message(FATAL_ERROR
            "Building bundled Protocol Buffers requires CMake >= 3.10. "
            "Use a system Protobuf package with this CMake version."
        )
    endif()

    if((NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0") AND
            CMAKE_VERSION VERSION_LESS "3.16")
        message(FATAL_ERROR
            "Building bundled Protocol Buffers ${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION} "
            "requires CMake >= 3.16. Use a system Protobuf package, or select "
            "an older bundled Protobuf version."
        )
    endif()

    _build_protobuf_find_zlib()

    set(UTF8_LIBRARIES
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_validity${CMAKE_STATIC_LIBRARY_SUFFIX}
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}utf8_range${CMAKE_STATIC_LIBRARY_SUFFIX}
    )

    set(ABSL_DIR_OPTION_ "")
    if(DEFINED absl_DIR)
        set(ABSL_DIR_OPTION_ "-Dabsl_DIR=${absl_DIR}")
    endif()

    set(GNSSSDR_ABSL_PROVIDER_ "")
    if(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
        set(GNSSSDR_ABSL_PROVIDER_ "-Dprotobuf_ABSL_PROVIDER=package")
    else()
        set(GNSSSDR_ABSL_PROVIDER_ "-Dprotobuf_LOCAL_DEPENDENCIES_ONLY=ON")
    endif()

    if(NOT DEFINED PROTOBUF_PATCH_COMMAND)
        set(PROTOBUF_PATCH_COMMAND "")
    endif()

    set(PROTOBUF_CXX_STANDARD_ "14")
    if(NOT GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
        set(PROTOBUF_CXX_STANDARD_ "17")
    endif()

    set(PROTOBUF_LIBPROTOBUF_
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX}
    )
    set(PROTOBUF_LIBPROTOBUF_DEBUG_
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}protobufd${CMAKE_STATIC_LIBRARY_SUFFIX}
    )
    set(PROTOBUF_PROTOC_EXECUTABLE_
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/bin/protoc${CMAKE_EXECUTABLE_SUFFIX}
    )
    set(PROTOBUF_BUILD_BYPRODUCTS_
        ${PROTOBUF_PROTOC_EXECUTABLE_}
        ${UTF8_LIBRARIES}
    )

    if(CMAKE_CONFIGURATION_TYPES)
        list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
            ${PROTOBUF_LIBPROTOBUF_}
            ${PROTOBUF_LIBPROTOBUF_DEBUG_}
        )
    elseif(CMAKE_BUILD_TYPE MATCHES "^(Debug|NoOptWithASM|Coverage|ASAN)$")
        list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
            ${PROTOBUF_LIBPROTOBUF_DEBUG_}
        )
    else()
        list(APPEND PROTOBUF_BUILD_BYPRODUCTS_
            ${PROTOBUF_LIBPROTOBUF_}
        )
    endif()

    set(CMAKE_PREFIX_PATH_OPTION_ "")
    if(CMAKE_PREFIX_PATH)
        string(REPLACE ";" "|" CMAKE_PREFIX_PATH_EXTERNAL_
            "${CMAKE_PREFIX_PATH}"
        )
        set(CMAKE_PREFIX_PATH_OPTION_
            "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_EXTERNAL_}"
        )
    endif()
endmacro()

macro(_build_protobuf_add_modern_external_project)
    ExternalProject_Add(protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        PREFIX ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
        GIT_TAG v${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        GIT_PROGRESS ON
        UPDATE_COMMAND ""
        PATCH_COMMAND ${PROTOBUF_PATCH_COMMAND}
        SOURCE_DIR ${GNSSSDR_BINARY_DIR}/thirdparty/protobuf/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        BINARY_DIR ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        LIST_SEPARATOR |
        CMAKE_ARGS
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DCMAKE_CXX_STANDARD=${PROTOBUF_CXX_STANDARD_}
            -DCMAKE_CXX_STANDARD_REQUIRED=ON
            -DCMAKE_CXX_EXTENSIONS=OFF
            ${CMAKE_PREFIX_PATH_OPTION_}
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_BUILD_TYPE=$<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            -DCMAKE_CXX_VISIBILITY_PRESET=hidden
            -DCMAKE_VISIBILITY_INLINES_HIDDEN=1
            -DCMAKE_INSTALL_PREFIX=${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
            "-DCMAKE_INSTALL_LIBDIR:PATH=${CMAKE_INSTALL_LIBDIR}"
            -Dprotobuf_BUILD_TESTS=OFF
            ${ABSL_DIR_OPTION_}
            ${GNSSSDR_ABSL_PROVIDER_}
            ${USE_ZLIB_}
        BUILD_COMMAND ${CMAKE_COMMAND}
            "--build" "${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}"
            "--config" $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>$<$<CONFIG:RelWithDebInfo>:RelWithDebInfo>$<$<CONFIG:MinSizeRel>:MinSizeRel>$<$<CONFIG:NoOptWithASM>:Debug>$<$<CONFIG:Coverage>:Debug>$<$<CONFIG:O2WithASM>:RelWithDebInfo>$<$<CONFIG:O3WithASM>:RelWithDebInfo>$<$<CONFIG:ASAN>:Debug>
            "--target" install
        BUILD_BYPRODUCTS ${PROTOBUF_BUILD_BYPRODUCTS_}
        INSTALL_COMMAND ""
    )
endmacro()

macro(_build_protobuf_codesign_modern_protoc)
    if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
        find_program(CODESIGN_EXECUTABLE codesign)
        if(CODESIGN_EXECUTABLE)
            ExternalProject_Add_Step(
                protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
                codesign_protoc
                COMMAND ${CODESIGN_EXECUTABLE}
                    --force
                    --sign -
                    ${PROTOBUF_PROTOC_EXECUTABLE_}
                DEPENDEES build
                COMMENT "Ad-hoc signing bundled protoc executable"
                VERBATIM
            )
        endif()
    endif()
endmacro()

macro(_build_protobuf_set_modern_extra_libraries)
    set(PROTOBUF_EXTRA_LIBRARIES_ "")
    if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
        find_library(GNSSSDR_COREFOUNDATION_LIBRARY CoreFoundation)
        if(GNSSSDR_COREFOUNDATION_LIBRARY)
            list(APPEND PROTOBUF_EXTRA_LIBRARIES_
                ${GNSSSDR_COREFOUNDATION_LIBRARY}
            )
        endif()
    endif()

    set(PROTOBUF_ABSL_USED_TARGETS
        absl::absl_check
        absl::absl_log
        absl::algorithm
        absl::base
        absl::bind_front
        absl::bits
        absl::btree
        absl::cleanup
        absl::cord
        absl::core_headers
        absl::debugging
        absl::die_if_null
        absl::dynamic_annotations
        absl::flags
        absl::flat_hash_map
        absl::flat_hash_set
        absl::function_ref
        absl::hash
        absl::layout
        absl::log_initialize
        absl::log_severity
        absl::memory
        absl::node_hash_map
        absl::node_hash_set
        absl::optional
        absl::span
        absl::status
        absl::statusor
        absl::strings
        absl::synchronization
        absl::time
        absl::type_traits
        absl::utility
        absl::variant
    )

    if(GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION VERSION_LESS "30.0")
        foreach(_gnsssdr_absl_target ${PROTOBUF_ABSL_USED_TARGETS})
            if(NOT TARGET ${_gnsssdr_absl_target})
                message(FATAL_ERROR
                    "Required Abseil target ${_gnsssdr_absl_target} was not found. "
                    "Install a compatible abseil-cpp package before building bundled "
                    "Protocol Buffers ${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}."
                )
            endif()
        endforeach()
        unset(_gnsssdr_absl_target)
    endif()
endmacro()

macro(_build_protobuf_create_modern_targets)
    file(MAKE_DIRECTORY
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include
    )
    file(MAKE_DIRECTORY
        ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/${CMAKE_INSTALL_LIBDIR}
    )

    _build_protobuf_set_modern_extra_libraries()

    if(NOT TARGET protobuf::libprotobuf)
        add_library(protobuf::libprotobuf STATIC IMPORTED GLOBAL)
        add_dependencies(
            protobuf::libprotobuf
            protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        )
        set_target_properties(protobuf::libprotobuf PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_CONFIGURATIONS "None;Debug;Release;RelWithDebInfo;MinSizeRel"
            MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
            MAP_IMPORTED_CONFIG_COVERAGE Debug
            MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_ASAN Debug
            IMPORTED_LOCATION ${PROTOBUF_LIBPROTOBUF_}
            IMPORTED_LOCATION_NONE ${PROTOBUF_LIBPROTOBUF_}
            IMPORTED_LOCATION_DEBUG ${PROTOBUF_LIBPROTOBUF_DEBUG_}
            IMPORTED_LOCATION_RELEASE ${PROTOBUF_LIBPROTOBUF_}
            IMPORTED_LOCATION_RELWITHDEBINFO ${PROTOBUF_LIBPROTOBUF_}
            IMPORTED_LOCATION_MINSIZEREL ${PROTOBUF_LIBPROTOBUF_}
            INTERFACE_INCLUDE_DIRECTORIES ${GNSSSDR_BINARY_DIR}/protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}/include
            INTERFACE_LINK_LIBRARIES "${ZLIB_LIBRARIES_};${PROTOBUF_EXTRA_LIBRARIES_};${UTF8_LIBRARIES};${PROTOBUF_ABSL_USED_TARGETS}"
        )
    endif()

    if(NOT TARGET protobuf::protoc)
        add_executable(protobuf::protoc IMPORTED GLOBAL)
        add_dependencies(
            protobuf::protoc
            protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
        )
        set(Protobuf_PROTOC_EXECUTABLE "${PROTOBUF_PROTOC_EXECUTABLE_}")
        set(PROTOBUF_PROTOC_EXECUTABLE "${PROTOBUF_PROTOC_EXECUTABLE_}")
        set_target_properties(protobuf::protoc PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            MAP_IMPORTED_CONFIG_NOOPTWITHASM Debug
            MAP_IMPORTED_CONFIG_COVERAGE Debug
            MAP_IMPORTED_CONFIG_O2WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_O3WITHASM RelWithDebInfo
            MAP_IMPORTED_CONFIG_ASAN Debug
            IMPORTED_LOCATION ${PROTOBUF_PROTOC_EXECUTABLE_}
        )
    endif()
endmacro()

macro(_build_protobuf_cleanup_modern)
    unset(GNSSSDR_ABSL_PROVIDER_)
    unset(USE_ZLIB_)
    unset(ZLIB_LIBRARIES_)
    unset(ABSL_DIR_OPTION_)
    unset(PROTOBUF_EXTRA_LIBRARIES_)
    unset(PROTOBUF_CXX_STANDARD_)
    unset(PROTOBUF_LIBPROTOBUF_)
    unset(PROTOBUF_LIBPROTOBUF_DEBUG_)
    unset(PROTOBUF_PROTOC_EXECUTABLE_)
    unset(PROTOBUF_BUILD_BYPRODUCTS_)
    unset(CMAKE_PREFIX_PATH_OPTION_)
    if(DEFINED CMAKE_PREFIX_PATH_EXTERNAL_)
        unset(CMAKE_PREFIX_PATH_EXTERNAL_)
    endif()
endmacro()

macro(_build_protobuf_add_modern)
    _build_protobuf_set_modern_options()
    _build_protobuf_add_modern_external_project()
    _build_protobuf_codesign_modern_protoc()
    _build_protobuf_create_modern_targets()
    _build_protobuf_cleanup_modern()
endmacro()

macro(_build_protobuf_set_standalone_outputs)
    if(NOT BUILD_PROTOBUF_PRESERVE_GNSSSDR_BEHAVIOR)
        set(Protobuf_FOUND TRUE)
        if(NOT Protobuf_VERSION OR "${Protobuf_VERSION}" VERSION_EQUAL "0.0.0")
            set(Protobuf_VERSION "${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}")
        endif()
        if(PROTOBUF_PROTOC_EXECUTABLE AND NOT Protobuf_PROTOC_EXECUTABLE)
            set(Protobuf_PROTOC_EXECUTABLE "${PROTOBUF_PROTOC_EXECUTABLE}")
        endif()
    endif()
endmacro()

_build_protobuf_initialize()
_build_protobuf_select_backend()

if(BUILD_PROTOBUF_USE_LEGACY)
    _build_protobuf_add_legacy()
else()
    _build_protobuf_add_modern()
endif()

_build_protobuf_set_standalone_outputs()

if(NOT COMMAND protobuf_generate_cpp)
    function(protobuf_generate_cpp SRCS HDRS)
        if(NOT ARGN)
            message(FATAL_ERROR
                "Error: protobuf_generate_cpp() called without any proto files"
            )
        endif()

        set(_protobuf_protoc_command "")
        if(TARGET protobuf::protoc)
            get_target_property(_protobuf_protoc_command
                protobuf::protoc
                IMPORTED_LOCATION
            )
        endif()

        if(NOT _protobuf_protoc_command)
            if(Protobuf_PROTOC_EXECUTABLE)
                set(_protobuf_protoc_command
                    "${Protobuf_PROTOC_EXECUTABLE}"
                )
            elseif(PROTOBUF_PROTOC_EXECUTABLE)
                set(_protobuf_protoc_command
                    "${PROTOBUF_PROTOC_EXECUTABLE}"
                )
            else()
                message(FATAL_ERROR
                    "Error: protobuf_generate_cpp() could not find protoc"
                )
            endif()
        endif()

        set(_protobuf_protoc_depends "")
        if(TARGET protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION})
            set(_protobuf_protoc_depends
                protobuf-${GNSSSDR_PROTOCOLBUFFERS_LOCAL_VERSION}
            )
        endif()

        set(_protobuf_generated_srcs "")
        set(_protobuf_generated_hdrs "")
        foreach(_protobuf_proto ${ARGN})
            get_filename_component(_protobuf_abs_file
                "${_protobuf_proto}"
                ABSOLUTE
            )
            get_filename_component(_protobuf_abs_path
                "${_protobuf_abs_file}"
                DIRECTORY
            )
            get_filename_component(_protobuf_name_we
                "${_protobuf_proto}"
                NAME_WE
            )

            set(_protobuf_generated_src
                "${CMAKE_CURRENT_BINARY_DIR}/${_protobuf_name_we}.pb.cc"
            )
            set(_protobuf_generated_hdr
                "${CMAKE_CURRENT_BINARY_DIR}/${_protobuf_name_we}.pb.h"
            )
            set(_protobuf_include_args -I "${_protobuf_abs_path}")
            foreach(_protobuf_import_dir
                    ${PROTOBUF_IMPORT_DIRS}
                    ${Protobuf_IMPORT_DIRS})
                get_filename_component(_protobuf_import_dir_abs
                    "${_protobuf_import_dir}"
                    ABSOLUTE
                )
                list(APPEND _protobuf_include_args
                    -I "${_protobuf_import_dir_abs}"
                )
            endforeach()

            add_custom_command(
                OUTPUT
                    "${_protobuf_generated_src}"
                    "${_protobuf_generated_hdr}"
                COMMAND ${_protobuf_protoc_command}
                ARGS
                    ${_protobuf_include_args}
                    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
                    "${_protobuf_abs_file}"
                DEPENDS
                    "${_protobuf_abs_file}"
                    ${_protobuf_protoc_depends}
                COMMENT "Running C++ protocol buffer compiler on ${_protobuf_proto}"
                VERBATIM
            )

            list(APPEND _protobuf_generated_srcs
                "${_protobuf_generated_src}"
            )
            list(APPEND _protobuf_generated_hdrs
                "${_protobuf_generated_hdr}"
            )
        endforeach()

        set_source_files_properties(
            ${_protobuf_generated_srcs}
            ${_protobuf_generated_hdrs}
            PROPERTIES GENERATED TRUE
        )
        set(${SRCS} ${_protobuf_generated_srcs} PARENT_SCOPE)
        set(${HDRS} ${_protobuf_generated_hdrs} PARENT_SCOPE)
    endfunction()
endif()

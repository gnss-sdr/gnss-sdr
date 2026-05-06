# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

execute_process(
    COMMAND uname -r
    OUTPUT_VARIABLE DARWIN_VERSION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "^[0-9]+" DARWIN_VERSION "${DARWIN_VERSION}")

if(NOT "${DARWIN_VERSION}" STREQUAL "" AND
        "${DARWIN_VERSION}" VERSION_GREATER "19")
    execute_process(
        COMMAND sw_vers -productVersion
        RESULT_VARIABLE MACOS_VERSION_RESULT
        OUTPUT_VARIABLE MACOS_VERSION
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(MACOS_NAME "")
    set(MACOS_LICENSE_FILE
        "/System/Library/CoreServices/Setup Assistant.app/Contents/Resources/en.lproj/OSXSoftwareLicense.rtf"
    )

    if(EXISTS "${MACOS_LICENSE_FILE}")
        execute_process(
            COMMAND awk "/SOFTWARE LICENSE AGREEMENT FOR macOS/" "${MACOS_LICENSE_FILE}"
            RESULT_VARIABLE MACOS_LICENSE_RESULT
            OUTPUT_VARIABLE MACOS_LICENSE_LINE
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if("${MACOS_LICENSE_RESULT}" STREQUAL "0" AND
                NOT "${MACOS_LICENSE_LINE}" STREQUAL "")
            string(REGEX MATCH
                "macOS[^\n\r}\\\\]*"
                MACOS_NAME
                "${MACOS_LICENSE_LINE}"
            )
            string(STRIP "${MACOS_NAME}" MACOS_NAME)

            # The license heading may say, for instance, "macOS Tahoe 26".
            # Keep the adaptive marketing name, but avoid printing the major
            # version twice when appending sw_vers -productVersion below.
            string(REGEX REPLACE
                " [0-9]+$"
                ""
                MACOS_NAME
                "${MACOS_NAME}"
            )
        endif()
    endif()

    if("${MACOS_NAME}" STREQUAL "")
        execute_process(
            COMMAND sw_vers -productName
            RESULT_VARIABLE MACOS_NAME_RESULT
            OUTPUT_VARIABLE MACOS_NAME
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(NOT "${MACOS_NAME_RESULT}" STREQUAL "0" OR
                "${MACOS_NAME}" STREQUAL "")
            set(MACOS_NAME "macOS")
        endif()
    endif()

    if("${MACOS_VERSION_RESULT}" STREQUAL "0" AND
            NOT "${MACOS_VERSION}" STREQUAL "")
        set(MACOS_DISTRIBUTION
            "${MACOS_NAME} ${MACOS_VERSION} (${CMAKE_SYSTEM_PROCESSOR})"
        )
    else()
        set(MACOS_DISTRIBUTION
            "${MACOS_NAME} (${CMAKE_SYSTEM_PROCESSOR})"
        )
    endif()

    unset(MACOS_LICENSE_FILE)
    unset(MACOS_LICENSE_LINE)
    unset(MACOS_LICENSE_RESULT)
    unset(MACOS_NAME_RESULT)
    unset(MACOS_VERSION_RESULT)
elseif("${DARWIN_VERSION}" STREQUAL "19")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Catalina 10.15")
elseif("${DARWIN_VERSION}" STREQUAL "18")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Mojave 10.14")
elseif("${DARWIN_VERSION}" STREQUAL "17")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS High Sierra 10.13")
elseif("${DARWIN_VERSION}" STREQUAL "16")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Sierra 10.12")
elseif("${DARWIN_VERSION}" STREQUAL "15")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "Mac OS X 10.11 El Capitan")
elseif("${DARWIN_VERSION}" STREQUAL "14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "Mac OS X 10.10 Yosemite")
elseif("${DARWIN_VERSION}" STREQUAL "13")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION
        "com.apple.compilers.llvm.clang.1_0"
    )
    set(MACOS_DISTRIBUTION "Mac OS X 10.9 Mavericks")
elseif("${DARWIN_VERSION}" STREQUAL "12")
    set(MACOS_DISTRIBUTION "Mac OS X 10.8 Mountain Lion")
elseif("${DARWIN_VERSION}" STREQUAL "11")
    set(MACOS_DISTRIBUTION "Mac OS X 10.7 Lion")
elseif("${DARWIN_VERSION}" STREQUAL "10")
    set(MACOS_DISTRIBUTION "Mac OS X 10.6 Snow Leopard")
endif()

if(NOT MACOS_DISTRIBUTION)
    set(MACOS_DISTRIBUTION "macOS (Unknown version)")
endif()

set(MACOS_PACKAGES_PREFIX "")

# Detect if MacPorts is installed on this system; if so, return base path
# and version.
find_program(MACPORTS_EXECUTABLE port)
if(MACPORTS_EXECUTABLE)
    # "/opt/local/bin/port", so we get the parent directory.
    get_filename_component(MACPORTS_PREFIX "${MACPORTS_EXECUTABLE}" DIRECTORY)
    # "/opt/local/bin", so we get the parent directory.
    get_filename_component(MACPORTS_PREFIX "${MACPORTS_PREFIX}" DIRECTORY)
    execute_process(
        COMMAND "${MACPORTS_EXECUTABLE}" version
        RESULT_VARIABLE DETECT_MACPORTS_VERSION
        OUTPUT_VARIABLE MACPORTS_VERSION
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX MATCH
        "[0-9]+\\.[0-9]+\\.[0-9]+"
        MACPORTS_VERSION
        "${MACPORTS_VERSION}"
    )
    set(MACOS_PACKAGES_PREFIX "${MACPORTS_PREFIX}")
endif()

# Detect if Homebrew is installed on this system; if so, return base path
# and version.
find_program(HOMEBREW_EXECUTABLE brew)
if(HOMEBREW_EXECUTABLE)
    execute_process(
        COMMAND "${HOMEBREW_EXECUTABLE}" --prefix
        RESULT_VARIABLE DETECT_HOMEBREW
        OUTPUT_VARIABLE HOMEBREW_PREFIX
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if("${DETECT_HOMEBREW}" STREQUAL "0")
        execute_process(
            COMMAND "${HOMEBREW_EXECUTABLE}" --version
            RESULT_VARIABLE DETECT_HOMEBREW_VERSION
            OUTPUT_VARIABLE HOMEBREW_VERSION
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        string(REGEX MATCH
            "[0-9]+\\.[0-9]+\\.[0-9]+"
            HOMEBREW_VERSION
            "${HOMEBREW_VERSION}"
        )
        set(MACOS_PACKAGES_PREFIX "${HOMEBREW_PREFIX}")
    endif()
endif()

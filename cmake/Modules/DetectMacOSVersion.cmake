# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})

if(${DARWIN_VERSION} VERSION_GREATER "19")
    execute_process(COMMAND awk "/SOFTWARE LICENSE AGREEMENT FOR macOS/" "/System/Library/CoreServices/Setup Assistant.app/Contents/Resources/en.lproj/OSXSoftwareLicense.rtf" OUTPUT_VARIABLE macOS_NAME)
    if(macOS_NAME)
        string(REGEX MATCH "macOS*([^\n\r]*)" macOS_NAME ${macOS_NAME})
        string(REGEX REPLACE "macOS " "" macOS_NAME ${macOS_NAME})
        string(REGEX REPLACE ".$" "" macOS_NAME ${macOS_NAME})
        execute_process(COMMAND sw_vers -productVersion OUTPUT_VARIABLE macOS_VERSION)
        string(REGEX REPLACE "\n$" "" macOS_VERSION ${macOS_VERSION})
        set(MACOS_DISTRIBUTION "macOS ${macOS_NAME} ${macOS_VERSION} (${CMAKE_SYSTEM_PROCESSOR})")
    endif()
endif()

if(${DARWIN_VERSION} MATCHES "19")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++17")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Catalina 10.15")
endif()

if(${DARWIN_VERSION} MATCHES "18")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Mojave 10.14")
endif()

if(${DARWIN_VERSION} MATCHES "17")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS High Sierra 10.13")
endif()

if(${DARWIN_VERSION} MATCHES "16")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "macOS Sierra 10.12")
endif()

if(${DARWIN_VERSION} MATCHES "15")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "Mac OS X 10.11 El Capitan")
endif()

if(${DARWIN_VERSION} MATCHES "14")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(MACOS_DISTRIBUTION "Mac OS X 10.10 Yosemite")
endif()

if(${DARWIN_VERSION} MATCHES "13")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION="com.apple.compilers.llvm.clang.1_0")
    set(MACOS_DISTRIBUTION "Mac OS X 10.9 Mavericks")
endif()

if(${DARWIN_VERSION} MATCHES "12")
    set(MACOS_DISTRIBUTION "Mac OS X 10.8 Mountain Lion")
endif()

if(${DARWIN_VERSION} MATCHES "11")
    set(MACOS_DISTRIBUTION "Mac OS X 10.7 Lion")
endif()

if(${DARWIN_VERSION} MATCHES "10")
    set(MACOS_DISTRIBUTION "Mac OS X 10.6 Snow Leopard")
endif()

if(NOT MACOS_DISTRIBUTION)
    set(MACOS_DISTRIBUTION "macOS (Unknown version)")
endif()

set(MACOS_PACKAGES_PREFIX "")
# Detect if MacPorts is installed on this system; if so, return base path and version
execute_process(COMMAND which port RESULT_VARIABLE DETECT_MACPORTS OUTPUT_VARIABLE MACPORTS_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${DETECT_MACPORTS} EQUAL 0)
    # "/opt/local/bin/port", so we get the parent directory
    get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
    # "/opt/local/bin", so we get the parent directory
    get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
    execute_process(COMMAND port version RESULT_VARIABLE DETECT_MACPORTS_VERSION OUTPUT_VARIABLE MACPORTS_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" MACPORTS_VERSION "${MACPORTS_VERSION}")
    set(MACOS_PACKAGES_PREFIX ${MACPORTS_PREFIX})
endif()

# Detect if Homebrew is installed on this system; if so, return base path and version
execute_process(COMMAND brew --prefix RESULT_VARIABLE DETECT_HOMEBREW OUTPUT_VARIABLE HOMEBREW_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
if(${DETECT_HOMEBREW} EQUAL 0)
    execute_process(COMMAND brew --version RESULT_VARIABLE DETECT_HOMEBREW_VERSION OUTPUT_VARIABLE HOMEBREW_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" HOMEBREW_VERSION "${HOMEBREW_VERSION}")
    set(MACOS_PACKAGES_PREFIX ${HOMEBREW_PREFIX})
endif()

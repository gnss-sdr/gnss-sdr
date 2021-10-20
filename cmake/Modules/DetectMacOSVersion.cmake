# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
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

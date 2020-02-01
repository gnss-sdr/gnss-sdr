# Copyright (C) 2020 (see AUTHORS file for a list of contributors)
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

execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})

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

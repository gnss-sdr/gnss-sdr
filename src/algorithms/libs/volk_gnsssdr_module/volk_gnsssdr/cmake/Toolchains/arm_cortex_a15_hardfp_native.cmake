# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Toolchain file for building native on a ARM Cortex A8 w/ NEON
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=<this file> <source directory>
########################################################################
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_FLAGS "-march=armv7-a -mtune=cortex-a15 -mfpu=neon -mfloat-abi=hard" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE)  # same flags for C sources
set(CMAKE_ASM_FLAGS "${CMAKE_CXX_FLAGS} -g" CACHE STRING "" FORCE)  # same flags for asm sources

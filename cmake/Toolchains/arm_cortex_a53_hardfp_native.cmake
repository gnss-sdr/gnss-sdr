# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Toolchain file for building native on a ARM Cortex A53 w/ NEON
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=<this file> <source directory>
########################################################################
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_FLAGS "-march=armv8-a -mtune=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE)  # same flags for C sources
set(CMAKE_ASM_FLAGS "${CMAKE_CXX_FLAGS} -mthumb -g" CACHE STRING "" FORCE)  # same flags for asm sources

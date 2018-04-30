# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
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

##########################################################
# Toolchain file for Zynq-7000 devices
##########################################################

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_SYSROOT /home/carlesfernandez/binary)  ### POINT THIS TO YOUR ROOTFS

set(CMAKE_C_COMPILER    /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER  /usr/bin/arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH  ${CMAKE_SYSROOT} )
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(ZYNQ_FLAGS  "-march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a7")
set(CMAKE_ASM_FLAGS ${ZYNQ_FLAGS} CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS   ${ZYNQ_FLAGS} CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS ${ZYNQ_FLAGS} CACHE STRING "" FORCE)

set(CMAKE_LIBRARY_PATH ${CMAKE_SYSROOT}/usr/lib
                       ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SYSROOT}/usr CACHE STRING "" FORCE)

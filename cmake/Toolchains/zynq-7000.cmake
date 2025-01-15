# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

##########################################################
# Toolchain file for Zynq-7000 devices
##########################################################

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_SYSROOT /home/carlesfernandez/binary)  ### POINT THIS TO YOUR ROOTFS

set(CMAKE_C_COMPILER    /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER  /usr/bin/arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(ZYNQ_FLAGS
    "-march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a7"
)
set(CMAKE_ASM_FLAGS ${ZYNQ_FLAGS} CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS   ${ZYNQ_FLAGS} CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS ${ZYNQ_FLAGS} CACHE STRING "" FORCE)

set(CMAKE_LIBRARY_PATH
    ${CMAKE_SYSROOT}/usr/lib
    ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf
)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SYSROOT}/usr CACHE STRING "" FORCE)

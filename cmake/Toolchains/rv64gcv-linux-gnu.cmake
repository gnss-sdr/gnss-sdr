# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2011-2025  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

set(CMAKE_C_COMPILER $ENV{CC})
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER $ENV{CXX})

set(CMAKE_C_FLAGS "$ENV{CFLAGS} -march=rv64gcv" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS ${CMAKE_C_FLAGS} CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -g" CACHE STRING "" FORCE)

set(CMAKE_OBJCOPY
    ${RISCV64_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy
    CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL
    ${RISCV64_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size
    CACHE INTERNAL "size tool")

set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})

set(QEMU_VLEN $ENV{VLEN})
if(NOT QEMU_VLEN)
    set(QEMU_VLEN "128")
endif()

set(CMAKE_CROSSCOMPILING_EMULATOR "qemu-riscv64-static -L /usr/riscv64-linux-gnu/ -cpu rv64,zba=true,zbb=true,v=on,vlen=${QEMU_VLEN},rvv_ta_all_1s=on,rvv_ma_all_1s=on")

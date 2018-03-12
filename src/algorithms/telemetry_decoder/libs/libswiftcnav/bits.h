/*!
 * \file bits.h
 * \author Fergus Noble <fergus@swift-nav.com>
 *
 * -------------------------------------------------------------------------
 * This file was originally borrowed from libswiftnav
 * <https://github.com/swift-nav/libswiftnav>,
 * a portable C library implementing GNSS related functions and algorithms,
 * and then modified by J. Arribas and C. Fernandez
 *
 * Copyright (C) 2013, 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSWIFTNAV_BITS_H
#define LIBSWIFTNAV_BITS_H

#include "swift_common.h"

u8 parity(u32 x);
u32 getbitu(const u8 *buff, u32 pos, u8 len);
s32 getbits(const u8 *buff, u32 pos, u8 len);
void setbitu(u8 *buff, u32 pos, u32 len, u32 data);
void setbits(u8 *buff, u32 pos, u32 len, s32 data);
void bitcopy(void *dst, u32 dst_index,
    const void *src, u32 src_index, u32 count);
void bitshl(void *buf, u32 size, u32 shift);
u8 count_bits_u64(u64 v, u8 bv);
u8 count_bits_u32(u32 v, u8 bv);
u8 count_bits_u16(u16 v, u8 bv);
u8 count_bits_u8(u8 v, u8 bv);

#endif /* LIBSWIFTNAV_BITS_H */

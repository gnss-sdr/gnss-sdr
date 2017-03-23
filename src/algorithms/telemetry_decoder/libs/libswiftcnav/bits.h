/*
 * Copyright (C) 2013, 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
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

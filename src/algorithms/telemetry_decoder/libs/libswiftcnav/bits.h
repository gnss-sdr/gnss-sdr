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
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GNSS_SDR_BITS_H_
#define GNSS_SDR_BITS_H_

#include "swift_common.h"

uint8_t parity(uint32_t x);
uint32_t getbitu(const uint8_t *buff, uint32_t pos, uint8_t len);
int32_t getbits(const uint8_t *buff, uint32_t pos, uint8_t len);
void setbitu(uint8_t *buff, uint32_t pos, uint32_t len, uint32_t data);
void setbits(uint8_t *buff, uint32_t pos, uint32_t len, int32_t data);
void bitcopy(void *dst, uint32_t dst_index,
    const void *src, uint32_t src_index, uint32_t count);
void bitshl(void *buf, uint32_t size, uint32_t shift);
uint8_t count_bits_u64(uint64_t v, uint8_t bv);
uint8_t count_bits_u32(uint32_t v, uint8_t bv);
uint8_t count_bits_u16(uint16_t v, uint8_t bv);
uint8_t count_bits_u8(uint8_t v, uint8_t bv);

#endif /* GNSS_SDR_BITS_H_ */

/*!
 * \file bits.h
 * \brief Utilities for bit manipulation of the libswiftnav library
 * \author Fergus Noble <fergus@swift-nav.com>
 *
 * -----------------------------------------------------------------------------
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * This file was originally borrowed from libswiftnav
 * <https://github.com/swift-nav/libswiftnav>,
 * a portable C library implementing GNSS related functions and algorithms,
 * and then modified by J. Arribas and C. Fernandez
 *
 * Copyright (C) 2013, 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * SPDX-License-Identifier: LGPL-3.0-only
 *
 */

#ifndef GNSS_SDR_BITS_H
#define GNSS_SDR_BITS_H

#include "swift_common.h"

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libswiftcnav telemetry_decoder_libswiftcnav
 * Utilities for CNAV message decoding by Swift Navigation Inc.
 * \{ */


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


/** \} */
/** \} */
#endif /* GNSS_SDR_BITS_H_ */

/*!
 * \file edc.h
 * \brief Utilities for CRC computation of the libswiftnav library
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
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * SPDX-License-Identifier: LGPL-3.0-only
 *
 */


#ifndef GNSS_SDR_EDC_H
#define GNSS_SDR_EDC_H

#include "swift_common.h"

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libswiftcnav
 * \{ */


uint32_t crc24q(const uint8_t *buf, uint32_t len, uint32_t crc);
uint32_t crc24q_bits(uint32_t crc, const uint8_t *buf, uint32_t n_bits, bool invert);


/** \} */
/** \} */
#endif /* GNSS_SDR_EDC_H */

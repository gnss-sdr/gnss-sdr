/*!
 * \file swift_common.h
 * \brief Common definitions used throughout the libswiftnav library
 * \author Henry Hallam <henry@swift-nav.com>
 *         Fergus Noble <fergus@swift-nav.com>
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
 * Copyright (C) 2012 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *          Fergus Noble <fergus@swift-nav.com>
 *
 * SPDX-License-Identifier: LGPL-3.0-only
 *
 */


#ifndef GNSS_SDR_SWIFT_COMMON_H
#define GNSS_SDR_SWIFT_COMMON_H

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libswiftcnav
 * \{ */


#define ABS(x) ((x) < 0 ? -(x) : (x))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define CLAMP_DIFF(a, b) (MAX((a), (b)) - (b))

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>


/** \} */
/** \} */
#endif /* GNSS_SDR_SWIFT_COMMON_H */

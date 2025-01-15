/*!
 * \file cnav_msg.h
 * \brief Utilities for CNAV message manipulation of the libswiftnav library
 * \author Valeri Atamaniouk <valeri@swift-nav.com>
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
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * SPDX-License-Identifier: LGPL-3.0-only
 *
 */


#ifndef GNSS_SDR_CNAV_MSG_H
#define GNSS_SDR_CNAV_MSG_H

#include "fec.h"
#include "swift_common.h"
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libswiftcnav
 * \{ */


/** Size of the Viterbi decoder history. */
#define GPS_L2_V27_HISTORY_LENGTH_BITS 64
/** Bits to accumulate before decoding starts. */
#define GPS_L2C_V27_INIT_BITS (32)
/** Bits to decode at a time. */
#define GPS_L2C_V27_DECODE_BITS (32)
/** Bits in decoder tail. We ignore them. */
#define GPS_L2C_V27_DELAY_BITS (32)
/**
 * GPS CNAV message container.
 *
 * @sa cnav_msg_decoder_add_symbol
 */
typedef struct
{
    uint8_t prn;                                                       /**< SV PRN. 0..31 */
    uint8_t msg_id;                                                    /**< Message id. 0..31 */
    uint32_t tow;                                                      /**< GPS ToW in 6-second units. Multiply to 6 to get seconds. */
    bool alert;                                                        /**< CNAV message alert flag */
    uint8_t raw_msg[GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS]; /**< RAW MSG for GNSS-SDR */
} cnav_msg_t;

/**
 * GPS CNAV decoder component.
 * This component controls symbol decoding string.
 *
 * @sa cnav_msg_decoder_t
 */
typedef struct
{
    v27_t dec; /**< Viterbi block decoder object */
    v27_decision_t decisions[GPS_L2_V27_HISTORY_LENGTH_BITS];
    /**< Decision graph */
    unsigned char symbols[(GPS_L2C_V27_INIT_BITS + GPS_L2C_V27_DECODE_BITS) * 2];
    /**< Symbol buffer */
    size_t n_symbols; /**< Count of symbols in the symbol buffer */
    unsigned char decoded[GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS];
    /**< Decode buffer */
    size_t n_decoded;   /**< Number of bits in the decode buffer */
    bool preamble_seen; /**< When true, the decode buffer is aligned on
                         *   preamble. */
    bool invert;        /**< When true, indicates the bits are inverted */
    bool message_lock;  /**< When true, indicates the message boundary
                         *   is found. */
    bool crc_ok;        /**< Flag that the last message had good CRC */
    size_t n_crc_fail;  /**< Counter for CRC failures */
    bool init;          /**< Initial state flag. When true, initial bits
                         *   do not produce output. */
} cnav_v27_part_t;

/**
 * GPS CNAV message lock and decoder object.
 *
 * Decoder uses two Viterbi decoder objects to ensure the lock is acquired when
 * the input symbol phase is not known.
 */
typedef struct
{
    cnav_v27_part_t part1; /**< Decoder for odd symbol pairs */
    cnav_v27_part_t part2; /**< Decoder for even symbol pairs */
} cnav_msg_decoder_t;

const v27_poly_t *cnav_msg_decoder_get_poly(void);
void cnav_msg_decoder_init(cnav_msg_decoder_t *dec);
bool cnav_msg_decoder_add_symbol(cnav_msg_decoder_t *dec,
    unsigned char symbol,
    cnav_msg_t *msg,
    uint32_t *delay);

/** \} */
/** \} */
#endif /* GNSS_SDR_CNAV_MSG_H_ */

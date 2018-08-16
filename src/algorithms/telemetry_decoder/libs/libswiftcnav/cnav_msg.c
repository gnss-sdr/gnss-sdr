/*!
 * \file cnav_msg.c
 * \author Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * -------------------------------------------------------------------------
 * This file was originally borrowed from libswiftnav
 * <https://github.com/swift-nav/libswiftnav>,
 * a portable C library implementing GNSS related functions and algorithms,
 * and then modified by J. Arribas and C. Fernandez
 *
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
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


#include "edc.h"
#include "bits.h"
#include "cnav_msg.h"

#include <limits.h>
#include <string.h>

/** \defgroup GPS_L2 GPS L2
 * GPS L2 operations
 * \{ */
/** \defgroup gps_cnav_decoder Decoder
 * GPS L2C CNAV message decoding.
 * \{ */
/*
 * Block Viterbi decoding parameters.
 */
/** Viterbi decoder reversed polynomial A */
#define GPS_L2C_V27_POLY_A (0x4F) /* 0b01001111 - reversed 0171*/

/** Viterbi decoder reversed polynomial B */
#define GPS_L2C_V27_POLY_B (0x6D) /* 0b01101101 - reversed 0133 */
/*
 * GPS L2C message constants.
 */

/** GPS L2C preamble */
const u32 GPS_CNAV_PREAMBLE1 = 0x989A73u;
/** Inverted GPS L2C preamble */
const u32 GPS_CNAV_PREAMBLE2 = 0x010F054u;
/** GPS L2C preamble length in bits */
#define GPS_CNAV_PREAMBLE_LENGTH (8)
/** GPS L2C CNAV message length in bits */
#define GPS_CNAV_MSG_LENGTH (300)
/** GPS LC2 CNAV CRC length in bits */
#define GPS_CNAV_MSG_CRC_LENGTH (24)
/** GPS L2C CNAV message payload length in bits */
#define GPS_CNAV_MSG_DATA_LENGTH (GPS_CNAV_MSG_LENGTH - GPS_CNAV_MSG_CRC_LENGTH)
/** GPS L2C CNAV message lock detector threshold */
#define GPS_CNAV_LOCK_MAX_CRC_FAILS (10)

/**
 * Computes CRC-24Q from a CNAV message buffer.
 * CRC-24Q is computed for 274 bits. For a purpose of 8-bit alignment, the
 * message is assumed to be prepended with four zero bits.
 *
 * \param[in] part Decoder component with payload
 *
 * \return CRC-24Q value.
 *
 * \private
 */
static u32 _cnav_compute_crc(cnav_v27_part_t *part)
{
    u32 crc = crc24q_bits(0, part->decoded, GPS_CNAV_MSG_DATA_LENGTH,
        part->invert);

    return crc;
}


/**
 * Extracts CRC-24Q from a CNAV message buffer.
 * CRC-24Q value is the last 24 bits from 300 bits message buffer.
 *
 * \param[in] part Decoded component with payload.
 *
 * \return CRC24-Q value.
 *
 * \private
 */
static u32 _cnav_extract_crc(const cnav_v27_part_t *part)
{
    u32 crc = getbitu(part->decoded, GPS_CNAV_MSG_DATA_LENGTH,
        GPS_CNAV_MSG_CRC_LENGTH);
    if (part->invert)
        {
            crc ^= 0xFFFFFF;
        }
    return crc;
}


/**
 * Helper to rescan for preamble in the received buffer.
 * Occasionally there could be a false lock on message contents if it the
 * preamble sequence is encountered in the message body. For this case, the
 * function performs for a scan for a preamble with a different offset:
 * - When found, the preamble octet is moved into the head of the buffer.
 * - When not found, only 7 bits are left in the buffer.
 *
 * \param[in,out] part Decoded component.
 *
 * \return None
 *
 * \private
 */
static void _cnav_rescan_preamble(cnav_v27_part_t *part)
{
    part->preamble_seen = false;

    if (part->n_decoded > GPS_CNAV_PREAMBLE_LENGTH + 1)
        {
            size_t i = 0;
            size_t j = 0;
            for (i = 1, j = part->n_decoded - GPS_CNAV_PREAMBLE_LENGTH; i < j; ++i)
                {
                    u32 c = getbitu(part->decoded, i, GPS_CNAV_PREAMBLE_LENGTH);
                    if (GPS_CNAV_PREAMBLE1 == c || GPS_CNAV_PREAMBLE2 == c)
                        {
                            part->preamble_seen = true;
                            part->invert = (GPS_CNAV_PREAMBLE2 == c);
                            /* We shift the accumulated bits to the beginning of the buffer */
                            bitshl(part->decoded, sizeof(part->decoded), i);
                            part->n_decoded -= i;
                            break;
                        }
                }
        }
    if (!part->preamble_seen && part->n_decoded >= GPS_CNAV_PREAMBLE_LENGTH)
        {
            bitshl(part->decoded, sizeof(part->decoded),
                part->n_decoded - GPS_CNAV_PREAMBLE_LENGTH + 1);
            part->n_decoded = GPS_CNAV_PREAMBLE_LENGTH - 1;
        }
}


/**
 * Feed a symbol into Viterbi decoder instance.
 *
 * The method uses block Viterbi decoder. It first accumulates initial number of
 * symbols, and after that runs decoding every time the buffer is full. Only
 * some of the decoded symbols are used.
 *
 * \param[in,out] part Decoder object
 * \param[in]     s    Symbol (0x00 - Hard 0, 0xFF - Hard 1)
 *
 * \return None
 *
 * \private
 */
static void _cnav_add_symbol(cnav_v27_part_t *part, u8 ch)
{
    part->symbols[part->n_symbols++] = ch;

    if (part->init)
        {
            /* Initial step - load more symbols without decoding. */
            if (part->n_symbols < (GPS_L2C_V27_INIT_BITS + GPS_L2C_V27_DECODE_BITS) * 2)
                {
                    return;
                }
            part->init = false;
        }
    else if (part->n_symbols < GPS_L2C_V27_DECODE_BITS * 2)
        {
            /* Wait until decoding block is accumulated */
            return;
        }

    /* Feed accumulated symbols into the buffer, reset the number of accumulated
     * symbols. */
    v27_update(&part->dec, part->symbols, part->n_symbols / 2);
    part->n_symbols = 0;

    /* Decode N+M bits, where:
     * - N - Number of bits to put into decoded buffer
     * - M - Number of bits in the tail to ignore.
     */
    unsigned char tmp_bits[(GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS +
                               CHAR_BIT - 1) /
                           CHAR_BIT];

    v27_chainback_likely(&part->dec, tmp_bits,
        GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS);

    /* Read decoded bits and add them to the decoded buffer */
    bitcopy(part->decoded, part->n_decoded, tmp_bits, 0, GPS_L2C_V27_DECODE_BITS);
    part->n_decoded += GPS_L2C_V27_DECODE_BITS;

    /* Depending on the decoder state, one of the following actions are
     * possible:
     * - If no message lock
     *   - If no preamble seen - look for preamble
     *   - If preamble seen - collect 300 bits
     *     - If 300 bits are collected - verify CRC
     *       - If CRC is OK - message lock is acquired
     *       - If CRC fails - rescan for preamble
     *         - If found - continue collecting 300 bits
     *         - If not found - continue preamble wait
     * - If message lock
     *   - If 300 bits collected, compute CRC
     *     - If CRC is OK, message can be decoded
     *     - If CRC is not OK, discard data
     */

    bool retry = true;
    while (retry)
        {
            retry = false;

            if (!part->preamble_seen)
                {
                    /* Rescan for preamble if possible. The first bit is ignored. */
                    _cnav_rescan_preamble(part);
                }
            if (part->preamble_seen && GPS_CNAV_MSG_LENGTH <= part->n_decoded)
                {
                    /* We have collected 300 bits starting from message preamble. Now try
                     * to compute CRC-24Q */
                    u32 crc = _cnav_compute_crc(part);
                    u32 crc2 = _cnav_extract_crc(part);

                    if (part->message_lock)
                        {
                            /* We have message lock */
                            part->crc_ok = (crc == crc2);
                            if (part->crc_ok)
                                {
                                    /* Reset message lock counter */
                                    part->n_crc_fail = 0;
                                }
                            else
                                {
                                    /* Increment message lock counter */
                                    part->n_crc_fail++;
                                    if (part->n_crc_fail > GPS_CNAV_LOCK_MAX_CRC_FAILS)
                                        {
                                            /* CRC has failed too many times - drop the lock. */
                                            part->n_crc_fail = 0;
                                            part->message_lock = false;
                                            part->preamble_seen = false;
                                            /* Try to find a new preamble, reuse data from buffer. */
                                            retry = true;
                                        }
                                }
                        }
                    else if (crc == crc2)
                        {
                            /* CRC match - message can be decoded */
                            part->message_lock = true;
                            part->crc_ok = true;
                            part->n_crc_fail = 0;
                        }
                    else
                        {
                            /* There is no message lock and the CRC check fails. Assume there is
                             * false positive lock - rescan for preamble. */
                            part->crc_ok = false;
                            part->preamble_seen = false;

                            /* CRC mismatch - try to re-scan for preamble */
                            retry = true;
                        }
                }
            else
                {
                    /* No preamble or preamble and less than 300 bits decoded */
                }
        }
}


/**
 * Invert message bits in the buffer.
 *
 * The method inverts bits of the decoded data.
 *
 * \param[in,out] part Decoder component with a message buffer.
 *
 * \return None
 */
static void _cnav_msg_invert(cnav_v27_part_t *part)
{
    size_t i = 0;
    for (i = 0; i < sizeof(part->decoded); i++)
        {
            part->decoded[i] ^= 0xFFu;
        }
}


/**
 * Performs CNAV message decoding.
 *
 * This function decoded CNAV message, if the following conditions are met:
 * - Buffer contains 300 bits.
 * - First 8 bits are matching direct or inverse preamble.
 * - Message data CRC matches one in the buffer.
 *
 * In case the message starts with inverted preamble, the data is inverted
 * before parsing.
 *
 * \param[in,out] part Decoder component.
 * \param[out]    msg  Container for a decoded message.
 * \param[out]    delay Delay of the message in symbols.
 *
 * \retval true The message has been decoded, and \a msg container is populated.
 * \retval false Not enough data or CRC is not correct.
 *
 * \private
 */
static bool _cnav_msg_decode(cnav_v27_part_t *part, cnav_msg_t *msg, u32 *delay)
{
    bool res = false;
    if (GPS_CNAV_MSG_LENGTH <= part->n_decoded)
        {
            if (part->crc_ok)
                {
                    /* CRC is OK */
                    if (part->invert)
                        {
                            _cnav_msg_invert(part);
                        }

                    msg->prn = getbitu(part->decoded, 8, 6);
                    msg->msg_id = getbitu(part->decoded, 14, 6);
                    msg->tow = getbitu(part->decoded, 20, 17);
                    msg->alert = getbitu(part->decoded, 37, 1) ? true : false;

                    /* copy RAW message for GNSS-SDR */
                    memcpy(msg->raw_msg, part->decoded, GPS_L2C_V27_DECODE_BITS + GPS_L2C_V27_DELAY_BITS);

                    *delay = (part->n_decoded - GPS_CNAV_MSG_LENGTH + GPS_L2C_V27_DELAY_BITS) * 2 + part->n_symbols;

                    if (part->invert)
                        {
                            _cnav_msg_invert(part);
                        }
                    res = true;
                }
            else
                {
                    /* CRC mismatch - no decoding */
                }
            bitshl(part->decoded, sizeof(part->decoded), GPS_CNAV_MSG_LENGTH);
            part->n_decoded -= GPS_CNAV_MSG_LENGTH;
        }

    return res;
}


/**
 * Initialize CNAV decoder.
 *
 * CNAV decoder contains two Viterbi decoders that are used to estimate bit and
 * message boundary.
 *
 * \param[out] dec Decoder structure.
 *
 * \return None
 */
void cnav_msg_decoder_init(cnav_msg_decoder_t *dec)
{
    memset(dec, 0, sizeof(*dec));
    v27_init(&dec->part1.dec,
        dec->part1.decisions,
        GPS_L2_V27_HISTORY_LENGTH_BITS,
        cnav_msg_decoder_get_poly(),
        0);
    v27_init(&dec->part2.dec,
        dec->part2.decisions,
        GPS_L2_V27_HISTORY_LENGTH_BITS,
        cnav_msg_decoder_get_poly(),
        0);
    dec->part1.init = true;
    dec->part2.init = true;
    _cnav_add_symbol(&dec->part2, 0x80);
}

/**
 * Adds a received symbol to decoder.
 *
 * The method feeds the symbol into the decoder. In case there is a sufficient
 * information to produce a message, the message is decoded and symbol delay is
 * reported.
 * The time of the last input symbol can be computed from the message ToW and
 * delay by the formulae:
 * \code
 * symbolTime_ms = msg->tow * 6000 + *pdelay * 20 (L2)
 * symbolTime_ms = msg->tow * 6000 + *pdelay * 10 (L5)
 * \endcode
 *
 * \param[in,out] dec    Decoder object.
 * \param[in]     symbol Symbol value probability, where 0x00 - 100% of 0,
 *                       0xFF - 100% of 1.
 * \param[out]    msg    Buffer for decoded message. The message is available
 *                       only when message lock is acquired and CRC is correct.
 * \param[out]    pdelay Delay of message generation in symbols.
 *
 * \retval true  The message has been decoded. ToW parameter is available.
 * \retval false More data is required.
 */
bool cnav_msg_decoder_add_symbol(cnav_msg_decoder_t *dec,
    u8 symbol,
    cnav_msg_t *msg,
    u32 *pdelay)
{
    _cnav_add_symbol(&dec->part1, symbol);
    _cnav_add_symbol(&dec->part2, symbol);

    if (dec->part1.message_lock)
        {
            /* Flush data in decoder. */
            dec->part2.n_decoded = 0;
            dec->part2.n_symbols = 0;
            return _cnav_msg_decode(&dec->part1, msg, pdelay);
        }
    if (dec->part2.message_lock)
        {
            /* Flush data in decoder. */
            dec->part1.n_decoded = 0;
            dec->part1.n_symbols = 0;
            return _cnav_msg_decode(&dec->part2, msg, pdelay);
        }

    return false;
}


/**
 * Provides a singleton polynomial object.
 *
 * The method constructs and returns polynomial object for CNAV message
 * decoding. The same polynomial can be used also for other message handling.
 *
 * The object is initialized on the first call. The method is thread-safe.
 *
 * @return Pointer to polynomial object for CNAV message decoding.
 */
const v27_poly_t *cnav_msg_decoder_get_poly(void)
{
    static v27_poly_t instance;
    static bool initialized = false;

    if (!initialized)
        {
            /* Coefficients for polynomial object */
            const signed char coeffs[2] = {GPS_L2C_V27_POLY_A, GPS_L2C_V27_POLY_B};

            /* Racing condition handling: the data can be potential initialized more
             * than once in case multiple threads request concurrent access. However,
             * nature of the v27_poly_init() function and data alignment ensure that
             * the data returned from the earlier finished call is consistent and can
             * be used even when re-initialization is happening.
             *
             * Other possible approaches are:
             * - Replace late initialization with an explicit call.
             * - Use POSIX synchronization objects like pthread_once_t.
             */
            v27_poly_init(&instance, coeffs);
            initialized = true;
        }
    return &instance;
}

/** \} */
/** \} */

/*!
 * \file gps_cnav_msg_decoder_test.cc
 * \brief  Tests for the GPS CNAV message decoder (libswiftcnav): preamble
 * search at every alignment of the message in the Viterbi decoding blocks
 * \author Ben Gant, 2026.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <cstddef>
#include <cstdint>
#include <vector>

extern "C"
{
#include "cnav_msg.h"
}


namespace gps_cnav_msg_decoder_test
{
const size_t message_bits = 300;      // IS-GPS-200, 30.3.3: message length
const size_t crc_bits = 24;           // IS-GPS-200, 30.3.5: CRC-24Q
const uint32_t preamble = 0x8BU;      // IS-GPS-200, 30.3.3: 10001011
const size_t idle_bits = 64;          // zero bits ahead of the first message
const size_t flush_bits = 128;        // zero bits that push the last message out
const size_t decode_block_bits = 32;  // GPS_L2C_V27_DECODE_BITS


void append_field(std::vector<uint8_t>& bits, uint32_t value, size_t width)
{
    for (size_t i = 0; i < width; i++)
        {
            bits.push_back(static_cast<uint8_t>((value >> (width - i - 1)) & 1U));
        }
}


// CRC-24Q, g(X) = 0x1864CFB, computed bit by bit over the message bits
uint32_t crc24q(const std::vector<uint8_t>& bits)
{
    uint32_t crc = 0;
    for (const auto bit : bits)
        {
            crc ^= static_cast<uint32_t>(bit) << 23U;
            crc <<= 1U;
            if ((crc & 0x1000000U) != 0U)
                {
                    crc ^= 0x1864CFBU;
                }
        }
    return crc & 0xFFFFFFU;
}


// A 300-bit CNAV message: preamble, PRN, message type ID, message TOW count,
// alert flag, 238 bits of pseudo-random payload and the CRC-24Q parity.
std::vector<uint8_t> make_message(uint32_t prn, uint32_t msg_id, uint32_t tow, uint32_t seed)
{
    std::vector<uint8_t> bits;
    bits.reserve(message_bits);
    append_field(bits, preamble, 8);
    append_field(bits, prn, 6);
    append_field(bits, msg_id, 6);
    append_field(bits, tow, 17);
    append_field(bits, 0, 1);
    uint32_t lcg = seed;
    while (bits.size() < message_bits - crc_bits)
        {
            lcg = lcg * 1664525U + 1013904223U;
            bits.push_back(static_cast<uint8_t>((lcg >> 24U) & 1U));
        }
    append_field(bits, crc24q(bits), crc_bits);
    return bits;
}


uint8_t parity7(uint32_t value)
{
    uint8_t p = 0;
    for (uint32_t i = 0; i < 7; i++)
        {
            p ^= static_cast<uint8_t>((value >> i) & 1U);
        }
    return p;
}


// Rate 1/2, constraint length 7 convolutional encoder, G1 = 171o, G2 = 133o
// (IS-GPS-200, 3.3.3.1.1), the G1 symbol first. Hard symbols: 0x00 or 0xFF.
std::vector<uint8_t> encode(const std::vector<uint8_t>& bits, bool invert)
{
    std::vector<uint8_t> symbols;
    symbols.reserve(2 * bits.size());
    uint32_t reg = 0;
    for (const auto bit : bits)
        {
            reg = ((reg << 1U) | bit) & 0x7FU;  // the newest bit is the LSB
            const uint8_t g1 = parity7(reg & 0x4FU);
            const uint8_t g2 = parity7(reg & 0x6DU);
            symbols.push_back(((g1 != 0) != invert) ? 0xFF : 0x00);
            symbols.push_back(((g2 != 0) != invert) ? 0xFF : 0x00);
        }
    return symbols;
}


// Feeds the symbols of \a bits to a new decoder and returns what it decodes.
// \a lead_symbols extra symbols ahead of the stream select which of the two
// Viterbi decoders of cnav_msg_decoder_t is the one aligned to symbol pairs.
std::vector<cnav_msg_t> decode(const std::vector<uint8_t>& bits, size_t lead_symbols, bool invert)
{
    std::vector<cnav_msg_t> messages;
    cnav_msg_decoder_t decoder;
    cnav_msg_decoder_init(&decoder);
    cnav_msg_t msg;
    uint32_t delay = 0;
    for (size_t i = 0; i < lead_symbols; i++)
        {
            if (cnav_msg_decoder_add_symbol(&decoder, 0x00, &msg, &delay))
                {
                    messages.push_back(msg);
                }
        }
    for (const auto symbol : encode(bits, invert))
        {
            if (cnav_msg_decoder_add_symbol(&decoder, symbol, &msg, &delay))
                {
                    messages.push_back(msg);
                }
        }
    return messages;
}


bool same_bits(const cnav_msg_t& msg, const std::vector<uint8_t>& bits)
{
    for (size_t i = 0; i < bits.size(); i++)
        {
            if (((msg.raw_msg[i / 8] >> (7U - i % 8)) & 1U) != bits[i])
                {
                    return false;
                }
        }
    return true;
}


void append(std::vector<uint8_t>& stream, const std::vector<uint8_t>& bits)
{
    stream.insert(stream.end(), bits.begin(), bits.end());
}
}  // namespace gps_cnav_msg_decoder_test


// Two consecutive messages; the preamble of the first one is put at each of
// the 32 positions it can take in the 32-bit blocks the Viterbi decoder
// delivers. The first message must be decoded at every position.
TEST(GpsCnavMsgDecoderTest, FirstMessageAtEveryBlockAlignment)
{
    using namespace gps_cnav_msg_decoder_test;
    const auto msg_1 = make_message(32, 10, 1001, 1);
    const auto msg_2 = make_message(32, 11, 1002, 2);

    for (const bool invert : {false, true})
        {
            for (size_t lead_symbols = 0; lead_symbols < 2; lead_symbols++)
                {
                    for (size_t shift = 0; shift < decode_block_bits; shift++)
                        {
                            SCOPED_TRACE(testing::Message() << "shift=" << shift << " bits, lead_symbols=" << lead_symbols << ", invert=" << invert);
                            std::vector<uint8_t> stream(idle_bits + shift, 0);
                            append(stream, msg_1);
                            append(stream, msg_2);
                            stream.insert(stream.end(), flush_bits, 0);

                            const auto messages = decode(stream, lead_symbols, invert);

                            ASSERT_FALSE(messages.empty());
                            EXPECT_EQ(messages.size(), 2U);
                            EXPECT_EQ(messages[0].prn, 32U);
                            EXPECT_EQ(messages[0].msg_id, 10U);
                            EXPECT_EQ(messages[0].tow, 1001U);
                            EXPECT_FALSE(messages[0].alert);
                            EXPECT_TRUE(same_bits(messages[0], msg_1));
                            if (messages.size() == 2)
                                {
                                    EXPECT_EQ(messages[1].msg_id, 11U);
                                    EXPECT_EQ(messages[1].tow, 1002U);
                                    EXPECT_TRUE(same_bits(messages[1], msg_2));
                                }
                        }
                }
        }
}


// A preamble pattern that is not followed by a valid message makes the decoder
// collect 300 bits and fail the CRC check. The rescan that follows must not
// stay on that pattern and must find the preamble of the first valid message,
// wherever it is with respect to the pattern and to the end of the decoded
// bits.
TEST(GpsCnavMsgDecoderTest, RescanAfterCrcFailure)
{
    using namespace gps_cnav_msg_decoder_test;
    const auto msg_1 = make_message(7, 30, 2001, 3);
    const auto msg_2 = make_message(7, 33, 2002, 4);

    for (size_t gap = 0; gap < message_bits + 2 * decode_block_bits; gap++)
        {
            SCOPED_TRACE(testing::Message() << "gap=" << gap << " bits");
            std::vector<uint8_t> stream(idle_bits, 0);
            append_field(stream, preamble, 8);
            stream.insert(stream.end(), gap, 0);
            append(stream, msg_1);
            append(stream, msg_2);
            stream.insert(stream.end(), flush_bits, 0);

            const auto messages = decode(stream, 0, false);

            ASSERT_FALSE(messages.empty());
            EXPECT_EQ(messages.size(), 2U);
            EXPECT_EQ(messages[0].prn, 7U);
            EXPECT_EQ(messages[0].msg_id, 30U);
            EXPECT_EQ(messages[0].tow, 2001U);
            EXPECT_TRUE(same_bits(messages[0], msg_1));
        }
}

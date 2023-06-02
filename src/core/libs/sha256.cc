/*!
 * \file sha256.cc
 * \brief Class foir computing SHA256
 * \author Carles Fernandez-Prades, 2023. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "sha256.h"
#include <cstring>
#include <iomanip>
#include <sstream>

SHA256::SHA256() : d_blocklen(0), d_bitlen(0)
{
    d_state[0] = 0x6a09e667;
    d_state[1] = 0xbb67ae85;
    d_state[2] = 0x3c6ef372;
    d_state[3] = 0xa54ff53a;
    d_state[4] = 0x510e527f;
    d_state[5] = 0x9b05688c;
    d_state[6] = 0x1f83d9ab;
    d_state[7] = 0x5be0cd19;
}

void SHA256::update(const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
        {
            d_data[d_blocklen++] = data[i];
            if (d_blocklen == 64)
                {
                    transform();

                    // End of the block
                    d_bitlen += 512;
                    d_blocklen = 0;
                }
        }
}

void SHA256::update(const std::string &data)
{
    update(reinterpret_cast<const uint8_t *>(data.c_str()), data.size());
}

uint8_t *SHA256::digest()
{
    uint8_t *hash = new uint8_t[32];

    pad();
    revert(hash);

    return hash;
}

uint32_t SHA256::rotr(uint32_t x, uint32_t n)
{
    return (x >> n) | (x << (32 - n));
}

uint32_t SHA256::choose(uint32_t e, uint32_t f, uint32_t g)
{
    return (e & f) ^ (~e & g);
}

uint32_t SHA256::majority(uint32_t a, uint32_t b, uint32_t c)
{
    return (a & (b | c)) | (b & c);
}

uint32_t SHA256::sig0(uint32_t x)
{
    return SHA256::rotr(x, 7) ^ SHA256::rotr(x, 18) ^ (x >> 3);
}

uint32_t SHA256::sig1(uint32_t x)
{
    return SHA256::rotr(x, 17) ^ SHA256::rotr(x, 19) ^ (x >> 10);
}

void SHA256::transform()
{
    uint32_t maj, xorA, ch, xorE, sum, newA, newE, m[64];
    uint32_t state[8];

    for (uint8_t i = 0, j = 0; i < 16; i++, j += 4)
        {  // Split data in 32 bit blocks for the 16 first words
            m[i] = (d_data[j] << 24) | (d_data[j + 1] << 16) | (d_data[j + 2] << 8) | (d_data[j + 3]);
        }

    for (uint8_t k = 16; k < 64; k++)
        {  // Remaining 48 blocks
            m[k] = SHA256::sig1(m[k - 2]) + m[k - 7] + SHA256::sig0(m[k - 15]) + m[k - 16];
        }

    for (uint8_t i = 0; i < 8; i++)
        {
            state[i] = d_state[i];
        }

    for (uint8_t i = 0; i < 64; i++)
        {
            maj = SHA256::majority(state[0], state[1], state[2]);
            xorA = SHA256::rotr(state[0], 2) ^ SHA256::rotr(state[0], 13) ^ SHA256::rotr(state[0], 22);

            ch = choose(state[4], state[5], state[6]);

            xorE = SHA256::rotr(state[4], 6) ^ SHA256::rotr(state[4], 11) ^ SHA256::rotr(state[4], 25);

            sum = m[i] + K[i] + state[7] + ch + xorE;
            newA = xorA + maj + sum;
            newE = state[3] + sum;

            state[7] = state[6];
            state[6] = state[5];
            state[5] = state[4];
            state[4] = newE;
            state[3] = state[2];
            state[2] = state[1];
            state[1] = state[0];
            state[0] = newA;
        }

    for (uint8_t i = 0; i < 8; i++)
        {
            d_state[i] += state[i];
        }
}

void SHA256::pad()
{
    uint64_t i = d_blocklen;
    uint8_t end = d_blocklen < 56 ? 56 : 64;

    d_data[i++] = 0x80;  // Append a bit 1
    while (i < end)
        {
            d_data[i++] = 0x00;  // Pad with zeros
        }

    if (d_blocklen >= 56)
        {
            transform();
            memset(d_data, 0, 56);
        }

    // Append to the padding the total message's length in bits and transform.
    d_bitlen += d_blocklen * 8;
    d_data[63] = d_bitlen;
    d_data[62] = d_bitlen >> 8;
    d_data[61] = d_bitlen >> 16;
    d_data[60] = d_bitlen >> 24;
    d_data[59] = d_bitlen >> 32;
    d_data[58] = d_bitlen >> 40;
    d_data[57] = d_bitlen >> 48;
    d_data[56] = d_bitlen >> 56;
    transform();
}

void SHA256::revert(uint8_t *hash)
{
    // SHA uses big endian byte ordering
    // Revert all bytes
    for (uint8_t i = 0; i < 4; i++)
        {
            for (uint8_t j = 0; j < 8; j++)
                {
                    hash[i + (j * 4)] = (d_state[j] >> (24 - i * 8)) & 0x000000ff;
                }
        }
}

std::string SHA256::toString(const uint8_t *digest)
{
    std::stringstream s;
    s << std::setfill('0') << std::hex;

    for (uint8_t i = 0; i < 32; i++)
        {
            s << std::setw(2) << (unsigned int)digest[i];
        }

    return s.str();
}
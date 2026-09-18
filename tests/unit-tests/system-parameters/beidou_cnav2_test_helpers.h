/*!
 * \file beidou_cnav2_test_helpers.h
 * \brief Independent B-CNAV2 reference encoder for tests
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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
#ifndef GNSS_SDR_BEIDOU_CNAV2_TEST_HELPERS_H
#define GNSS_SDR_BEIDOU_CNAV2_TEST_HELPERS_H

#include "beidou_cnav2_ldpc.h"
#include <array>
#include <stdexcept>
#include <utility>

namespace BeidouCnav2Test
{
// Polynomial arithmetic independent of the decoder's logarithm lookup tables.
inline uint8_t multiply(uint8_t a, uint8_t b)
{
    uint8_t product = 0;
    while (b != 0)
        {
            if ((b & 1U) != 0)
                {
                    product ^= a;
                }
            b >>= 1U;
            a <<= 1U;
            if ((a & 0x40U) != 0)
                {
                    a ^= 0x43U;
                }
        }
    return product;
}

// Solve H2 * parity = H1 * information by Gaussian elimination over GF(64).
inline std::array<uint8_t, 576> encode(const std::array<uint8_t, 288>& bits)
{
    std::array<uint8_t, 96> symbols{};
    for (size_t i = 0; i < bits.size(); i++)
        {
            symbols[i / 6] = static_cast<uint8_t>((symbols[i / 6] << 1U) | bits[i]);
        }
    std::array<std::array<uint8_t, 49>, 48> augmented{};
    for (size_t row = 0; row < 48; row++)
        {
            for (size_t edge = 0; edge < 4; edge++)
                {
                    // Each printed row contains four groups of four entries.
                    const size_t offset = (row % 12) * 16 + (row / 12) * 4 + edge;
                    const auto col = BEIDOU_CNAV2_H48_96_INDEX[offset];
                    const auto h = BEIDOU_CNAV2_H48_96_ELEMENT[offset];
                    if (col < 48)
                        {
                            augmented[row][48] ^= multiply(h, symbols[col]);
                        }
                    else
                        {
                            augmented[row][col - 48] = h;
                        }
                }
        }
    for (size_t col = 0; col < 48; col++)
        {
            size_t pivot = col;
            while (pivot < 48 && augmented[pivot][col] == 0)
                {
                    pivot++;
                }
            if (pivot == 48)
                {
                    throw std::runtime_error("Singular B-CNAV2 parity matrix");
                }
            std::swap(augmented[col], augmented[pivot]);
            uint8_t inverse = 1;
            while (multiply(augmented[col][col], inverse) != 1)
                {
                    inverse++;
                }
            for (size_t j = col; j <= 48; j++)
                {
                    augmented[col][j] = multiply(augmented[col][j], inverse);
                }
            for (size_t row = 0; row < 48; row++)
                {
                    if (row == col)
                        {
                            continue;
                        }
                    const auto factor = augmented[row][col];
                    for (size_t j = col; j <= 48; j++)
                        {
                            augmented[row][j] ^= multiply(factor, augmented[col][j]);
                        }
                }
        }
    for (size_t row = 0; row < 48; row++)
        {
            symbols[48 + row] = augmented[row][48];
        }
    std::array<uint8_t, 576> codeword{};
    for (size_t bit = 0; bit < codeword.size(); bit++)
        {
            codeword[bit] = (symbols[bit / 6] >> (5 - bit % 6)) & 1U;
        }
    return codeword;
}
}  // namespace BeidouCnav2Test

#endif  // GNSS_SDR_BEIDOU_CNAV2_TEST_HELPERS_H

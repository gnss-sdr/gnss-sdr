/*!
 * \file beidou_cnav_test_helpers.h
 * \brief Independent B-CNAV1/B-CNAV2 reference encoder for tests
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
#ifndef GNSS_SDR_BEIDOU_CNAV_TEST_HELPERS_H
#define GNSS_SDR_BEIDOU_CNAV_TEST_HELPERS_H

#include "beidou_cnav1_ldpc.h"
#include "beidou_cnav2_ldpc.h"
#include <array>
#include <stdexcept>
#include <utility>

namespace BeidouCnavTest
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
template <int N>
inline std::array<uint8_t, N> encode(const std::array<uint8_t, N / 2>& bits)
{
    if (N != 1200 && N != 528 && N != 576)
        {
            throw std::runtime_error("Invalid B-CNAV1/B-CNAV2 encoded subframe size");
        }
    std::array<uint8_t, N / 6> symbols{};
    for (size_t i = 0; i < bits.size(); i++)
        {
            symbols[i / 6] = static_cast<uint8_t>((symbols[i / 6] << 1U) | bits[i]);
        }
    std::array<std::array<uint8_t, N / 12 + 1>, N / 12> augmented{};
    for (size_t row = 0; row < N / 12; row++)
        {
            for (size_t edge = 0; edge < 4; edge++)
                {
                    // Each printed row contains four groups of four entries.
                    const size_t offset = (row % (N / 48)) * 16 + (row / (N / 48)) * 4 + edge;
                    uint16_t col = 0;
                    uint8_t h = 0;
                    switch (N)
                        {
                        case 1200:
                            col = BEIDOU_CNAV1_H100_200_INDEX[offset];
                            h = BEIDOU_CNAV1_H100_200_ELEMENT[offset];
                            break;
                        case 528:
                            col = BEIDOU_CNAV1_H44_88_INDEX[offset];
                            h = BEIDOU_CNAV1_H44_88_ELEMENT[offset];
                            break;
                        case 576:
                            col = BEIDOU_CNAV2_H48_96_INDEX[offset];
                            h = BEIDOU_CNAV2_H48_96_ELEMENT[offset];
                            break;
                        }
                    if (col < N / 12)
                        {
                            augmented[row][N / 12] ^= multiply(h, symbols[col]);
                        }
                    else
                        {
                            augmented[row][col - N / 12] = h;
                        }
                }
        }
    for (size_t col = 0; col < N / 12; col++)
        {
            size_t pivot = col;
            while (pivot < N / 12 && augmented[pivot][col] == 0)
                {
                    pivot++;
                }
            if (pivot == N / 12)
                {
                    throw std::runtime_error("Singular B-CNAV1/B-CNAV2 parity matrix");
                }
            std::swap(augmented[col], augmented[pivot]);
            uint8_t inverse = 1;
            while (multiply(augmented[col][col], inverse) != 1)
                {
                    inverse++;
                }
            for (size_t j = col; j <= N / 12; j++)
                {
                    augmented[col][j] = multiply(augmented[col][j], inverse);
                }
            for (size_t row = 0; row < N / 12; row++)
                {
                    if (row == col)
                        {
                            continue;
                        }
                    const auto factor = augmented[row][col];
                    for (size_t j = col; j <= N / 12; j++)
                        {
                            augmented[row][j] ^= multiply(factor, augmented[col][j]);
                        }
                }
        }
    for (size_t row = 0; row < N / 12; row++)
        {
            symbols[N / 12 + row] = augmented[row][N / 12];
        }
    std::array<uint8_t, N> codeword{};
    for (size_t bit = 0; bit < codeword.size(); bit++)
        {
            codeword[bit] = (symbols[bit / 6] >> (5 - bit % 6)) & 1U;
        }
    return codeword;
}
}  // namespace BeidouCnavTest

#endif  // GNSS_SDR_BEIDOU_CNAV_TEST_HELPERS_H

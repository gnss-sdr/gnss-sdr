/*!
 * \file galileo_ism_test.cc
 * \brief Tests for Galileo Integrity Support Message
 * \author Carles Fernandez-Prades, 2024. cfernandez(at)cttc.es
 *
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

#include "galileo_ism.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <bitset>
#include <vector>

TEST(GalileoISMTest, CRC)
{
    Galileo_ISM gal_ism{};
    uint32_t expected_crc = 3002390191;
    std::bitset<96> input{"010110000010101010101010101010101010101010101010101010101010101010101010101010101010101010101010"};
    std::vector<uint8_t> data_bytes;
    for (size_t i = 0; i < input.size(); i += 8)
        {
            std::bitset<8> byte;
            for (size_t j = 0; j < 8; j++)
                {
                    byte[j] = input[i + j];
                }
            data_bytes.push_back(static_cast<uint8_t>(byte.to_ulong()));
        }

    std::reverse(data_bytes.begin(), data_bytes.end());
    auto computed_crc = gal_ism.compute_crc(data_bytes);
    EXPECT_TRUE(computed_crc == expected_crc);
}
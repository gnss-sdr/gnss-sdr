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
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_ism.h"
#include <gtest/gtest.h>

TEST(GalileoISMTest, CRC)
{
    // Example from ANNEX G Galileo ICD
    Galileo_ISM gal_ism{};
    std::bitset<128> input{"01011000001010101010101010101010101010101010101010101010101010101010101010101010101010101010101010110010111101001101011010101111"};
    bool result = gal_ism.check_ism_crc(input);
    EXPECT_TRUE(result);
    // Check if it can be used twice
    bool result2 = gal_ism.check_ism_crc(input);
    EXPECT_TRUE(result2);
    // Check if it fails
    input.set(127);
    bool result3 = gal_ism.check_ism_crc(input);
    EXPECT_TRUE(!result3);
}
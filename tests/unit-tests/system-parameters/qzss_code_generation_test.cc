/*!
 * \file qzss_code_generation_test.cc
 * \brief  This file implements unit tests for the QZSS code generation
 * \author Carles Fernández-Prades, 2026. cfernandez (at)) cttc.es
 *
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

#include "qzss_signal_replica.h"
#include <gtest/gtest.h>
#include <vector>


TEST(QzssL1Code, Periodicity)
{
    constexpr uint32_t prn = 193;
    std::vector<float> code1(1023);
    std::vector<float> code2(1023);

    qzss_l1_code_gen_float(code1, prn);
    qzss_l1_code_gen_float(code2, prn);

    EXPECT_EQ(code1, code2);
}


TEST(QzssL1Code, GoldenFirst32Chips)
{
    constexpr uint32_t prn = 193;
    std::vector<float> code(1023);
    qzss_l1_code_gen_float(code, prn);

    const float golden[32] = {
        -1, 1, 1, 1, -1, 1, -1, 1,
        1, 1, -1, -1, -1, -1, 1, 1,
        -1, -1, -1, -1, -1, -1, 1, -1,
        1, -1, -1, -1, -1, -1, -1, 1};

    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden[i]);
        }
    constexpr uint32_t prn2 = 195;
    qzss_l1_code_gen_float(code, prn2);
    const float golden2[32] = {
        -1, -1, -1, -1, -1, 1, 1, -1,
        -1, -1, -1, 1, -1, 1, -1, 1,
        1, 1, 1, 1, -1, -1, 1, 1,
        -1, -1, 1, -1, 1, 1, -1, -1};
    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden2[i]);
        }

    constexpr uint32_t prn3 = 199;
    qzss_l1_code_gen_float(code, prn3);
    const float golden3[32] = {
        1, -1, -1, -1, 1, -1, 1, -1,
        -1, -1, -1, -1, 1, -1, -1, -1,
        -1, 1, -1, -1, 1, -1, -1, -1,
        -1, 1, 1, 1, -1, 1, -1, -1};
    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden3[i]);
        }
}


TEST(QzssL5Code, L5IGoldenFirst32Chips)
{
    std::vector<float> code(10230);
    qzss_l5i_code_gen_float(code, 193);

    const float golden[32] = {
        1, -1, -1, -1, 1, -1, 1, 1,
        1, 1, -1, -1, 1, 1, 1, -1,
        1, -1, -1, 1, -1, -1, 1, -1,
        -1, 1, 1, 1, -1, -1, -1, 1};

    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden[i]);
        }

    qzss_l5i_code_gen_float(code, 199);

    const float golden2[32] = {
        1, 1, -1, 1, 1, -1, 1, -1,
        -1, 1, -1, 1, -1, -1, -1, -1,
        1, 1, -1, 1, -1, -1, -1, 1,
        -1, 1, 1, 1, 1, -1, 1, -1};

    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden2[i]);
        }
}


TEST(QzssL5Code, L5QGoldenFirst32Chips)
{
    std::vector<float> code(10230);
    qzss_l5q_code_gen_float(code, 193);

    const float golden[32] = {
        -1, -1, -1, 1, 1, 1, 1, -1,
        -1, -1, 1, 1, -1, 1, 1, -1,
        -1, 1, 1, -1, -1, -1, -1, -1,
        1, 1, -1, -1, -1, -1, -1, 1};

    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden[i]);
        }

    qzss_l5q_code_gen_float(code, 199);

    const float golden2[32] = {
        -1, 1, 1, 1, -1, -1, -1, 1,
        1, -1, -1, 1, 1, 1, 1, -1,
        -1, -1, -1, -1, -1, 1, 1, 1,
        -1, 1, 1, -1, 1, -1, -1, 1};

    for (int i = 0; i < 32; ++i)
        {
            EXPECT_FLOAT_EQ(code[i], golden2[i]);
        }
}
/*!
 * \file sbas_rtklib_corrections_test.cc
 * \brief Unit tests for feeding raw SBAS messages into RTKLIB correction state
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

#include "SBAS_L1.h"
#include "rtklib_rtkcmn.h"
#include "sbas_raw_message.h"
#include "sbas_rtklib_corrections.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <vector>

namespace sbas_rtklib_corrections_test
{
void set_unsigned_bits(std::vector<uint8_t>& frame, int32_t position, int32_t length, uint32_t value)
{
    for (int32_t bit = 0; bit < length; ++bit)
        {
            const int32_t frame_bit = position + bit;
            const uint8_t mask = static_cast<uint8_t>(1U << (7 - frame_bit % 8));
            if ((value >> (length - bit - 1)) & 1U)
                {
                    frame[static_cast<size_t>(frame_bit / 8)] |= mask;
                }
            else
                {
                    frame[static_cast<size_t>(frame_bit / 8)] &= static_cast<uint8_t>(~mask);
                }
        }
}


std::vector<uint8_t> make_message(uint32_t type)
{
    std::vector<uint8_t> frame(SBAS_L1_MSG_LENGTH_BYTES, 0U);
    frame[0] = SBAS_L1_PREAMBLE_1;
    set_unsigned_bits(frame, 8, 6, type);
    return frame;
}
}  // namespace sbas_rtklib_corrections_test


TEST(SbasRtklibCorrectionsTest, DecodesMasksAndFastCorrectionsWithGpsTime)
{
    using namespace sbas_rtklib_corrections_test;
    Sbas_Rtklib_Corrections corrections(136);

    auto mask_message = make_message(1);
    set_unsigned_bits(mask_message, 14, 1, 1);   // GPS PRN 1
    set_unsigned_bits(mask_message, 224, 2, 2);  // IODP
    corrections.push(Sbas_Raw_Message(99.0, 136, mask_message));
    corrections.update(2400, 100.0, 100.0);

    ASSERT_EQ(corrections.active_prn(), 136);
    ASSERT_EQ(corrections.satellite_corrections().iodp, 2);
    ASSERT_EQ(corrections.satellite_corrections().nsat, 1);
    EXPECT_EQ(corrections.satellite_corrections().sat[0].sat, satno(SYS_GPS, 1));

    auto fast_correction_message = make_message(2);
    set_unsigned_bits(fast_correction_message, 14, 2, 1);    // IODF
    set_unsigned_bits(fast_correction_message, 16, 2, 2);    // IODP
    set_unsigned_bits(fast_correction_message, 18, 12, 80);  // 80 * 0.125 m = 10 m
    set_unsigned_bits(fast_correction_message, 174, 4, 0);   // UDREI 0
    corrections.push(Sbas_Raw_Message(100.0, 136, fast_correction_message));
    corrections.update(2400, 101.0, 100.0);

    const auto& fast_correction = corrections.satellite_corrections().sat[0].fcorr;
    EXPECT_DOUBLE_EQ(fast_correction.prc, 10.0);
    EXPECT_EQ(fast_correction.udre, 1);
    EXPECT_EQ(fast_correction.iodf, 1);

    int correction_week = 0;
    const double correction_tow = time2gpst(fast_correction.t0, &correction_week);
    EXPECT_EQ(correction_week, 2400);
    EXPECT_NEAR(correction_tow, 101.0, 1e-9);

    auto long_correction_message = make_message(25);
    const int32_t half_positions[2] = {14, 120};
    for (const int32_t half_position : half_positions)
        {
            set_unsigned_bits(long_correction_message, half_position + 103, 2, 2);  // IODP
            const int32_t correction_positions[2] = {half_position + 1, half_position + 52};
            for (const int32_t correction_position : correction_positions)
                {
                    set_unsigned_bits(long_correction_message, correction_position, 6, 1);       // Mask slot 1
                    set_unsigned_bits(long_correction_message, correction_position + 6, 8, 7);   // IODE
                    set_unsigned_bits(long_correction_message, correction_position + 14, 9, 8);  // 1 m X offset
                }
        }
    corrections.push(Sbas_Raw_Message(102.0, 136, long_correction_message));
    corrections.update(2400, 102.0, 102.0);

    const auto& long_correction = corrections.satellite_corrections().sat[0].lcorr;
    EXPECT_EQ(long_correction.iode, 7);
    EXPECT_DOUBLE_EQ(long_correction.dpos[0], 1.0);

    nav_t navigation_data{};
    corrections.copy_to(navigation_data);
    EXPECT_DOUBLE_EQ(navigation_data.sbssat.sat[0].fcorr.prc, 10.0);
    EXPECT_EQ(navigation_data.ns, NSATSBS * 2);
    EXPECT_NE(navigation_data.seph, nullptr);
}


TEST(SbasRtklibCorrectionsTest, DecodesIonosphericGridDelay)
{
    using namespace sbas_rtklib_corrections_test;
    Sbas_Rtklib_Corrections corrections(136);

    auto igp_mask_message = make_message(18);
    set_unsigned_bits(igp_mask_message, 18, 4, 4);  // IGP band 4
    set_unsigned_bits(igp_mask_message, 22, 2, 1);  // IODI
    set_unsigned_bits(igp_mask_message, 24, 1, 1);  // First IGP in the band
    corrections.push(Sbas_Raw_Message(200.0, 136, igp_mask_message));
    corrections.update(2400, 200.0, 200.0);

    ASSERT_EQ(corrections.ionosphere_corrections()[4].iodi, 1);
    ASSERT_EQ(corrections.ionosphere_corrections()[4].nigp, 1);

    auto ionosphere_message = make_message(26);
    set_unsigned_bits(ionosphere_message, 14, 4, 4);   // IGP band 4
    set_unsigned_bits(ionosphere_message, 18, 4, 0);   // Block 0
    set_unsigned_bits(ionosphere_message, 22, 9, 40);  // 40 * 0.125 m = 5 m
    set_unsigned_bits(ionosphere_message, 31, 4, 2);   // GIVEI 2
    set_unsigned_bits(ionosphere_message, 217, 2, 1);  // IODI
    corrections.push(Sbas_Raw_Message(201.0, 136, ionosphere_message));
    corrections.update(2400, 201.0, 201.0);

    const auto& igp = corrections.ionosphere_corrections()[4].igp[0];
    EXPECT_FLOAT_EQ(igp.delay, 5.0F);
    EXPECT_EQ(igp.give, 3);
}


TEST(SbasRtklibCorrectionsTest, KeepsOneCorrectionStreamAndDefersFutureMessages)
{
    using namespace sbas_rtklib_corrections_test;
    Sbas_Rtklib_Corrections corrections;

    auto first_mask = make_message(1);
    set_unsigned_bits(first_mask, 14, 1, 1);
    set_unsigned_bits(first_mask, 224, 2, 1);
    corrections.push(Sbas_Raw_Message(10.0, 123, first_mask));

    auto other_mask = make_message(1);
    set_unsigned_bits(other_mask, 14, 1, 1);
    set_unsigned_bits(other_mask, 224, 2, 3);
    corrections.push(Sbas_Raw_Message(10.1, 136, other_mask));

    auto future_mask = make_message(1);
    set_unsigned_bits(future_mask, 14, 1, 1);
    set_unsigned_bits(future_mask, 224, 2, 2);
    corrections.push(Sbas_Raw_Message(12.0, 123, future_mask));

    corrections.update(2400, 10.0, 10.0);
    EXPECT_EQ(corrections.active_prn(), 123);
    EXPECT_EQ(corrections.satellite_corrections().iodp, 1);
    EXPECT_EQ(corrections.pending_messages(), 1U);

    corrections.update(2400, 12.0, 12.0);
    EXPECT_EQ(corrections.satellite_corrections().iodp, 2);
    EXPECT_EQ(corrections.pending_messages(), 0U);
}

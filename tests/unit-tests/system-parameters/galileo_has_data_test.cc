/*!
 * \file galileo_has_data_test.cc
 * \brief Tests for the validity interval and clock correction sentinels of
 * the Galileo HAS data class
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "Galileo_CNAV.h"
#include "galileo_has_data.h"
#include <gtest/gtest.h>
#include <cstdint>
#include <vector>

namespace
{
// HAS data with a single Galileo satellite (PRN 1) in the mask, in the clock
// full-set block and in the clock subset block
Galileo_HAS_data make_single_satellite_clock_data(int16_t delta_clock_correction, uint8_t multiplier)
{
    Galileo_HAS_data has_data;
    has_data.Nsys = 1;
    has_data.gnss_id_mask = {HAS_MSG_GALILEO_SYSTEM};
    has_data.satellite_mask = {uint64_t{1} << (HAS_MSG_NUMBER_SATELLITE_IDS - 1)};  // MSB is Satellite Index 0 (PRN 1)
    has_data.delta_clock_multiplier = {multiplier};
    has_data.delta_clock_correction = {delta_clock_correction};
    has_data.Nsys_sub = 1;
    has_data.gnss_id_clock_subset = {HAS_MSG_GALILEO_SYSTEM};
    has_data.satellite_submask = {1};  // one bit, set, for the only satellite in the mask
    has_data.delta_clock_multiplier_clock_subset = {multiplier};
    has_data.delta_clock_correction_clock_subset = {{delta_clock_correction}};
    return has_data;
}


bool is_inside_interval(uint32_t t_mt1, uint32_t validity_interval_s, uint32_t tow)
{
    const uint32_t elapsed_s = (tow % GALILEO_HAS_SECONDS_PER_WEEK + GALILEO_HAS_SECONDS_PER_WEEK - t_mt1) % GALILEO_HAS_SECONDS_PER_WEEK;
    return elapsed_s < validity_interval_s;
}
}  // namespace


TEST(HAS_Data_Test, ValidityIntervalAcrossHalfWeek)
{
    // t_MT1 ten seconds before TOW 302400 with VI = 300 s
    const uint32_t valid_until = 302390U + 300U;
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302390U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302395U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302399U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302400U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302689U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(302390U, valid_until, 302690U));
}


TEST(HAS_Data_Test, ValidityIntervalAcrossWeekRollover)
{
    // t_MT1 one hundred seconds before the end of the week with VI = 300 s
    const uint32_t valid_until = 604700U + 300U;
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(604700U, valid_until, 604700U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(604700U, valid_until, 604799U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(604700U, valid_until, 0U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(604700U, valid_until, 199U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(604700U, valid_until, 200U));
}


TEST(HAS_Data_Test, ValidityIntervalRejectsExpiredCorrections)
{
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(700U, 1000U, 1000U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(700U, 1000U, 1001U));
    // expired more than half a week ago
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(700U, 1000U, 400000U));
    // expired in the previous week
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(604490U, 604790U, 100U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(604490U, 604790U, 302500U));
}


TEST(HAS_Data_Test, ValidityIntervalMatchesDefinition)
{
    // HAS SIS ICD 1.0 Section 5.2.2.1: the interval starts at t_MT1 and
    // lasts VI seconds
    const std::vector<uint32_t> message_times = {0U, 1U, 3599U, 302390U, 302399U, 302400U, 302401U, 604199U, 604799U};
    for (const auto t_mt1 : message_times)
        {
            for (const auto& validity_interval : HAS_VALIDITY_INTERVALS)
                {
                    const uint32_t vi_s = validity_interval.second;
                    const uint32_t valid_until = t_mt1 + vi_s;
                    for (int32_t elapsed_s = -10; elapsed_s <= static_cast<int32_t>(vi_s) + 10; elapsed_s++)
                        {
                            const uint32_t tow = (static_cast<int64_t>(t_mt1) + elapsed_s + GALILEO_HAS_SECONDS_PER_WEEK) % GALILEO_HAS_SECONDS_PER_WEEK;
                            EXPECT_EQ(Galileo_HAS_data::has_validity_interval_at_tow(t_mt1, valid_until, tow),
                                is_inside_interval(t_mt1, vi_s, tow))
                                << "t_MT1=" << t_mt1 << " VI=" << vi_s << " tow=" << tow;
                        }
                }
        }
}


TEST(HAS_Data_Test, ClockSentinelsAreCheckedBeforeMultiplier)
{
    for (uint8_t multiplier = 1; multiplier <= 4; multiplier++)
        {
            const auto unavailable = make_single_satellite_clock_data(HAS_MSG_DELTA_CLOCK_UNAVAILABLE_RAW, multiplier);
            EXPECT_TRUE(Galileo_HAS_data::is_clock_unavailable(unavailable.get_clock_correction_mult_m("Galileo", 1)));
            EXPECT_FALSE(Galileo_HAS_data::is_clock_do_not_use(unavailable.get_clock_correction_mult_m("Galileo", 1)));
            EXPECT_TRUE(Galileo_HAS_data::is_clock_unavailable(unavailable.get_clock_subset_correction_mult_m("Galileo", 1)));
            EXPECT_FALSE(Galileo_HAS_data::is_clock_do_not_use(unavailable.get_clock_subset_correction_mult_m("Galileo", 1)));

            const auto do_not_use = make_single_satellite_clock_data(HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW, multiplier);
            EXPECT_TRUE(Galileo_HAS_data::is_clock_do_not_use(do_not_use.get_clock_correction_mult_m("Galileo", 1)));
            EXPECT_FALSE(Galileo_HAS_data::is_clock_unavailable(do_not_use.get_clock_correction_mult_m("Galileo", 1)));
            EXPECT_TRUE(Galileo_HAS_data::is_clock_do_not_use(do_not_use.get_clock_subset_correction_mult_m("Galileo", 1)));
            EXPECT_FALSE(Galileo_HAS_data::is_clock_unavailable(do_not_use.get_clock_subset_correction_mult_m("Galileo", 1)));
        }
}


TEST(HAS_Data_Test, ValidClockCorrectionsAreNeverSentinels)
{
    // Every transmissible DCC except the two sentinels, with every multiplier.
    // Before the fix, (DCM, DCC) = (2, -2048), (3, 1365), (3, 2730), (4, -1024),
    // (4, -2048) and (4, -3072) were misread as sentinels.
    for (uint8_t multiplier = 1; multiplier <= 4; multiplier++)
        {
            for (int32_t dcc = HAS_MSG_DELTA_CLOCK_UNAVAILABLE_RAW + 1; dcc < HAS_MSG_DELTA_CLOCK_DO_NOT_USE_RAW; dcc++)
                {
                    const auto has_data = make_single_satellite_clock_data(static_cast<int16_t>(dcc), multiplier);
                    const float expected_m = static_cast<float>(dcc) * HAS_MSG_DELTA_CLOCK_SCALE_FACTOR * static_cast<float>(multiplier);
                    const float full_set_m = has_data.get_clock_correction_mult_m("Galileo", 1);
                    const float subset_m = has_data.get_clock_subset_correction_mult_m("Galileo", 1);
                    EXPECT_FALSE(Galileo_HAS_data::is_clock_unavailable(full_set_m)) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                    EXPECT_FALSE(Galileo_HAS_data::is_clock_do_not_use(full_set_m)) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                    EXPECT_FALSE(Galileo_HAS_data::is_clock_unavailable(subset_m)) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                    EXPECT_FALSE(Galileo_HAS_data::is_clock_do_not_use(subset_m)) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                    EXPECT_NEAR(full_set_m, expected_m, 1e-4) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                    EXPECT_NEAR(subset_m, expected_m, 1e-4) << "DCM=" << static_cast<int>(multiplier) << " DCC=" << dcc;
                }
        }
}


TEST(HAS_Data_Test, ValidityIntervalRequiresBothBoundaries)
{
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 610U, 599U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 610U, 600U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 610U, 609U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 610U, 610U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(0U, 5U, 604799U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 600U, 600U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 599U, 600U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 4201U, 600U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_at_tow(600U, 610U, GALILEO_HAS_INVALID_TOW));
}


TEST(HAS_Data_Test, ExpiryDoesNotDiscardFutureIntervals)
{
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_expired_at_tow(610U, 599U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_expired_at_tow(610U, 609U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_expired_at_tow(610U, 610U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_expired_at_tow(610U, 611U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_expired_at_tow(604810U, 604799U));
    EXPECT_FALSE(Galileo_HAS_data::has_validity_interval_expired_at_tow(604810U, 9U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_expired_at_tow(604810U, 10U));
    EXPECT_TRUE(Galileo_HAS_data::has_validity_interval_expired_at_tow(604790U, 0U));
}

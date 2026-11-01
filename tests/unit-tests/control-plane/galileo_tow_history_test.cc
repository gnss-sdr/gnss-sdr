/*!
 * \file galileo_tow_history_test.cc
 * \brief Checks the deterministic Galileo time of week handoff to the E6 decoders
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

#include "deterministic_clock.h"
#include "galileo_telemetry_decoder_gs.h"
#include "galileo_tow_map.h"
#include "gnss_satellite.h"
#include "tlm_conf.h"
#include <gtest/gtest.h>
#include <cstdint>


class GalileoTowHistoryTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        GalileoTowHistory::instance().reset();
    }

    void TearDown() override
    {
        GalileoTowHistory::instance().reset();
        DeterministicClock::instance().disable();
    }

    static GalileoTowMapEntry entry(uint32_t tow_ms, uint64_t sample_counter)
    {
        return GalileoTowMapEntry{1300, tow_ms, sample_counter};
    }

    GalileoTowHistory& history = GalileoTowHistory::instance();
};


TEST_F(GalileoTowHistoryTest, LookupReturnsTheEntryInForceAtTheSample)
{
    history.record(11, 2, 100, entry(5000, 100));
    history.record(11, 2, 400, entry(5300, 400));

    EXPECT_EQ(history.lookup(11, 99).week, GALILEO_TOW_MAP_INVALID_WEEK);
    EXPECT_EQ(history.lookup(11, 100).tow_ms, 5000U);
    EXPECT_EQ(history.lookup(11, 399).tow_ms, 5000U);
    EXPECT_EQ(history.lookup(11, 1000).tow_ms, 5300U);
    EXPECT_EQ(history.lookup(12, 1000).week, GALILEO_TOW_MAP_INVALID_WEEK);
}


TEST_F(GalileoTowHistoryTest, WithdrawalHidesOnlyItsChannel)
{
    history.record(11, 5, 50, entry(4950, 50));
    history.record(11, 2, 100, entry(5000, 100));
    history.record(11, 2, 200, GalileoTowMapEntry{});

    EXPECT_EQ(history.lookup(11, 150).tow_ms, 5000U);
    EXPECT_EQ(history.lookup(11, 250).tow_ms, 4950U);
}


TEST_F(GalileoTowHistoryTest, WithdrawalWinsOverAnEntryWithTheSameStamp)
{
    history.record(11, 2, 100, entry(5000, 100));
    history.record(11, 2, 100, GalileoTowMapEntry{});
    EXPECT_EQ(history.lookup(11, 100).week, GALILEO_TOW_MAP_INVALID_WEEK);

    history.reset();
    history.record(11, 2, 100, GalileoTowMapEntry{});
    history.record(11, 2, 100, entry(5000, 100));
    EXPECT_EQ(history.lookup(11, 100).week, GALILEO_TOW_MAP_INVALID_WEEK);
}


TEST_F(GalileoTowHistoryTest, NewestEntryWinsAndTiesGoToTheLowestChannel)
{
    history.record(11, 7, 100, entry(5007, 100));
    history.record(11, 3, 100, entry(5003, 100));
    EXPECT_EQ(history.lookup(11, 100).tow_ms, 5003U);

    history.record(11, 9, 120, entry(5020, 120));
    EXPECT_EQ(history.lookup(11, 119).tow_ms, 5003U);
    EXPECT_EQ(history.lookup(11, 120).tow_ms, 5020U);
}


TEST_F(GalileoTowHistoryTest, RetentionKeepsTheEntryInForce)
{
    history.reset(1000);
    history.record(11, 2, 100, entry(5000, 100));
    history.record(11, 2, 500, entry(5400, 500));
    history.record(11, 2, 2000, entry(6900, 2000));

    EXPECT_EQ(history.lookup(11, 1000).tow_ms, 5400U);
    EXPECT_EQ(history.lookup(11, 3000).tow_ms, 6900U);
    // Older than the retention
    EXPECT_EQ(history.lookup(11, 499).week, GALILEO_TOW_MAP_INVALID_WEEK);
}


TEST_F(GalileoTowHistoryTest, ResetForgetsTheRecords)
{
    history.record(11, 2, 100, entry(5000, 100));
    history.reset();
    EXPECT_EQ(history.lookup(11, 1000).week, GALILEO_TOW_MAP_INVALID_WEEK);
}


TEST_F(GalileoTowHistoryTest, SourceDecoderResetWithdrawsItsEntryAtTheEventStamp)
{
    DeterministicClock::instance().enable(1000.0, 100, 100);
    Tlm_Conf conf{};
    conf.there_are_e6_channels = true;
    auto e1_decoder = galileo_make_telemetry_decoder_gs(conf, 1);
    e1_decoder->set_channel(3);
    e1_decoder->set_satellite(Gnss_Satellite("Galileo", 11));  // no stamp, nothing recorded
    EXPECT_EQ(history.lookup(11, 1000).week, GALILEO_TOW_MAP_INVALID_WEEK);

    history.record(11, 3, 100, entry(5000, 100));
    {
        const DeterministicClock::Scope cause(200);
        e1_decoder->reset();
    }
    EXPECT_EQ(history.lookup(11, 199).tow_ms, 5000U);
    EXPECT_EQ(history.lookup(11, 200).week, GALILEO_TOW_MAP_INVALID_WEEK);
}


TEST_F(GalileoTowHistoryTest, E6DecoderResetRecordsNothing)
{
    DeterministicClock::instance().enable(1000.0, 100, 100);
    Tlm_Conf conf{};
    conf.there_are_e6_channels = true;
    auto e6_decoder = galileo_make_telemetry_decoder_gs(conf, 3);
    e6_decoder->set_channel(4);
    e6_decoder->set_satellite(Gnss_Satellite("Galileo", 11));

    history.record(11, 4, 100, entry(5000, 100));
    {
        const DeterministicClock::Scope cause(200);
        e6_decoder->reset();
    }
    EXPECT_EQ(history.lookup(11, 300).tow_ms, 5000U);
}

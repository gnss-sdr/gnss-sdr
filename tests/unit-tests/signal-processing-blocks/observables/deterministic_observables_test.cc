/*!
 * \file deterministic_observables_test.cc
 * \brief Checks progress bounds for deterministic observables with short lookaheads
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
#include "gnss_synchro.h"
#include "hybrid_observables_gs.h"
#include "obs_conf.h"
#include "stream_batcher_limits.h"
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <vector>


class DeterministicObservablesTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        DeterministicClock::instance().enable(4000000.0, 4000, 400000);
    }

    void TearDown() override
    {
        DeterministicClock::instance().disable();
    }

    void check_clock_progress(bool always_output, bool expand_lookahead)
    {
        Obs_Conf conf;
        conf.nchannels_in = 2;
        conf.nchannels_out = 1;
        conf.always_output_gs = always_output;
        auto observables = hybrid_observables_gs_make(conf);
        auto& clock = DeterministicClock::instance();
        if (expand_lookahead)
            {
                // Flowgraph construction creates observables before channels,
                // whose acquisition/tracking spans may raise the lookahead.
                clock.require_span(400000);
            }

        // Allocate stream buffers, then drive work manually to control exactly
        // when the downstream PVT consumer finishes each output epoch.
        const auto item_size = sizeof(Gnss_Synchro);
        auto channel_source = gr::blocks::vector_source_b::make(std::vector<uint8_t>{}, false, item_size);
        auto clock_source = gr::blocks::vector_source_b::make(std::vector<uint8_t>{}, false, item_size);
        auto pvt_sink = gr::blocks::null_sink::make(item_size);
        auto top_block = gr::make_top_block("deterministic_observables_test");
        top_block->connect(channel_source, 0, observables, 0);
        top_block->connect(clock_source, 0, observables, 1);
        top_block->connect(observables, 0, pvt_sink, 0);
        top_block->run();

        Gnss_Synchro channel{};
        Gnss_Synchro clock_input{};
        Gnss_Synchro output{};
        clock_input.fs = 4000000;
        gr_vector_const_void_star inputs{&channel, &clock_input};
        gr_vector_void_star outputs{&output};
        gr_vector_int available{0, 1};
        const uint64_t samples_per_tick = conf.observable_interval_ms * 4000ULL;
        const uint64_t history_ticks = observables_clock_ticks(conf.observable_interval_ms);
        const uint64_t quantum = std::max<uint64_t>(1, clock.lookahead() / 8);
        for (uint64_t tick = 1; tick <= 50; ++tick)
            {
                SCOPED_TRACE(tick);
                clock_input.Tracking_sample_counter = tick * samples_per_tick;
                const uint64_t horizon = clock.min_progress() + clock.lookahead();
                // The same rounding used by the control thread must still
                // release the next clock tick; otherwise the graph deadlocks.
                ASSERT_GE(horizon - horizon % quantum, clock_input.Tracking_sample_counter);
                const int produced = observables->general_work(1, available, inputs, outputs);
                ASSERT_EQ(produced, always_output ? 1 : 0);
                if (produced > 0)
                    {
                        observables->produce(0, produced);
                        const uint64_t pending_progress = clock.min_progress();
                        pvt_sink->consume_each(produced);
                        if (tick >= history_ticks)
                            {
                                // An unread PVT epoch must retain its bound,
                                // even though observables has computed it.
                                EXPECT_GT(clock.min_progress(), pending_progress);
                            }
                    }
            }
    }
};


TEST_F(DeterministicObservablesTest, ShortLookaheadWithPvtOutput)
{
    check_clock_progress(true, false);
}


TEST_F(DeterministicObservablesTest, ShortLookaheadWithoutValidObservations)
{
    check_clock_progress(false, false);
}


TEST_F(DeterministicObservablesTest, ExpandedLookaheadWithPvtOutput)
{
    check_clock_progress(true, true);
}


TEST_F(DeterministicObservablesTest, ExpandedLookaheadWithoutValidObservations)
{
    check_clock_progress(false, true);
}

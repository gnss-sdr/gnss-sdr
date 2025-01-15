/*!
 * \file gnss_sdr_valve_test.cc
 * \brief  This file implements unit tests for the valve custom block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/top_block.h>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_f.h>
#endif
#include "concurrent_queue.h"
#include "gnss_sdr_valve.h"
#include <gnuradio/blocks/null_sink.h>
#include <pmt/pmt.h>

TEST(ValveTest, CheckEventSentAfter100Samples)
{
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    auto top_block = gr::make_top_block("gnss_sdr_valve_test");

    auto source = gr::analog::sig_source_f::make(100, gr::analog::GR_CONST_WAVE, 100, 1, 0);
    auto valve = gnss_sdr_make_valve(sizeof(float), 100, queue.get());
    auto sink = gr::blocks::null_sink::make(sizeof(float));

    bool expected0 = false;
    pmt::pmt_t msg;
    EXPECT_EQ(expected0, queue->timed_wait_and_pop(msg, 100));

    top_block->connect(source, 0, valve, 0);
    top_block->connect(valve, 0, sink, 0);

    top_block->run();
    top_block->stop();

    bool expected1 = true;
    EXPECT_EQ(expected1, queue->timed_wait_and_pop(msg, 100));
}

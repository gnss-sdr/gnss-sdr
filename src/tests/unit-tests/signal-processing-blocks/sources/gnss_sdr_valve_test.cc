/*!
 * \file gnss_sdr_valve_test.cc
 * \brief  This file implements unit tests for the valve custom block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
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
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    gr::top_block_sptr top_block = gr::make_top_block("gnss_sdr_valve_test");

    gr::analog::sig_source_f::sptr source = gr::analog::sig_source_f::make(100, gr::analog::GR_CONST_WAVE, 100, 1, 0);
    boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(float), 100, queue);
    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(float));

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

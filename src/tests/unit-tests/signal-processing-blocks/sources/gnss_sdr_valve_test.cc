/*!
 * \file gnss_sdr_valve_test.cc
 * \brief  This file implements unit tests for the valve custom block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#include <gnuradio/top_block.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_f.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/msg_queue.h>
#include "gnss_sdr_valve.h"

TEST(ValveTest, CheckEventSentAfter100Samples)
{
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    gr::top_block_sptr top_block = gr::make_top_block("gnss_sdr_valve_test");

    gr::analog::sig_source_f::sptr source = gr::analog::sig_source_f::make(100, gr::analog::GR_CONST_WAVE, 100, 1, 0);
    boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(float), 100, queue);
    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(float));

    unsigned int expected0 = 0;
    EXPECT_EQ(expected0, queue->count());

    top_block->connect(source, 0, valve, 0);
    top_block->connect(valve, 0, sink, 0);

    top_block->run();
    top_block->stop();

    unsigned int expected1 = 1;
    EXPECT_EQ(expected1, queue->count());
}

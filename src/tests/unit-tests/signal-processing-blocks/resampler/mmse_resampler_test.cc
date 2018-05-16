/*!
 * \file mmse_resampler_test.cc
 * \brief  Executes a resampler based on some input parameters.
 * \author Carles Fernandez-Prades 2018 cfernandez (at) cttc.cat
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

#include <chrono>
#include <gnuradio/top_block.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include "gnss_sdr_valve.h"
#include "mmse_resampler_conditioner.h"

TEST(MmseResamplerTest, InstantiationAndRunTestWarning)
{
    double fs_in = 8000000.0;   // Input sampling frequency in Hz
    double fs_out = 4000000.0;  // sampling freuqncy of the resampled signal in Hz
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int nsamples = 1000000;  //Number of samples to be computed
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    gr::top_block_sptr top_block = gr::make_top_block("mmse_resampler_conditioner_cc_test");
    boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
    boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);

    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Resampler.sample_freq_in", std::to_string(fs_in));
    config->set_property("Resampler.sample_freq_out", std::to_string(fs_out));

    std::shared_ptr<MmseResamplerConditioner> resampler = std::make_shared<MmseResamplerConditioner>(config.get(), "Resampler", 1, 1);

    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(gr_complex));

    EXPECT_NO_THROW({
        resampler->connect(top_block);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, resampler->get_left_block(), 0);
        top_block->connect(resampler->get_right_block(), 0, sink, 0);
    }) << "Connection failure of direct_resampler_conditioner.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
        top_block->stop();
    }) << "Failure running direct_resampler_conditioner.";

    std::cout << "Resampled " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST(MmseResamplerTest, InstantiationAndRunTest2)
{
    double fs_in = 8000000.0;   // Input sampling frequency in Hz
    double fs_out = 4000000.0;  // sampling freuqncy of the resampled signal in Hz
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int nsamples = 1000000;  //Number of samples to be computed
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    gr::top_block_sptr top_block = gr::make_top_block("mmse_resampler_conditioner_cc_test");
    boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
    boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);

    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Resampler.sample_freq_in", std::to_string(fs_in));
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_out));

    std::shared_ptr<MmseResamplerConditioner> resampler = std::make_shared<MmseResamplerConditioner>(config.get(), "Resampler", 1, 1);

    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(gr_complex));

    EXPECT_NO_THROW({
        resampler->connect(top_block);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, resampler->get_left_block(), 0);
        top_block->connect(resampler->get_right_block(), 0, sink, 0);
    }) << "Connection failure of direct_resampler_conditioner.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
        top_block->stop();
    }) << "Failure running direct_resampler_conditioner.";

    std::cout << "Resampled " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

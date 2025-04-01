/*!
 * \file mmse_resampler_test.cc
 * \brief  Executes a resampler based on some input parameters.
 * \author Carles Fernandez-Prades 2018 cfernandez (at) cttc.cat
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
#include <chrono>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include "concurrent_queue.h"
#include "gnss_sdr_valve.h"
#include "mmse_resampler_conditioner.h"
#include <gnuradio/blocks/null_sink.h>

TEST(MmseResamplerTest, InstantiationAndRunTestWarning)
{
    double fs_in = 8000000.0;   // Input sampling frequency in Hz
    double fs_out = 4000000.0;  // sampling freuqncy of the resampled signal in Hz
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int nsamples = 1000000;  // Number of samples to be computed
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto top_block = gr::make_top_block("mmse_resampler_conditioner_cc_test");
    auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
    auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());

    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Resampler.sample_freq_in", std::to_string(fs_in));
    config->set_property("Resampler.sample_freq_out", std::to_string(fs_out));

    auto resampler = std::make_shared<MmseResamplerConditioner>(config.get(), "Resampler", 1, 1);

    auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));

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

    std::cout << "Resampled " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST(MmseResamplerTest, InstantiationAndRunTest2)
{
    double fs_in = 8000000.0;   // Input sampling frequency in Hz
    double fs_out = 4000000.0;  // sampling freuqncy of the resampled signal in Hz
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int nsamples = 1000000;  // Number of samples to be computed
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto top_block = gr::make_top_block("mmse_resampler_conditioner_cc_test");
    auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
    auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());

    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Resampler.sample_freq_in", std::to_string(fs_in));
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_out));

    auto resampler = std::make_shared<MmseResamplerConditioner>(config.get(), "Resampler", 1, 1);

    auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));

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

    std::cout << "Resampled " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

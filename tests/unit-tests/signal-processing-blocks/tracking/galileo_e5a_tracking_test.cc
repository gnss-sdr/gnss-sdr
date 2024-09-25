/*!
 * \file galileo_e1_dll_pll_veml_tracking_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "concurrent_queue.h"
#include "galileo_e5a_dll_pll_tracking.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <chrono>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif


class GalileoE5aTrackingTest : public ::testing::Test
{
protected:
    GalileoE5aTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GalileoE5aTrackingTest() = default;

    void init();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
};


void GalileoE5aTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "5X";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("GNSS-SDR.internal_fs_sps", "32000000");
    config->set_property("Tracking_5X.implementation", "Galileo_E5a_DLL_PLL_Tracking");
    config->set_property("Tracking_5X.item_type", "gr_complex");
    config->set_property("Tracking_5X.dump", "false");
    config->set_property("Tracking_5X.dump_filename", "../data/e5a_tracking_ch_");
    config->set_property("Tracking_5X.early_late_space_chips", "0.5");
    config->set_property("Tracking_5X.order", "2");
    config->set_property("Tracking_5X.pll_bw_hz", "20.0");
    config->set_property("Tracking_5X.dll_bw_hz", "5.0");
    config->set_property("Tracking_5X.pll_bw_narrow_hz", "2.0");
    config->set_property("Tracking_5X.pll_bw_narrow_hz", "2.0");
    config->set_property("Tracking_5X.ti_ms", "1");
}


TEST_F(GalileoE5aTrackingTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int fs_in = 32000000;
    int nsamples = 32000000 * 5;
    init();
    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config.get(), "Tracking_5X", 1, 1);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

    // REAL
    gnss_synchro.Acq_delay_samples = 10;  // 32 Msps
    //    gnss_synchro.Acq_doppler_hz = 3500; // 32 Msps
    gnss_synchro.Acq_doppler_hz = 2000;  // 500 Hz resolution
    //    gnss_synchro.Acq_samplestamp_samples = 98000;
    gnss_synchro.Acq_samplestamp_samples = 0;

    ASSERT_NO_THROW({
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block.";

    ASSERT_NO_THROW({
        gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
    }) << "Failure connecting the blocks of tracking test.";

    tracking->start_tracking();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    std::cout << "Tracked " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

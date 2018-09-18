/*!
 * \file galileo_e1_dll_pll_veml_tracking_test.cc
 * \brief  This class implements a tracking test for GalileoE1DllPllVemlTracking
 *  class based on some input parameters.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "galileo_e1_dll_pll_veml_tracking.h"


class GalileoE1DllPllVemlTrackingInternalTest : public ::testing::Test
{
protected:
    GalileoE1DllPllVemlTrackingInternalTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GalileoE1DllPllVemlTrackingInternalTest()
    {
    }

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
};


void GalileoE1DllPllVemlTrackingInternalTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1B";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("GNSS-SDR.internal_fs_sps", "8000000");
    config->set_property("Tracking_1B.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B.item_type", "gr_complex");
    config->set_property("Tracking_1B.dump", "false");
    config->set_property("Tracking_1B.dump_filename", "../data/veml_tracking_ch_");
    config->set_property("Tracking_1B.early_late_space_chips", "0.15");
    config->set_property("Tracking_1B.very_early_late_space_chips", "0.6");
    config->set_property("Tracking_1B.pll_bw_hz", "30.0");
    config->set_property("Tracking_1B.dll_bw_hz", "2.0");
}


TEST_F(GalileoE1DllPllVemlTrackingInternalTest, Instantiate)
{
    init();
    auto tracking = factory->GetBlock(config, "Tracking_1B", "Galileo_E1_DLL_PLL_VEML_Tracking", 1, 1);
    EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());
}


TEST_F(GalileoE1DllPllVemlTrackingInternalTest, ConnectAndRun)
{
    int fs_in = 8000000;
    int nsamples = 40000000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking_1B", "Galileo_E1_DLL_PLL_VEML_Tracking", 1, 1);
    std::shared_ptr<GalileoE1DllPllVemlTracking> tracking = std::dynamic_pointer_cast<GalileoE1DllPllVemlTracking>(trk_);

    ASSERT_NO_THROW({
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        tracking->connect(top_block);
        gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
    }) << "Failure connecting the blocks of tracking test.";

    tracking->start_tracking();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  //Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(GalileoE1DllPllVemlTrackingInternalTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    // int num_samples = 40000000; // 4 Msps
    // unsigned int skiphead_sps = 24000000; // 4 Msps
    int num_samples = 80000000;           // 8 Msps
    unsigned int skiphead_sps = 8000000;  // 8 Msps
    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking_1B", "Galileo_E1_DLL_PLL_VEML_Tracking", 1, 1);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

    // gnss_synchro.Acq_delay_samples = 1753; // 4 Msps
    // gnss_synchro.Acq_doppler_hz = -9500; // 4 Msps
    gnss_synchro.Acq_delay_samples = 17256;  // 8 Msps
    gnss_synchro.Acq_doppler_hz = -8750;     // 8 Msps
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
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        gr::blocks::skiphead::sptr skip_head = gr::blocks::skiphead::make(sizeof(gr_complex), skiphead_sps);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), num_samples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, skip_head, 0);
        top_block->connect(skip_head, 0, valve, 0);
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

    std::cout << "Tracked " << num_samples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

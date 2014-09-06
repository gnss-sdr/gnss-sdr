/*!
 * \file galileo_e1_dll_pll_veml_tracking_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2014  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <ctime>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "galileo_e5a_dll_pll_tracking.h"


class GalileoE5aTrackingTest: public ::testing::Test
{
protected:
    GalileoE5aTrackingTest()
    {
        queue = gr::msg_queue::make(0);
        top_block = gr::make_top_block("Tracking test");
        std::shared_ptr<GNSSBlockFactory> factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
    }

    ~GalileoE5aTrackingTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    concurrent_queue<int> channel_internal_queue;
    bool stop;
    int message;
};


void GalileoE5aTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "5Q";
    signal.copy(gnss_synchro.Signal, 2, 0);


    config->set_property("GNSS-SDR.internal_fs_hz", "32000000");
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "../data/e5a_tracking_ch_");
    config->set_property("Tracking.implementation", "Galileo_E5a_DLL_PLL_Tracking");
    config->set_property("Tracking.early_late_space_chips", "0.5");

    config->set_property("Tracking.pll_bw_hz_init","20.0");
    //    config->set_property("Tracking.pll_bw_hz_init","5.0");
    config->set_property("Tracking.dll_bw_hz_init","2.0");

    config->set_property("Tracking.pll_bw_hz", "5");
    config->set_property("Tracking.dll_bw_hz", "2");
    config->set_property("Tracking.ti_ms","1");

    //    config->set_property("Tracking.pll_bw_hz", "5");
    //    config->set_property("Tracking.dll_bw_hz", "2");
    //    config->set_property("Tracking.ti_ms","1");
    //config->set_property("Tracking.fll_bw_hz", "10.0");
}
/*
TEST_F(GalileoE5aTrackingTest, InstantiateTrack)
{

    init();
    auto tracking = factory->GetBlock(config, "Tracking", "Galileo_E5a_DLL_PLL_Tracking", 1, 1, queue);
    EXPECT_STREQ("Galileo_E5a_DLL_PLL_Tracking", tracking->implementation().c_str());
//    auto tracking = factory->GetBlock(config, "Tracking", "Galileo_E1_DLL_PLL_VEML_Tracking", 1, 1, queue);
//    EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());

}*/
/*
TEST_F(GalileoE5aTrackingTest, ConnectAndRun)
{
    int fs_in = 21000000;
    int nsamples = 21000000;
    struct timeval tv;
    long long int begin;
    long long int end;
    init();

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "Galileo_E5a_DLL_PLL_Tracking", 1, 1, queue);
    std::shared_ptr<GalileoE5aDllPllTracking> tracking = std::dynamic_pointer_cast<GalileoE5aDllPllTracking>(trk_);

    ASSERT_NO_THROW( {
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;


    ASSERT_NO_THROW( {
        tracking->connect(top_block);
        gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);

    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run();   //Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}
 */
TEST_F(GalileoE5aTrackingTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int fs_in = 3200000000;
    int nsamples = 3200000000*1.5;
    //int num_samples = 320000000*1.5; // 32 Msps
    //unsigned int skiphead_sps = 98000; // 1 Msample
    unsigned int skiphead_sps = 0; // 1 Msampl
    //    unsigned int skiphead_sps = 104191; // 1 Msampl
    init();

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", "Galileo_E5a_DLL_PLL_Tracking", 1, 1, queue);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

    //REAL
    gnss_synchro.Acq_delay_samples = 15579+1; // 32 Msps
    //    gnss_synchro.Acq_doppler_hz = 3500; // 32 Msps
    gnss_synchro.Acq_doppler_hz = 3750; // 500 Hz resolution
    //    gnss_synchro.Acq_samplestamp_samples = 98000;
    gnss_synchro.Acq_samplestamp_samples = 0;
    //SIM
    //    gnss_synchro.Acq_delay_samples = 14001+1; // 32 Msps
    //    //gnss_synchro.Acq_doppler_hz = 2750; // 32 Msps (real 2800)
    ////    gnss_synchro.Acq_doppler_hz = 2800; // 32 Msps (real 2800)
    //    gnss_synchro.Acq_doppler_hz = 0; // 32 Msps (real 2800)
    ////    gnss_synchro.Acq_samplestamp_samples = 98000;
    //    gnss_synchro.Acq_samplestamp_samples = 0;

    //SIM2
    //    gnss_synchro.Acq_delay_samples = 5810; // 32 Msps
    //    gnss_synchro.Acq_doppler_hz = 2800;
    //    gnss_synchro.Acq_samplestamp_samples = 0;

    ASSERT_NO_THROW( {
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;

    ASSERT_NO_THROW( {
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);



        /**   std::string file =  "/home/marc/E5a_acquisitions/32MS_complex.dat";
        //std::string file =  "/home/marc/E5a_acquisitions/sim_32M_sec94_PRN11_long.dat";
        //std::string file =  "/home/marc/E5a_acquisitions/sim_32M_sec94_PRN11_long_0dopp.dat";
        gnss_synchro.PRN = 19;//real
        //gnss_synchro.PRN = 11;//sim

        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex),file_name,false);
        gr::blocks::skiphead::sptr skip_head = gr::blocks::skiphead::make(sizeof(gr_complex), skiphead_sps);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), num_samples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, skip_head, 0);
        top_block->connect(skip_head, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);*/
    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Tracked " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}


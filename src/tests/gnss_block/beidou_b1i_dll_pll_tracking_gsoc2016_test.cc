/*!
 * \file beidou_b1i_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for BeiDou_B1I_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Enric Juan, 2016. enric.juan.92(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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
#include "beidou_b1i_dll_pll_tracking.h"

//TODO: Test of BeiDou signal Tracking Block TBD
//!!!! All parameters has to be re-defined (at least revised)

class BeiDouB1iDllPllTrackingTest: public ::testing::Test
{
protected:
    BeiDouB1iDllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~BeiDouB1iDllPllTrackingTest(){};

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


void BeiDouB1iDllPllTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'B';
    std::string signal = "1I";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("GNSS-SDR.internal_fs_hz", "32000000");
    config->set_property("Tracking_BeiDou.item_type", "gr_complex");
    config->set_property("Tracking_BeiDou.dump", "false");
    config->set_property("Tracking_BeiDou.dump_filename", "../data/b1i_tracking_ch_");
    config->set_property("Tracking_BeiDou.implementation", "BeiDou_B1i_DLL_PLL_Tracking");
    //TODO: Discriminators TBD
    config->set_property("Tracking_BeiDou.early_late_space_chips", "0.5");
    config->set_property("Tracking_BeiDou.order", "2");
    config->set_property("Tracking_BeiDou.pll_bw_hz_init","20.0");
    config->set_property("Tracking_BeiDou.pll_bw_hz", "5");
    config->set_property("Tracking_BeiDou.dll_bw_hz_init","2.0");
    config->set_property("Tracking_BeiDou.dll_bw_hz", "2");
    config->set_property("Tracking_BeiDou.ti_ms", "1");
}

TEST_F(BeiDouB1iDllPllTrackingTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int fs_in = 32000000;
    int nsamples = 32000000*5;
    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    // Example using smart pointers and the block factory
    std::shared_ptr<GNSSBlockInterface> trk_ =
        factory->GetBlock(config, "Tracking", "BeiDou_B1i_DLL_PLL_Tracking", 1, 1);
    
    std::shared_ptr<TrackingInterface> tracking =
        std::dynamic_pointer_cast<TrackingInterface>(trk_);

    // REAL
    gnss_synchro.Acq_delay_samples = 10; // 32 Msps
    //    gnss_synchro.Acq_doppler_hz = 3500; // 32 Msps
    gnss_synchro.Acq_doppler_hz = 2000; // 500 Hz resolution
    //    gnss_synchro.Acq_samplestamp_samples = 98000;
    gnss_synchro.Acq_samplestamp_samples = 0;

    ASSERT_NO_THROW( {
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        gr::analog::sig_source_c::sptr source =
            gr::analog::sig_source_c::make(fs_in,
                                           gr::analog::GR_SIN_WAVE,
                                           1000,
                                           1,
                                           gr_complex(0));
        
        boost::shared_ptr<gr::block> valve =
            gnss_sdr_make_valve(sizeof(gr_complex),
                                nsamples,
                                queue);
        
        gr::blocks::null_sink::sptr sink =
            gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);

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


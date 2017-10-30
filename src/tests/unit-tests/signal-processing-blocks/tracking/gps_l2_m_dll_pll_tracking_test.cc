/*!
 * \file gps_l2_m_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#include <chrono>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l2_m_dll_pll_tracking.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL2MDllPllTrackingTest_msg_rx;

typedef boost::shared_ptr<GpsL2MDllPllTrackingTest_msg_rx> GpsL2MDllPllTrackingTest_msg_rx_sptr;

GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make();

class GpsL2MDllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL2MDllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~GpsL2MDllPllTrackingTest_msg_rx(); //!< Default destructor

};


GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make()
{
    return GpsL2MDllPllTrackingTest_msg_rx_sptr(new GpsL2MDllPllTrackingTest_msg_rx());
}


void GpsL2MDllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
    {
            long int message = pmt::to_long(msg);
            rx_message = message;
    }
    catch(boost::bad_any_cast& e)
    {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
    }
}


GpsL2MDllPllTrackingTest_msg_rx::GpsL2MDllPllTrackingTest_msg_rx() :
            gr::block("GpsL2MDllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL2MDllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL2MDllPllTrackingTest_msg_rx::~GpsL2MDllPllTrackingTest_msg_rx()
{}


// ###########################################################

class GpsL2MDllPllTrackingTest: public ::testing::Test
{
protected:
    GpsL2MDllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL2MDllPllTrackingTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GpsL2MDllPllTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "2S";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 7;

    config->set_property("GNSS-SDR.internal_fs_sps", "5000000");
    config->set_property("Tracking_2S.item_type", "gr_complex");
    config->set_property("Tracking_2S.dump", "false");
    config->set_property("Tracking_2S.dump_filename", "../data/L2m_tracking_ch_");
    config->set_property("Tracking_2S.implementation", "GPS_L2_M_DLL_PLL_Tracking");
    config->set_property("Tracking_2S.early_late_space_chips", "0.5");
    config->set_property("Tracking_2S.order", "2");
    config->set_property("Tracking_2S.pll_bw_hz", "2");
    config->set_property("Tracking_2S.dll_bw_hz", "0.5");
}


TEST_F(GpsL2MDllPllTrackingTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int fs_in = 5000000;
    int nsamples = fs_in*9;

    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL2MDllPllTracking>(config.get(), "Tracking_2S", 1, 1);
    boost::shared_ptr<GpsL2MDllPllTrackingTest_msg_rx> msg_rx = GpsL2MDllPllTrackingTest_msg_rx_make();

    gnss_synchro.Acq_delay_samples = 1;
    gnss_synchro.Acq_doppler_hz = 1200;
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
        //gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        std::string path = std::string(TEST_PATH);
        std::string file =  path + "signal_samples/gps_l2c_m_prn7_5msps.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    // TODO: Verify tracking results
    std::cout <<  "Tracked " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


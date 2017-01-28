/*!
 * \file gps_l1_ca_dll_pll_tracking_test.cc
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


#include <ctime>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_dll_pll_tracking.h"

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CADllPllTrackingTest_msg_rx;

typedef boost::shared_ptr<GpsL1CADllPllTrackingTest_msg_rx> GpsL1CADllPllTrackingTest_msg_rx_sptr;

GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make();

class GpsL1CADllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTrackingTest_msg_rx(); //!< Default destructor

};

GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make()
{
    return GpsL1CADllPllTrackingTest_msg_rx_sptr(new GpsL1CADllPllTrackingTest_msg_rx());
}

void GpsL1CADllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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

GpsL1CADllPllTrackingTest_msg_rx::GpsL1CADllPllTrackingTest_msg_rx() :
            gr::block("GpsL1CADllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL1CADllPllTrackingTest_msg_rx::~GpsL1CADllPllTrackingTest_msg_rx()
{}


// ###########################################################


class GpsL1CADllPllTrackingTest: public ::testing::Test
{
protected:
    GpsL1CADllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CADllPllTrackingTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GpsL1CADllPllTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    config->set_property("GNSS-SDR.internal_fs_hz", "2600000");
    // Set Tracking
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", "0");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "30.0");
    config->set_property("Tracking_1C.dll_bw_hz", "4.0");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
}

TEST_F(GpsL1CADllPllTrackingTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int fs_in = 2600000;
    //int nsamples = fs_in*3;

    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    boost::shared_ptr<GpsL1CADllPllTrackingTest_msg_rx> msg_rx = GpsL1CADllPllTrackingTest_msg_rx_make();

    gnss_synchro.Acq_delay_samples = (1023-994.622/1023)*fs_in*1e-3;
    gnss_synchro.Acq_doppler_hz = -2583.86;
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
        std::string path = std::string(TEST_PATH);
        std::string file =  path + "signal_samples/gps_l1ca_prn1_2.6msps.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        //boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::interleaved_char_to_complex::sptr  gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        //top_block->connect(gr_interleaved_char_to_complex, 0, valve, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of tracking test." << std::endl;

    tracking->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    // TODO: Verify tracking results
    std::cout <<  "Signal tracking completed in " << (end - begin) << " microseconds" << std::endl;
}


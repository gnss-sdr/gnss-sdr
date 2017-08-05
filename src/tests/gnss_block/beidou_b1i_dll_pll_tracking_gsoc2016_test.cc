/*!
 * \file beidou_b1i_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for BeiDou_B1I_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Enric Juan, 2016. enric.juan.92(at)gmail.com
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
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "beidou_b1i_dll_pll_tracking.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class BeiDouB1iDllPllTrackingTest_msg_rx;

typedef boost::shared_ptr <BeiDouB1iDllPllTrackingTest_msg_rx> BeiDouB1iDllPllTrackingTest_msg_rx_sptr;

BeiDouB1iDllPllTrackingTest_msg_rx_sptr BeiDouB1iDllPllTrackingTest_msg_rx_make();

class BeiDouB1iDllPllTrackingTest_msg_rx : public gr::block {
private:
    friend BeiDouB1iDllPllTrackingTest_msg_rx_sptr BeiDouB1iDllPllTrackingTest_msg_rx_make();

    void msg_handler_events(pmt::pmt_t msg);

    BeiDouB1iDllPllTrackingTest_msg_rx();

public:
    int rx_message;

    ~BeiDouB1iDllPllTrackingTest_msg_rx(); //!< Default destructor

};

BeiDouB1iDllPllTrackingTest_msg_rx_sptr BeiDouB1iDllPllTrackingTest_msg_rx_make() {
    return BeiDouB1iDllPllTrackingTest_msg_rx_sptr(new BeiDouB1iDllPllTrackingTest_msg_rx());
}

void BeiDouB1iDllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg) {
    try {
        long int message = pmt::to_long(msg);
        rx_message = message;
    }
    catch (boost::bad_any_cast &e) {
        LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
        rx_message = 0;
    }
}

BeiDouB1iDllPllTrackingTest_msg_rx::BeiDouB1iDllPllTrackingTest_msg_rx() :
        gr::block("BeiDouB1iDllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0),
                  gr::io_signature::make(0, 0, 0)) {
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
                          boost::bind(&BeiDouB1iDllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

BeiDouB1iDllPllTrackingTest_msg_rx::~BeiDouB1iDllPllTrackingTest_msg_rx() {}


// ###########################################################


class BeiDouB1iDllPllTrackingTest : public ::testing::Test {
protected:
    BeiDouB1iDllPllTrackingTest() {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~BeiDouB1iDllPllTrackingTest() {}

    void config_test();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr <GNSSBlockFactory> factory;
    std::shared_ptr <InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;

    double f_if = 0;
    double space_chips = 0;
    double pll_bw_hz = 0;
    double dll_bw_hz = 0;
    double delay_samples = 0;
    double doppler_hz = 0;

    int fs_in = 0;
    int nsamples = 0;

    long long int begin = 0;
    long long int end = 0;

    std::string signal_path = "";
};


void BeiDouB1iDllPllTrackingTest::config_test() {
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);

/******** CONFIGURATION PARAMETERS ********/
    gnss_synchro.PRN = 20;                  // PRN satellite number
    fs_in = 16000000;            // Sampling Frequency
    nsamples = fs_in * 10;
    f_if = 0.0;                 // Intermediate Frequency
    space_chips = 0.5;                 // Space b/w chips
    pll_bw_hz = 20;                  // PLL Bandwidth
    dll_bw_hz = 4;                   // DLL Bandwidth
    delay_samples = 3767.0;              // Code delay in samples
    doppler_hz = 1650;
    signal_path = "signal_samples/test_beidou_15s.dat";

/******************************************/

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.if", std::to_string(f_if));
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "../src/tests/data/tracking_beidou/tracking_ch_");
    config->set_property("Tracking.implementation", "BeiDou_B1i_DLL_PLL_Tracking");
    config->set_property("Tracking.early_late_space_chips", std::to_string(space_chips));
    config->set_property("Tracking.order", "2");
    config->set_property("Tracking.pll_bw_hz", std::to_string(pll_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(dll_bw_hz));
}

TEST_F(BeiDouB1iDllPllTrackingTest, ValidationOfResults
)
{
struct timeval tv;

config_test();

queue = gr::msg_queue::make(0);
top_block = gr::make_top_block("Tracking test");

std::shared_ptr <TrackingInterface> tracking = std::make_shared<BeiDouB1iDllPllTracking>(config.get(), "Tracking", 1,
                                                                                         1);
boost::shared_ptr <BeiDouB1iDllPllTrackingTest_msg_rx> msg_rx = BeiDouB1iDllPllTrackingTest_msg_rx_make();

gnss_synchro.
Acq_delay_samples = delay_samples;
gnss_synchro.
Acq_doppler_hz = doppler_hz;
gnss_synchro.
Acq_samplestamp_samples = 0;

ASSERT_NO_THROW( {
tracking->
set_channel(gnss_synchro
.Channel_ID);
}) << "Failure setting channel." <<
std::endl;

ASSERT_NO_THROW( {
tracking->
set_gnss_synchro(&gnss_synchro);
}) << "Failure setting gnss_synchro." <<
std::endl;

ASSERT_NO_THROW( {
tracking->
connect(top_block);
}) << "Failure connecting tracking to the top_block." <<
std::endl;

ASSERT_NO_THROW( {
std::string path = std::string(TEST_PATH);
std::string file = path + signal_path;
const char *file_name = file.c_str();

gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
boost::shared_ptr <gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));

top_block->
connect(file_source,
0, valve, 0);
top_block->
connect(valve,
0, tracking->

get_left_block(),

0);
top_block->
connect(tracking
->

get_right_block(),

0, sink, 0);
top_block->
msg_connect(tracking
->

get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events")

);
}) << "Failure connecting the blocks of tracking test." <<
std::endl;

tracking->

start_tracking();

EXPECT_NO_THROW( {
gettimeofday(&tv, NULL
);
begin = tv.tv_sec * 1000000 + tv.tv_usec;
top_block->

run(); // Start threads and wait
gettimeofday(&tv, NULL
);
end = tv.tv_sec * 1000000 + tv.tv_usec;
}) << "Failure running the top_block." <<
std::endl;

// TODO: Verify tracking results
std::cout <<  "Tracked " << nsamples << " samples in " << (end - begin) << " microseconds" <<
std::endl;
}



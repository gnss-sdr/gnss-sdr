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
#include <cstdlib>
#include <iostream>
#include <boost/chrono.hpp>
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
#include "BEIDOU_B1I.h"

#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class BeiDouB1iDllPllTrackingGenSourceTest_msg_rx;

typedef boost::shared_ptr<BeiDouB1iDllPllTrackingGenSourceTest_msg_rx> BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_sptr;

BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_sptr BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_make();

class BeiDouB1iDllPllTrackingGenSourceTest_msg_rx : public gr::block
{
private:
    friend BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_sptr BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    BeiDouB1iDllPllTrackingGenSourceTest_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~BeiDouB1iDllPllTrackingGenSourceTest_msg_rx(); //!< Default destructor

};

BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_sptr BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_make(concurrent_queue<int>& queue)
{
    return BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_sptr(new BeiDouB1iDllPllTrackingGenSourceTest_msg_rx(queue));
}

void BeiDouB1iDllPllTrackingGenSourceTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
    {
        long int message = pmt::to_long(msg);
        rx_message = message;
        channel_internal_queue.push(rx_message);
    }
    catch(boost::bad_any_cast& e)
    {
        LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
        rx_message = 0;
    }
}

BeiDouB1iDllPllTrackingGenSourceTest_msg_rx::BeiDouB1iDllPllTrackingGenSourceTest_msg_rx(concurrent_queue<int>& queue) :
        gr::block("BeiDouB1iDllPllTrackingGenSourceTest_msg_rx",
                  gr::io_signature::make(0, 0, 0),
                  gr::io_signature::make(0, 0, 0)),
        channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&BeiDouB1iDllPllTrackingGenSourceTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

BeiDouB1iDllPllTrackingGenSourceTest_msg_rx::~BeiDouB1iDllPllTrackingGenSourceTest_msg_rx()
{}


// ###########################################################


class BeiDouB1iDllPllTrackingGenSourceTest: public ::testing::Test
{
protected:
    BeiDouB1iDllPllTrackingGenSourceTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~BeiDouB1iDllPllTrackingGenSourceTest()
    {}

    void init();

    concurrent_queue<int> channel_internal_queue;

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;

    std::shared_ptr<GNSSBlockFactory> factory;
//    std::shared_ptr<TrackingInterface> tracking;
    std::shared_ptr<BeiDouB1iDllPllTracking> tracking;
    std::shared_ptr<InMemoryConfiguration> config;

    Gnss_Synchro gnss_synchro;

    size_t item_size;

    unsigned int intg_time_ms = 0;
    unsigned int fs_in = 0;
    int nsamples = 0;

    double expected_doppler_hz = 0.0;
    double expected_delay_chips = 0.0;
    double expected_delay_samples = 0.0;

    long long int begin = 0;
    long long int end = 0;
};


void BeiDouB1iDllPllTrackingGenSourceTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);

    gnss_synchro.PRN = 20;

    intg_time_ms = 1;           // Tested with a period of integration > 1 ms
    fs_in = 16.000e6;           // set 16.000 MHz
    nsamples = fs_in*9;

    expected_delay_samples = 3767.0;                           // [samples]
    expected_doppler_hz    = 1650.0;                             // [Hz]
    expected_delay_chips   = static_cast<float>(expected_delay_samples * BEIDOU_B1I_CODE_RATE_HZ / static_cast<float>(fs_in));

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.num_satellites", "1");
    config->set_property("SignalSource.system_0", "C");
    config->set_property("SignalSource.PRN_0", std::to_string(gnss_synchro.PRN));
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));
    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");
    config->set_property("SignalSource.dump", "true");
    config->set_property("SignalSource.dump_filename", "../src/tests/signal_samples/signal_source_beidou_100ms.dat");

    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", "11");
    config->set_property("InputFilter.number_of_bands", "2");
    config->set_property("InputFilter.band1_begin", "0.0");
    config->set_property("InputFilter.band1_end", "0.97");
    config->set_property("InputFilter.band2_begin", "0.98");
    config->set_property("InputFilter.band2_end", "1.0");
    config->set_property("InputFilter.ampl1_begin", "1.0");
    config->set_property("InputFilter.ampl1_end", "1.0");
    config->set_property("InputFilter.ampl2_begin", "0.0");
    config->set_property("InputFilter.ampl2_end", "0.0");
    config->set_property("InputFilter.band1_error", "1.0");
    config->set_property("InputFilter.band2_error", "1.0");
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", "16");

    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.if", "0.0");
    config->set_property("Tracking.dump_filename", "../src/tests/data/tracking_beidou/tracking_ch_");
    config->set_property("Tracking.implementation", "BeiDou_B1i_DLL_PLL_Tracking");
    config->set_property("Tracking.early_late_space_chips", "0.5");
    config->set_property("Tracking.order", "2");
    config->set_property("Tracking.pll_bw_hz", "50");
    config->set_property("Tracking.dll_bw_hz", "2");
}

TEST_F(BeiDouB1iDllPllTrackingGenSourceTest, ValidationOfResults)
{
    struct timeval tv;

    init();

    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");

    tracking = std::make_shared<BeiDouB1iDllPllTracking>(config.get(), "Tracking", 1, 1);
    boost::shared_ptr<BeiDouB1iDllPllTrackingGenSourceTest_msg_rx> msg_rx = BeiDouB1iDllPllTrackingGenSourceTest_msg_rx_make(channel_internal_queue);

    gnss_synchro.Acq_delay_samples = 3767;
    gnss_synchro.Acq_doppler_hz = 1650;
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

    // USING SIGNAL GENERATOR
    ASSERT_NO_THROW( {
    boost::shared_ptr<GenSignalSource> signal_source;
    SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);

    FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
    filter->connect(top_block);

    signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
    signal_source->connect(top_block);

    top_block->connect(signal_source->get_right_block(), 0, tracking->get_left_block(), 0);
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
    std::cout <<  "Tracked " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}



/*!
 * \file gps_l2_m_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l2_m_dll_pll_tracking.h"
#include "in_memory_configuration.h"
#include "tracking_interface.h"
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <utility>
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL2MDllPllTrackingTest_msg_rx;

using GpsL2MDllPllTrackingTest_msg_rx_sptr = gnss_shared_ptr<GpsL2MDllPllTrackingTest_msg_rx>;

GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make();

class GpsL2MDllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    GpsL2MDllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~GpsL2MDllPllTrackingTest_msg_rx();  //!< Default destructor
};


GpsL2MDllPllTrackingTest_msg_rx_sptr GpsL2MDllPllTrackingTest_msg_rx_make()
{
    return GpsL2MDllPllTrackingTest_msg_rx_sptr(new GpsL2MDllPllTrackingTest_msg_rx());
}


void GpsL2MDllPllTrackingTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


GpsL2MDllPllTrackingTest_msg_rx::GpsL2MDllPllTrackingTest_msg_rx() : gr::block("GpsL2MDllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&GpsL2MDllPllTrackingTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&GpsL2MDllPllTrackingTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


GpsL2MDllPllTrackingTest_msg_rx::~GpsL2MDllPllTrackingTest_msg_rx() = default;


// ###########################################################

class GpsL2MDllPllTrackingTest : public ::testing::Test
{
protected:
    GpsL2MDllPllTrackingTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL2MDllPllTrackingTest() = default;

    void init();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
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
    std::chrono::duration<double> elapsed_seconds(0.0);
    int fs_in = 5000000;
    int nsamples = fs_in * 9;

    init();
    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Tracking test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL2MDllPllTracking>(config.get(), "Tracking_2S", 1, 1);
    auto msg_rx = GpsL2MDllPllTrackingTest_msg_rx_make();

    gnss_synchro.Acq_delay_samples = 1;
    gnss_synchro.Acq_doppler_hz = 1200;
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
        // gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/gps_l2c_m_prn7_5msps.dat";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of tracking test.";

    tracking->start_tracking();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    // TODO: Verify tracking results
    std::cout << "Tracked " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

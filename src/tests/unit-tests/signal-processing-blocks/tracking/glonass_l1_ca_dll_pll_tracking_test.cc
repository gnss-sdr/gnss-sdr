/*!
 * \file glonass_l1_ca_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for GLONASS_L1_CA_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#include "concurrent_queue.h"
#include "glonass_l1_ca_dll_pll_tracking.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include "tracking_interface.h"
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <chrono>
#include <utility>
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind.hpp>
#endif
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GlonassL1CaDllPllTrackingTest_msg_rx;

#if GNURADIO_USES_STD_POINTERS
using GlonassL1CaDllPllTrackingTest_msg_rx_sptr = std::shared_ptr<GlonassL1CaDllPllTrackingTest_msg_rx>;
#else
using GlonassL1CaDllPllTrackingTest_msg_rx_sptr = boost::shared_ptr<GlonassL1CaDllPllTrackingTest_msg_rx>;
#endif

GlonassL1CaDllPllTrackingTest_msg_rx_sptr GlonassL1CaDllPllTrackingTest_msg_rx_make();

class GlonassL1CaDllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend GlonassL1CaDllPllTrackingTest_msg_rx_sptr GlonassL1CaDllPllTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GlonassL1CaDllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~GlonassL1CaDllPllTrackingTest_msg_rx();  //!< Default destructor
};

GlonassL1CaDllPllTrackingTest_msg_rx_sptr GlonassL1CaDllPllTrackingTest_msg_rx_make()
{
    return GlonassL1CaDllPllTrackingTest_msg_rx_sptr(new GlonassL1CaDllPllTrackingTest_msg_rx());
}

void GlonassL1CaDllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

GlonassL1CaDllPllTrackingTest_msg_rx::GlonassL1CaDllPllTrackingTest_msg_rx() : gr::block("GlonassL1CaDllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](pmt::pmt_t&& PH1) { msg_handler_events(PH1); });
#else
        boost::bind(&GlonassL1CaDllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
#endif
    rx_message = 0;
}

GlonassL1CaDllPllTrackingTest_msg_rx::~GlonassL1CaDllPllTrackingTest_msg_rx() = default;


// ###########################################################


class GlonassL1CaDllPllTrackingTest : public ::testing::Test
{
protected:
    GlonassL1CaDllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GlonassL1CaDllPllTrackingTest() = default;

    void init();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GlonassL1CaDllPllTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'R';
    std::string signal = "1G";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("GNSS-SDR.internal_fs_sps", "6625000");
    config->set_property("Tracking_1G.implementation", "GLONASS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1G.item_type", "gr_complex");
    config->set_property("Tracking_1G.dump", "false");
    config->set_property("Tracking_1G.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1G.early_late_space_chips", "0.5");
    config->set_property("Tracking_1G.order", "2");
    config->set_property("Tracking_1G.pll_bw_hz", "2");
    config->set_property("Tracking_1G.dll_bw_hz", "0.5");
}

TEST_F(GlonassL1CaDllPllTrackingTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    int fs_in = 6625000;
    int nsamples = fs_in * 4e-3 * 2;

    init();
    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Tracking test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GlonassL1CaDllPllTracking>(config.get(), "Tracking_1G", 1, 1);
    auto msg_rx = GlonassL1CaDllPllTrackingTest_msg_rx_make();

    gnss_synchro.Acq_delay_samples = 1343;
    gnss_synchro.Acq_doppler_hz = -2750;
    // gnss_synchro.Acq_doppler_hz = -2750;
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
        gr::analog::sig_source_c::sptr sin_source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
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
    std::cout << "Tracked " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

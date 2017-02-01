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
#include "telemetry_decoder_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_c_aid_tracking.h"
#include "tracking_obs_reader.h"

// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TRACKING MESSAGES #########
class GpsL1CADllPllTelemetryDecoderTest_msg_rx;

typedef boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_msg_rx> GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr;

GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();

class GpsL1CADllPllTelemetryDecoderTest_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTelemetryDecoderTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTelemetryDecoderTest_msg_rx(); //!< Default destructor

};

GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make()
{
    return GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr(new GpsL1CADllPllTelemetryDecoderTest_msg_rx());
}

void GpsL1CADllPllTelemetryDecoderTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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

GpsL1CADllPllTelemetryDecoderTest_msg_rx::GpsL1CADllPllTelemetryDecoderTest_msg_rx() :
            gr::block("GpsL1CADllPllTelemetryDecoderTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTelemetryDecoderTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL1CADllPllTelemetryDecoderTest_msg_rx::~GpsL1CADllPllTelemetryDecoderTest_msg_rx()
{}


// ###########################################################


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TLM MESSAGES #########
class GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx;

typedef boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx> GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr;

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();

class GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx(); //!< Default destructor

};

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make()
{
    return GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr(new GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx());
}

void GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::msg_handler_events(pmt::pmt_t msg)
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

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx() :
            gr::block("GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::~GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx()
{}


// ###########################################################


class GpsL1CATelemetryDecoderTest: public ::testing::Test
{
protected:
    GpsL1CATelemetryDecoderTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CATelemetryDecoderTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GpsL1CATelemetryDecoderTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    config->set_property("GNSS-SDR.internal_fs_hz", "2600000");

    // Set Tracking
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", "0");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "20.0");
    config->set_property("Tracking_1C.dll_bw_hz", "1.5");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");

    config->set_property("TelemetryDecoder_1C.dump","true");
    config->set_property("TelemetryDecoder_1C.decimation_factor","1");

}

TEST_F(GpsL1CATelemetryDecoderTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    int fs_in = 2600000;

    init();

    //todo: call here the gnss simulator

    //open true observables log file written by the simulator
    tracking_obs_reader true_obs_data;
    ASSERT_NO_THROW({
        if (true_obs_data.open_obs_file("signal_samples/gps_l1_ca_obs_prn1.dat")==false)
            {
                throw std::exception();
            };
    })<< "Failure opening true observables file" << std::endl;


    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Telemetry_Decoder test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    //std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL1CaDllPllCAidTracking>(config.get(), "Tracking_1C", 1, 1);

    boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_msg_rx> msg_rx = GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();

    gnss_synchro.Acq_delay_samples = (1023-994.622/1023)*fs_in*1e-3;
    gnss_synchro.Acq_doppler_hz = -2583.86;
    gnss_synchro.Acq_samplestamp_samples = 0;

    std::shared_ptr<TelemetryDecoderInterface> tlm(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C",1, 1));
    tlm->set_channel(0);

    boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx> tlm_msg_rx = GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();

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
        std::string file =  path + "signal_samples/signal_out.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        //boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::interleaved_char_to_complex::sptr  gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        //top_block->connect(gr_interleaved_char_to_complex, 0, valve, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, tlm->get_left_block(), 0);
        top_block->connect(tlm->get_right_block(), 0, sink, 0);
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


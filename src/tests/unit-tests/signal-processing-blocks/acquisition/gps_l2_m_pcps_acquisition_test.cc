/*!
 * \file gps_l2_m_pcps_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsAcquisition class based on some input parameters.
 * \author Javier Arribas, 2015 (jarribas@cttc.es)
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
#include <cstring>
#include <iostream>
#include <boost/chrono.hpp>
#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include <gnuradio/blocks/char_to_short.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "GPS_L2C.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL2MPcpsAcquisitionTest_msg_rx;

typedef boost::shared_ptr<GpsL2MPcpsAcquisitionTest_msg_rx> GpsL2MPcpsAcquisitionTest_msg_rx_sptr;

GpsL2MPcpsAcquisitionTest_msg_rx_sptr GpsL2MPcpsAcquisitionTest_msg_rx_make();

class GpsL2MPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GpsL2MPcpsAcquisitionTest_msg_rx_sptr GpsL2MPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL2MPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~GpsL2MPcpsAcquisitionTest_msg_rx(); //!< Default destructor

};

GpsL2MPcpsAcquisitionTest_msg_rx_sptr GpsL2MPcpsAcquisitionTest_msg_rx_make()
{
    return GpsL2MPcpsAcquisitionTest_msg_rx_sptr(new GpsL2MPcpsAcquisitionTest_msg_rx());
}

void GpsL2MPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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

GpsL2MPcpsAcquisitionTest_msg_rx::GpsL2MPcpsAcquisitionTest_msg_rx() :
            gr::block("GpsL2MPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL2MPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL2MPcpsAcquisitionTest_msg_rx::~GpsL2MPcpsAcquisitionTest_msg_rx()
{}


// ###########################################################

class GpsL2MPcpsAcquisitionTest: public ::testing::Test
{
protected:
    GpsL2MPcpsAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        sampling_freqeuncy_hz = 0;
        nsamples = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL2MPcpsAcquisitionTest()
    {}

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    int sampling_freqeuncy_hz;
    int nsamples;
};


void GpsL2MPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "2S";
    //strncpy(gnss_synchro.Signal, signal.c_str(), 3);
    std::memcpy((void*)gnss_synchro.Signal, signal.c_str(), 3); // copy string into synchro char array: 2 char + null
    gnss_synchro.Signal[2] = 0; // make sure that string length is only two characters
    gnss_synchro.PRN = 7;

    sampling_freqeuncy_hz  = 5000000;
    nsamples = round((double)sampling_freqeuncy_hz*GPS_L2_M_PERIOD)*2;
    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(sampling_freqeuncy_hz));
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.implementation", "GPS_L2_M_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.001");
    config->set_property("Acquisition.doppler_max", "5000");
    config->set_property("Acquisition.doppler_step", "100");
    config->set_property("Acquisition.repeat_satellite", "false");
}


TEST_F(GpsL2MPcpsAcquisitionTest, Instantiate)
{
    init();
    queue = gr::msg_queue::make(0);
    std::shared_ptr<GpsL2MPcpsAcquisition> acquisition = std::make_shared<GpsL2MPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
}

TEST_F(GpsL2MPcpsAcquisitionTest, ConnectAndRun)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    init();
    std::shared_ptr<GpsL2MPcpsAcquisition> acquisition = std::make_shared<GpsL2MPcpsAcquisition>(config.get(), "Acquisition", 1, 1);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(sampling_freqeuncy_hz, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        boost::shared_ptr<GpsL2MPcpsAcquisitionTest_msg_rx> msg_rx = GpsL2MPcpsAcquisitionTest_msg_rx_make();

    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}

TEST_F(GpsL2MPcpsAcquisitionTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);
    double expected_delay_samples = 1;//2004;
    double expected_doppler_hz = 1200;//3000;
    init();
    std::shared_ptr<GpsL2MPcpsAcquisition> acquisition = std::make_shared<GpsL2MPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GpsL2MPcpsAcquisitionTest_msg_rx> msg_rx = GpsL2MPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(0.001);
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(5000);
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(10);
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        //std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
        std::string file = path + "signal_samples/gps_l2c_m_prn7_5msps.dat";
        //std::string file = "/datalogger/signals/Fraunhofer/L125_III1b_210s_L2_resampled.bin";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        //gr::blocks::interleaved_short_to_complex::sptr gr_interleaved_short_to_complex_ = gr::blocks::interleaved_short_to_complex::make();
        //gr::blocks::char_to_short::sptr gr_char_to_short_ = gr::blocks::char_to_short::make();
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        //top_block->connect(file_source, 0, gr_char_to_short_, 0);
        //top_block->connect(gr_char_to_short_, 0, gr_interleaved_short_to_complex_ , 0);
        top_block->connect(file_source, 0, valve , 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test." << std::endl;


    ASSERT_NO_THROW( {
        acquisition->set_state(1); // Ensure that acquisition starts at the first sample
        acquisition->init();
    }) << "Failure set_state and init acquisition test" << std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    //unsigned long int Acq_samplestamp_samples = gnss_synchro.Acq_samplestamp_samples;
    std::cout <<  "Acquisition process runtime duration: " << (end - begin) << " microseconds" << std::endl;

    std::cout <<  "gnss_synchro.Acq_doppler_hz = " << gnss_synchro.Acq_doppler_hz << " Hz" << std::endl;
    std::cout <<  "gnss_synchro.Acq_delay_samples = " << gnss_synchro.Acq_delay_samples << " Samples" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = (float)(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 200) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";

}

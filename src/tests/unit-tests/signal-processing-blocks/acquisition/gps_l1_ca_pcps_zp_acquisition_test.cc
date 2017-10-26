/*!
 * \file gps_l1_ca_pcps_zp_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsZPAcquisition class based on some input parameters.
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

DEFINE_int32(doppler_max_pcps_zpa, 5000,"Max. Doppler frequency in the Acquisition block");
DEFINE_int32(doppler_step_pcps_zpa, 250,"Doppler step in the Acquisition block");
DEFINE_int32(ntests_pcps_zpa, 1000,"Number of averaged tests");

#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/analog/noise_source_c.h>
#include <gnuradio/analog/noise_type.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_pcps_zp_acquisition.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CaPcpsZPAcquisitionTest_msg_rx;

typedef boost::shared_ptr<GpsL1CaPcpsZPAcquisitionTest_msg_rx> GpsL1CaPcpsZPAcquisitionTest_msg_rx_sptr;

GpsL1CaPcpsZPAcquisitionTest_msg_rx_sptr GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();

class GpsL1CaPcpsZPAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GpsL1CaPcpsZPAcquisitionTest_msg_rx_sptr GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CaPcpsZPAcquisitionTest_msg_rx();
public:
    int rx_message;
    ~GpsL1CaPcpsZPAcquisitionTest_msg_rx(); //!< Default destructor
};


GpsL1CaPcpsZPAcquisitionTest_msg_rx_sptr GpsL1CaPcpsZPAcquisitionTest_msg_rx_make()
{
    return GpsL1CaPcpsZPAcquisitionTest_msg_rx_sptr(new GpsL1CaPcpsZPAcquisitionTest_msg_rx());
}


void GpsL1CaPcpsZPAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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


GpsL1CaPcpsZPAcquisitionTest_msg_rx::GpsL1CaPcpsZPAcquisitionTest_msg_rx() :
    gr::block("GpsL1CaPcpsZPAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CaPcpsZPAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CaPcpsZPAcquisitionTest_msg_rx::~GpsL1CaPcpsZPAcquisitionTest_msg_rx()
{}


// ###########################################################

class GpsL1CaPcpsZPAcquisitionTest: public ::testing::Test
{
protected:
    GpsL1CaPcpsZPAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CaPcpsZPAcquisitionTest()
    {}

    void init();
    void init_speed_test(int freq, bool blocking);

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GpsL1CaPcpsZPAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms", "1");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_ZP_Acquisition");
    config->set_property("Acquisition.threshold", "0.00001");
    config->set_property("Acquisition.doppler_max", "5000");
    config->set_property("Acquisition.doppler_step", "500");
    config->set_property("Acquisition.repeat_satellite", "false");
    config->set_property("Acquisition.use_CFAR_algorithm", "false");
    config->set_property("Acquisition.blocking", "true");
}

void GpsL1CaPcpsZPAcquisitionTest::init_speed_test(int freq, bool blocking)
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(freq));
    if (blocking)
    {
        config->set_property("Acquisition.blocking", "true");
    }
    else
    {
    	config->set_property("Acquisition.blocking", "false");
    }
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms", "1");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_ZP_Acquisition");
    config->set_property("Acquisition.threshold", "22");
    config->set_property("Acquisition.doppler_max", "45000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.repeat_satellite", "false");
}

TEST_F(GpsL1CaPcpsZPAcquisitionTest, Instantiate)
{
    init();
    boost::shared_ptr<GpsL1CaPcpsZPAcquisition> acquisition = boost::make_shared<GpsL1CaPcpsZPAcquisition>(config.get(), "Acquisition", 1, 1);
}


TEST_F(GpsL1CaPcpsZPAcquisitionTest, ConnectAndRun)
{
    int fs_in = 4000000;
    int nsamples = 4000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    top_block = gr::make_top_block("Acquisition test");
    init();
    boost::shared_ptr<GpsL1CaPcpsZPAcquisition> acquisition = boost::make_shared<GpsL1CaPcpsZPAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GpsL1CaPcpsZPAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

TEST_F(GpsL1CaPcpsZPAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 524;
    double expected_doppler_hz = 1680;

    init();

    std::shared_ptr<GpsL1CaPcpsZPAcquisition> acquisition = std::make_shared<GpsL1CaPcpsZPAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GpsL1CaPcpsZPAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(0.00001);
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(5000);
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(100);
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    acquisition->set_local_code();
    acquisition->set_state(0); // Ensure that acquisition starts at the first sample
    acquisition->init();
    acquisition->reset();

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout <<  "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}

TEST_F(GpsL1CaPcpsZPAcquisitionTest, SpeedTestBlocking)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int fs_in[12] = {1000000, 1024000, 2000000, 2048000, 4000000, 4096000, 8000000, 8192000, 16000000, 16384000};
    int ntests = FLAGS_ntests_pcps_zpa;
    int nfreqtest = 10;
    double elapsed_times[nfreqtest];

    for (int nfreq = 0; nfreq < nfreqtest; nfreq++)
    {
    	std::chrono::duration<double> elapsed_seconds(0);
    	top_block = gr::make_top_block("Acquisition test");
   	    config = std::make_shared<InMemoryConfiguration>();
   	    init_speed_test(fs_in[nfreq],true);
   	    std::shared_ptr<GpsL1CaPcpsZPAcquisition> acquisition = std::make_shared<GpsL1CaPcpsZPAcquisition>(config.get(), "Acquisition", 1, 1);
   	    boost::shared_ptr<GpsL1CaPcpsZPAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();
   	    ASSERT_NO_THROW( {
   	        acquisition->set_channel(1);
   	    }) << "Failure setting channel." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_gnss_synchro(&gnss_synchro);
   	    }) << "Failure setting gnss_synchro." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_threshold(22);
   	    }) << "Failure setting threshold." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_doppler_max(FLAGS_doppler_max_pcps_zpa);
   	    }) << "Failure setting doppler_max." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_doppler_step(FLAGS_doppler_step_pcps_zpa);
   	    }) << "Failure setting doppler_step." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->connect(top_block);
   	    }) << "Failure connecting acquisition to the top_block." << std::endl;

   	    ASSERT_NO_THROW( {
   	        gr::analog::noise_source_c::sptr noise_source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
   	        unsigned int nsamples = static_cast<unsigned int>(fs_in[nfreq]) * static_cast<unsigned int>(ntests) / 1000;
   	        gr::msg_queue::sptr queue = gr::msg_queue::make(0);
   	        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
   	        top_block->connect(noise_source, 0, valve, 0);
   	        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
   	        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
   	    }) << "Failure connecting the blocks of acquisition test." << std::endl;

   	    acquisition->set_local_code();
   	    acquisition->set_state(0); // Ensure that acquisition starts at the first sample
   	    acquisition->init();
   	    acquisition->reset();
   	    acquisition->set_testing(true);

    	EXPECT_NO_THROW( {
    	    start = std::chrono::system_clock::now();
    	    top_block->run(); // Start threads and wait
    	    end = std::chrono::system_clock::now();
    	    elapsed_seconds = end - start;
    	}) << "Failure running the top_block." << std::endl;
    	ASSERT_EQ(2, msg_rx->rx_message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
    	elapsed_times[nfreq] = elapsed_seconds.count() / static_cast<double>(ntests);
    }
    std::cout << "BLOCKING Acquisition" << std::endl;
    for (int aux = 0; aux < nfreqtest; aux++)
    {
    	std::cout << "Sampling frequency: " << static_cast<float>(fs_in[aux]) / 1e6 << "MHz. Averaged ACQ time: " << elapsed_times[aux] * 1e3 << " ms." << std::endl;
    }
}

TEST_F(GpsL1CaPcpsZPAcquisitionTest, SpeedTestNonBlocking)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int fs_in[12] = {1000000, 1024000, 2000000, 2048000, 4000000, 4096000, 8000000, 8192000, 16000000, 16384000};
    int ntests = FLAGS_ntests_pcps_zpa;
    int nfreqtest = 10;
    double elapsed_times[nfreqtest];

    for (int nfreq = 0; nfreq < nfreqtest; nfreq++)
    {
    	std::chrono::duration<double> elapsed_seconds(0);
    	top_block = gr::make_top_block("Acquisition test");
   	    config = std::make_shared<InMemoryConfiguration>();
   	    init_speed_test(fs_in[nfreq],false);
   	    std::shared_ptr<GpsL1CaPcpsZPAcquisition> acquisition = std::make_shared<GpsL1CaPcpsZPAcquisition>(config.get(), "Acquisition", 1, 1);
   	    boost::shared_ptr<GpsL1CaPcpsZPAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsZPAcquisitionTest_msg_rx_make();
   	    ASSERT_NO_THROW( {
   	        acquisition->set_channel(1);
   	    }) << "Failure setting channel." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_gnss_synchro(&gnss_synchro);
   	    }) << "Failure setting gnss_synchro." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_threshold(22);
   	    }) << "Failure setting threshold." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_doppler_max(FLAGS_doppler_max_pcps_zpa);
   	    }) << "Failure setting doppler_max." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->set_doppler_step(FLAGS_doppler_step_pcps_zpa);
   	    }) << "Failure setting doppler_step." << std::endl;

   	    ASSERT_NO_THROW( {
   	        acquisition->connect(top_block);
   	    }) << "Failure connecting acquisition to the top_block." << std::endl;

   	    ASSERT_NO_THROW( {
   	        gr::analog::noise_source_c::sptr noise_source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
   	        unsigned int nsamples = static_cast<unsigned int>(fs_in[nfreq]) * static_cast<unsigned int>(ntests) / 1000;
   	        gr::msg_queue::sptr queue = gr::msg_queue::make(0);
   	        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
   	        top_block->connect(noise_source, 0, valve, 0);
   	        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
   	        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
   	    }) << "Failure connecting the blocks of acquisition test." << std::endl;

   	    acquisition->set_local_code();
   	    acquisition->set_state(0); // Ensure that acquisition starts at the first sample
   	    acquisition->init();
   	    acquisition->reset();
   	    acquisition->set_testing(true);

    	EXPECT_NO_THROW( {
    	    start = std::chrono::system_clock::now();
    	    top_block->run(); // Start threads and wait
    	    end = std::chrono::system_clock::now();
    	    elapsed_seconds = end - start;
    	}) << "Failure running the top_block." << std::endl;
    	ASSERT_EQ(2, msg_rx->rx_message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
    	elapsed_times[nfreq] = elapsed_seconds.count() / static_cast<double>(ntests);
    }
    std::cout << "NON BLOCKING Acquisition" << std::endl;
    for (int aux = 0; aux < nfreqtest; aux++)
    {
    	std::cout << "Sampling frequency: " << static_cast<float>(fs_in[aux]) / 1e6 << "MHz. Averaged ACQ time: " << elapsed_times[aux] * 1e3 << " ms." << std::endl;
    }
}

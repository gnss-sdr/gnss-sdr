/*!
 * \file beidou_B1I_pcps_acquisition_gensource_test.cc
 * \brief  This class implements an acquisition test for
 * BeidouB1iPcpsAcquisition class based on some input parameters.
 * \author Enric Juan, 2016. enric.juan.92(at)gmail.com
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
#include <iostream>
#include <boost/chrono.hpp>
//#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "beidou_b1i_pcps_acquisition.h"
#include "BEIDOU_B1I.h"

#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h" 


// ######## GNURADIO BLOCK MESSAGE RECEIVER #########
class BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx;

typedef boost::shared_ptr<BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx> BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_sptr;

BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_sptr BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_make(concurrent_queue<int>& queue);

class BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx : public gr::block
{
private:
    friend BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_sptr BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;
public:
    int rx_message;
    ~BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx(); //!< Default destructor
};


BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_sptr BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_make(concurrent_queue<int>& queue)
{
    return BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_sptr(new BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx(queue));
}


void BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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


BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx::BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx(concurrent_queue<int>& queue) :
    gr::block("BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
    channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx::~BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx()
{}


// ###########################################################

class BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest: public ::testing::Test
{
protected:
    BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest()
    {
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest()
    {}

    void init();
    void config_gensource();
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();

    concurrent_queue<int> channel_internal_queue;

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;

    std::shared_ptr<BeidouB1iPcpsAcquisition> acquisition;
    std::shared_ptr<InMemoryConfiguration> config;

    Gnss_Synchro gnss_synchro;

    size_t item_size;
    bool stop;
    int message;
    boost::thread ch_thread;

    unsigned int intg_time_ms = 0;
    unsigned int fs_in = 0;

    double expected_doppler_hz = 0.0;
    double expected_delay_chips = 0.0;
    double expected_delay_samples = 0.0;
    double expected_delay_sec = 0.0;
    double ts_in = 0.0;

    float max_doppler_error_hz = 0.0;
    float max_delay_error_chips = 0.0;

    unsigned int num_of_realizations = 0;
    unsigned int realization_counter = 0;
    unsigned int detection_counter = 0;
    unsigned int correct_estimation_counter = 0;
    unsigned int acquired_samples = 0;
    unsigned int mean_acq_time_us = 0;

    double mse_doppler = 0.0;
    double mse_delay = 0.0;

    double Pd = 0.0;
    double Pfa_p = 0.0;
    double Pfa_a = 0.0;
};


void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::init()
{
    message = 0;
    realization_counter = 0;
    detection_counter = 0;
    correct_estimation_counter = 0;
    acquired_samples = 0;
    mse_doppler = 0;
    mse_delay = 0;
    mean_acq_time_us = 0;
    Pd = 0;
    Pfa_p = 0;
    Pfa_a = 0;
}

void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::config_gensource()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';                                                         // "BeiDou" = "C"                               see gnss_satellite.h
    std::string signal = "1C";                                                         // "1C" is for GPS L1 C/A (have to be changed)  see gnss_signal.h
    signal.copy(gnss_synchro.Signal, 2, 0);

/******** CONFIGURATION PARAMETERS ********/

    gnss_synchro.PRN = 20;          // [1:37]
    intg_time_ms = 100;             // Tested with a period of integration > 1 ms
    fs_in = 16.000e6;               // set 16.000 MHz
    ts_in = (1/static_cast<double>(fs_in));

    expected_delay_samples = 3767.0;                                    // [samples]
    expected_delay_sec     = ts_in * expected_delay_samples * 1e3;      // [sec]
    expected_doppler_hz    = 1650.0;                                    // [Hz]
    expected_delay_chips   = static_cast<float>(expected_delay_samples * BEIDOU_B1I_CODE_RATE_HZ / static_cast<float>(fs_in));
    num_of_realizations = 1;

/******************************************/

    config = std::make_shared<InMemoryConfiguration>();

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

    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0.0");
    config->set_property("Acquisition.coherent_integration_time_ms", std::to_string(intg_time_ms));
    config->set_property("Acquisition.implementation", "BeiDou_B1I_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.001");
    config->set_property("Acquisition.doppler_max", "5000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.repeat_satellite", "false");
    config->set_property("Acquisition.dump", "true");
    config->set_property("Acquisition.dump_filename", "../src/tests/data/acquisition_beidou/acquisition_beidou.dat");
    config->set_property("Acquisition.pfa", "0.0");
}

void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::wait_message, this);
}

void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::wait_message()
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;

    while (!stop)
        {
            acquisition->reset();

            gettimeofday(&tv, NULL);
            begin = tv.tv_sec *1e6 + tv.tv_usec;

            channel_internal_queue.wait_and_pop(message);

            gettimeofday(&tv, NULL);
            end = tv.tv_sec*1e6 + tv.tv_usec;

            mean_acq_time_us += (end - begin);

            process_message();
        }
}

void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::process_message()
{
    if (message == 1)
        {
            detection_counter++;

            // The term -5 is here to correct the additional delay introduced by the FIR filter
            double delay_error_chips = std::abs((double)expected_delay_chips - (double)(gnss_synchro.Acq_delay_samples-5)*1023.0/((double)fs_in*1e-3));
            double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

            mse_delay += std::pow(delay_error_chips, 2);
            mse_doppler += std::pow(doppler_error_hz, 2);

            if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
                {
                    correct_estimation_counter++;
                }
        }

    realization_counter++;

    std::cout << "Progress: " << round((float)realization_counter/num_of_realizations*100) << "% \r" << std::flush;

    if (realization_counter == num_of_realizations)
        {
            mse_delay /= num_of_realizations;
            mse_doppler /= num_of_realizations;

            Pd = (double)correct_estimation_counter / (double)num_of_realizations;
            Pfa_a = (double)detection_counter / (double)num_of_realizations;
            Pfa_p = (double)(detection_counter-correct_estimation_counter) / (double)num_of_realizations;

            mean_acq_time_us /= num_of_realizations;

            stop_queue();
            top_block->stop();

            std::cout << std::endl;
        }
}

void BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest::stop_queue()
{
    stop = true;
}


TEST_F(BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest, Instantiate)
{
    config_gensource();
    acquisition = std::make_shared<BeidouB1iPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
}

TEST_F(BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest, ConnectAndRun)
{
    int nsamples = floor(fs_in * intg_time_ms*1e-3);
    // int nsamples =      16000;

    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    config_gensource();
    acquisition = std::make_shared<BeidouB1iPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx> msg_rx = BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
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

TEST_F(BeiDouB1iPcpsAcquisitionGSoC2016GenSourceTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end   = 0;

    config_gensource();
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    acquisition = std::make_shared<BeidouB1iPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx> msg_rx = BeidouB1iPcpsAcquisitionGenSourceTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 250));
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 0.001));
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    // USING SIGNAL GENERATOR
    ASSERT_NO_THROW( {
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);

        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        filter->connect(top_block);
        
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    init();
    acquisition->set_state(1);
    acquisition->init();
    
    start_queue();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    stop_queue();
    ch_thread.join(); 

    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout <<  "\nAcquired " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    // float delay_error_chips = expected_delay_chips - static_cast<float>(gnss_synchro.Acq_delay_samples * BEIDOU_B1I_CODE_RATE_HZ / static_cast<float>(fs_in));
    float delay_error_chips = static_cast<float>(delay_error_samples * BEIDOU_B1I_CODE_RATE_HZ / static_cast<float>(fs_in));
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    std::cout << "The delay_error_samples is "             << delay_error_samples            << std::endl;
    std::cout << "The delay_error_chips is "               << delay_error_chips              << std::endl;
    std::cout << "The doppler_error_hz is "                << doppler_error_hz               << std::endl;

    std::cout << "\n" << "The gnss_synchro.Acq_delay_samples is  " << gnss_synchro.Acq_delay_samples << std::endl;
    std::cout << "The gnss_synchro.Acq_doppler_hz is  "    << gnss_synchro.Acq_doppler_hz    << "\n" << std::endl;

    EXPECT_LE(doppler_error_hz,  500)   <<    "Doppler error exceeds the expected value: 500 Hz = 2*doppler_step";         
    EXPECT_LT(delay_error_chips,  10)   <<    "Delay error exceeds the expected value: 1 chips";  
}

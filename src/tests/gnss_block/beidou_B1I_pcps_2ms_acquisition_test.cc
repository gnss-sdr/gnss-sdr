/*!
 * \file beidou_B1I_pcps_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * BeidouB1iPcps2msAcquisition class based on some input parameters.
 * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
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
 
#include "beidou_b1i_pcps_2ms_acquisition.h"   

class BeidouB1iPcps2msAcquisitionTest: public ::testing::Test
{
protected:
    BeidouB1iPcps2msAcquisitionTest()
    {
        //queue = gr::msg_queue::make(0);
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~BeidouB1iPcps2msAcquisitionTest()
    {}

    void init();
    void start_queue();
    void wait_message();
    void stop_queue();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    concurrent_queue<int> channel_internal_queue;
    bool stop;
    int message;
    boost::thread ch_thread;
};


void BeidouB1iPcps2msAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';                                                         // "BeiDou" = "C"                               see gnss_satellite.h              
    std::string signal = "1C";                                                         // "1C" is for GPS L1 C/A (have to be canched)  see gnss_signal.h 
    signal.copy(gnss_synchro.Signal, 2, 0);

    gnss_synchro.PRN = 30;                                                              // [1:37]

    config->set_property("GNSS-SDR.internal_fs_hz", "16000000");                       // set 16.000 MHz
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "98000");                                   // see "Development of a PC-Based Software Receiver for the Reception of Beidou Navigation Satellite Signals"
    config->set_property("Acquisition.coherent_integration_time_ms", "8");             // Tested with a period of integration > 1 ms
    config->set_property("Acquisition.dump", "true");                                  // set "true"
    config->set_property("Acquisition.implementation", "BeiDou_B1I_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.001");
    config->set_property("Acquisition.doppler_max", "5000");
    config->set_property("Acquisition.doppler_step", "500");                            
    config->set_property("Acquisition.repeat_satellite", "false");
    config->set_property("Acquisition.pfa", "0.0");
}

void BeidouB1iPcps2msAcquisitionTest::start_queue()
{
    ch_thread = boost::thread(&BeidouB1iPcps2msAcquisitionTest::wait_message, this);
}

void BeidouB1iPcps2msAcquisitionTest::wait_message()
{
    while (!stop)
        {
            channel_internal_queue.wait_and_pop(message);
            stop_queue();
        }
}

void BeidouB1iPcps2msAcquisitionTest::stop_queue()
{
    stop = true;
}

TEST_F(BeidouB1iPcps2msAcquisitionTest, Instantiate)
{
    init();
    queue = gr::msg_queue::make(0);
    std::shared_ptr<BeidouB1iPcps2msAcquisition> acquisition = std::make_shared<BeidouB1iPcps2msAcquisition>(config.get(), "Acquisition", 1, 1, queue);
}

TEST_F(BeidouB1iPcps2msAcquisitionTest, ConnectAndRun)
{
    int fs_in    =   16000000;                           
    int nsamples =      16000;                           
    struct timeval tv;
    long long int begin = 0;
    long long int end   = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    init();
    std::shared_ptr<BeidouB1iPcps2msAcquisition> acquisition = std::make_shared<BeidouB1iPcps2msAcquisition>(config.get(), "Acquisition", 1, 1, queue);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
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

TEST_F(BeidouB1iPcps2msAcquisitionTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end   = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue     = gr::msg_queue::make(0);

    double expected_delay_samples = 3565;          // [samples]
    double expected_doppler_hz    = 1500;          // [Hz]
    init();
    start_queue();
    std::shared_ptr<BeidouB1iPcps2msAcquisition> acquisition = std::make_shared<BeidouB1iPcps2msAcquisition>(config.get(), "Acquisition", 1, 1, queue);


    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(0.1);                        
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(10000);                    
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(250);                     
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/FFF030_test_1.dat";                                      //  set the name of the file
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;


    acquisition->set_state(1); // Ensure that acquisition starts at the first sample
    acquisition->init();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    stop_queue();

    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout <<  "Acquired " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;

    ASSERT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = (float)(delay_error_samples * 2046 / 16000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    std::cout << "The gnss_synchro.Acq_delay_samples is  " << gnss_synchro.Acq_delay_samples << std::endl;
    std::cout << "The gnss_synchro.Acq_doppler_hz is  "    << gnss_synchro.Acq_doppler_hz    << std::endl;

    EXPECT_LE(doppler_error_hz,  500)   <<    "Doppler error exceeds the expected value: 500 Hz = 2*doppler_step";         
    EXPECT_LT(delay_error_chips,  10)   <<    "Delay error exceeds the expected value: 1 chips";                             
    ch_thread.join();
}

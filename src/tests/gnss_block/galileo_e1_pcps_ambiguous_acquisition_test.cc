/*!
 * \file galileo_e1_pcps_ambiguous_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * GalileoE1PcpsAmbiguousAcquisition class based on some input parameters.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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


#include <cstdlib>
#include <ctime>
#include <iostream>
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
#include "gnss_signal.h"
#include "gnss_synchro.h"

#include "galileo_e1_pcps_ambiguous_acquisition.h"

class GalileoE1PcpsAmbiguousAcquisitionTest: public ::testing::Test
{
protected:
    GalileoE1PcpsAmbiguousAcquisitionTest()
{
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
}

    ~GalileoE1PcpsAmbiguousAcquisitionTest()
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


void GalileoE1PcpsAmbiguousAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    config->set_property("GNSS-SDR.internal_fs_hz", "4000000");
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms", "4");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition.threshold", "0.0001");
    config->set_property("Acquisition.doppler_max", "10000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.repeat_satellite", "false");
    config->set_property("Acquisition1.cboc", "true");
}

void GalileoE1PcpsAmbiguousAcquisitionTest::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&GalileoE1PcpsAmbiguousAcquisitionTest::wait_message, this);
}


void GalileoE1PcpsAmbiguousAcquisitionTest::wait_message()
{
    while (!stop)
        {
            channel_internal_queue.wait_and_pop(message);
            stop_queue();
        }
}

void GalileoE1PcpsAmbiguousAcquisitionTest::stop_queue()
{
    stop = true;
}



TEST_F(GalileoE1PcpsAmbiguousAcquisitionTest, Instantiate)
{
    init();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1, queue);
    std::shared_ptr<AcquisitionInterface> acquisition = std::dynamic_pointer_cast<AcquisitionInterface>(acq_);
}


TEST_F(GalileoE1PcpsAmbiguousAcquisitionTest, ConnectAndRun)
{
    int fs_in = 4000000;
    int nsamples = 4*fs_in;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);
    init();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1, queue);
    std::shared_ptr<AcquisitionInterface> acquisition = std::dynamic_pointer_cast<AcquisitionInterface>(acq_);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec*1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec*1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;
    std::cout <<  "Processed " << nsamples << " samples in " << (end-begin) << " microseconds" << std::endl;
}


TEST_F(GalileoE1PcpsAmbiguousAcquisitionTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    double expected_delay_samples = 2920; //18250;
    double expected_doppler_hz = -632;
    init();
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1, queue);
    std::shared_ptr<GalileoE1PcpsAmbiguousAcquisition> acquisition = std::dynamic_pointer_cast<GalileoE1PcpsAmbiguousAcquisition>(acq_);

    ASSERT_NO_THROW( {
        acquisition->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 1e-9));
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 250));
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    ASSERT_NO_THROW( {
        start_queue();
    }) << "Failure while starting the queue" << std::endl;

    acquisition->init();
    acquisition->reset();
    acquisition->set_state(1);

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
    EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    std::cout << "Delay: " << gnss_synchro.Acq_delay_samples << std::endl;
    std::cout << "Doppler: " << gnss_synchro.Acq_doppler_hz << std::endl;

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = (float)(delay_error_samples * 1023 / 4000000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 166) << "Doppler error exceeds the expected value: 166 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.175) << "Delay error exceeds the expected value: 0.175 chips";

    ch_thread.join();
}


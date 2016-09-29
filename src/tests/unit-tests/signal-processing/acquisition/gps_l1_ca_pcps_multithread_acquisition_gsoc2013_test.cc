/*!
 * \file gps_l1_ca_pcps_multithread_acquisition_gsoc2013_test.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsMultithreadAcquisition class.
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "configuration_interface.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_pcps_multithread_acquisition.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h"
#include "gnss_sdr_valve.h"
#include "signal_generator.h"
#include "signal_generator.cc"
#include "signal_generator_c.h"
#include "signal_generator_c.cc"


class GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test: public ::testing::Test
{
protected:
    GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test()
    {
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test()
    {
    }

    void init();
    void config_1();
    void config_2();
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GpsL1CaPcpsMultithreadAcquisition> acquisition;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
    boost::thread ch_thread;

    unsigned int integration_time_ms = 0;
    unsigned int fs_in = 0;

    double expected_delay_chips = 0.0;
    double expected_doppler_hz = 0.0;
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

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::init()
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

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::config_1()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal,2,0);

    integration_time_ms = 1;
    fs_in = 4e6;

    expected_delay_chips = 600;
    expected_doppler_hz = 750;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "1");

    config->set_property("SignalSource.system_0", "G");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));

    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");

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
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms",
                         std::to_string(integration_time_ms));
    config->set_property("Acquisition.max_dwells", "1");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Multithread_Acquisition");
    config->set_property("Acquisition.threshold", "0.8");
    config->set_property("Acquisition.doppler_max", "10000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.bit_transition_flag", "false");
    config->set_property("Acquisition.dump", "false");
}

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::config_2()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal,2,0);

    integration_time_ms = 1;
    fs_in = 4e6;

    expected_delay_chips = 600;
    expected_doppler_hz = 750;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    num_of_realizations = 100;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "4");

    config->set_property("SignalSource.system_0", "G");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));

    config->set_property("SignalSource.system_1", "G");
    config->set_property("SignalSource.PRN_1", "15");
    config->set_property("SignalSource.CN0_dB_1", "44");
    config->set_property("SignalSource.doppler_Hz_1", "1000");
    config->set_property("SignalSource.delay_chips_1", "100");

    config->set_property("SignalSource.system_2", "G");
    config->set_property("SignalSource.PRN_2", "21");
    config->set_property("SignalSource.CN0_dB_2", "44");
    config->set_property("SignalSource.doppler_Hz_2", "2000");
    config->set_property("SignalSource.delay_chips_2", "200");

    config->set_property("SignalSource.system_3", "G");
    config->set_property("SignalSource.PRN_3", "22");
    config->set_property("SignalSource.CN0_dB_3", "44");
    config->set_property("SignalSource.doppler_Hz_3", "3000");
    config->set_property("SignalSource.delay_chips_3", "300");

    config->set_property("SignalSource.noise_flag", "true");
    config->set_property("SignalSource.data_flag", "true");
    config->set_property("SignalSource.BW_BB", "0.97");

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
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms",
                         std::to_string(integration_time_ms));
    config->set_property("Acquisition.max_dwells", "1");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Multithread_Acquisition");
    config->set_property("Acquisition.pfa", "0.1");
    config->set_property("Acquisition.doppler_max", "10000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.bit_transition_flag", "false");
    config->set_property("Acquisition.dump", "false");
}

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::wait_message, this);
}

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::wait_message()
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
            end = tv.tv_sec *1e6 + tv.tv_usec;

            mean_acq_time_us += (end-begin);

            process_message();
        }
}

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::process_message()
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

void GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test::stop_queue()
{
    stop = true;
}

TEST_F(GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test, Instantiate)
{
    config_1();
    acquisition = std::make_shared<GpsL1CaPcpsMultithreadAcquisition>(config.get(), "Acquisition", 1, 1);
}

TEST_F(GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test, ConnectAndRun)
{
    int nsamples = floor(fs_in*integration_time_ms*1e-3);
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    config_1();
    acquisition = std::make_shared<GpsL1CaPcpsMultithreadAcquisition>(config.get(), "Acquisition", 1, 1);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1e6 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1e6 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}

TEST_F(GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test, ValidationOfResults)
{
    config_1();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    acquisition = std::make_shared<GpsL1CaPcpsMultithreadAcquisition>(config.get(), "Acquisition", 1, 1);

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
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    acquisition->init();

    ASSERT_NO_THROW( {
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 10; // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20; // This satellite is not visible
                }

            acquisition->set_local_code();

            start_queue();

            EXPECT_NO_THROW( {
                top_block->run(); // Start threads and wait
            }) << "Failure running the top_block." << std::endl;

            if (i == 0)
            {
                EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                if (message == 1)
                    {
                        EXPECT_EQ((unsigned int)1, correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                    }

            }
            else if (i == 1)
            {
                EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
            }
        }
}

TEST_F(GpsL1CaPcpsMultithreadAcquisitionGSoC2013Test, ValidationOfResultsProbabilities)
{
    config_2();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    acquisition = std::make_shared<GpsL1CaPcpsMultithreadAcquisition>(config.get(), "Acquisition", 1, 1);

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
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    acquisition->init();

    ASSERT_NO_THROW( {
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    std::cout << "Probability of false alarm (target) = " << 0.1 << std::endl;

    // i = 0 --> satellite in acquisition is visible (prob of detection and prob of detection with wrong estimation)
    // i = 1 --> satellite in acquisition is not visible (prob of false detection)
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 10; // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20; // This satellite is not visible
                }

            acquisition->set_local_code();

            start_queue();

            EXPECT_NO_THROW( {
                top_block->run(); // Start threads and wait
            }) << "Failure running the top_block." << std::endl;

            if (i == 0)
            {
                std::cout << "Probability of detection = " << Pd << std::endl;
                std::cout << "Probability of false alarm (satellite present) = " << Pfa_p << std::endl;
//                std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;
            }
            else if (i == 1)
            {
                std::cout << "Probability of false alarm (satellite absent) = " << Pfa_a << std::endl;
//                std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;
            }
        }

}

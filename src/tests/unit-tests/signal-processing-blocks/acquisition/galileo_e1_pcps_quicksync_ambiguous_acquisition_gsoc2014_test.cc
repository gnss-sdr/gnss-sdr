/*!
 * \file galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc
 * \brief  This class implements an acquisition test for
 * GalileoE1PcpsQuickSyncAmbiguousAcquisition class.
 * \author Damian Miralles, 2014. dmiralles2009@gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <chrono>
#include <fstream>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <glog/logging.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include <gnuradio/blocks/null_sink.h>
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_synchro.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h"
#include "gnss_sdr_valve.h"
#include "galileo_e1_pcps_quicksync_ambiguous_acquisition.h"

DEFINE_double(e1_value_threshold, 0.3, "Value of the threshold for the acquisition");
DEFINE_int32(e1_value_CN0_dB_0, 50, "Value for the CN0_dB_0 in channel 0");

using google::LogMessage;


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx;

typedef boost::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx> GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_sptr;

GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_sptr GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(concurrent_queue<int>& queue);


class GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx : public gr::block
{
private:
    friend GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_sptr GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx();  //!< Default destructor
};


GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_sptr GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(concurrent_queue<int>& queue)
{
    return GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_sptr(new GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx(queue));
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx::GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx(concurrent_queue<int>& queue) : gr::block("GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx::~GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx()
{
}


// ###########################################################

class GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test : public ::testing::Test
{
protected:
    GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
        init();
    }
    ~GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test()
    {
    }

    void init();
    void config_1();
    void config_2();
    void config_3();
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();

    concurrent_queue<int> channel_internal_queue;
    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisition> acquisition;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
    boost::thread ch_thread;

    unsigned int integration_time_ms = 0;
    unsigned int fs_in = 0;
    unsigned int folding_factor = 0;

    double expected_delay_chips = 0.0;
    double expected_doppler_hz = 0.0;
    float max_doppler_error_hz = 0.0;
    float max_delay_error_chips = 0.0;

    unsigned int num_of_realizations = 0;
    unsigned int realization_counter;
    unsigned int detection_counter;
    unsigned int correct_estimation_counter;
    unsigned int acquired_samples;
    unsigned int mean_acq_time_us;

    double mse_doppler;
    double mse_delay;

    double Pd;     // Probability of detection
    double Pfa_p;  // Probability of false alarm on present satellite
    double Pfa_a;  // Probability of false alarm on absent satellite
    double Pmd;    // Probability of miss detection

    std::ofstream pdpfafile;
    unsigned int miss_detection_counter;
    bool dump_test_results = false;
};


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::init()
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

    miss_detection_counter = 0;
    Pmd = 0;
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::config_1()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);

    integration_time_ms = 8;
    fs_in = 4e6;

    expected_delay_chips = 600;
    expected_doppler_hz = 750;
    max_doppler_error_hz = 2 / (3 * integration_time_ms * 1e-3);
    max_delay_error_chips = 0.50;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "1");

    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0",
        std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0",
        std::to_string(expected_delay_chips));

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

    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B.item_type", "gr_complex");
    config->set_property("Acquisition_1B.coherent_integration_time_ms",
        std::to_string(integration_time_ms));
    config->set_property("Acquisition_1B.max_dwells", "1");
    config->set_property("Acquisition_1B.bit_transition_flag", "false");
    config->set_property("Acquisition_1B.threshold", "1");
    config->set_property("Acquisition_1Bdoppler_max", "10000");
    config->set_property("Acquisition_1B.doppler_step", "250");
    config->set_property("Acquisition_1B.folding_factor", "2");
    config->set_property("Acquisition_1B.dump", "false");
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::config_2()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);

    integration_time_ms = 8;
    fs_in = 4e6;

    expected_delay_chips = 600;
    expected_doppler_hz = 750;
    max_doppler_error_hz = 2 / (3 * integration_time_ms * 1e-3);
    max_delay_error_chips = 0.50;

    /*Unset this flag to eliminates data logging for the Validation of results
    probabilities test*/
    dump_test_results = true;

    num_of_realizations = 100;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "4");

    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", std::to_string(FLAGS_e1_value_CN0_dB_0));
    config->set_property("SignalSource.doppler_Hz_0",
        std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0",
        std::to_string(expected_delay_chips));

    config->set_property("SignalSource.system_1", "E");
    config->set_property("SignalSource.PRN_1", "15");
    config->set_property("SignalSource.CN0_dB_1", "44");
    config->set_property("SignalSource.doppler_Hz_1", "1000");
    config->set_property("SignalSource.delay_chips_1", "100");

    config->set_property("SignalSource.system_2", "E");
    config->set_property("SignalSource.PRN_2", "21");
    config->set_property("SignalSource.CN0_dB_2", "44");
    config->set_property("SignalSource.doppler_Hz_2", "2000");
    config->set_property("SignalSource.delay_chips_2", "200");

    config->set_property("SignalSource.system_3", "E");
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

    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B.item_type", "gr_complex");
    config->set_property("Acquisition_1B.coherent_integration_time_ms",
        std::to_string(integration_time_ms));
    config->set_property("Acquisition_1B.max_dwells", "1");
    config->set_property("Acquisition_1B.bit_transition_flag", "false");
    config->set_property("Acquisition_1B.threshold", std::to_string(FLAGS_e1_value_threshold));
    config->set_property("Acquisition_1B.doppler_max", "10000");
    config->set_property("Acquisition_1B.doppler_step", "125");
    config->set_property("Acquisition_1B.folding_factor", "2");
    config->set_property("Acquisition_1B.dump", "false");
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::config_3()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);

    integration_time_ms = 8;
    fs_in = 4e6;

    expected_delay_chips = 600;
    expected_doppler_hz = 750;
    max_doppler_error_hz = 2 / (3 * integration_time_ms * 1e-3);
    max_delay_error_chips = 0.50;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "4");

    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.PRN_0", "10");
    config->set_property("SignalSource.CN0_dB_0", std::to_string(FLAGS_e1_value_CN0_dB_0));
    config->set_property("SignalSource.doppler_Hz_0",
        std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0",
        std::to_string(expected_delay_chips));

    config->set_property("SignalSource.system_1", "E");
    config->set_property("SignalSource.PRN_1", "15");
    config->set_property("SignalSource.CN0_dB_1", "44");
    config->set_property("SignalSource.doppler_Hz_1", "1000");
    config->set_property("SignalSource.delay_chips_1", "100");

    config->set_property("SignalSource.system_2", "E");
    config->set_property("SignalSource.PRN_2", "21");
    config->set_property("SignalSource.CN0_dB_2", "44");
    config->set_property("SignalSource.doppler_Hz_2", "2000");
    config->set_property("SignalSource.delay_chips_2", "200");

    config->set_property("SignalSource.system_3", "E");
    config->set_property("SignalSource.PRN_3", "22");
    config->set_property("SignalSource.CN0_dB_3", "44");
    config->set_property("SignalSource.doppler_Hz_3", "3000");
    config->set_property("SignalSource.delay_chips_3", "300");

    config->set_property("SignalSource.noise_flag", "false");  //
    config->set_property("SignalSource.data_flag", "false");   //
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

    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B.item_type", "gr_complex");
    config->set_property("Acquisition_1B.coherent_integration_time_ms",
        std::to_string(integration_time_ms));
    config->set_property("Acquisition_1B.max_dwells", "1");
    config->set_property("Acquisition_1B.bit_transition_flag", "false");
    config->set_property("Acquisition_1B.threshold", "0.2");
    config->set_property("Acquisition_1B.doppler_max", "10000");
    config->set_property("Acquisition_1B.doppler_step", "125");
    config->set_property("Acquisition_1B.folding_factor", "4");
    config->set_property("Acquisition_1B.dump", "false");
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::wait_message, this);
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::wait_message()
{
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);

    while (!stop)
        {
            acquisition->reset();

            begin = std::chrono::system_clock::now();

            channel_internal_queue.wait_and_pop(message);

            end = std::chrono::system_clock::now();
            elapsed_seconds = end - begin;

            mean_acq_time_us += elapsed_seconds.count() * 1e6;

            process_message();
        }
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::process_message()
{
    if (message == 1)
        {
            detection_counter++;

            // The term -5 is here to correct the additional delay introduced by the FIR filter
            double delay_error_chips = std::abs(static_cast<double>(expected_delay_chips) - static_cast<double>(gnss_synchro.Acq_delay_samples - 5) * 1023.0 / (static_cast<double>(fs_in) * 1e-3));
            double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

            mse_delay += std::pow(delay_error_chips, 2);
            mse_doppler += std::pow(doppler_error_hz, 2);

            if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
                {
                    correct_estimation_counter++;
                }
        }
    else if (message == 2 && gnss_synchro.PRN == 10)
        {
            /*
               if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
        {
               miss_detection_counter++;
        }
             */
            miss_detection_counter++;
        }

    realization_counter++;

    std::cout << "Progress: " << round(static_cast<float>(realization_counter) / static_cast<float>(num_of_realizations) * 100.0) << "% \r" << std::flush;

    if (realization_counter == num_of_realizations)
        {
            mse_delay /= static_cast<double>(num_of_realizations);
            mse_doppler /= static_cast<double>(num_of_realizations);

            Pd = static_cast<double>(correct_estimation_counter) / static_cast<double>(num_of_realizations);
            Pfa_a = static_cast<double>(detection_counter) / static_cast<double>(num_of_realizations);
            Pfa_p = static_cast<double>(detection_counter - correct_estimation_counter) / static_cast<double>(num_of_realizations);
            Pmd = static_cast<double>(miss_detection_counter) / static_cast<double>(num_of_realizations);

            mean_acq_time_us /= static_cast<double>(num_of_realizations);

            stop_queue();
            top_block->stop();
        }
}


void GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test::stop_queue()
{
    stop = true;
}


TEST_F(GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test, Instantiate)
{
    config_1();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", 1, 0);
    acquisition = std::dynamic_pointer_cast<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(acq_);
}


TEST_F(GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test, ConnectAndRun)
{
    LOG(INFO) << "**Start connect and run test";
    int nsamples = floor(fs_in * integration_time_ms * 1e-3);
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    config_1();

    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", 1, 0);
    acquisition = std::dynamic_pointer_cast<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(acq_);
    boost::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx> msg_rx = GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source =
            gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve =
            gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        begin = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - begin;
    }) << "Failure running the top_block.";

    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
    LOG(INFO) << "----end connect and run test-----";
    LOG(INFO) << "**End connect and run test";
}


TEST_F(GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test, ValidationOfResults)
{
    LOG(INFO) << "Start validation of results test";
    config_1();
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", 1, 0);
    acquisition = std::dynamic_pointer_cast<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(acq_);
    boost::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx> msg_rx = GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->set_channel(0);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(config->property("Acquisition_1B.doppler_max", 10000));
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(config->property("Acquisition_1B.doppler_step", 125));
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(1);
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    acquisition->init();
    acquisition->reset();

    ASSERT_NO_THROW({
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";


    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 10;  // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20;  // This satellite is not visible
                }
            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->set_local_code();
            acquisition->reset();
            acquisition->set_state(1);
            start_queue();

            EXPECT_NO_THROW({
                top_block->run();  // Start threads and wait
            }) << "Failure running the top_block.";

            stop_queue();

            if (i == 0)
                {
                    EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                    EXPECT_EQ(static_cast<unsigned int>(1), correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                }
            else if (i == 1)
                {
                    EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
                }

            ch_thread.join();
        }
    DLOG(INFO) << "End validation of results test";
}


TEST_F(GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test, ValidationOfResultsWithNoise)
{
    LOG(INFO) << "Start validation of results with noise+interference test";
    config_3();
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", 1, 0);
    acquisition = std::dynamic_pointer_cast<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(acq_);
    boost::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx> msg_rx = GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(config->property("Acquisition_1B.doppler_max", 10000));
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(50);
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(5);
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    acquisition->init();
    acquisition->reset();

    ASSERT_NO_THROW({
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 10;  // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20;  // This satellite is not visible
                }

            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->set_local_code();
            acquisition->reset();
            acquisition->set_state(1);
            start_queue();

            EXPECT_NO_THROW({
                top_block->run();  // Start threads and wait
            }) << "Failure running the top_block.";

            stop_queue();
            if (i == 0)
                {
                    EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                    EXPECT_EQ(static_cast<unsigned int>(1), correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                }
            else if (i == 1)
                {
                    EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
                }
            ch_thread.join();
        }
    DLOG(INFO) << "End validation of results with noise+interference test";
}


TEST_F(GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test, ValidationOfResultsProbabilities)
{
    config_2();
    top_block = gr::make_top_block("Acquisition test");
    queue = gr::msg_queue::make(0);

    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", 1, 0);
    acquisition = std::dynamic_pointer_cast<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(acq_);
    boost::shared_ptr<GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx> msg_rx = GalileoE1PcpsQuickSyncAmbiguousAcquisitionGSoC2014Test_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(config->property("Acquisition_1B.doppler_max", 10000));
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(config->property("Acquisition_1B.doppler_step", 500));
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(config->property("Acquisition_1B.threshold", 0.0));
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    acquisition->init();

    ASSERT_NO_THROW({
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    std::cout << "Probability of false alarm (target) = " << 0.1 << std::endl;

    // i = 0 --> satellite in acquisition is visible (prob of detection and prob of detection with wrong estimation)
    // i = 1 --> satellite in acquisition is not visible (prob of false detection)
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 10;  // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20;  // This satellite is not visible
                }

            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->set_local_code();
            acquisition->reset();
            acquisition->set_state(1);
            start_queue();

            EXPECT_NO_THROW({
                top_block->run();  // Start threads and wait
            }) << "Failure running the top_block.";

            stop_queue();

            if (i == 0)
                {
                    std::cout << "Estimated probability of detection = " << Pd << std::endl;
                    std::cout << "Estimated probability of false alarm (satellite present) = " << Pfa_p << std::endl;
                    std::cout << "Estimated probability of miss detection (satellite present) = " << Pmd << std::endl;
                    std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;

                    if (dump_test_results)
                        {
                            std::stringstream filenamepd;
                            filenamepd.str("");
                            filenamepd << "../data/test_statistics_" << gnss_synchro.System
                                       << "_" << gnss_synchro.Signal << "_sat_"
                                       << gnss_synchro.PRN << "CN0_dB_0_" << FLAGS_e1_value_CN0_dB_0 << "_dBHz.csv";

                            pdpfafile.open(filenamepd.str().c_str(), std::ios::app | std::ios::out);
                            pdpfafile << FLAGS_e1_value_threshold << "," << Pd << "," << Pfa_p << "," << Pmd << std::endl;
                            pdpfafile.close();
                        }
                }
            else if (i == 1)
                {
                    std::cout << "Estimated probability of false alarm (satellite absent) = " << Pfa_a << std::endl;
                    std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;

                    if (dump_test_results)
                        {
                            std::stringstream filenamepf;
                            filenamepf.str("");
                            filenamepf << "../data/test_statistics_" << gnss_synchro.System
                                       << "_" << gnss_synchro.Signal << "_sat_"
                                       << gnss_synchro.PRN << "CN0_dB_0_" << FLAGS_e1_value_CN0_dB_0 << "_dBHz.csv";

                            pdpfafile.open(filenamepf.str().c_str(), std::ios::app | std::ios::out);
                            pdpfafile << FLAGS_e1_value_threshold << "," << Pfa_a << std::endl;
                            pdpfafile.close();
                        }
                }
            ch_thread.join();
        }
}

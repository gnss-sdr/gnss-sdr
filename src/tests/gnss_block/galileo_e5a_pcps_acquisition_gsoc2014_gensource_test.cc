/*!
 * \file galileo_e5a_pcps_acquisition_gsoc2014_gensource_test.cc
 * \brief  This class implements an acquisition test for
 * GalileoE5a3msNoncoherentIQAcquisition class.
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
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
#include <boost/chrono.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "configuration_interface.h"
#include "gnss_synchro.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h"
#include "gnss_sdr_valve.h"
#include "pass_through.h"

#include "gnss_block_factory.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx;

typedef boost::shared_ptr<GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx> GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_sptr;

GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_sptr GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_make(concurrent_queue<int>& queue);


class GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx : public gr::block
{
private:
    friend GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_sptr GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;
public:
    int rx_message;
    ~GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx(); //!< Default destructor
};


GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_sptr GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_make(concurrent_queue<int>& queue)
{
    return GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_sptr(new GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx(queue));
}


void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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


GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx::GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx(concurrent_queue<int>& queue) :
    gr::block("GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx::~GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx()
{}


class GalileoE5aPcpsAcquisitionGSoC2014GensourceTest: public ::testing::Test
{
protected:
    GalileoE5aPcpsAcquisitionGSoC2014GensourceTest()
    {
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
        init();
    }

    ~GalileoE5aPcpsAcquisitionGSoC2014GensourceTest()
    {}

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
    //std::shared_ptr<GNSSBlockFactory> factory = std::make_shared<GNSSBlockFactory>();
    std::shared_ptr<GalileoE5aNoncoherentIQAcquisitionCaf> acquisition;

    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
    boost::thread ch_thread;

    unsigned int integration_time_ms = 0;
    unsigned int fs_in = 0;

    double expected_delay_chips = 0.0;
    double expected_delay_sec = 0.0;
    double expected_doppler_hz = 0.0;
    double expected_delay_chips1 = 0.0;
    double expected_delay_sec1 = 0.0;
    double expected_doppler_hz1 = 0.0;
    double expected_delay_chips2 = 0.0;
    double expected_delay_sec2 = 0.0;
    double expected_doppler_hz2 = 0.0;
    double expected_delay_chips3 = 0.0;
    double expected_delay_sec3 = 0.0;
    double expected_doppler_hz3 = 0.0;
    float max_doppler_error_hz = 0.0;
    float max_delay_error_chips = 0.0;
    int CAF_window_hz = 0;
    int Zero_padding = 0;

    unsigned int num_of_realizations = 0;
    unsigned int realization_counter;
    unsigned int detection_counter;
    unsigned int correct_estimation_counter;
    unsigned int acquired_samples;
    unsigned int mean_acq_time_us;

    double mse_doppler;
    double mse_delay;

    double Pd;
    double Pfa_p;
    double Pfa_a;

    int sat = 0;
};

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::init()
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

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::config_1()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    //    std::string signal = "5I";
    //    std::string signal = "5Q";
    std::string signal = "5X";
    signal.copy(gnss_synchro.Signal,2,0);


    integration_time_ms = 3;
    //fs_in = 11e6;
    //fs_in = 18e6;
    fs_in = 32e6;
    //fs_in = 30.69e6;
    //fs_in = 20.47e6;

    //    unsigned int delay_samples = (delay_chips_[sat] % codelen)
    //                          * samples_per_code_[sat] / codelen;
    expected_delay_chips = round(14000*((double)10230000/(double)fs_in));
    expected_doppler_hz = 2800;
    //expected_doppler_hz = 0;
    expected_delay_sec = 94;
    //    CAF_window_hz = 3000;
    CAF_window_hz = 0;
    Zero_padding = 0;

    //expected_delay_chips = 1000;
    //expected_doppler_hz = 250;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    //max_doppler_error_hz = 1000;
    //max_delay_error_chips = 1;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("Channel.signal",signal);
    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));
    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.num_satellites", "1");
    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.signal_0", "5X");
    config->set_property("SignalSource.PRN_0", "11");
    config->set_property("SignalSource.CN0_dB_0", "50");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));
    config->set_property("SignalSource.delay_sec_0", std::to_string(expected_delay_sec));

    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");

    config->set_property("SignalSource.dump", "false");
    config->set_property("SignalSource.dump_filename", "../data/signal_source.dat");

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

    config->set_property("Acquisition_Galileo.item_type", "gr_complex");
    config->set_property("Acquisition_Galileo.if", "0");
    config->set_property("Acquisition_Galileo.coherent_integration_time_ms",
            std::to_string(integration_time_ms));
    config->set_property("Acquisition_Galileo.max_dwells", "1");
    config->set_property("Acquisition_Galileo.CAF_window_hz",std::to_string(CAF_window_hz));
    config->set_property("Acquisition_Galileo.Zero_padding",std::to_string(Zero_padding));

    config->set_property("Acquisition_Galileo.implementation", "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF");
    config->set_property("Acquisition_Galileo.pfa","0.003");
    //    config->set_property("Acquisition_Galileo.threshold", "0.01");
    config->set_property("Acquisition_Galileo.doppler_max", "10000");
    config->set_property("Acquisition_Galileo.doppler_step", "250");
    //    config->set_property("Acquisition_Galileo.doppler_step", "500");
    config->set_property("Acquisition_Galileo.bit_transition_flag", "false");
    config->set_property("Acquisition_Galileo.dump", "false");
    config->set_property("SignalSource.dump_filename", "../data/acquisition.dat");
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::config_2()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "5Q";
    //std::string signal = "5X";
    signal.copy(gnss_synchro.Signal,2,0);

    integration_time_ms = 3;
    //fs_in = 10.24e6;
    //fs_in = 12e6;
    fs_in = 12e6;

    //expected_delay_chips = 600;
    //expected_doppler_hz = 750;

    expected_delay_chips = 1000;
    expected_doppler_hz = 250;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    //max_doppler_error_hz = 1000;
    //max_delay_error_chips = 1;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("Acquisition_Galileo.item_type", "gr_complex");
    config->set_property("Acquisition_Galileo.if", "0");
    config->set_property("Acquisition_Galileo.coherent_integration_time_ms",
            std::to_string(integration_time_ms));
    config->set_property("Acquisition_Galileo.max_dwells", "1");
    config->set_property("Acquisition_Galileo.implementation", "Galileo_E5a_PCPS_Acquisition");
    //config->set_property("Acquisition_Galileo.implementation", "Galileo_E5a_Pilot_3ms_Acquisition");
    //config->set_property("Acquisition_Galileo.implementation", "Galileo_E5ax_2ms_Pcps_Acquisition");
    config->set_property("Acquisition_Galileo.threshold", "0.1");
    config->set_property("Acquisition_Galileo.doppler_max", "10000");
    config->set_property("Acquisition_Galileo.doppler_step", "250");
    config->set_property("Acquisition_Galileo.bit_transition_flag", "false");
    config->set_property("Acquisition_Galileo.dump", "false");
    config->set_property("SignalSource.dump_filename", "../data/acquisition.dat");
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::config_3()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    //std::string signal = "5Q";
    std::string signal = "5X";
    signal.copy(gnss_synchro.Signal,2,0);


    integration_time_ms = 3;
    //fs_in = 10.24e6;
    //fs_in = 12e6;
    fs_in = 12e6;

    //expected_delay_chips = 600;
    //expected_doppler_hz = 750;

    expected_delay_chips = 0;
    expected_delay_sec = 0;
    expected_doppler_hz = 0;
    expected_delay_chips1 = 6000;
    expected_delay_sec1 = 10;
    expected_doppler_hz1 = 700;
    expected_delay_chips2 = 9000;
    expected_delay_sec2 = 26;
    expected_doppler_hz2 = -1500;
    expected_delay_chips3 = 2000;
    expected_delay_sec3 = 77;
    expected_doppler_hz3 = 5000;

    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;
    //max_doppler_error_hz = 1000;
    //max_delay_error_chips = 1;

    num_of_realizations = 10;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "4");

    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.signal_0", "5X");
    config->set_property("SignalSource.PRN_0", "11");
    config->set_property("SignalSource.CN0_dB_0", "46");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));
    config->set_property("SignalSource.delay_sec_0", std::to_string(expected_delay_sec));

    config->set_property("SignalSource.system_1", "E");
    config->set_property("SignalSource.signal_1", "5X");
    config->set_property("SignalSource.PRN_1", "12");
    config->set_property("SignalSource.CN0_dB_1", "46");
    config->set_property("SignalSource.doppler_Hz_1", std::to_string(expected_doppler_hz1));
    config->set_property("SignalSource.delay_chips_1", std::to_string(expected_delay_chips1));
    config->set_property("SignalSource.delay_sec_1", std::to_string(expected_delay_sec1));

    config->set_property("SignalSource.system_2", "E");
    config->set_property("SignalSource.signal_2", "5X");
    config->set_property("SignalSource.PRN_2", "19");
    config->set_property("SignalSource.CN0_dB_2", "43");
    config->set_property("SignalSource.doppler_Hz_2", std::to_string(expected_doppler_hz2));
    config->set_property("SignalSource.delay_chips_2", std::to_string(expected_delay_chips2));
    config->set_property("SignalSource.delay_sec_2", std::to_string(expected_delay_sec2));

    config->set_property("SignalSource.system_3", "E");
    config->set_property("SignalSource.signal_3", "5X");
    config->set_property("SignalSource.PRN_3", "20");
    config->set_property("SignalSource.CN0_dB_3", "39");
    config->set_property("SignalSource.doppler_Hz_3", std::to_string(expected_doppler_hz3));
    config->set_property("SignalSource.delay_chips_3", std::to_string(expected_delay_chips3));
    config->set_property("SignalSource.delay_sec_3", std::to_string(expected_delay_sec3));

    config->set_property("SignalSource.noise_flag", "true");
    config->set_property("SignalSource.data_flag", "true");
    config->set_property("SignalSource.BW_BB", "0.97");

    config->set_property("SignalSource.dump", "false");
    config->set_property("SignalSource.dump_filename", "../data/signal_source.dat");

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

    config->set_property("Acquisition_Galileo.item_type", "gr_complex");
    config->set_property("Acquisition_Galileo.if", "0");
    config->set_property("Acquisition_Galileo.coherent_integration_time_ms",
            std::to_string(integration_time_ms));
    config->set_property("Acquisition_Galileo.max_dwells", "1");
    config->set_property("Acquisition_Galileo.implementation", "Galileo_E5a_PCPS_Acquisition");
    //config->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    //config->set_property("Acquisition.implementation", "Galileo_E5a_Pilot_3ms_Acquisition");

    config->set_property("Acquisition_Galileo.threshold", "0.5");
    config->set_property("Acquisition_Galileo.doppler_max", "10000");
    config->set_property("Acquisition_Galileo.doppler_step", "250");
    config->set_property("Acquisition_Galileo.bit_transition_flag", "false");
    config->set_property("Acquisition_Galileo.dump", "false");
    config->set_property("SignalSource.dump_filename", "../data/acquisition.dat");
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::wait_message, this);
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::wait_message()
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

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::process_message()
{
    if (message == 1)
        {
            double delay_error_chips = 0.0;
            double doppler_error_hz = 0.0;
            switch (sat)
            {
            case 0:
                delay_error_chips = std::abs((double)expected_delay_chips - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
                doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);
                break;
            case 1:
                delay_error_chips = std::abs((double)expected_delay_chips1 - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
                doppler_error_hz = std::abs(expected_doppler_hz1 - gnss_synchro.Acq_doppler_hz);
                break;
            case 2:
                delay_error_chips = std::abs((double)expected_delay_chips2 - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
                doppler_error_hz = std::abs(expected_doppler_hz2 - gnss_synchro.Acq_doppler_hz);
                break;
            case 3:
                delay_error_chips = std::abs((double)expected_delay_chips3 - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
                doppler_error_hz = std::abs(expected_doppler_hz3 - gnss_synchro.Acq_doppler_hz);
                break;
            default: // case 3
                std::cout << "Error: message from unexpected acquisition channel" << std::endl;
                break;
            }
            detection_counter++;

            // The term -5 is here to correct the additional delay introduced by the FIR filter
            /*
            double delay_error_chips = abs((double)expected_delay_chips - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
            double doppler_error_hz = abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);
             */
            mse_delay += std::pow(delay_error_chips, 2);
            mse_doppler += std::pow(doppler_error_hz, 2);

            if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
                {
                    correct_estimation_counter++;
                }
        }

    realization_counter++;

    //std::cout << correct_estimation_counter << "correct estimation counter" << std::endl;
    std::cout << "Progress: " << round((float)realization_counter/num_of_realizations*100) << "% \r" << std::flush;
    //std::cout << message << "message" <<std::endl;
    if (realization_counter == num_of_realizations)
        {
            mse_delay /= num_of_realizations;
            mse_doppler /= num_of_realizations;

            Pd = (double)correct_estimation_counter / (double)num_of_realizations;
            Pfa_a = (double)detection_counter / (double)num_of_realizations;
            Pfa_p = (double)(detection_counter - correct_estimation_counter) / (double)num_of_realizations;

            mean_acq_time_us /= num_of_realizations;

            stop_queue();
            top_block->stop();
        }
}


void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::stop_queue()
{
    stop = true;
}


TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, Instantiate)
{
    config_1();
    acquisition = std::make_shared<GalileoE5aNoncoherentIQAcquisitionCaf>(config.get(), "Acquisition", 1, 1);
}


TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ConnectAndRun)
{
    config_1();
    //int nsamples = floor(5*fs_in*integration_time_ms*1e-3);
    int nsamples = 21000*3;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    acquisition = std::make_shared<GalileoE5aNoncoherentIQAcquisitionCaf>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx> msg_rx = GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_make(channel_internal_queue);
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test."<< std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1e6 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1e6 + tv.tv_usec;
    }) << "Failure running the top_block."<< std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}

/*
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, SOURCEValidation)
{
    config_1();
    ASSERT_NO_THROW( {
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);


        //top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);

    }) << "Failure generating signal" << std::endl;
}
 */
/*
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, SOURCEValidationTOFILE)
{
    config_1();
    ASSERT_NO_THROW( {
    std::string filename_ = "../data/Tiered_sinknull.dat";
    boost::shared_ptr<gr::blocks::file_sink> file_sink_;

        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1);
        signal_source.reset(new GenSignalSource(signal_generator, filter, "SignalSource", queue));
        //signal_source->connect(top_block);
        file_sink_=gr::blocks::file_sink::make(sizeof(gr_complex), filename_.c_str());

        top_block->connect(signal_source->get_right_block(),0,file_sink_,0);

        //top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);

    }) << "Failure generating signal" << std::endl;
}
 */

TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ValidationOfSIM)
{
    config_1();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");
    acquisition = std::make_shared<GalileoE5aNoncoherentIQAcquisitionCaf>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx> msg_rx = GalileoE5aPcpsAcquisitionGSoC2014GensourceTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW( {
        acquisition->set_channel(0);
    }) << "Failure setting channel."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(config->property("Acquisition_Galileo.doppler_max", 10000));
    }) << "Failure setting doppler_max."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(config->property("Acquisition_Galileo.doppler_step", 500));
    }) << "Failure setting doppler_step."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition_Galileo.threshold", 0.0));
    }) << "Failure setting threshold."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block."<< std::endl;

    acquisition->init();
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

    acquisition->reset();
    acquisition->init();

    // USING SIGNAL FROM FILE SOURCE
    //unsigned int skiphead_sps = 28000+32000; // 32 Msps
    //    unsigned int skiphead_sps = 0;
    //    unsigned int skiphead_sps = 84000;

    // ASSERT_NO_THROW( {
    //   //noiseless sim
    //   //std::string file =  "/home/marc/E5a_acquisitions/sim_32M_sec94_PRN11_long.dat";
    //   // real
    // std::string file =  "/home/marc/E5a_acquisitions/32MS_complex.dat";
    //
    // const char * file_name = file.c_str();
    // gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
    //
    // gr::blocks::skiphead::sptr skip_head = gr::blocks::skiphead::make(sizeof(gr_complex), skiphead_sps);
    // top_block->connect(file_source, 0, skip_head, 0);
    // top_block->connect(skip_head, 0, acquisition->get_left_block(), 0);
    //
    //   // top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    //  }) << "Failure connecting the blocks of acquisition test." << std::endl;

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 1; i++)
        {
            init();

            switch (i)
            {
            case 0:
                {
                    gnss_synchro.PRN = 19; // present
                }
            case 1:
                {
                    gnss_synchro.PRN = 11;
                }
            }

            start_queue();

            acquisition->reset();
            acquisition->init();
            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->set_local_code();
            acquisition->set_state(1);

            EXPECT_NO_THROW( {
                top_block->run(); // Start threads and wait
            }) << "Failure running the top_block."<< std::endl;

            stop_queue();

            ch_thread.join();
            //std::cout << gnss_synchro.Acq_delay_samples << "acq delay" <<std::endl;
            //std::cout << gnss_synchro.Acq_doppler_hz << "acq doppler" <<std::endl;
            //std::cout << gnss_synchro.Acq_samplestamp_samples << "acq samples" <<std::endl;
            if (i == 0)
                {
                    EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                    if (message == 1)
                        {
                            //std::cout << gnss_synchro.Acq_delay_samples << "acq delay" <<std::endl;
                            EXPECT_EQ((unsigned int) 1, correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                        }

                }
            else if (i == 1)
                {
                    EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
                }
        }
}




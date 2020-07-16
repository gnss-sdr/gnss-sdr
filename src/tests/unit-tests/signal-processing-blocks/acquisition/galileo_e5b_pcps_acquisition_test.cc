/*!
 * \file Galileo_E5b_pcps_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * GalileoE5bPcpsAcquisition class based on some input parameters.
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as GSoC 2020 Program.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
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
#include "fir_filter.h"
#include "galileo_e5b_pcps_acquisition.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include "pass_through.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstdlib>
#include <memory>
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

#if GNURADIO_USES_STD_POINTERS
#else
#include <boost/make_shared.hpp>
#endif

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GalileoE5bPcpsAcquisitionTest_msg_rx;

#if GNURADIO_USES_STD_POINTERS
using GalileoE5bPcpsAcquisitionTest_msg_rx_sptr = std::shared_ptr<GalileoE5bPcpsAcquisitionTest_msg_rx>;
#else
using GalileoE5bPcpsAcquisitionTest_msg_rx_sptr = boost::shared_ptr<GalileoE5bPcpsAcquisitionTest_msg_rx>;
#endif

GalileoE5bPcpsAcquisitionTest_msg_rx_sptr GalileoE5bPcpsAcquisitionTest_msg_rx_make(Concurrent_Queue<int>& queue);

class GalileoE5bPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GalileoE5bPcpsAcquisitionTest_msg_rx_sptr GalileoE5bPcpsAcquisitionTest_msg_rx_make(Concurrent_Queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    explicit GalileoE5bPcpsAcquisitionTest_msg_rx(Concurrent_Queue<int>& queue);
    Concurrent_Queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~GalileoE5bPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


GalileoE5bPcpsAcquisitionTest_msg_rx_sptr GalileoE5bPcpsAcquisitionTest_msg_rx_make(Concurrent_Queue<int>& queue)
{
    return GalileoE5bPcpsAcquisitionTest_msg_rx_sptr(new GalileoE5bPcpsAcquisitionTest_msg_rx(queue));
}


void GalileoE5bPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (boost::bad_any_cast& e)
        {
            std::cout << "msg_handler_telemetry Bad any cast!" << std::endl;
            rx_message = 0;
        }
}


GalileoE5bPcpsAcquisitionTest_msg_rx::GalileoE5bPcpsAcquisitionTest_msg_rx(Concurrent_Queue<int>& queue) : gr::block("GalileoE5bPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](pmt::pmt_t&& PH1) { msg_handler_events(PH1); });
#else
#if BOOST_173_OR_GREATER
        boost::bind(&GalileoE5bPcpsAcquisitionTest_msg_rx::msg_handler_events, this, boost::placeholders::_1));
#else
        boost::bind(&GalileoE5bPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
#endif
#endif
    rx_message = 0;
}


GalileoE5bPcpsAcquisitionTest_msg_rx::~GalileoE5bPcpsAcquisitionTest_msg_rx() = default;


// ###########################################################

class GalileoE5bPcpsAcquisitionTest : public ::testing::Test
{
protected:
    GalileoE5bPcpsAcquisitionTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
        stop = false;
        message = 0;
    }

    ~GalileoE5bPcpsAcquisitionTest() = default;

    void init();
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();

    Concurrent_Queue<int> channel_internal_queue;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<GalileoE5bPcpsAcquisition> acquisition;
#else
    boost::shared_ptr<GalileoE5bPcpsAcquisition> acquisition;
#endif
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    std::thread ch_thread;

    Gnss_Synchro gnss_synchro;

    size_t item_size;

    double fs_in = 32e6;
    double expected_doppler_hz = -632.0;
    double expected_delay_chips = 1234;
    double expected_delay_sec = 51;

    double mse_doppler = 0.0;
    double mse_delay = 0.0;
    double Pd = 0.0;
    double Pfa_p = 0.0;
    double Pfa_a = 0.0;

    float integration_time_ms = 1;
    float max_doppler_error_hz = 2 / (3 * integration_time_ms * 1e-3);
    float max_delay_error_chips = 0.5;

    unsigned int num_of_realizations = 1;
    unsigned int realization_counter = 0;
    unsigned int detection_counter = 0;
    unsigned int correct_estimation_counter = 0;
    unsigned int acquired_samples = 0;
    unsigned int mean_acq_time_us = 0;

    bool stop;
    int message;
};


void GalileoE5bPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "7X";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(fs_in));
    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.num_satellites", "1");
    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.signal_0", "7X");
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
    config->set_property("Acquisition_7X.implementation", "Galileo_E5b_PCPS_Acquisition");
    config->set_property("Acquisition_7X.item_type", "gr_complex");
    config->set_property("Acquisition_7X.coherent_integration_time_ms", std::to_string(integration_time_ms));
    config->set_property("Acquisition_7X.dump", "true");
    config->set_property("Acquisition_7X.dump_filename", "./acquisition");
    config->set_property("Acquisition_7X.threshold", "0.001");
    config->set_property("Acquisition_7X.doppler_max", "10000");
    config->set_property("Acquisition_7X.doppler_step", "250");
    config->set_property("Acquisition_7X.repeat_satellite", "false");
}

void GalileoE5bPcpsAcquisitionTest::start_queue()
{
    stop = false;
    ch_thread = std::thread(&GalileoE5bPcpsAcquisitionTest::wait_message, this);
}


void GalileoE5bPcpsAcquisitionTest::wait_message()
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    while (!stop)
        {
            acquisition->reset();

            start = std::chrono::system_clock::now();

            channel_internal_queue.wait_and_pop(message);

            end = std::chrono::system_clock::now();
            elapsed_seconds = end - start;

            mean_acq_time_us += elapsed_seconds.count() * 1e6;

            process_message();
        }
}


void GalileoE5bPcpsAcquisitionTest::process_message()
{
    if (message == 1)
        {
            double delay_error_chips = std::abs(static_cast<double>(expected_delay_chips) - static_cast<double>(gnss_synchro.Acq_delay_samples - 5) * 10230.0 / (static_cast<double>(fs_in) * 1e-3));
            double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);
            // The term -5 is here to correct the additional delay introduced by the FIR filter
            /*
            double delay_error_chips = abs((double)expected_delay_chips - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
            double doppler_error_hz = abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);
             */

            detection_counter++;

            mse_delay += std::pow(delay_error_chips, 2);
            mse_doppler += std::pow(doppler_error_hz, 2);

            if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
                {
                    correct_estimation_counter++;
                }
        }

    realization_counter++;

    // std::cout << correct_estimation_counter << "correct estimation counter\n";
    std::cout << "Progress: " << round(static_cast<float>(realization_counter / num_of_realizations * 100)) << "% \r" << std::flush;
    // std::cout << message << "message'\n'";
    if (realization_counter == num_of_realizations)
        {
            mse_delay /= num_of_realizations;
            mse_doppler /= num_of_realizations;

            Pd = static_cast<double>(correct_estimation_counter) / static_cast<double>(num_of_realizations);
            Pfa_a = static_cast<double>(detection_counter) / static_cast<double>(num_of_realizations);
            Pfa_p = static_cast<double>(detection_counter - correct_estimation_counter) / static_cast<double>(num_of_realizations);

            mean_acq_time_us /= num_of_realizations;

            stop_queue();
            top_block->stop();
        }
}


void GalileoE5bPcpsAcquisitionTest::stop_queue()
{
    stop = true;
}


TEST_F(GalileoE5bPcpsAcquisitionTest, Instantiate)
{
    init();
#if GNURADIO_USES_STD_POINTERS
    acquisition = std::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#else
    acquisition = boost::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#endif
}


TEST_F(GalileoE5bPcpsAcquisitionTest, ConnectAndRun)
{
    int nsamples = 21000 * 3;
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);

    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Acquisition test");

    init();

#if GNURADIO_USES_STD_POINTERS
    acquisition = std::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#else
    acquisition = boost::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#endif

    auto msg_rx = GalileoE5bPcpsAcquisitionTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
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
}


TEST_F(GalileoE5bPcpsAcquisitionTest, ValidationOfResults)
{
    top_block = gr::make_top_block("Acquisition test");

    init();

#if GNURADIO_USES_STD_POINTERS
    acquisition = std::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#else
    acquisition = boost::make_shared<GalileoE5bPcpsAcquisition>(config.get(), "Acquisition_7X", 1, 0);
#endif
    std::shared_ptr<FirFilter> input_filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    auto msg_rx = GalileoE5bPcpsAcquisitionTest_msg_rx_make(channel_internal_queue);
    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    ASSERT_NO_THROW({
        acquisition->set_channel(0);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(0.0001);
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(5000);
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(100);
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    ASSERT_NO_THROW({
        std::shared_ptr<GNSSBlockInterface> signal_generator = std::make_shared<SignalGenerator>(config.get(), "SignalSource", 0, 1, queue.get());
        std::shared_ptr<GNSSBlockInterface> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
        std::shared_ptr<GNSSBlockInterface> signal_source = std::make_shared<GenSignalSource>(signal_generator, filter, "SignalSource", queue.get());
        filter->connect(top_block);
        signal_source->connect(top_block);
        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    acquisition->reset();
    acquisition->init();

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 1; i++)
        {
            init();

            switch (i)
                {
                case 0:
                    {
                        gnss_synchro.PRN = 11;  // present
                        break;
                    }
                case 1:
                    {
                        gnss_synchro.PRN = 19;  // not present
                        break;
                    }
                }

            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->set_local_code();
            acquisition->set_state(1);
            start_queue();

            EXPECT_NO_THROW({
                top_block->run();  // Start threads and wait
            }) << "Failure running the top_block.";

            stop_queue();

            ch_thread.join();

            if (i == 0)
                {
                    EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                    if (message == 1)
                        {
                            // std::cout << gnss_synchro.Acq_delay_samples << "acq delay'\n'";
                            // std::cout << gnss_synchro.Acq_doppler_hz << "acq doppler'\n'";
                            EXPECT_EQ(static_cast<unsigned int>(1), correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                        }
                }
            else if (i == 1)
                {
                    EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
                }
        }
}

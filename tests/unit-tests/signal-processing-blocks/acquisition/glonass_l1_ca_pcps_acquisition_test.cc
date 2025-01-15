/*!
 * \file glonass_l1_ca_pcps_acquisition_test.cc
 * \brief  Tests a PCPS acquisition block for Glonass L1 C/A signals
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "concurrent_queue.h"
#include "freq_xlating_fir_filter.h"
#include "glonass_l1_ca_pcps_acquisition.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include <boost/make_shared.hpp>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstdlib>
#include <utility>
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GlonassL1CaPcpsAcquisitionTest_msg_rx;

using GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr = gnss_shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx>;

GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

class GlonassL1CaPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    GlonassL1CaPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~GlonassL1CaPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make()
{
    return GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr(new GlonassL1CaPcpsAcquisitionTest_msg_rx());
}


void GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (const wht::bad_any_cast& e)
        {
            std::cout << "msg_handler_telemetry Bad any cast!\n";
            rx_message = 0;
        }
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::GlonassL1CaPcpsAcquisitionTest_msg_rx() : gr::block("GlonassL1CaPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::~GlonassL1CaPcpsAcquisitionTest_msg_rx() = default;


// ###########################################################

class GlonassL1CaPcpsAcquisitionTest : public ::testing::Test
{
protected:
    GlonassL1CaPcpsAcquisitionTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GlonassL1CaPcpsAcquisitionTest() = default;

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GlonassL1CaPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'R';
    std::string signal = "1G";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "62314000");
    config->set_property("InputFilter.IF", "9540000");
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
    config->set_property("InputFilter.sampling_frequency", "62314000");
    config->set_property("Acquisition_1G.item_type", "gr_complex");
    config->set_property("Acquisition_1G.coherent_integration_time_ms", "1");
    config->set_property("Acquisition_1G.dump", "true");
    config->set_property("Acquisition_1G.dump_filename", "./acquisition");
    config->set_property("Acquisition_1G.implementation", "Glonass_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1G.threshold", "0.001");
    config->set_property("Acquisition_1G.doppler_max", "5000");
    config->set_property("Acquisition_1G.doppler_step", "500");
    config->set_property("Acquisition_1G.repeat_satellite", "false");
    // config->set_property("Acquisition_1G.pfa", "0.0");
}


TEST_F(GlonassL1CaPcpsAcquisitionTest, Instantiate)
{
    init();
    auto acquisition = gnss_make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 0);
}


TEST_F(GlonassL1CaPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 62314000;
    int nsamples = 62314;
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    top_block = gr::make_top_block("Acquisition test");
    init();
    auto acquisition = gnss_make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 0);

    auto msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

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

    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(GlonassL1CaPcpsAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 31874;
    double expected_doppler_hz = -9500;
    init();
    std::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = std::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 0);
    std::shared_ptr<FreqXlatingFirFilter> input_filter = std::make_shared<FreqXlatingFirFilter>(config.get(), "InputFilter", 1, 1);
    auto msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(0.005);
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(10000);
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(500);
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    acquisition->set_local_code();
    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->init();

    ASSERT_NO_THROW({
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, input_filter->get_left_block(), 0);
        top_block->connect(input_filter->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        begin = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - begin;
    }) << "Failure running the top_block.";

    uint64_t nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples) * 511.0 / 62316.0;
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}

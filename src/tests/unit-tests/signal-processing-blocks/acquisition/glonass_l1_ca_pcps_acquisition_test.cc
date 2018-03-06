/*!
 * \file glonass_l1_ca_pcps_acquisition_test.cc
 * \brief  Tests a PCPS acquisition block for Glonass L1 C/A signals
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
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

#include <chrono>
#include <cstdlib>
#include <boost/chrono.hpp>
#include <boost/make_shared.hpp>
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
#include "glonass_l1_ca_pcps_acquisition.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GlonassL1CaPcpsAcquisitionTest_msg_rx;

typedef boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr;

GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

class GlonassL1CaPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GlonassL1CaPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~GlonassL1CaPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make()
{
    return GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr(new GlonassL1CaPcpsAcquisitionTest_msg_rx());
}


void GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            std::cout << "msg_handler_telemetry Bad any cast!" << std::endl;
            rx_message = 0;
        }
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::GlonassL1CaPcpsAcquisitionTest_msg_rx() : gr::block("GlonassL1CaPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::~GlonassL1CaPcpsAcquisitionTest_msg_rx()
{
}


// ###########################################################

class GlonassL1CaPcpsAcquisitionTest : public ::testing::Test
{
protected:
    GlonassL1CaPcpsAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GlonassL1CaPcpsAcquisitionTest()
    {
    }

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
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
    config->set_property("Acquisition_1G.item_type", "gr_complex");
    config->set_property("Acquisition_1G.if", "9540000");
    config->set_property("Acquisition_1G.coherent_integration_time_ms", "1");
    config->set_property("Acquisition_1G.dump", "true");
    config->set_property("Acquisition_1G.dump_filename", "./acquisition");
    config->set_property("Acquisition_1G.implementation", "Glonass_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1G.threshold", "0.001");
    config->set_property("Acquisition_1G.doppler_max", "5000");
    config->set_property("Acquisition_1G.doppler_step", "500");
    config->set_property("Acquisition_1G.repeat_satellite", "false");
    //config->set_property("Acquisition_1G.pfa", "0.0");
}


TEST_F(GlonassL1CaPcpsAcquisitionTest, Instantiate)
{
    init();
    boost::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = boost::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 1);
}


TEST_F(GlonassL1CaPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 62314000;
    int nsamples = 62314;
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    top_block = gr::make_top_block("Acquisition test");
    init();
    boost::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = boost::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 1);
    boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
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


TEST_F(GlonassL1CaPcpsAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 31874;
    double expected_doppler_hz = -8000;
    init();
    std::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = std::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 1);

    boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

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
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        begin = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - begin;
    }) << "Failure running the top_block.";

    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples) * 511.0 / 62316.0;
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}

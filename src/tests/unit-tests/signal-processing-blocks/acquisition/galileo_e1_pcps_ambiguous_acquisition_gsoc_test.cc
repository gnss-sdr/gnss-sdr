/*!
 * \file galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc
 * \brief  This class implements an acquisition test for
 *  GalileoE1PcpsAmbiguousAcquisition class based on GSoC 2012 experiments.
 *
 * This test is a part of an experiment performed by Luis Esteve in the
 * framework of the Google Summer of Code (GSoC) 2012, with the collaboration
 * of Javier Arribas and Carles Fernández, related to the extension of GNSS-SDR
 *  to Galileo. The objective is perform a positive acquisition of in-orbit
 *  Galileo signals in the E1 band.
 *
 *  Report:
 *  https://docs.google.com/document/d/1SZ3m1K7Qf9GsZQGEF7VSOEewBDCjbylCClw9rSXwG7Y/edit?pli=1
 *
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


#include <chrono>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_signal.h"
#include "gnss_synchro.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx;

typedef boost::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx> GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_sptr;

GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_sptr GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_make(concurrent_queue<int>& queue);


class GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx : public gr::block
{
private:
    friend GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_sptr GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx();  //!< Default destructor
};


GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_sptr GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_make(concurrent_queue<int>& queue)
{
    return GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_sptr(new GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx(queue));
}


void GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx::GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx(concurrent_queue<int>& queue) : gr::block("GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx::~GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx()
{
}


// ###########################################################


class GalileoE1PcpsAmbiguousAcquisitionGSoCTest : public ::testing::Test
{
protected:
    GalileoE1PcpsAmbiguousAcquisitionGSoCTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
        gnss_synchro = Gnss_Synchro();
    }

    ~GalileoE1PcpsAmbiguousAcquisitionGSoCTest()
    {
    }

    void init();
    void start_queue();
    void wait_message();
    void stop_queue();

    concurrent_queue<int> channel_internal_queue;
    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    bool stop;
    int message;
    boost::thread ch_thread;
};


void GalileoE1PcpsAmbiguousAcquisitionGSoCTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 11;

    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition_1B.item_type", "gr_complex");
    config->set_property("Acquisition_1B.coherent_integration_time_ms", "4");
    config->set_property("Acquisition_1B.dump", "false");
    config->set_property("Acquisition_1B.threshold", "0.1");
    config->set_property("Acquisition_1B.doppler_max", "10000");
    config->set_property("Acquisition_1B.doppler_step", "125");
    config->set_property("Acquisition_1B.repeat_satellite", "false");
    config->set_property("Acquisition_1B.cboc", "true");
}


void GalileoE1PcpsAmbiguousAcquisitionGSoCTest::start_queue()
{
    ch_thread = boost::thread(&GalileoE1PcpsAmbiguousAcquisitionGSoCTest::wait_message, this);
}


void GalileoE1PcpsAmbiguousAcquisitionGSoCTest::wait_message()
{
    while (!stop)
        {
            try
                {
                    channel_internal_queue.wait_and_pop(message);
                    stop_queue();
                }
            catch (const boost::exception& e)
                {
                    DLOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                }
        }
}


void GalileoE1PcpsAmbiguousAcquisitionGSoCTest::stop_queue()
{
    stop = true;
}


TEST_F(GalileoE1PcpsAmbiguousAcquisitionGSoCTest, Instantiate)
{
    init();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1);
    std::shared_ptr<AcquisitionInterface> acquisition = std::dynamic_pointer_cast<AcquisitionInterface>(acq_);
    EXPECT_STREQ("Galileo_E1_PCPS_Ambiguous_Acquisition", acquisition->implementation().c_str());
}


TEST_F(GalileoE1PcpsAmbiguousAcquisitionGSoCTest, ConnectAndRun)
{
    int fs_in = 4000000;
    int nsamples = 4 * fs_in;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    init();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition_1B", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1);
    std::shared_ptr<AcquisitionInterface> acquisition = std::dynamic_pointer_cast<AcquisitionInterface>(acq_);
    boost::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx> msg_rx = GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(GalileoE1PcpsAmbiguousAcquisitionGSoCTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    init();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(config, "Acquisition", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1);
    std::shared_ptr<GalileoE1PcpsAmbiguousAcquisition> acquisition = std::dynamic_pointer_cast<GalileoE1PcpsAmbiguousAcquisition>(acq_);
    boost::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx> msg_rx = GalileoE1PcpsAmbiguousAcquisitionGSoCTest_msg_rx_make(channel_internal_queue);

    ASSERT_NO_THROW({
        acquisition->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(config->property("Acquisition_1B.threshold", 0.00001));
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(config->property("Acquisition_1B.doppler_max", 10000));
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(config->property("Acquisition_1B.doppler_step", 250));
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    ASSERT_NO_THROW({
        std::string path = std::string(TEST_PATH);
        //std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
        std::string file = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    ASSERT_NO_THROW({
        start_queue();
        acquisition->set_local_code();
        acquisition->init();
        acquisition->reset();
        acquisition->set_state(1);
    }) << "Failure starting acquisition";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    stop_queue();

    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;

    EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 0=ACQ STOP.";

    ch_thread.join();
}

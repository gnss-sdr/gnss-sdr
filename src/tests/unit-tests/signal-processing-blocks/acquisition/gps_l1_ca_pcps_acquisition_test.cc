/*!
 * \file gps_l1_ca_pcps_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsAcquisition class based on some input parameters.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gnuplot_i.h"
#include "GPS_L1_CA.h"
#include "test_flags.h"
#include "acquisition_dump_reader.h"
#include "gps_l1_ca_pcps_acquisition.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CaPcpsAcquisitionTest_msg_rx;

typedef boost::shared_ptr<GpsL1CaPcpsAcquisitionTest_msg_rx> GpsL1CaPcpsAcquisitionTest_msg_rx_sptr;

GpsL1CaPcpsAcquisitionTest_msg_rx_sptr GpsL1CaPcpsAcquisitionTest_msg_rx_make();

class GpsL1CaPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GpsL1CaPcpsAcquisitionTest_msg_rx_sptr GpsL1CaPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CaPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CaPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


GpsL1CaPcpsAcquisitionTest_msg_rx_sptr GpsL1CaPcpsAcquisitionTest_msg_rx_make()
{
    return GpsL1CaPcpsAcquisitionTest_msg_rx_sptr(new GpsL1CaPcpsAcquisitionTest_msg_rx());
}


void GpsL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


GpsL1CaPcpsAcquisitionTest_msg_rx::GpsL1CaPcpsAcquisitionTest_msg_rx() : gr::block("GpsL1CaPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CaPcpsAcquisitionTest_msg_rx::~GpsL1CaPcpsAcquisitionTest_msg_rx()
{
}


// ###########################################################

class GpsL1CaPcpsAcquisitionTest : public ::testing::Test
{
protected:
    GpsL1CaPcpsAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
        doppler_max = 5000;
        doppler_step = 100;
    }

    ~GpsL1CaPcpsAcquisitionTest()
    {
    }

    void init();
    void plot_grid();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    unsigned int doppler_max;
    unsigned int doppler_step;
};


void GpsL1CaPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.coherent_integration_time_ms", "1");
    if (FLAGS_plot_acq_grid == true)
        {
            config->set_property("Acquisition_1C.dump", "true");
        }
    else
        {
            config->set_property("Acquisition_1C.dump", "false");
        }
    config->set_property("Acquisition_1C.dump_filename", "./tmp-acq-gps1/acquisition");
    config->set_property("Acquisition_1C.dump_channel", "1");
    config->set_property("Acquisition_1C.threshold", "0.00001");
    config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_1C.repeat_satellite", "false");
    //config->set_property("Acquisition_1C.pfa", "0.0");
}


void GpsL1CaPcpsAcquisitionTest::plot_grid()
{
    //load the measured values
    std::string basename = "./tmp-acq-gps1/acquisition_G_1C";
    unsigned int sat = static_cast<unsigned int>(gnss_synchro.PRN);

    unsigned int samples_per_code = static_cast<unsigned int>(round(4000000 / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));  // !!
    acquisition_dump_reader acq_dump(basename, sat, doppler_max, doppler_step, samples_per_code, 1);

    if (!acq_dump.read_binary_acq()) std::cout << "Error reading files" << std::endl;

    std::vector<int> *doppler = &acq_dump.doppler;
    std::vector<unsigned int> *samples = &acq_dump.samples;
    std::vector<std::vector<float> > *mag = &acq_dump.mag;

    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
    if (gnuplot_executable.empty())
        {
            std::cout << "WARNING: Although the flag plot_acq_grid has been set to TRUE," << std::endl;
            std::cout << "gnuplot has not been found in your system." << std::endl;
            std::cout << "Test results will not be plotted." << std::endl;
        }
    else
        {
            std::cout << "Plotting the acquisition grid. This can take a while..." << std::endl;
            try
                {
                    boost::filesystem::path p(gnuplot_executable);
                    boost::filesystem::path dir = p.parent_path();
                    std::string gnuplot_path = dir.native();
                    Gnuplot::set_GNUPlotPath(gnuplot_path);

                    Gnuplot g1("lines");
                    if (FLAGS_show_plots)
                        {
                            g1.showonscreen();  // window output
                        }
                    else
                        {
                            g1.disablescreen();
                        }
                    g1.set_title("GPS L1 C/A signal acquisition for satellite PRN #" + std::to_string(gnss_synchro.PRN));
                    g1.set_xlabel("Doppler [Hz]");
                    g1.set_ylabel("Sample");
                    //g1.cmd("set view 60, 105, 1, 1");
                    g1.plot_grid3d(*doppler, *samples, *mag);

                    g1.savetops("GPS_L1_acq_grid");
                    g1.savetopdf("GPS_L1_acq_grid");
                }
            catch (const GnuplotException &ge)
                {
                    std::cout << ge.what() << std::endl;
                }
        }
    std::string data_str = "./tmp-acq-gps1";
    if (boost::filesystem::exists(data_str))
        {
            boost::filesystem::remove_all(data_str);
        }
}


TEST_F(GpsL1CaPcpsAcquisitionTest, Instantiate)
{
    init();
    boost::shared_ptr<GpsL1CaPcpsAcquisition> acquisition = boost::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
}


TEST_F(GpsL1CaPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 4000000;
    int nsamples = 4000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    top_block = gr::make_top_block("Acquisition test");
    init();
    boost::shared_ptr<GpsL1CaPcpsAcquisition> acquisition = boost::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
    boost::shared_ptr<GpsL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsAcquisitionTest_msg_rx_make();

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


TEST_F(GpsL1CaPcpsAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0.0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 524;
    double expected_doppler_hz = 1680;

    init();

    if (FLAGS_plot_acq_grid == true)
        {
            std::string data_str = "./tmp-acq-gps1";
            if (boost::filesystem::exists(data_str))
                {
                    boost::filesystem::remove_all(data_str);
                }
            boost::filesystem::create_directory(data_str);
        }

    std::shared_ptr<GpsL1CaPcpsAcquisition> acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
    boost::shared_ptr<GpsL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GpsL1CaPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(0.001);
    }) << "Failure setting threshold.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(doppler_max);
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(doppler_step);
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    ASSERT_NO_THROW({
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
        const char *file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    acquisition->set_local_code();
    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->init();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    uint64_t nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";

    if (FLAGS_plot_acq_grid == true)
        {
            plot_grid();
        }
}

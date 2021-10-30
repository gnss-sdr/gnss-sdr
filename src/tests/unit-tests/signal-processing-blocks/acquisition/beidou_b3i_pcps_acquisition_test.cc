/*!
 * \file beidou_b3i_pcps_acquisition_test.cc
 * \brief  This class implements an acquisition test for
 * BeidouB3iPcpsAcquisition class based on some input parameters.
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
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


#include "Beidou_B3I.h"
#include "acquisition_dump_reader.h"
#include "beidou_b3i_pcps_acquisition.h"
#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gnuplot_i.h"
#include "in_memory_configuration.h"
#include "test_flags.h"
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
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
class BeidouB3iPcpsAcquisitionTest_msg_rx;

using BeidouB3iPcpsAcquisitionTest_msg_rx_sptr = std::shared_ptr<BeidouB3iPcpsAcquisitionTest_msg_rx>;

BeidouB3iPcpsAcquisitionTest_msg_rx_sptr BeidouB3iPcpsAcquisitionTest_msg_rx_make();

class BeidouB3iPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend BeidouB3iPcpsAcquisitionTest_msg_rx_sptr BeidouB3iPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    BeidouB3iPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~BeidouB3iPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


BeidouB3iPcpsAcquisitionTest_msg_rx_sptr BeidouB3iPcpsAcquisitionTest_msg_rx_make()
{
    return BeidouB3iPcpsAcquisitionTest_msg_rx_sptr(new BeidouB3iPcpsAcquisitionTest_msg_rx());
}


void BeidouB3iPcpsAcquisitionTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (const wht::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


BeidouB3iPcpsAcquisitionTest_msg_rx::BeidouB3iPcpsAcquisitionTest_msg_rx() : gr::block("BeidouB3iPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto &&PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&BeidouB3iPcpsAcquisitionTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&BeidouB3iPcpsAcquisitionTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


BeidouB3iPcpsAcquisitionTest_msg_rx::~BeidouB3iPcpsAcquisitionTest_msg_rx() = default;


// ###########################################################

class BeidouB3iPcpsAcquisitionTest : public ::testing::Test
{
protected:
    BeidouB3iPcpsAcquisitionTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
        doppler_max = 5000;
        doppler_step = 100;
    }

    ~BeidouB3iPcpsAcquisitionTest() = default;

    void init();
    void plot_grid();

    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro{};
    size_t item_size;
    unsigned int doppler_max;
    unsigned int doppler_step;
};


void BeidouB3iPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';
    std::string signal = "B3";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "50000000");
    config->set_property("Acquisition_B3.implementation", "BEIDOU_B3I_PCPS_Acquisition");
    config->set_property("Acquisition_B3.item_type", "gr_complex");
    config->set_property("Acquisition_B3.coherent_integration_time_ms", "1");
    if (FLAGS_plot_acq_grid == true)
        {
            config->set_property("Acquisition_B3.dump", "true");
        }
    else
        {
            config->set_property("Acquisition_B3.dump", "false");
        }
    config->set_property("Acquisition_B3.dump_filename", "./tmp-acq-bds-b3i/acquisition");
    config->set_property("Acquisition_B3.dump_channel", "1");
    config->set_property("Acquisition_B3.threshold", "0.00001");
    config->set_property("Acquisition_B3.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_B3.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_B3.repeat_satellite", "false");
}


void BeidouB3iPcpsAcquisitionTest::plot_grid()
{
    // load the measured values
    std::string basename = "./tmp-acq-bds-b3i/acquisition_C_B3";
    auto sat = static_cast<unsigned int>(gnss_synchro.PRN);

    auto samples_per_code = static_cast<unsigned int>(round(50000000 / (BEIDOU_B3I_CODE_RATE_CPS / BEIDOU_B3I_CODE_LENGTH_CHIPS)));  // !!
    Acquisition_Dump_Reader acq_dump(basename, sat, doppler_max, doppler_step, samples_per_code, 1);

    if (!acq_dump.read_binary_acq())
        {
            std::cout << "Error reading files\n";
        }

    std::vector<int> *doppler = &acq_dump.doppler;
    std::vector<unsigned int> *samples = &acq_dump.samples;
    std::vector<std::vector<float>> *mag = &acq_dump.mag;

    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
    if (gnuplot_executable.empty())
        {
            std::cout << "WARNING: Although the flag plot_acq_grid has been set to TRUE,\n";
            std::cout << "gnuplot has not been found in your system.\n";
            std::cout << "Test results will not be plotted.\n";
        }
    else
        {
            std::cout << "Plotting the acquisition grid. This can take a while...\n";
            try
                {
                    fs::path p(gnuplot_executable);
                    fs::path dir = p.parent_path();
                    const std::string &gnuplot_path = dir.native();
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
                    g1.set_title("BeiDou B3I signal acquisition for satellite PRN #" + std::to_string(gnss_synchro.PRN));
                    g1.set_xlabel("Doppler [Hz]");
                    g1.set_ylabel("Sample");
                    // g1.cmd("set view 60, 105, 1, 1");
                    g1.plot_grid3d(*doppler, *samples, *mag);

                    g1.savetops("BDS_B3I_acq_grid");
                    g1.savetopdf("BDS_B3I_acq_grid");
                }
            catch (const GnuplotException &ge)
                {
                    std::cout << ge.what() << '\n';
                }
        }
    std::string data_str = "./tmp-acq-bds-b3i";
    if (fs::exists(data_str))
        {
            fs::remove_all(data_str);
        }
}


TEST_F(BeidouB3iPcpsAcquisitionTest, Instantiate)
{
    init();
    std::shared_ptr<BeidouB3iPcpsAcquisition> acquisition = boost::make_shared<BeidouB3iPcpsAcquisition>(config.get(), "Acquisition_B3", 1, 0);
}


TEST_F(BeidouB3iPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 50000000;
    int nsamples = 50000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    top_block = gr::make_top_block("Acquisition test");
    init();
    std::shared_ptr<BeidouB3iPcpsAcquisition> acquisition = boost::make_shared<BeidouB3iPcpsAcquisition>(config.get(), "Acquisition_B3", 1, 0);
    std::shared_ptr<BeidouB3iPcpsAcquisitionTest_msg_rx> msg_rx = BeidouB3iPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
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

    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(BeidouB3iPcpsAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0.0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 3380;
    double expected_doppler_hz = 700;

    init();

    if (FLAGS_plot_acq_grid == true)
        {
            std::string data_str = "./tmp-acq-bds-b3i";
            if (fs::exists(data_str))
                {
                    fs::remove_all(data_str);
                }
            fs::create_directory(data_str);
        }

    std::shared_ptr<BeidouB3iPcpsAcquisition> acquisition = std::make_shared<BeidouB3iPcpsAcquisition>(config.get(), "Acquisition_B3", 1, 0);
    std::shared_ptr<BeidouB3iPcpsAcquisitionTest_msg_rx> msg_rx = BeidouB3iPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_threshold(0.0002);
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
        std::string file = path + "signal_samples/BdsB3IStr01_fs50e6_if0_4ms.dat";
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
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    auto delay_error_chips = static_cast<float>(delay_error_samples * BEIDOU_B3I_CODE_LENGTH_CHIPS / 50000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";

    if (FLAGS_plot_acq_grid == true)
        {
            plot_grid();
        }
}

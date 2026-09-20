/*!
 * \file gps_l1_ca_pcps_acquisition_cuda_test.cc
 * \brief End-to-end test of GPS_L1_CA_PCPS_Acquisition with use_cuda=true on a
 *        real capture, compared against the CPU path on the same data.
 * \author Phil Vu, 2026. dr.phil.vu(at)orboticsystems.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"
#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "in_memory_configuration.h"
#include "test_flags.h"
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif
#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif


// ######## GNURADIO BLOCK MESSAGE RECEIVER #########
class GpsL1CaPcpsAcquisitionCudaTest_msg_rx;

using GpsL1CaPcpsAcquisitionCudaTest_msg_rx_sptr = gnss_shared_ptr<GpsL1CaPcpsAcquisitionCudaTest_msg_rx>;

GpsL1CaPcpsAcquisitionCudaTest_msg_rx_sptr GpsL1CaPcpsAcquisitionCudaTest_msg_rx_make();

class GpsL1CaPcpsAcquisitionCudaTest_msg_rx : public gr::block
{
private:
    friend GpsL1CaPcpsAcquisitionCudaTest_msg_rx_sptr GpsL1CaPcpsAcquisitionCudaTest_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t &msg);
    GpsL1CaPcpsAcquisitionCudaTest_msg_rx();

public:
    int rx_message{0};
};


GpsL1CaPcpsAcquisitionCudaTest_msg_rx_sptr GpsL1CaPcpsAcquisitionCudaTest_msg_rx_make()
{
    return GpsL1CaPcpsAcquisitionCudaTest_msg_rx_sptr(new GpsL1CaPcpsAcquisitionCudaTest_msg_rx());
}


void GpsL1CaPcpsAcquisitionCudaTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t &msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (const wht::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


GpsL1CaPcpsAcquisitionCudaTest_msg_rx::GpsL1CaPcpsAcquisitionCudaTest_msg_rx()
    : gr::block("GpsL1CaPcpsAcquisitionCudaTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto &&PH1) { msg_handler_channel_events(std::forward<decltype(PH1)>(PH1)); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&GpsL1CaPcpsAcquisitionCudaTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&GpsL1CaPcpsAcquisitionCudaTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
}


// ###########################################################

class GpsL1CaPcpsAcquisitionCudaTest : public ::testing::Test
{
protected:
    struct Outcome
    {
        int message{0};
        double doppler_hz{0.0};
        double delay_samples{0.0};
        double elapsed_us{0.0};
    };

    std::shared_ptr<InMemoryConfiguration> make_config(bool use_cuda, bool two_steps) const;
    Outcome run_once(bool use_cuda, bool two_steps = false);

    unsigned int doppler_max{5000};
    unsigned int doppler_step{100};
};


std::shared_ptr<InMemoryConfiguration> GpsL1CaPcpsAcquisitionCudaTest::make_config(bool use_cuda, bool two_steps) const
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.coherent_integration_time_ms", "1");
    config->set_property("Acquisition_1C.dump", "false");
    config->set_property("Acquisition_1C.threshold", "0.001");
    config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_1C.repeat_satellite", "false");
    config->set_property("Acquisition_1C.use_cuda", use_cuda ? "true" : "false");
    if (two_steps)
        {
            config->set_property("Acquisition_1C.make_two_steps", "true");
            config->set_property("Acquisition_1C.second_nbins", "5");
            config->set_property("Acquisition_1C.second_doppler_step", "20");
        }
    return config;
}


GpsL1CaPcpsAcquisitionCudaTest::Outcome GpsL1CaPcpsAcquisitionCudaTest::run_once(bool use_cuda, bool two_steps)
{
    Outcome out;
    auto config = make_config(use_cuda, two_steps);
    auto top_block = gr::make_top_block("Acquisition CUDA test");

    Gnss_Synchro gnss_synchro = Gnss_Synchro();
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    auto acquisition = gnss_make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
    auto msg_rx = GpsL1CaPcpsAcquisitionCudaTest_msg_rx_make();

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->connect(top_block);

    std::string path = std::string(TEST_PATH);
    // The two-step search needs more than the 2 ms of the single-step capture
    std::string file = path + (two_steps ? "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat" : "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat");
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file.c_str(), false);
    top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->set_local_code();
    acquisition->reset();

    const auto start = std::chrono::steady_clock::now();
    top_block->run();
    const auto end = std::chrono::steady_clock::now();

    out.message = msg_rx->rx_message;
    out.doppler_hz = gnss_synchro.Acq_doppler_hz;
    out.delay_samples = gnss_synchro.Acq_delay_samples;
    out.elapsed_us = std::chrono::duration<double, std::micro>(end - start).count();
    return out;
}


TEST_F(GpsL1CaPcpsAcquisitionCudaTest /*unused*/, Instantiate /*unused*/)
{
    auto config = make_config(true, false);
    std::shared_ptr<GpsL1CaPcpsAcquisition> acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
}


TEST_F(GpsL1CaPcpsAcquisitionCudaTest /*unused*/, ValidationOfResults /*unused*/)
{
    const double expected_delay_samples = 524;
    const double expected_doppler_hz = 1680;

    Outcome gpu;
    ASSERT_NO_THROW({ gpu = run_once(true); }) << "Failure running the top_block with use_cuda=true.";
    std::cout << "CUDA acquisition: message=" << gpu.message << " Doppler=" << gpu.doppler_hz
              << " Hz, delay=" << gpu.delay_samples << " samples, flowgraph time " << gpu.elapsed_us << " us\n";

    ASSERT_EQ(1, gpu.message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    const double delay_error_samples = std::abs(expected_delay_samples - gpu.delay_samples);
    const auto delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    const double doppler_error_hz = std::abs(expected_doppler_hz - gpu.doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}


TEST_F(GpsL1CaPcpsAcquisitionCudaTest /*unused*/, SameEstimateAsCpu /*unused*/)
{
    Outcome cpu;
    Outcome gpu;
    ASSERT_NO_THROW({ cpu = run_once(false); }) << "Failure running the top_block with use_cuda=false.";
    ASSERT_NO_THROW({ gpu = run_once(true); }) << "Failure running the top_block with use_cuda=true.";

    std::cout << "CPU:  Doppler=" << cpu.doppler_hz << " Hz, delay=" << cpu.delay_samples << " samples (" << cpu.elapsed_us << " us)\n";
    std::cout << "CUDA: Doppler=" << gpu.doppler_hz << " Hz, delay=" << gpu.delay_samples << " samples (" << gpu.elapsed_us << " us)\n";

    ASSERT_EQ(cpu.message, gpu.message);
    ASSERT_EQ(1, cpu.message);
    // Same grid, same peak search: the estimates must coincide to the bin
    EXPECT_NEAR(cpu.doppler_hz, gpu.doppler_hz, static_cast<double>(doppler_step) / 2.0);
    EXPECT_NEAR(cpu.delay_samples, gpu.delay_samples, 1.0);
}


TEST_F(GpsL1CaPcpsAcquisitionCudaTest /*unused*/, SameEstimateAsCpuMakeTwoStep /*unused*/)
{
    // Exercises the fine-Doppler (STEP2_GRID) path of the engine through the real block
    Outcome cpu;
    Outcome gpu;
    ASSERT_NO_THROW({ cpu = run_once(false, true); }) << "Failure running the top_block with use_cuda=false.";
    ASSERT_NO_THROW({ gpu = run_once(true, true); }) << "Failure running the top_block with use_cuda=true.";

    std::cout << "CPU  (two steps): Doppler=" << cpu.doppler_hz << " Hz, delay=" << cpu.delay_samples << " samples (" << cpu.elapsed_us << " us)
";
    std::cout << "CUDA (two steps): Doppler=" << gpu.doppler_hz << " Hz, delay=" << gpu.delay_samples << " samples (" << gpu.elapsed_us << " us)
";

    ASSERT_EQ(1, cpu.message) << "CPU acquisition failure. Expected message: 1=ACQ SUCCESS.";
    ASSERT_EQ(1, gpu.message) << "CUDA acquisition failure. Expected message: 1=ACQ SUCCESS.";
    EXPECT_NEAR(cpu.doppler_hz, gpu.doppler_hz, 20.0);  // second_doppler_step
    EXPECT_NEAR(cpu.delay_samples, gpu.delay_samples, 1.0);
}

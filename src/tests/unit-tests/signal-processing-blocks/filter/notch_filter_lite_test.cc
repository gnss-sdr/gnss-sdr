/*!
 * \file notch_filter_lite_test.cc
 * \brief Implements Unit Test for the NotchFilterLite class.
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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

#include <gflags/gflags.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/top_block.h>
#include <chrono>
#include <complex>
#include <cstdint>
#include <thread>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include "concurrent_queue.h"
#include "file_signal_source.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_sdr_valve.h"
#include "in_memory_configuration.h"
#include "notch_filter_lite.h"
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>


DEFINE_int32(notch_filter_lite_test_nsamples, 1000000, "Number of samples to filter in the tests (max: 2147483647)");

class NotchFilterLiteTest : public ::testing::Test
{
protected:
    NotchFilterLiteTest() : item_size(sizeof(gr_complex)),
                            nsamples(FLAGS_notch_filter_lite_test_nsamples)
    {
        queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
        config = std::make_shared<InMemoryConfiguration>();
    }

    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();
    void init();
    void configure_gr_complex_gr_complex();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    pmt::pmt_t message;
    std::thread ch_thread;
    size_t item_size;
    int nsamples;
    bool stop{false};
};


void NotchFilterLiteTest::start_queue()
{
    stop = false;
    ch_thread = std::thread(&NotchFilterLiteTest::wait_message, this);
}


void NotchFilterLiteTest::wait_message()
{
    while (!stop)
        {
            queue->wait_and_pop(message);
            process_message();
        }
}


void NotchFilterLiteTest::process_message()
{
    stop_queue();
    top_block->stop();
}


void NotchFilterLiteTest::stop_queue()
{
    stop = true;
}


void NotchFilterLiteTest::init()
{
    config->set_property("InputFilter.pfa", "0.01");
    config->set_property("InputFilter.p_c_factor", "0.9");
    config->set_property("InputFilter.length", "32");
    config->set_property("InputFilter.segments_est", "12500");
    config->set_property("InputFilter.segments_reset", "5000000");
}


void NotchFilterLiteTest::configure_gr_complex_gr_complex()
{
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
}


TEST_F(NotchFilterLiteTest, InstantiateGrComplexGrComplex)
{
    init();
    configure_gr_complex_gr_complex();
    auto filter = std::make_unique<NotchFilterLite>(config.get(), "InputFilter", 1, 1);
    int res = 0;
    if (filter)
        {
            res = 1;
        }
    ASSERT_EQ(1, res);
}


TEST_F(NotchFilterLiteTest, ConnectAndRun)
{
    int fs_in = 4000000;
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Notch filter lite test");
    init();
    configure_gr_complex_gr_complex();
    auto filter = std::make_shared<NotchFilterLite>(config.get(), "InputFilter", 1, 1);
    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);
        auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
        auto null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    start_queue();
    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    ch_thread.join();
    std::cout << "Filtered " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(NotchFilterLiteTest, ConnectAndRunGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Notch filter lite test");
    init();
    configure_gr_complex_gr_complex();
    auto filter = std::make_shared<NotchFilterLite>(config.get(), "InputFilter", 1, 1);
    auto config2 = std::make_shared<InMemoryConfiguration>();

    config2->set_property("Test_Source.samples", std::to_string(nsamples));
    config2->set_property("Test_Source.sampling_frequency", "4000000");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config2->set_property("Test_Source.filename", filename);
    config2->set_property("Test_Source.item_type", "gr_complex");
    config2->set_property("Test_Source.repeat", "true");

    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);

        auto source = std::make_shared<FileSignalSource>(config2.get(), "Test_Source", 0, 1, queue.get());
        source->connect(top_block);

        auto null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source->get_right_block(), 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    start_queue();
    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    ch_thread.join();
    std::cout << "Filtered " << nsamples << " gr_complex samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

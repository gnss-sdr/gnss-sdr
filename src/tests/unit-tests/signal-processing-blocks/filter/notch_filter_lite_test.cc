/*!
 * \file notch_filter_lite_test.cc
 * \brief Implements Unit Test for the NotchFilterLite class.
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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
#include <complex>
#include <cstdint>
#include <gflags/gflags.h>
#include <gnuradio/top_block.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "notch_filter_lite.h"
#include "file_signal_source.h"


DEFINE_int32(notch_filter_lite_test_nsamples, 1000000, "Number of samples to filter in the tests (max: 2147483647)");

class NotchFilterLiteTest : public ::testing::Test
{
protected:
    NotchFilterLiteTest()
    {
        queue = gr::msg_queue::make(0);
        item_size = sizeof(gr_complex);
        config = std::make_shared<InMemoryConfiguration>();
        nsamples = FLAGS_notch_filter_lite_test_nsamples;
    }
    ~NotchFilterLiteTest()
    {
    }

    void init();
    void configure_gr_complex_gr_complex();
    boost::shared_ptr<gr::msg_queue> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    size_t item_size;
    int nsamples;
};


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
    std::unique_ptr<NotchFilterLite> filter(new NotchFilterLite(config.get(), "InputFilter", 1, 1));
    int res = 0;
    if (filter) res = 1;
    ASSERT_EQ(1, res);
}

TEST_F(NotchFilterLiteTest, ConnectAndRun)
{
    int fs_in = 4000000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Notch filter lite test");
    init();
    configure_gr_complex_gr_complex();
    std::shared_ptr<NotchFilterLite> filter = std::make_shared<NotchFilterLite>(config.get(), "InputFilter", 1, 1);
    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);
        boost::shared_ptr<gr::block> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000.0, 1.0, gr_complex(0.0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Filtered " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(NotchFilterLiteTest, ConnectAndRunGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Notch filter lite test");
    init();
    configure_gr_complex_gr_complex();
    std::shared_ptr<NotchFilterLite> filter = std::make_shared<NotchFilterLite>(config.get(), "InputFilter", 1, 1);
    std::shared_ptr<InMemoryConfiguration> config2 = std::make_shared<InMemoryConfiguration>();

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

        boost::shared_ptr<FileSignalSource> source(new FileSignalSource(config2.get(), "Test_Source", 1, 1, queue));
        source->connect(top_block);

        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source->get_right_block(), 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Filtered " << nsamples << " gr_complex samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

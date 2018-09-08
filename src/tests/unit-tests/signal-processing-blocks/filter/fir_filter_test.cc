/*!
 * \file fir_filter_test.cc
 * \brief Implements Unit Test for the FirFilter class.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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
#include "interleaved_byte_to_complex_byte.h"
#include "interleaved_short_to_complex_short.h"
#include "fir_filter.h"
#include "file_signal_source.h"


DEFINE_int32(filter_test_nsamples, 1000000, "Number of samples to filter in the tests (max: 2147483647)");

class FirFilterTest : public ::testing::Test
{
protected:
    FirFilterTest()
    {
        queue = gr::msg_queue::make(0);
        item_size = sizeof(gr_complex);
        config = std::make_shared<InMemoryConfiguration>();
    }
    ~FirFilterTest()
    {
    }

    void init();
    void configure_cbyte_cbyte();
    void configure_cbyte_gr_complex();
    void configure_gr_complex_gr_complex();
    void configure_cshort_cshort();
    boost::shared_ptr<gr::msg_queue> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    size_t item_size;
    int nsamples = FLAGS_filter_test_nsamples;
};


void FirFilterTest::init()
{
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", "5");
    config->set_property("InputFilter.number_of_bands", "2");

    config->set_property("InputFilter.band1_begin", "0.0");
    config->set_property("InputFilter.band1_end", "0.45");
    config->set_property("InputFilter.band2_begin", "0.55");
    config->set_property("InputFilter.band2_end", "1.0");

    config->set_property("InputFilter.ampl1_begin", "1.0");
    config->set_property("InputFilter.ampl1_end", "1.0");
    config->set_property("InputFilter.ampl2_begin", "0.0");
    config->set_property("InputFilter.ampl2_end", "0.0");

    config->set_property("InputFilter.band1_error", "1.0");
    config->set_property("InputFilter.band2_error", "1.0");

    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", "16");
    //config->set_property("InputFilter.dump", "true");
}


void FirFilterTest::configure_cbyte_cbyte()
{
    config->set_property("InputFilter.input_item_type", "cbyte");
    config->set_property("InputFilter.output_item_type", "cbyte");
}


void FirFilterTest::configure_gr_complex_gr_complex()
{
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
}


void FirFilterTest::configure_cshort_cshort()
{
    config->set_property("InputFilter.input_item_type", "cshort");
    config->set_property("InputFilter.output_item_type", "cshort");
}


void FirFilterTest::configure_cbyte_gr_complex()
{
    config->set_property("InputFilter.input_item_type", "cbyte");
    config->set_property("InputFilter.output_item_type", "gr_complex");
}


TEST_F(FirFilterTest, InstantiateGrComplexGrComplex)
{
    init();
    configure_gr_complex_gr_complex();
    std::unique_ptr<FirFilter> filter(new FirFilter(config.get(), "InputFilter", 1, 1));
    int res = 0;
    if (filter) res = 1;
    ASSERT_EQ(1, res);
}

TEST_F(FirFilterTest, InstantiateCshortCshort)
{
    init();
    configure_cshort_cshort();
    std::unique_ptr<FirFilter> filter(new FirFilter(config.get(), "InputFilter", 1, 1));
    int res = 0;
    if (filter) res = 1;
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, InstantiateCbyteCbyte)
{
    init();
    configure_cbyte_cbyte();
    std::unique_ptr<FirFilter> filter(new FirFilter(config.get(), "InputFilter", 1, 1));
    int res = 0;
    if (filter) res = 1;
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, InstantiateCbyteGrComplex)
{
    init();
    configure_cbyte_gr_complex();
    std::unique_ptr<FirFilter> filter(new FirFilter(config.get(), "InputFilter", 1, 1));
    int res = 0;
    if (filter) res = 1;
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, ConnectAndRun)
{
    int fs_in = 4000000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_gr_complex_gr_complex();
    std::shared_ptr<FirFilter> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);
        boost::shared_ptr<gr::block> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
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


TEST_F(FirFilterTest, ConnectAndRunGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_gr_complex_gr_complex();
    std::shared_ptr<FirFilter> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
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

        boost::shared_ptr<FileSignalSource> source(new FileSignalSource(config2.get(), "Test_Source", 0, 1, queue));
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

TEST_F(FirFilterTest, ConnectAndRunCshorts)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cshort_cshort();
    std::shared_ptr<FirFilter> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    std::shared_ptr<InMemoryConfiguration> config2 = std::make_shared<InMemoryConfiguration>();

    config2->set_property("Test_Source.samples", std::to_string(nsamples));
    config2->set_property("Test_Source.sampling_frequency", "4000000");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config2->set_property("Test_Source.filename", filename);
    config2->set_property("Test_Source.item_type", "ishort");
    config2->set_property("Test_Source.repeat", "true");

    item_size = sizeof(std::complex<int16_t>);
    ASSERT_NO_THROW({
        filter->connect(top_block);

        boost::shared_ptr<FileSignalSource> source(new FileSignalSource(config2.get(), "Test_Source", 0, 1, queue));
        source->connect(top_block);

        interleaved_short_to_complex_short_sptr ishort_to_cshort_ = make_interleaved_short_to_complex_short();
        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source->get_right_block(), 0, ishort_to_cshort_, 0);
        top_block->connect(ishort_to_cshort_, 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Filtered " << nsamples << " std::complex<int16_t> samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(FirFilterTest, ConnectAndRunCbytes)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cbyte_cbyte();
    std::shared_ptr<FirFilter> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    std::shared_ptr<InMemoryConfiguration> config2 = std::make_shared<InMemoryConfiguration>();

    config2->set_property("Test_Source.samples", std::to_string(nsamples));
    config2->set_property("Test_Source.sampling_frequency", "4000000");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config2->set_property("Test_Source.filename", filename);
    config2->set_property("Test_Source.item_type", "ibyte");
    config2->set_property("Test_Source.repeat", "true");

    item_size = sizeof(std::complex<int8_t>);
    ASSERT_NO_THROW({
        filter->connect(top_block);

        boost::shared_ptr<FileSignalSource> source(new FileSignalSource(config2.get(), "Test_Source", 0, 1, queue));
        source->connect(top_block);

        interleaved_byte_to_complex_byte_sptr ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();
        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source->get_right_block(), 0, ibyte_to_cbyte_, 0);
        top_block->connect(ibyte_to_cbyte_, 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block.";

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Filtered " << nsamples << " std::complex<int8_t> samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(FirFilterTest, ConnectAndRunCbyteGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cbyte_gr_complex();
    std::shared_ptr<FirFilter> filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    std::shared_ptr<InMemoryConfiguration> config2 = std::make_shared<InMemoryConfiguration>();

    config2->set_property("Test_Source.samples", std::to_string(nsamples));
    config2->set_property("Test_Source.sampling_frequency", "4000000");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    config2->set_property("Test_Source.filename", filename);
    config2->set_property("Test_Source.item_type", "ibyte");
    config2->set_property("Test_Source.repeat", "true");

    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);

        boost::shared_ptr<FileSignalSource> source(new FileSignalSource(config2.get(), "Test_Source", 0, 1, queue));
        source->connect(top_block);

        interleaved_byte_to_complex_byte_sptr ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();
        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source->get_right_block(), 0, ibyte_to_cbyte_, 0);
        top_block->connect(ibyte_to_cbyte_, 0, filter->get_left_block(), 0);
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

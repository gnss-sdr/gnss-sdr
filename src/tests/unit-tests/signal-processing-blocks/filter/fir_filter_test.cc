/*!
 * \file fir_filter_test.cc
 * \brief Implements Unit Test for the FirFilter class.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include "concurrent_queue.h"
#include "file_signal_source.h"
#include "fir_filter.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_sdr_valve.h"
#include "in_memory_configuration.h"
#include "interleaved_byte_to_complex_byte.h"
#include "interleaved_short_to_complex_short.h"
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>


DEFINE_int32(filter_test_nsamples, 1000000, "Number of samples to filter in the tests (max: 2147483647)");

class FirFilterTest : public ::testing::Test
{
protected:
    FirFilterTest() : item_size(sizeof(gr_complex)),
                      nsamples(FLAGS_filter_test_nsamples)
    {
        queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
        config = std::make_shared<InMemoryConfiguration>();
    }

    void init();
    void configure_cbyte_cbyte();
    void configure_cbyte_gr_complex();
    void configure_gr_complex_gr_complex();
    void configure_cshort_cshort();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<InMemoryConfiguration> config;
    size_t item_size;
    int nsamples;
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
    // config->set_property("InputFilter.dump", "true");
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
    auto filter = std::make_unique<FirFilter>(config.get(), "InputFilter", 1, 1);
    int res = 0;
    if (filter)
        {
            res = 1;
        }
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, InstantiateCshortCshort)
{
    init();
    configure_cshort_cshort();
    auto filter = std::make_unique<FirFilter>(config.get(), "InputFilter", 1, 1);
    int res = 0;
    if (filter)
        {
            res = 1;
        }
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, InstantiateCbyteCbyte)
{
    init();
    configure_cbyte_cbyte();
    auto filter = std::make_unique<FirFilter>(config.get(), "InputFilter", 1, 1);
    int res = 0;
    if (filter)
        {
            res = 1;
        }
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, InstantiateCbyteGrComplex)
{
    init();
    configure_cbyte_gr_complex();
    auto filter = std::make_unique<FirFilter>(config.get(), "InputFilter", 1, 1);
    int res = 0;
    if (filter)
        {
            res = 1;
        }
    ASSERT_EQ(1, res);
}


TEST_F(FirFilterTest, ConnectAndRun)
{
    int fs_in = 4000000;
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_gr_complex_gr_complex();
    auto filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    item_size = sizeof(gr_complex);
    ASSERT_NO_THROW({
        filter->connect(top_block);
        auto source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
        auto null_sink = gr::blocks::null_sink::make(item_size);

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
    std::cout << "Filtered " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(FirFilterTest, ConnectAndRunGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_gr_complex_gr_complex();
    auto filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
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

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";
    std::cout << "Filtered " << nsamples << " gr_complex samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(FirFilterTest, ConnectAndRunCshorts)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cshort_cshort();
    auto filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    auto config2 = std::make_shared<InMemoryConfiguration>();

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

        auto source = std::make_shared<FileSignalSource>(config2.get(), "Test_Source", 0, 1, queue.get());
        source->connect(top_block);

        interleaved_short_to_complex_short_sptr ishort_to_cshort_ = make_interleaved_short_to_complex_short();
        auto null_sink = gr::blocks::null_sink::make(item_size);

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
    std::cout << "Filtered " << nsamples << " std::complex<int16_t> samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(FirFilterTest, ConnectAndRunCbytes)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cbyte_cbyte();
    auto filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    auto config2 = std::make_shared<InMemoryConfiguration>();

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

        auto source = std::make_shared<FileSignalSource>(config2.get(), "Test_Source", 0, 1, queue.get());
        source->connect(top_block);

        interleaved_byte_to_complex_byte_sptr ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();
        auto null_sink = gr::blocks::null_sink::make(item_size);

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
    std::cout << "Filtered " << nsamples << " std::complex<int8_t> samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}


TEST_F(FirFilterTest, ConnectAndRunCbyteGrcomplex)
{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Fir filter test");

    init();
    configure_cbyte_gr_complex();
    auto filter = std::make_shared<FirFilter>(config.get(), "InputFilter", 1, 1);
    auto config2 = std::make_shared<InMemoryConfiguration>();

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

        auto source = std::make_shared<FileSignalSource>(config2.get(), "Test_Source", 0, 1, queue.get());
        source->connect(top_block);

        interleaved_byte_to_complex_byte_sptr ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();
        auto null_sink = gr::blocks::null_sink::make(item_size);

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
    std::cout << "Filtered " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

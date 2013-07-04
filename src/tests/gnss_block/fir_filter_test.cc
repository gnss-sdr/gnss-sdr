/*!
 * \file fir_filter_test.cc
 * \brief Implements Unit Test for the FirFilter class.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <sys/time.h>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "fir_filter.h"

DEFINE_string(filter_test_output_filename, "../src/tests/data/fir_filter_test_output.dat", "Dump filename");

class Fir_Filter_Test: public ::testing::Test
{
protected:
    Fir_Filter_Test()
    {
        queue = gr::msg_queue::make(0);
        top_block = gr::make_top_block("Fir filter test");
        config = new InMemoryConfiguration();
        item_size = sizeof(gr_complex);
    }
    ~Fir_Filter_Test()
    {
        delete config;
    }
    void init();
    boost::shared_ptr<gr::msg_queue> queue;
    gr::top_block_sptr top_block;
    InMemoryConfiguration* config;
    size_t item_size;
};

void Fir_Filter_Test::init()
{
    config->set_property("InputFilter.number_of_taps", "4");
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


TEST_F(Fir_Filter_Test, Instantiate)
{
    init();
    FirFilter *filter = new FirFilter(config, "InputFilter", 1, 1, queue);
    delete filter;
}


TEST_F(Fir_Filter_Test, ConnectAndRun)
{
    int fs_in = 8000000;
    int nsamples = 10000000;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;

    init();

    FirFilter *filter = new FirFilter(config, "InputFilter", 1, 1, queue);

    ASSERT_NO_THROW( {
        filter->connect(top_block);
        boost::shared_ptr<gr::block> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        boost::shared_ptr<gr::block> null_sink = gr::blocks::null_sink::make(item_size);

        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, filter->get_left_block(), 0);
        top_block->connect(filter->get_right_block(), 0, null_sink, 0);
    }) << "Failure connecting the top_block."<< std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1000000 + tv.tv_usec;
    }) << "Failure running he top_block."<< std::endl;
    std::cout <<  "Filtered " << nsamples << " samples in " << (end-begin) << " microseconds" << std::endl;

    delete filter;
}

/*!
 * \file gr_thread_generation.cc
 * \brief  This class implements a gr::thread::thread object generation test
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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
#include <iostream>
#include <string>
#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/analog/noise_source_c.h>
#include <gnuradio/analog/noise_type.h>
#include <gnuradio/msg_queue.h>
#include <gtest/gtest.h>
#include "gnss_sdr_valve.h"


// ######## GNURADIO VOID BLOCK #########
class GrBlockVoid;

typedef boost::shared_ptr<GrBlockVoid> GrBlockVoid_sptr;

GrBlockVoid_sptr GrBlockVoid_make(bool blocking, bool inlineproc);

class GrBlockVoid : public gr::block
{
private:
    friend GrBlockVoid_sptr GrBlockVoid_make(bool blocking, bool inlineproc);
    GrBlockVoid(bool blocking, bool inlineproc);
    void thread_core();
    unsigned int counter;
    bool d_blocking;
    bool d_inlineproc;
public:
    int general_work(int noutput_items, gr_vector_int &ninput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
    ~GrBlockVoid(); //!< Default destructor
};


GrBlockVoid_sptr GrBlockVoid_make(bool blocking, bool inlineproc)
{
    return GrBlockVoid_sptr(new GrBlockVoid(blocking, inlineproc));
}


GrBlockVoid::GrBlockVoid(bool blocking, bool inlineproc) :
    gr::block("GrBlockVoid", gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::make(0, 0, sizeof(gr_complex)))
{
    counter = 0;
    d_blocking = blocking;
    d_inlineproc = inlineproc;
}

int GrBlockVoid::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)), gr_vector_const_void_star &input_items __attribute__((unused)), gr_vector_void_star &output_items __attribute__((unused)))
{
	gr::thread::scoped_lock l(d_setlock);
	if (d_inlineproc)
	{
		counter++;
		consume_each(1);
	}
	else if (d_blocking)
	{
		consume_each(1);
		l.unlock();
		gr::thread::thread d_worker(&GrBlockVoid::thread_core, this);
		d_worker.join();
	}
	else
	{
		consume_each(1);
		l.unlock();
		gr::thread::thread d_worker(&GrBlockVoid::thread_core, this);

	}
	return 0;
}

void GrBlockVoid::thread_core()
{
	gr::thread::scoped_lock l(d_setlock);
	counter++;
}

GrBlockVoid::~GrBlockVoid()
{}


// ###########################################################

class GrBlockVoidTest: public ::testing::Test
{
protected:
    GrBlockVoidTest()
    {}

    ~GrBlockVoidTest()
    {}

    gr::top_block_sptr top_block;
};

TEST_F(GrBlockVoidTest, Instantiate)
{
    EXPECT_NO_THROW( {
    	GrBlockVoid_sptr block_ = GrBlockVoid_make(true, true);
    }) << "Failure instantiating the GrBlockVoid." << std::endl;
}


TEST_F(GrBlockVoidTest, ConnectAndRun)
{
    int nsamples = 5;
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    GrBlockVoid_sptr block_ = GrBlockVoid_make(true, true);
    top_block = gr::make_top_block("GrBlock Void test");

    EXPECT_NO_THROW( {
        boost::shared_ptr<gr::analog::noise_source_c> source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, block_, 0);
    }) << "Failure connecting the blocks of GrBlockVoid test." << std::endl;

    EXPECT_NO_THROW( {
        top_block->run(); // Start threads and wait
    }) << "Failure running the top_block." << std::endl;
}

TEST_F(GrBlockVoidTest, Inlineprocess)
{
    int nsamples = 100000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    GrBlockVoid_sptr block_ = GrBlockVoid_make(true, true);
    top_block = gr::make_top_block("GrBlockVoid test");

    EXPECT_NO_THROW( {
        boost::shared_ptr<gr::analog::noise_source_c> source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, block_, 0);
    }) << "Failure connecting the blocks of GrBlockVoid test." << std::endl;

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    std::cout << "Inline process: Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " us" << std::endl;
    std::cout << "Inline process: Time per sample = " << elapsed_seconds.count() * 1e6 / static_cast<double>(nsamples) << " us" << std::endl;
}

TEST_F(GrBlockVoidTest, BlockingAndWorker)
{
    int nsamples = 100000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    GrBlockVoid_sptr block_ = GrBlockVoid_make(true, false);
    top_block = gr::make_top_block("GrBlockVoid test");

    EXPECT_NO_THROW( {
        boost::shared_ptr<gr::analog::noise_source_c> source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, block_, 0);
    }) << "Failure connecting the blocks of GrBlockVoid test." << std::endl;

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    std::cout << "Blocking and worker: Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " us" << std::endl;
    std::cout << "Blocking and worker: Time per sample = " << elapsed_seconds.count() * 1e6 / static_cast<double>(nsamples) << " us" << std::endl;
}

TEST_F(GrBlockVoidTest, NonBlockingAndWorker)
{
    int nsamples = 100000;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);
    GrBlockVoid_sptr block_ = GrBlockVoid_make(false, false);
    top_block = gr::make_top_block("GrBlockVoid test");

    EXPECT_NO_THROW( {
        boost::shared_ptr<gr::analog::noise_source_c> source = gr::analog::noise_source_c::make(gr::analog::GR_GAUSSIAN, 1.0);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, block_, 0);
    }) << "Failure connecting the blocks of GrBlockVoid test." << std::endl;

    EXPECT_NO_THROW( {
        start = std::chrono::system_clock::now();
        top_block->run(); // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block." << std::endl;

    std::cout << "Non Blocking and worker: Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " us" << std::endl;
    std::cout << "Non Blocking and worker: Time per sample = " << elapsed_seconds.count() * 1e6 / static_cast<double>(nsamples) << " us" << std::endl;
}

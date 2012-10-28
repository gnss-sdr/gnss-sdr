/*!
 * \file galileo_e1_dll_pll_veml_tracking_test.cc
 * \brief  This class implements a tracking test for GalileoE1DllPllVemlTracking
 *  class based on some input parameters.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012  (see AUTHORS file for a list of contributors)
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



#include <gtest/gtest.h>
#include <sys/time.h>
#include <iostream>
#include <gnuradio/gr_top_block.h>
#include <gnuradio/gr_file_source.h>
#include <gnuradio/gr_sig_source_c.h>
#include <gnuradio/gr_msg_queue.h>
#include <gnuradio/gr_null_sink.h>
#include <gnuradio/gr_skiphead.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"

#include "galileo_e1_dll_pll_veml_tracking.h"

class GalileoE1DllPllVemlTrackingInternalTest: public ::testing::Test
{
protected:
	GalileoE1DllPllVemlTrackingInternalTest()
	{
		queue = gr_make_msg_queue(0);
		top_block = gr_make_top_block("Tracking test");
		factory = new GNSSBlockFactory();
		config = new InMemoryConfiguration();
		item_size = sizeof(gr_complex);
		stop = false;
		message = 0;
	}

	~GalileoE1DllPllVemlTrackingInternalTest()
	{
		delete factory;
		delete config;
	}

	void init();

	gr_msg_queue_sptr queue;
	gr_top_block_sptr top_block;
	GNSSBlockFactory* factory;
	InMemoryConfiguration* config;
	Gnss_Synchro gnss_synchro;
	size_t item_size;
	concurrent_queue<int> channel_internal_queue;
	bool stop;
	int message;
	boost::thread ch_thread;
};


void GalileoE1DllPllVemlTrackingInternalTest::init()
{
	gnss_synchro.Channel_ID=0;
	gnss_synchro.System = 'E';
	std::string signal = "1B";
	signal.copy(gnss_synchro.Signal,2,0);
	gnss_synchro.PRN=11;

	config->set_property("GNSS-SDR.internal_fs_hz", "8000000");
	config->set_property("Tracking.item_type", "gr_complex");
	config->set_property("Tracking.dump", "true");
	config->set_property("Tracking.dump_filename", "../data/veml_tracking_ch_");
	config->set_property("Tracking.implementation", "Galileo_E1_DLL_PLL_Tracking");
	config->set_property("Tracking.early_late_space_chips", "0.15");
	config->set_property("Tracking.very_early_late_space_chips", "0.6");
	config->set_property("Tracking.pll_bw_hz", "30.0");
	config->set_property("Tracking.dll_bw_hz", "2.0");
}



TEST_F(GalileoE1DllPllVemlTrackingInternalTest, Instantiate)
{

	init();
	GalileoE1DllPllVemlTracking *tracking = new GalileoE1DllPllVemlTracking(config, "Tracking", 1, 1, queue);
	EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());
	delete tracking;
}

//TEST_F(GalileoE1DllPllVemlTrackingInternalTest, ConnectAndRun)
//{
//    int fs_in = 8000000;
//    int nsamples = 80000000;
//    struct timeval tv;
//    long long int begin;
//    long long int end;
//
//    init();
//    GalileoE1DllPllVemlTracking *tracking = new GalileoE1DllPllVemlTracking(config, "Tracking", 1, 1, queue);
//
//    ASSERT_NO_THROW( {
//        tracking->connect(top_block);
//
//            gr_sig_source_c_sptr source = gr_make_sig_source_c(fs_in,GR_SIN_WAVE, 1000, 1, gr_complex(0));
//            gr_block_sptr valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
//            gr_null_sink_sptr sink = gr_make_null_sink(sizeof(Gnss_Synchro));
//            top_block->connect(source, 0, valve, 0);
//            top_block->connect(valve, 0, tracking->get_left_block(), 0);
//            top_block->connect(tracking->get_right_block(), 0, sink, 0);
//
//        }) << "Failure connecting the blocks of tracking test."<< std::endl;
//
//    EXPECT_NO_THROW( {
//            gettimeofday(&tv, NULL);
//            begin = tv.tv_sec *1000000 + tv.tv_usec;
//            top_block->run(); // Start threads and wait
//            gettimeofday(&tv, NULL);
//            end = tv.tv_sec *1000000 + tv.tv_usec;
//        }) << "Failure running he top_block."<< std::endl;
//
//    delete tracking;
//    std::cout <<  "Processed " << nsamples << " samples in " << (end-begin) << " microseconds" << std::endl;
//
//}

TEST_F(GalileoE1DllPllVemlTrackingInternalTest, ValidationOfResults)
{
	struct timeval tv;
	long long int begin;
	long long int end;
	// int num_samples = 40000000; // 4 Msps
	// unsigned int skiphead_sps = 24000000; // 4 Msps
	int num_samples = 80000000; // 8 Msps
	unsigned int skiphead_sps = 8000000; // 8 Msps
	init();
	GalileoE1DllPllVemlTracking *tracking = new GalileoE1DllPllVemlTracking(config, "Tracking", 1, 1, queue);

	// gnss_synchro.Acq_delay_samples=1753; // 4 Msps
	// gnss_synchro.Acq_doppler_hz=-9500; // 4 Msps
	gnss_synchro.Acq_delay_samples=17256; // 8 Msps
	gnss_synchro.Acq_doppler_hz=-8750; // 8 Msps
	gnss_synchro.Acq_samplestamp_samples=0;

	ASSERT_NO_THROW( {
		tracking->set_channel(gnss_synchro.Channel_ID);
	}) << "Failure setting channel."<< std::endl;

	ASSERT_NO_THROW( {
		tracking->set_gnss_synchro(&gnss_synchro);
	}) << "Failure setting gnss_synchro."<< std::endl;

	ASSERT_NO_THROW( {
		tracking->set_channel_queue(&channel_internal_queue);
	}) << "Failure setting channel_internal_queue."<< std::endl;

	ASSERT_NO_THROW( {
		tracking->connect(top_block);
	}) << "Failure connecting tracking to the top_block."<< std::endl;

	ASSERT_NO_THROW( {
		std::string file = "/media/DATA/Proyectos/Signals/cttc_2012_07_26/cp_cttc_2012_07_26_n6_8Msps.dat";
		// std::string file = "/media/DATA/Proyectos/Signals/cttc_2012_07_26/cp_cttc_2012_07_26_n4_4Msps.dat";
		// std::string file = "/media/DATA/Proyectos/Signals/prueba.dat";
		// std::string file = "../data/resampler.dat";
		const char * file_name = file.c_str();
		gr_file_source_sptr file_source = gr_make_file_source(sizeof(gr_complex),file_name,false);
		gr_skiphead_sptr skip_head = gr_make_skiphead(sizeof(gr_complex), skiphead_sps);
		gr_block_sptr valve = gnss_sdr_make_valve(sizeof(gr_complex), num_samples, queue);
		gr_null_sink_sptr sink = gr_make_null_sink(sizeof(Gnss_Synchro));
		top_block->connect(file_source, 0, skip_head, 0);
		top_block->connect(skip_head, 0, valve, 0);
		top_block->connect(valve, 0, tracking->get_left_block(), 0);
		top_block->connect(tracking->get_right_block(), 0, sink, 0);
	}) << "Failure connecting the blocks of tracking test."<< std::endl;


	tracking->start_tracking();

	EXPECT_NO_THROW( {
		gettimeofday(&tv, NULL);
		begin = tv.tv_sec *1000000 + tv.tv_usec;
		top_block->run(); // Start threads and wait
		gettimeofday(&tv, NULL);
		end = tv.tv_sec *1000000 + tv.tv_usec;
	}) << "Failure running he top_block."<< std::endl;

	std::cout <<  "Tracked " << num_samples << " samples in " << (end-begin) << " microseconds" << std::endl;

	delete tracking;
}

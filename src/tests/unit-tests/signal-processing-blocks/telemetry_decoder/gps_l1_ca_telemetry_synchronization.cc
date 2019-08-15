/*!
 * \file gps_l1_ca_telemetry_decoder_lucas_test.cc
 * \brief  This class implements a telemetry decoder test for GPS_L1_CA_Telemetry_Decoder
 *  implementation based on some input parameters.
 * \author Lucas Ventura, 2015. lucas.ventura.r(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include <armadillo>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/top_block.h>
#include <chrono>
#include <exception>
#include <string>
#include <unistd.h>
#include <utility>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include "GPS_L1_CA.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include "telemetry_decoder_interface.h"
#include "tlm_dump_reader.h"
#include "tracking_dump_reader.h"
#include "tracking_interface.h"
#include "tracking_true_obs_reader.h"
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>

#include <random>

#define vector_size 1500 	// 5 frames with 300 bits


class GpsL1CATelemetrySynchronizationTest : public ::testing::Test
{
public:

	Gnss_Synchro gnss_synchro;
	std::vector<Gnss_Synchro> synchro_vector;
	
	std::vector<int> initial_vector;
	
	GpsL1CATelemetrySynchronizationTest()
	{
		gnss_synchro = Gnss_Synchro();
	}
	
	
	void make_vector();
	void fill_gnss_synchro();

};


void GpsL1CATelemetrySynchronizationTest::make_vector()
{
	std::random_device rd;  //Otain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<> dis(0, 1);
	 
	for (int32_t i = 0; i < vector_size; i++)
	{
		// SW = 1 0 0 0 1 0 1 1
		/*
    	if(i%300 == 0)
            initial_vector.push_back(1);
    	else if((i-1)%300 == 0)
            initial_vector.push_back(0);
    	else if((i-2)%300 == 0)
    		initial_vector.push_back(0);
    	else if((i-3)%300 == 0)
    	    initial_vector.push_back(0);
    	else if((i-4)%300 == 0)
    		initial_vector.push_back(1);
    	else if((i-5)%300 == 0)
    		initial_vector.push_back(0);
    	else if((i-6)%300 == 0)
    		initial_vector.push_back(1);
    	else if((i-6)%300 == 0)
    		initial_vector.push_back(1);
    	else*/
		//Transform the random unsigned int generated by gen into an int in [0, 1]
	    //std::cout << dis(gen)*2-1 << ' ';
    		initial_vector.push_back(dis(gen)*2-1);
	}
	std::cout << '\n';
}



void GpsL1CATelemetrySynchronizationTest::fill_gnss_synchro()
{
	// Satellite and signal info
	gnss_synchro.System = 'G';
	std::string signal = "1C";
	signal.copy(gnss_synchro.Signal, 2, 0);
	gnss_synchro.PRN = FLAGS_test_satellite_PRN;
	gnss_synchro.Channel_ID = 0;
	
	// Acquisition
	gnss_synchro.Acq_delay_samples = 0;
	gnss_synchro.Acq_doppler_hz = 0;
	gnss_synchro.Acq_samplestamp_samples = 0;
	gnss_synchro.Flag_valid_acquisition = true;
	
	// Tracking
	gnss_synchro.fs = 2000000;
	// gnss_synchro.Prompt_I;
	// gnss_synchro.Prompt_Q;
	gnss_synchro.CN0_dB_hz = 10;
	gnss_synchro.Carrier_Doppler_hz = 0.0;
	gnss_synchro.Carrier_phase_rads = 0.0;
	gnss_synchro.Code_phase_samples = 0.0;
	gnss_synchro.Tracking_sample_counter = 2000000 * 0.02;
	gnss_synchro.Flag_valid_symbol_output = true;
	gnss_synchro.correlation_length_ms = 20;
	
	
    // Random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.1;
    auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
                          std::mt19937(std::random_device{}()));
	
    for (auto& x : initial_vector) 
    {
    	gnss_synchro.Prompt_I = x + dist();
    	
    	synchro_vector.push_back(gnss_synchro);
    	
    }
	
}

// for (int32_t i = 0; i < vector_size; i++)
//	{
	
//	}


TEST_F(GpsL1CATelemetrySynchronizationTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();
	
    make_vector();
    fill_gnss_synchro();
    
    
    //std::cout << "Synchro vector " << synchro_vector[0].Prompt_I;
    
    std::cout << "\n";
	
    
    
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "GPSL1CA Telemetry decoder Test completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

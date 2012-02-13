
/*!
 * \file direct_resampler_conditioner_cc_test.cc
 * \brief  Executes a resampler based on some input parameters.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
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


#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <gnuradio/gr_top_block.h>
#include <gnuradio/gr_sig_source_c.h>
#include <gnuradio/gr_msg_queue.h>
#include <gnuradio/gr_null_sink.h>

#include "gnss_sdr_valve.h"
#include "direct_resampler_conditioner_cc.h"



DEFINE_string(signal_file, "../data/signal.dat",
		"Path to the file containing the signal samples");

DEFINE_double(fs_in, 8000000.0, "FS of the signal in Hz");
DEFINE_double(fs_out, 4000000.0, "FS of the resampled signal in Hz");

using namespace std;

TEST(Direct_Resampler_Conditioner_Cc_Test, InstantiationAndRunTest) {


	try {

		gr_msg_queue_sptr queue = gr_make_msg_queue(0);

		gr_top_block_sptr top_block = gr_make_top_block("direct_resampler_conditioner_cc_test");
	    gr_sig_source_c_sptr source = gr_make_sig_source_c(FLAGS_fs_in, GR_SIN_WAVE, 1000, 1, 0);
	    gr_block_sptr valve = gnss_sdr_make_valve(sizeof(gr_complex), 1000000, queue);
		direct_resampler_conditioner_cc_sptr resampler = direct_resampler_make_conditioner_cc(FLAGS_fs_in, FLAGS_fs_out);
		gr_block_sptr sink = gr_make_null_sink(sizeof(gr_complex));

	    top_block->connect(source, 0, valve, 0);
		top_block->connect(valve, 0, resampler, 0);
		top_block->connect(resampler, 0, sink, 0);
		top_block->run(); // Start threads and wait
		top_block->stop();
		}

	catch(std::runtime_error &e){

		ADD_FAILURE() << "Runtime error";
	}
	catch(...){

		ADD_FAILURE() << "Uncaught exception";
	}

}

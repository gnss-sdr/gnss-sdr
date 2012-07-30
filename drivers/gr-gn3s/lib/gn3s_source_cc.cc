/*!
 * \file gn3s_source_cc.cc
 * \brief GNU Radio source block to acces to SiGe GN3S USB sampler v2.
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gn3s_source_cc.h>
#include <gn3s_defines.h>
#include <gr_io_signature.h>


/*
 * Create a new instance of howto_square_ff and return
 * a boost shared_ptr.  This is effectively the public constructor.
 */
gn3s_source_cc_sptr
gn3s_make_source_cc ()
{
  return gnuradio::get_initial_sptr(new gn3s_source_cc ());
}

/*
 * Specify constraints on number of input and output streams.
 * This info is used to construct the input and output signatures
 * (2nd & 3rd args to gr_block's constructor).  The input and
 * output signatures are used by the runtime system to
 * check that a valid number and type of inputs and outputs
 * are connected to this block.  In this case, we accept
 * only 1 input and 1 output.
 */
static const int MIN_IN = 0;	// mininum number of input streams
static const int MAX_IN = 0;	// maximum number of input streams
static const int MIN_OUT = 1;	// minimum number of output streams
static const int MAX_OUT = 1;	// maximum number of output streams

/*
 * The private constructor
 */
gn3s_source_cc::gn3s_source_cc ()
  : gr_block ("gn3s_cc",
	      gr_make_io_signature (MIN_IN, MAX_IN, sizeof (gr_complex)),
	      gr_make_io_signature (MIN_OUT, MAX_OUT, sizeof (gr_complex)))
{
  // constructor code here
  gn3s_drv = new gn3s_Source();
  fprintf(stdout,"GN3S Start\n");
}

/*
 * Our virtual destructor.
 */
gn3s_source_cc::~gn3s_source_cc ()
{
  // destructor code here
	if(gn3s_drv != NULL)
	{
		fprintf(stdout,"Destructing GN3S\n");
		delete gn3s_drv;
		//delete packet;
	}
}

int 
gn3s_source_cc::general_work (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
int n_samples_rx;
  gr_complex *out = (gr_complex *) output_items[0];
  
if (noutput_items<=GN3S_SAMPS_5MS)
{
  gn3s_drv->Read(&packet,noutput_items);
  n_samples_rx=noutput_items;
}else{
  gn3s_drv->Read(&packet,GN3S_SAMPS_5MS);
  n_samples_rx=GN3S_SAMPS_5MS;
}
  for (int i = 0; i < n_samples_rx; i++){
	out[i]=gr_complex(packet.data[i].i,packet.data[i].q);
  }

  // Tell runtime system how many output items we produced.
  return n_samples_rx;
}


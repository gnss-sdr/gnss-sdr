/*!
 * \file beamformer.cc
 *
 * \brief Unpacks byte samples to NSR 2 bits samples
 * \author Javier Arribas jarribas (at) cttc.es
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


#include "beamformer.h"
#include <iostream>
#include <gnuradio/io_signature.h>



beamformer_sptr make_beamformer()
{
    return beamformer_sptr(new beamformer());
}

beamformer::beamformer()
	: gr::sync_block("beamformer",
            		gr::io_signature::make(8, 8,sizeof(gr_complex)),
            		gr::io_signature::make(1, 1,sizeof(gr_complex)))
{

}

beamformer::~beamformer()
{

}

int beamformer::work(int noutput_items,gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    //gr_complex *out = (gr_complex *) output_items[0];
	  // channel output buffers
	//  gr_complex *ch1 = (gr_complex *) output_items[0];
	//  gr_complex *ch2 = (gr_complex *) output_items[1];
	//  gr_complex *ch3 = (gr_complex *) output_items[2];
	//  gr_complex *ch4 = (gr_complex *) output_items[3];
	//  gr_complex *ch5 = (gr_complex *) output_items[4];
	//  gr_complex *ch6 = (gr_complex *) output_items[5];
	//  gr_complex *ch7 = (gr_complex *) output_items[6];
	//  gr_complex *ch8 = (gr_complex *) output_items[7];


    return noutput_items;
}

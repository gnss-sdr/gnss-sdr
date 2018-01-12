/*!
 * \file unpack_intspir_1bit_samples.cc
 *
 * \brief Unpacks SPIR int samples to NSR 1 bit samples
 * \author Fran Fabra fabra (at) ice.csic.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is not part of GNSS-SDR.
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


#include "unpack_spir_gss6450_samples.h"
#include <gnuradio/io_signature.h>



unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int n_chann, unsigned int sel_ch, size_t item_size)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(n_chann, sel_ch, item_size));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(
        unsigned int n_chann, unsigned int sel_ch, size_t item_size) : gr::block("unpack_spir_gss6450_samples",
        gr::io_signature::make(1, 1, sizeof(int)),
        gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    d_channels = n_chann;
    d_sel_ch = sel_ch;
    item_size_ = item_size;
    ch_processing = 1;
}

void unpack_spir_gss6450_samples::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_channels * noutput_items;
}


unpack_spir_gss6450_samples::~unpack_spir_gss6450_samples()
{}


int unpack_spir_gss6450_samples::general_work(int noutput_items,
                                              gr_vector_const_void_star &input_items,
                                              gr_vector_void_star &output_items)
{
    const int *in = reinterpret_cast<const int *>(input_items[0]);
    gr_complex *out = reinterpret_cast<gr_complex*>(output_items[0]);




    return noutput_items;
}

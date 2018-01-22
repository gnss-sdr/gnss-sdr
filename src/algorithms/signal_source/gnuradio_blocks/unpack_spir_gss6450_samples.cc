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



unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int adc_nbit)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(adc_nbit));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(unsigned int adc_nbit) : gr::sync_interpolator("unpack_spir_gss6450_samples",
        gr::io_signature::make(1, 1, sizeof(int)),
        gr::io_signature::make(1, 1, sizeof(gr_complex)), 16 / adc_nbit)
{
    adc_bits = adc_nbit;
    i_data = 0;
    q_data = 0;
    samples_per_int = 16 / adc_bits;
    if(adc_bits == 2)
    {
        mask_data = 0x00000003;
        map_ = {0, 1, -2, -1};
    }
    else
    {
        mask_data = 0x0000000F;
        map_ = {0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1};
    }
}


unpack_spir_gss6450_samples::~unpack_spir_gss6450_samples()
{}

void unpack_spir_gss6450_samples::process_sample(gr_complex& out)
{
    out = gr_complex(0.5, 0.5);
    compute_two_complement(i_data);
    compute_two_complement(q_data);
    out += gr_complex(static_cast<float>(i_data), static_cast<float>(q_data));
}


int unpack_spir_gss6450_samples::work(int noutput_items,
                      gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    const int* in = reinterpret_cast<const int*>(input_items[0]);
    gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);
    unsigned int n_sample = 0;
    unsigned int in_counter = 0;
    for(int i = 0; i < noutput_items; i++)
    {
        int sample_aux = in[in_counter];
        int aux_i = sample_aux;
        int aux_q = sample_aux;
        int i_shift = adc_bits * 2 * (samples_per_int - n_sample - 1) + adc_bits;
        int q_shift = adc_bits * 2 * (samples_per_int - n_sample - 1);
        i_data = (aux_i >> i_shift) & mask_data;
        q_data = (aux_q >> q_shift) & mask_data;
        process_sample(out[samples_per_int * in_counter + samples_per_int - n_sample - 1]);
        n_sample++;
        if(n_sample == samples_per_int)
        {
            n_sample = 0;
            in_counter++;
        }
    }
    return noutput_items;
}

void unpack_spir_gss6450_samples::compute_two_complement(int& data)
{
    data = map_[data];
}

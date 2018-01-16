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



unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int n_chann, unsigned int sel_ch, int samp_item, size_t item_size)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(n_chann, sel_ch, samp_item, item_size));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(
        unsigned int n_chann, unsigned int sel_ch, int samp_item, size_t item_size) : gr::block("unpack_spir_gss6450_samples",
        gr::io_signature::make(1, 1, sizeof(int)),
        gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    d_channels = n_chann;
    d_sel_ch = sel_ch;
    item_size_ = item_size;
    ch_processing = 1;
    d_samp_item = samp_item;
    samp_frame = 0;
    adc_bits = 16 / d_samp_item;
    i_ = true;
    new_sample = false;
    i_data = 0;
    q_data = 0;
}


unpack_spir_gss6450_samples::~unpack_spir_gss6450_samples()
{}

void unpack_spir_gss6450_samples::process_sample(gr_complex* out)
{
    gr_complex result = gr_complex(0.5, 0.5);
    compute_two_complement(i_data);
    compute_two_complement(q_data);
    result += gr_complex(static_cast<float>(i_data), static_cast<float>(q_data));
    *out = result;
}

int unpack_spir_gss6450_samples::general_work(int noutput_items,
                                              gr_vector_const_void_star &input_items,
                                              gr_vector_void_star &output_items)
{
    const int* in = reinterpret_cast<const int*>(input_items[0]);
    gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);
    int samples_produced = 0;
    for(int i = 0; i < noutput_items; i++)
    {
        if(ch_processing == d_sel_ch)
        {
            if(i_)
            {
                i_data = in[i];
                swap_data(i_data);
                i_ = false;
            }
            else
            {
                q_data = in[i];
                swap_data(q_data);
                i_ = true;
                process_sample(out);
                out++;
                samples_produced++;
                new_sample = true;
            }
        }
        else
        {
            if(i_) { i_ = false;}
            else
            {
                i_ = true;
                new_sample = true;
            }
        }
        if(new_sample)
        {
            new_sample = false;
            samp_frame++;
            if(samp_frame == d_samp_item)
            {
                samp_frame = 0;
                ch_processing++;
                if(ch_processing > d_channels) { ch_processing = 1; }
            }
        }
    }
    consume_each(noutput_items);
    return samples_produced;
}

void unpack_spir_gss6450_samples::swap_data(int& data)
{
    int result = 0;
    int aux = data;
    int mask = 1;
    for (int i = 0; i < adc_bits; i++)
    {
        result = result << 1;
        result += (aux & mask);
        aux = aux >> 1;
    }
    data = result;
}

void unpack_spir_gss6450_samples::compute_two_complement(int& data)
{
    int result = 0;
    int mask = 1;
    for(int i = 0; i < (adc_bits - 1); i++)
    {
        result = result << 1;
        result += (data >> i) & mask;
    }
    if((data >> (adc_bits - 1)) == 1)
    {
        if(adc_bits == 2) { result -= 2; }
        else { result -= 8; }
    }
    data = result;
}

/*!
 * \file unpack_spir_gss6450_samples.cc
 *
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos,  antonio(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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
#include <cmath>

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int adc_nbit)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(adc_nbit));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(unsigned int adc_nbit) : gr::sync_interpolator("unpack_spir_gss6450_samples",
                                                                                      gr::io_signature::make(1, 1, sizeof(int)),
                                                                                      gr::io_signature::make(1, 1, sizeof(gr_complex)), 16 / adc_nbit)
{
    adc_bits = adc_nbit;
    samples_per_int = 16 / adc_bits;
    i_data.resize(adc_bits, false);
    q_data.resize(adc_bits, false);
    adc_bits_two_pow = static_cast<int>(std::exp2(adc_bits));
    two_compl_thres = adc_bits_two_pow / 2;
}


unpack_spir_gss6450_samples::~unpack_spir_gss6450_samples()
{
}


int unpack_spir_gss6450_samples::compute_two_complement(unsigned long data)
{
    int res = 0;
    if (static_cast<int>(data) < two_compl_thres)
        {
            res = static_cast<int>(data);
        }
    else
        {
            res = static_cast<int>(data) - adc_bits_two_pow;
        }
    return res;
}


int unpack_spir_gss6450_samples::work(int noutput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
{
    const int* in = reinterpret_cast<const int*>(input_items[0]);
    gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);
    unsigned int n_sample = 0;
    unsigned int in_counter = 0;
    std::bitset<32> bs;
    for (int i = 0; i < noutput_items; i++)
        {
            bs = in[in_counter];
            int i_shift = adc_bits * 2 * (samples_per_int - n_sample - 1) + adc_bits;
            int q_shift = adc_bits * 2 * (samples_per_int - n_sample - 1);
            for (unsigned int k = 0; k < adc_bits; k++)
                {
                    i_data[k] = bs[i_shift + k];
                    q_data[k] = bs[q_shift + k];
                }
            out[i] = gr_complex(static_cast<float>(compute_two_complement(i_data.to_ulong())) + 0.5,
                static_cast<float>(compute_two_complement(q_data.to_ulong())) + 0.5);
            n_sample++;
            if (n_sample == samples_per_int)
                {
                    n_sample = 0;
                    in_counter++;
                }
        }
    return noutput_items;
}

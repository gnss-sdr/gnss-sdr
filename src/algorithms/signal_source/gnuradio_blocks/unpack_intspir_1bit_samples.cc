/*!
 * \file unpack_intspir_1bit_samples.cc
 *
 * \brief Unpacks SPIR int samples to NSR 1 bit samples
 * \author Fran Fabra fabra (at) ice.csic.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "unpack_intspir_1bit_samples.h"
#include <gnuradio/io_signature.h>


unpack_intspir_1bit_samples_sptr make_unpack_intspir_1bit_samples()
{
    return unpack_intspir_1bit_samples_sptr(new unpack_intspir_1bit_samples());
}


unpack_intspir_1bit_samples::unpack_intspir_1bit_samples() : sync_interpolator("unpack_intspir_1bit_samples",
                                                                 gr::io_signature::make(1, 1, sizeof(int)),
                                                                 gr::io_signature::make(1, 1, sizeof(float)),
                                                                 2)
{
}


unpack_intspir_1bit_samples::~unpack_intspir_1bit_samples()
{
}


int unpack_intspir_1bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const signed int *in = reinterpret_cast<const signed int *>(input_items[0]);
    float *out = reinterpret_cast<float *>(output_items[0]);

    int n = 0;
    int channel = 1;
    for (int i = 0; i < noutput_items / 2; i++)
        {
            // Read packed input sample (1 byte = 1 complex sample)
            // For historical reasons, values are float versions of short int limits (32767)
            signed int val = in[i];
            if (((val >> ((channel - 1) * 2)) & 1) == 1)
                {
                    out[n++] = static_cast<float>(32767.0);
                }
            else
                {
                    out[n++] = static_cast<float>(-32767.0);
                }
            if (((val >> (2 * channel - 1)) & 1) == 1)
                {
                    out[n++] = static_cast<float>(32767.0);
                }
            else
                {
                    out[n++] = static_cast<float>(-32767.0);
                }
        }
    return noutput_items;
}

/*!
 * \file unpack_byte_2bit_cpx_samples.cc
 *
 * \brief Unpacks byte samples to 2 bits complex samples.
 *     Packing Order
 *     Most Significant Nibble  - Sample n
 *     Least Significant Nibble - Sample n+1
 *     Packing order in Nibble Q1 Q0 I1 I0
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


#include "unpack_byte_2bit_cpx_samples.h"
#include <gnuradio/io_signature.h>
#include <cstdint>

struct byte_2bit_struct
{
    signed two_bit_sample : 2;  // <- 2 bits wide only
};


unpack_byte_2bit_cpx_samples_sptr make_unpack_byte_2bit_cpx_samples()
{
    return unpack_byte_2bit_cpx_samples_sptr(new unpack_byte_2bit_cpx_samples());
}


unpack_byte_2bit_cpx_samples::unpack_byte_2bit_cpx_samples() : sync_interpolator("unpack_byte_2bit_cpx_samples",
                                                                   gr::io_signature::make(1, 1, sizeof(int8_t)),
                                                                   gr::io_signature::make(1, 1, sizeof(int16_t)),
                                                                   4)
{
}


int unpack_byte_2bit_cpx_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const int8_t *>(input_items[0]);
    auto *out = reinterpret_cast<int16_t *>(output_items[0]);

    byte_2bit_struct sample{};
    int n = 0;
    for (int i = 0; i < noutput_items / 4; i++)
        {
            // Read packed input sample (1 byte = 2 complex samples)
            //*     Packing Order
            //*     Most Significant Nibble  - Sample n
            //*     Least Significant Nibble - Sample n+1
            //*     Packing order in Nibble Q1 Q0 I1 I0
            //normal
            //  int8_t c = in[i];
            //  //Q[n]
            //  sample.two_bit_sample = (c>>6) & 3;
            //  out[n++] = (2*(int16_t)sample.two_bit_sample+1);
            //  //I[n]
            //  sample.two_bit_sample = (c>>4) & 3;
            //  out[n++] = (2*(int16_t)sample.two_bit_sample+1);
            //  //Q[n+1]
            //  sample.two_bit_sample = (c>>2) & 3;
            //  out[n++] = (2*(int16_t)sample.two_bit_sample+1);
            //  //I[n+1]
            //  sample.two_bit_sample = c & 3;
            //  out[n++] = (2*(int16_t)sample.two_bit_sample+1);

            //I/Q swap
            int8_t c = in[i];
            //I[n]
            sample.two_bit_sample = (c >> 4) & 3;
            out[n++] = (2 * static_cast<int16_t>(sample.two_bit_sample) + 1);
            //Q[n]
            sample.two_bit_sample = (c >> 6) & 3;
            out[n++] = (2 * static_cast<int16_t>(sample.two_bit_sample) + 1);
            //I[n+1]
            sample.two_bit_sample = c & 3;
            out[n++] = (2 * static_cast<int16_t>(sample.two_bit_sample) + 1);
            //Q[n+1]
            sample.two_bit_sample = (c >> 2) & 3;
            out[n++] = (2 * static_cast<int16_t>(sample.two_bit_sample) + 1);
        }
    return noutput_items;
}

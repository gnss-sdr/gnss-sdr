/*!
 * \file unpack_byte_4bit_samples.cc
 *
 * \brief Unpacks byte samples to 4 bits samples.
 *     Packing Order
 *     Packing order in Nibble I0 I1 I2 I3 I0 I1 I2 I3
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "unpack_byte_4bit_samples.h"
#include <gnuradio/io_signature.h>

unpack_byte_4bit_samples_sptr make_unpack_byte_4bit_samples()
{
    return unpack_byte_4bit_samples_sptr(new unpack_byte_4bit_samples());
}


unpack_byte_4bit_samples::unpack_byte_4bit_samples() : sync_interpolator("unpack_byte_4bit_samples",
                                                           gr::io_signature::make(1, 1, sizeof(signed char)),
                                                           gr::io_signature::make(1, 1, sizeof(signed char)),
                                                           2)
{
}


unpack_byte_4bit_samples::~unpack_byte_4bit_samples()
{
}


int unpack_byte_4bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const signed char *in = reinterpret_cast<const signed char *>(input_items[0]);
    signed char *out = reinterpret_cast<signed char *>(output_items[0]);
    int n = 0;
    unsigned char tmp_char2;
    for (int i = 0; i < noutput_items / 2; i++)
        {
            tmp_char2 = in[i] & 0x0F;
            if (tmp_char2 >= 8)
                {
                    out[n++] = 2 * (tmp_char2 - 16) + 1;
                }
            else
                {
                    out[n++] = 2 * tmp_char2 + 1;
                }
            tmp_char2 = in[i] >> 4;
            tmp_char2 = tmp_char2 & 0x0F;
            if (tmp_char2 >= 8)
                {
                    out[n++] = 2 * (tmp_char2 - 16) + 1;
                }
            else
                {
                    out[n++] = 2 * tmp_char2 + 1;
                }
        }
    return noutput_items;
}

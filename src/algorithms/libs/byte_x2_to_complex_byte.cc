/*!
 * \file byte_x2_to_complex_byte.cc
 * \brief Adapts two signed char streams into a std::complex<signed char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "byte_x2_to_complex_byte.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>


byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte()
{
    return byte_x2_to_complex_byte_sptr(new byte_x2_to_complex_byte());
}


byte_x2_to_complex_byte::byte_x2_to_complex_byte() : sync_block("byte_x2_to_complex_byte",
                                                         gr::io_signature::make(2, 2, sizeof(int8_t)),    // int8_t, defined in stdint.h and included in volk.h (signed char)
                                                         gr::io_signature::make(1, 1, sizeof(lv_8sc_t)))  // lv_8sc_t is a Volk's typedef for std::complex<signed char>
{
    const int alignment_multiple = volk_get_alignment() / sizeof(lv_8sc_t);
    set_alignment(std::max(1, alignment_multiple));
}


int byte_x2_to_complex_byte::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const int8_t *in0 = reinterpret_cast<const int8_t *>(input_items[0]);
    const int8_t *in1 = reinterpret_cast<const int8_t *>(input_items[1]);
    lv_8sc_t *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
    // This could be put into a volk kernel
    int8_t real_part;
    int8_t imag_part;
    for (int number = 0; number < noutput_items; number++)
        {
            // lv_cmake(r, i) defined at volk/volk_complex.h
            real_part = *in0++;
            imag_part = *in1++;
            *out++ = lv_cmake(real_part, imag_part);
        }
    return noutput_items;
}

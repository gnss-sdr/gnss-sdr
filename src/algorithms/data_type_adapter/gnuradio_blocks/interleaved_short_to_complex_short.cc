/*!
 * \file interleaved_short_to_complex_short.cc
 * \brief Adapts a short (16-bits) interleaved sample stream into a std::complex<short> stream
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


#include "interleaved_short_to_complex_short.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>


interleaved_short_to_complex_short_sptr make_interleaved_short_to_complex_short()
{
    return interleaved_short_to_complex_short_sptr(new interleaved_short_to_complex_short());
}


interleaved_short_to_complex_short::interleaved_short_to_complex_short() : sync_decimator("interleaved_short_to_complex_short",
                                                                               gr::io_signature::make(1, 1, sizeof(int16_t)),
                                                                               gr::io_signature::make(1, 1, sizeof(lv_16sc_t)),  // lv_16sc_t is a Volk's typedef for std::complex<short int>
                                                                               2)
{
    const int alignment_multiple = volk_get_alignment() / sizeof(lv_16sc_t);
    set_alignment(std::max(1, alignment_multiple));
}


int interleaved_short_to_complex_short::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const int16_t *in = reinterpret_cast<const int16_t *>(input_items[0]);
    lv_16sc_t *out = reinterpret_cast<lv_16sc_t *>(output_items[0]);
    // This could be put into a Volk kernel
    int16_t real_part;
    int16_t imag_part;
    for (int number = 0; number < noutput_items; number++)
        {
            // lv_cmake(r, i) defined at volk/volk_complex.h
            real_part = *in++;
            imag_part = *in++;
            *out++ = lv_cmake(real_part, imag_part);
        }
    return noutput_items;
}

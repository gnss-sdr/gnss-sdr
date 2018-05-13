/*!
 * \file complex_float_to_complex_byte.cc
 * \brief Adapts a gr_complex stream into a std::complex<signed char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
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


#include "complex_float_to_complex_byte.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


complex_float_to_complex_byte_sptr make_complex_float_to_complex_byte()
{
    return complex_float_to_complex_byte_sptr(new complex_float_to_complex_byte());
}


complex_float_to_complex_byte::complex_float_to_complex_byte() : sync_block("complex_float_to_complex_byte",
                                                                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                                                     gr::io_signature::make(1, 1, sizeof(lv_8sc_t)))  // lv_8sc_t is a Volk's typedef for std::complex<signed char>
{
    const int alignment_multiple = volk_gnsssdr_get_alignment() / sizeof(lv_8sc_t);
    set_alignment(std::max(1, alignment_multiple));
}


int complex_float_to_complex_byte::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    lv_8sc_t *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
    volk_gnsssdr_32fc_convert_8ic(out, in, noutput_items);
    return noutput_items;
}

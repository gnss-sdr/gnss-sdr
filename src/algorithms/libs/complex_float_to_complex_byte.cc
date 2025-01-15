/*!
 * \file complex_float_to_complex_byte.cc
 * \brief Adapts a gr_complex stream into a std::complex<signed char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "complex_float_to_complex_byte.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for max

complex_float_to_complex_byte_sptr make_complex_float_to_complex_byte()
{
    return complex_float_to_complex_byte_sptr(new complex_float_to_complex_byte());
}


complex_float_to_complex_byte::complex_float_to_complex_byte() : sync_block("complex_float_to_complex_byte",
                                                                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                                                     gr::io_signature::make(1, 1, sizeof(lv_8sc_t)))  // lv_8sc_t is a Volk's typedef for std::complex<signed char>
{
    const auto alignment_multiple = static_cast<int>(volk_gnsssdr_get_alignment() / sizeof(lv_8sc_t));
    set_alignment(std::max(1, alignment_multiple));
}


int complex_float_to_complex_byte::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    auto *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
    volk_gnsssdr_32fc_convert_8ic(out, in, noutput_items);
    return noutput_items;
}

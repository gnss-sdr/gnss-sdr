/*!
 * \file interleaved_byte_to_complex_byte.cc
 * \brief Adapts an 8-bits interleaved sample stream into a 16-bits complex stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
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


#include "interleaved_byte_to_complex_byte.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>  // for max


interleaved_byte_to_complex_byte_sptr make_interleaved_byte_to_complex_byte()
{
    return interleaved_byte_to_complex_byte_sptr(new interleaved_byte_to_complex_byte());
}


interleaved_byte_to_complex_byte::interleaved_byte_to_complex_byte()
    : sync_decimator("interleaved_byte_to_complex_byte",
          gr::io_signature::make(1, 1, sizeof(int8_t)),
          gr::io_signature::make(1, 1, sizeof(lv_8sc_t)),  // lv_8sc_t is a Volk's typedef for std::complex<signed char>
          2)
{
    const auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(lv_8sc_t));
    set_alignment(std::max(1, alignment_multiple));
}


int interleaved_byte_to_complex_byte::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const int8_t *>(input_items[0]);
    auto *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
    // This could be put into a Volk kernel
    int8_t real_part;
    int8_t imag_part;
    for (int number = 0; number < noutput_items; number++)
        {
            // lv_cmake(r, i) defined at volk/volk_complex.h
            real_part = *in++;
            imag_part = *in++;
            *out++ = lv_cmake(real_part, imag_part);
        }
    return noutput_items;
}

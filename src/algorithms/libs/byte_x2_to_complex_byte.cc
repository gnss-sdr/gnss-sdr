/*!
 * \file byte_x2_to_complex_byte.cc
 * \brief Adapts two signed char streams into a std::complex<signed char> stream
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


#include "byte_x2_to_complex_byte.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for max
#include <complex>    // for complex
#include <cstdint>    // for int8_t


byte_x2_to_complex_byte_sptr make_byte_x2_to_complex_byte()
{
    return byte_x2_to_complex_byte_sptr(new byte_x2_to_complex_byte());
}


byte_x2_to_complex_byte::byte_x2_to_complex_byte() : sync_block("byte_x2_to_complex_byte",
                                                         gr::io_signature::make(2, 2, sizeof(int8_t)),    // int8_t, defined in stdint.h and included in volk.h (signed char)
                                                         gr::io_signature::make(1, 1, sizeof(lv_8sc_t)))  // lv_8sc_t is a Volk's typedef for std::complex<signed char>
{
    const auto alignment_multiple = static_cast<int>(volk_gnsssdr_get_alignment() / sizeof(lv_8sc_t));
    set_alignment(std::max(1, alignment_multiple));
}


int byte_x2_to_complex_byte::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in0 = reinterpret_cast<const int8_t *>(input_items[0]);
    const auto *in1 = reinterpret_cast<const int8_t *>(input_items[1]);
    auto *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
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

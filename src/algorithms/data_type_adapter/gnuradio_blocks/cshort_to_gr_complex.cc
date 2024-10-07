/*!
 * \file cshort_to_gr_complex.cc
 * \brief Adapts a complex short (16 + 16 bits) sample stream into a
 *   std::complex<float> stream (32 + 32 bits)
 * \author Carles Fernandez Prades, 2014 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "cshort_to_gr_complex.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for max


cshort_to_gr_complex_sptr make_cshort_to_gr_complex()
{
    return cshort_to_gr_complex_sptr(new cshort_to_gr_complex());
}


cshort_to_gr_complex::cshort_to_gr_complex()
    : sync_block("cshort_to_gr_complex",
          gr::io_signature::make(1, 1, sizeof(lv_16sc_t)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    const auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(lv_16sc_t));
    set_alignment(std::max(1, alignment_multiple));
}


int cshort_to_gr_complex::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const lv_16sc_t *>(input_items[0]);
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    volk_gnsssdr_16ic_convert_32fc(out, in, noutput_items);
    return noutput_items;
}

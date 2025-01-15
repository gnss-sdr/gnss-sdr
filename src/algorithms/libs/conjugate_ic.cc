/*!
 * \file conjugate_ic.cc
 * \brief Conjugate a stream of lv_8sc_t ( std::complex<char> )
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

#include "conjugate_ic.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for max


conjugate_ic_sptr make_conjugate_ic()
{
    return conjugate_ic_sptr(new conjugate_ic());
}


conjugate_ic::conjugate_ic() : gr::sync_block("conjugate_ic",
                                   gr::io_signature::make(1, 1, sizeof(lv_8sc_t)),
                                   gr::io_signature::make(1, 1, sizeof(lv_8sc_t)))
{
    const auto alignment_multiple = static_cast<int>(volk_gnsssdr_get_alignment() / sizeof(lv_8sc_t));
    set_alignment(std::max(1, alignment_multiple));
}


int conjugate_ic::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const lv_8sc_t *>(input_items[0]);
    auto *out = reinterpret_cast<lv_8sc_t *>(output_items[0]);
    volk_gnsssdr_8ic_conjugate_8ic(out, in, noutput_items);
    return noutput_items;
}

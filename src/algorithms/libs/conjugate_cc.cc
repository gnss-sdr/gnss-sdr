/*!
 * \file conjugate_cc.cc
 * \brief Conjugate a stream of gr_complex
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

#include "conjugate_cc.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>  // for max


conjugate_cc_sptr make_conjugate_cc()
{
    return conjugate_cc_sptr(new conjugate_cc());
}


conjugate_cc::conjugate_cc() : gr::sync_block("conjugate_cc",
                                   gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                   gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    const auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(gr_complex));
    set_alignment(std::max(1, alignment_multiple));
}


int conjugate_cc::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    volk_32fc_conjugate_32fc(out, in, noutput_items);
    return noutput_items;
}

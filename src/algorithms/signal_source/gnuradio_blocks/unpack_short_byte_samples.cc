/*!
 * \file unpack_short_byte_samples.cc
 *
 * \brief Unpacks shorts samples to byte samples (1 short = 2 byte samples).
 *     Packing Order
 *     Packing order in Nibble I0 I1
 * \author Javier Arribas jarribas (at) cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "unpack_short_byte_samples.h"
#include <gnuradio/io_signature.h>

unpack_short_byte_samples_sptr make_unpack_short_byte_samples()
{
    return unpack_short_byte_samples_sptr(new unpack_short_byte_samples());
}


unpack_short_byte_samples::unpack_short_byte_samples() : sync_interpolator("unpack_short_byte_samples",
                                                             gr::io_signature::make(1, 1, sizeof(int16_t)),
                                                             gr::io_signature::make(1, 1, sizeof(int8_t)),
                                                             2)
{
}


void unpack_short_byte_samples::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int32_t>(noutput_items) / 2;
        }
}

int unpack_short_byte_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    //    const auto *in = reinterpret_cast<const int16_t *>(input_items[0]);
    //    auto *out = reinterpret_cast<int8_t *>(output_items[0]);

    memcpy(reinterpret_cast<int8_t *>(output_items[0]), reinterpret_cast<const int8_t *>(input_items[0]), noutput_items);

    return noutput_items;
}

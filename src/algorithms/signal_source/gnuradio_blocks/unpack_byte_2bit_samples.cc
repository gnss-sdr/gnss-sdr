/*!
 * \file unpack_byte_2bit_samples.cc
 * \brief Unpacks byte samples to NSR 2 bits samples
 * \author Javier Arribas jarribas (at) cttc.es
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


#include "unpack_byte_2bit_samples.h"
#include <gnuradio/io_signature.h>

struct byte_2bit_struct
{
    signed two_bit_sample : 2;  // <- 2 bits wide only
};


unpack_byte_2bit_samples_sptr make_unpack_byte_2bit_samples()
{
    return unpack_byte_2bit_samples_sptr(new unpack_byte_2bit_samples());
}


unpack_byte_2bit_samples::unpack_byte_2bit_samples() : sync_interpolator("unpack_byte_2bit_samples",
                                                           gr::io_signature::make(1, 1, sizeof(signed char)),
                                                           gr::io_signature::make(1, 1, sizeof(float)),
                                                           4)
{
}


int unpack_byte_2bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const signed char *>(input_items[0]);
    auto *out = reinterpret_cast<float *>(output_items[0]);

    byte_2bit_struct sample{};
    int n = 0;
    for (int i = 0; i < noutput_items / 4; i++)
        {
            // Read packed input sample (1 byte = 4 samples)
            signed char c = in[i];
            sample.two_bit_sample = c & 3;
            out[n++] = static_cast<float>(sample.two_bit_sample);

            sample.two_bit_sample = (c >> 2) & 3;
            out[n++] = static_cast<float>(sample.two_bit_sample);

            sample.two_bit_sample = (c >> 4) & 3;
            out[n++] = static_cast<float>(sample.two_bit_sample);

            sample.two_bit_sample = (c >> 6) & 3;
            out[n++] = static_cast<float>(sample.two_bit_sample);
        }
    return noutput_items;
}

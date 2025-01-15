/*!
 * \file unpack_intspir_1bit_samples.cc
 * \brief Unpacks SPIR int samples to NSR 1 bit samples
 * \author Fran Fabra fabra (at) ice.csic.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is not part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "unpack_intspir_1bit_samples.h"
#include <gnuradio/io_signature.h>


unpack_intspir_1bit_samples_sptr make_unpack_intspir_1bit_samples()
{
    return unpack_intspir_1bit_samples_sptr(new unpack_intspir_1bit_samples());
}


unpack_intspir_1bit_samples::unpack_intspir_1bit_samples() : sync_interpolator("unpack_intspir_1bit_samples",
                                                                 gr::io_signature::make(1, 1, sizeof(int)),
                                                                 gr::io_signature::make(1, 1, sizeof(float)),
                                                                 2)
{
}


int unpack_intspir_1bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const signed int *>(input_items[0]);
    auto *out = reinterpret_cast<float *>(output_items[0]);

    int n = 0;
    int channel = 1;
    for (int i = 0; i < noutput_items / 2; i++)
        {
            // Read packed input sample (1 byte = 1 complex sample)
            // For historical reasons, values are float versions of short int limits (32767)
            signed int val = in[i];
            if (((val >> ((channel - 1) * 2)) & 1) == 1)
                {
                    out[n++] = static_cast<float>(32767.0);
                }
            else
                {
                    out[n++] = static_cast<float>(-32767.0);
                }
            if (((val >> (2 * channel - 1)) & 1) == 1)
                {
                    out[n++] = static_cast<float>(32767.0);
                }
            else
                {
                    out[n++] = static_cast<float>(-32767.0);
                }
        }
    return noutput_items;
}

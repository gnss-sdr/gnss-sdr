/*!
 * \file unpack_ntlab_2bit_samples.cc
 *
 * \brief Unpacks multichannel 2-bit samples into 4 real-valued floats
 * per input byte.
 * \author Pedro Pereira pereirapedrocp (at) gmail.com
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


#include "unpack_ntlab_2bit_samples.h"
#include <gnuradio/io_signature.h>


unpack_ntlab_2bit_samples_sptr make_unpack_ntlab_2bit_samples(size_t item_size,
    int nchannels)
{
    return unpack_ntlab_2bit_samples_sptr(
        new unpack_ntlab_2bit_samples(item_size,
            nchannels));
}


unpack_ntlab_2bit_samples::unpack_ntlab_2bit_samples(size_t item_size,
    int nchannels)
    : sync_interpolator("unpack_ntlab_2bit_samples",
          gr::io_signature::make(1, 1, item_size),
          gr::io_signature::make(nchannels, nchannels, sizeof(float)),
          SAMPLES_PER_BYTE / nchannels),
      nchannels_(nchannels)
{
}


int unpack_ntlab_2bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto const *in = reinterpret_cast<signed char const *>(input_items[0]);
    int const nch = nchannels_;

    std::vector<float *> out(nch);
    for (int ch = 0; ch < nch; ++ch)
        {
            out[ch] = static_cast<float *>(output_items[ch]);
        }

    for (int i = 0; i < noutput_items; ++i)
        {
            auto b = static_cast<uint8_t>(in[i]);
            int j = 0;
            for (int n = 0; n < SAMPLES_PER_BYTE; ++n)
                {
                    int shift = 2 * (3 - n);           // 6, 4, 2, 0
                    int M = (b >> (shift + 1)) & 0x1;  // magnitude bit
                    int S = (b >> shift) & 0x1;        // sign bit
                    int mag = M ? 3 : 1;
                    int val = S ? +mag : -mag;

                    out[j][i] = static_cast<float>(val);

                    if (++j == nch)  // iterate through each channel
                        {
                            j = 0;
                        }
                }
        }

    return noutput_items;
}

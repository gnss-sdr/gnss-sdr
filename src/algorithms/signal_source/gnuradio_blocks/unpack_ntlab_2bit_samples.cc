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
          1),  // we make 4 floats out for every byte in
      item_size_(item_size),
      nchannels_(nchannels)
{
}


int unpack_ntlab_2bit_samples::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto const *in = reinterpret_cast<signed char const *>(input_items[0]);

    float *out[4];
    for (int n = 0; n < nchannels_; ++n)
        {
            out[n] = static_cast<float *>(output_items[n]);
        }

    const int nbytes = noutput_items;

    for (int i = 0; i < nbytes; ++i)
        {
            // Unpack each of the four 2-bit samples in the byte 'b' into four real-valued outputs.
            //
            // The NTLAB format encodes samples as sign+magnitude pairs in each byte:
            //   bits 7-6 = [M0 S0] -> sample 0
            //   bits 5-4 = [M1 S1] -> sample 1
            //   bits 3-2 = [M2 S2] -> sample 2
            //   bits 1-0 = [M3 S3] -> sample 3
            //
            // Here we loop over channel index n = 0...3, compute the bit shift to extract
            // the two bits for that channel (shift = 6,4,2,0), then:
            //   - M = magnitude bit (1->|sample|=3, 0->|sample|=1)
            //   - S = sign     bit (1->positive, 0->negative)
            // We reconstruct the signed sample value (+/-1 or +/-3) and store it in out[n][i].
            uint8_t b = static_cast<uint8_t>(in[i]);
            for (int n = 0; n < nchannels_; ++n)
                {
                    int shift = 2 * (3 - n);           // 6, 4, 2, 0
                    int M = (b >> (shift + 1)) & 0x1;  // magnitude bit
                    int S = (b >> shift) & 0x1;        // sign bit
                    int mag = M ? 3 : 1;
                    int val = S ? +mag : -mag;
                    out[n][i] = static_cast<float>(val);
                }
        }

    return nbytes;
}

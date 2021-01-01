/*!
 * \file beamformer.cc
 *
 * \brief Unpacks byte samples to NSR 2 bits samples
 * \author Javier Arribas jarribas (at) cttc.es
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


#include "beamformer.h"
#include <gnuradio/io_signature.h>
#include <cstddef>


beamformer_sptr make_beamformer_sptr()
{
    return beamformer_sptr(new beamformer());
}


beamformer::beamformer()
    : gr::sync_block("beamformer",
          gr::io_signature::make(GNSS_SDR_BEAMFORMER_CHANNELS, GNSS_SDR_BEAMFORMER_CHANNELS, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
}


int beamformer::work(int noutput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    // channel output buffers
    //  gr_complex *ch1 = (gr_complex *) input_items[0];
    //  gr_complex *ch2 = (gr_complex *) input_items[1];
    //  gr_complex *ch3 = (gr_complex *) input_items[2];
    //  gr_complex *ch4 = (gr_complex *) input_items[3];
    //  gr_complex *ch5 = (gr_complex *) input_items[4];
    //  gr_complex *ch6 = (gr_complex *) input_items[5];
    //  gr_complex *ch7 = (gr_complex *) input_items[6];
    //  gr_complex *ch8 = (gr_complex *) input_items[7];

    // NON-VOLK beamforming operation
    // TODO: Implement VOLK SIMD-accelerated beamformer!
    gr_complex sum;
    for (int n = 0; n < noutput_items; n++)
        {
            sum = gr_complex(0, 0);
            for (size_t i = 0; i < weight_vector.size(); i++)
                {
                    sum = sum + (reinterpret_cast<const gr_complex *>(input_items[i]))[n] * weight_vector[i];
                }
            out[n] = sum;
        }

    return noutput_items;
}

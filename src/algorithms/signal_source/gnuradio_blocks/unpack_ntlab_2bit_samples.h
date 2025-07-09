/*!
 * \file unpack_ntlab_2bit_samples.h
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

#ifndef GNSS_SDR_UNPACK_NTLAB_2BIT_SAMPLES_H
#define GNSS_SDR_UNPACK_NTLAB_2BIT_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>
#include <cstdint>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_ntlab_2bit_samples;

using unpack_ntlab_2bit_samples_sptr = gnss_shared_ptr<unpack_ntlab_2bit_samples>;

unpack_ntlab_2bit_samples_sptr make_unpack_ntlab_2bit_samples(
    size_t item_size,
    int nchannels = 4);

/*!
 * \brief This class implements conversion between byte packet multichannel samples
 *  to 2bit samples 1 byte = 4 2bit samples
 */
class unpack_ntlab_2bit_samples : public gr::sync_interpolator
{
public:
    ~unpack_ntlab_2bit_samples() = default;

    unpack_ntlab_2bit_samples(size_t item_size,
        int nchannels);

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend unpack_ntlab_2bit_samples_sptr make_unpack_ntlab_2bit_samples_sptr(
        size_t item_size,
        int nchannels);

    std::vector<int8_t> work_buffer_;
    size_t item_size_;
    int nchannels_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_NTLAB_2BIT_SAMPLES_H

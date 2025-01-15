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

#ifndef GNSS_SDR_UNPACK_SHORT_BYTE_SAMPLES_H
#define GNSS_SDR_UNPACK_SHORT_BYTE_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_short_byte_samples;

using unpack_short_byte_samples_sptr = gnss_shared_ptr<unpack_short_byte_samples>;

unpack_short_byte_samples_sptr make_unpack_short_byte_samples();

/*!
 * \brief This class implements conversion between short packet samples to byte samples
 *  1 short = 2 byte samples
 */
class unpack_short_byte_samples : public gr::sync_interpolator
{
public:
    unpack_short_byte_samples();
    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    ~unpack_short_byte_samples() = default;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend unpack_short_byte_samples_sptr make_unpack_short_byte_samples_sptr();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_SHORT_BYTE_SAMPLES_H

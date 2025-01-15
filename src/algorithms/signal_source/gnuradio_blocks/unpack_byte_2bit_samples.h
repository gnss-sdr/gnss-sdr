/*!
 * \file unpack_byte_2bit_samples.h
 *
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

#ifndef GNSS_SDR_UNPACK_BYTE_2BIT_SAMPLES_H
#define GNSS_SDR_UNPACK_BYTE_2BIT_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */

class unpack_byte_2bit_samples;

using unpack_byte_2bit_samples_sptr = gnss_shared_ptr<unpack_byte_2bit_samples>;

unpack_byte_2bit_samples_sptr make_unpack_byte_2bit_samples();

/*!
 * \brief This class implements conversion between byte packet samples to 2bit samples
 *  1 byte = 4 2bit samples
 */
class unpack_byte_2bit_samples : public gr::sync_interpolator
{
public:
    unpack_byte_2bit_samples();
    ~unpack_byte_2bit_samples() = default;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend unpack_byte_2bit_samples_sptr make_unpack_byte_2bit_samples_sptr();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_BYTE_2BIT_SAMPLES_H

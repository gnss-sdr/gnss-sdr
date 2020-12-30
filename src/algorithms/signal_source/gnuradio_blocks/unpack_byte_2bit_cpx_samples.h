/*!
 * \file unpack_byte_2bit_cpx_samples.h
 *
 * \brief Unpacks byte samples to 2 bits complex samples.
 *     Packing Order
 *     Most Significant Nibble  - Sample n
 *     Least Significant Nibble - Sample n+1
 *     Packing order in Nibble Q1 Q0 I1 I0
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

#ifndef GNSS_SDR_UNPACK_BYTE_2BIT_CPX_SAMPLES_H
#define GNSS_SDR_UNPACK_BYTE_2BIT_CPX_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_byte_2bit_cpx_samples;

using unpack_byte_2bit_cpx_samples_sptr = gnss_shared_ptr<unpack_byte_2bit_cpx_samples>;

unpack_byte_2bit_cpx_samples_sptr make_unpack_byte_2bit_cpx_samples();

/*!
 * \brief This class implements conversion between byte packet samples to 2bit_cpx samples
 *  1 byte = 2 x complex 2bit I, + 2bit Q samples
 */
class unpack_byte_2bit_cpx_samples : public gr::sync_interpolator
{
public:
    unpack_byte_2bit_cpx_samples();
    ~unpack_byte_2bit_cpx_samples() = default;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend unpack_byte_2bit_cpx_samples_sptr make_unpack_byte_2bit_cpx_samples_sptr();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_BYTE_2BIT_CPX_SAMPLES_H

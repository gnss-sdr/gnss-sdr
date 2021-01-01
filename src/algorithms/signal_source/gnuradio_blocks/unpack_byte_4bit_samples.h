/*!
 * \file unpack_byte_4bit_samples.h
 *
 * \brief Unpacks byte samples to 4 bits samples.
 *     Packing Order
 *     Packing order in Nibble I0 I1 I2 I3 I0 I1 I2 I3
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

#ifndef GNSS_SDR_UNPACK_BYTE_4BIT_SAMPLES_H
#define GNSS_SDR_UNPACK_BYTE_4BIT_SAMPLES_H

#include <gnuradio/sync_interpolator.h>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_byte_4bit_samples;

using unpack_byte_4bit_samples_sptr = std::shared_ptr<unpack_byte_4bit_samples>;

unpack_byte_4bit_samples_sptr make_unpack_byte_4bit_samples();

/*!
 * \brief This class implements conversion between byte packet samples to 4bit_cpx samples
 *  1 byte = 1 x complex 4bit I, + 4bit Q samples
 */
class unpack_byte_4bit_samples : public gr::sync_interpolator
{
public:
    unpack_byte_4bit_samples();
    ~unpack_byte_4bit_samples() = default;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend unpack_byte_4bit_samples_sptr make_unpack_byte_4bit_samples_sptr();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_BYTE_4BIT_SAMPLES_H

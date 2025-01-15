/*!
 * \file unpack_spir_gss6450_samples.h
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos, antonio.ramos(at)cttc.es
 * \author Javier Arribas jarribas (at) cttc.es
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

#ifndef GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H
#define GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_spir_gss6450_samples;

using unpack_spir_gss6450_samples_sptr = gnss_shared_ptr<unpack_spir_gss6450_samples>;

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(int adc_nbit_);


class unpack_spir_gss6450_samples : public gr::sync_interpolator
{
public:
    explicit unpack_spir_gss6450_samples(int adc_nbit);
    ~unpack_spir_gss6450_samples() = default;
    void decode_4bits_word(uint32_t input_uint32, gr_complex *out, int adc_bits_);
    int work(int noutput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples_sptr(int adc_nbit);
    int adc_bits;
    int samples_per_int;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

/*!
 * \file unpack_spir_gss6450_samples.h
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos, antonio.ramos(at)cttc.es
 * \author Javier Arribas jarribas (at) cttc.es
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H
#define GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_interpolator.h>
#include <cstdint>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class unpack_spir_gss6450_samples;

using unpack_spir_gss6450_samples_sptr = gnss_shared_ptr<unpack_spir_gss6450_samples>;

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(int adc_nbit_, bool gss6425_compatible = false);


class unpack_spir_gss6450_samples : public gr::sync_interpolator
{
public:
    explicit unpack_spir_gss6450_samples(int adc_nbit, bool gss6425_compatible = false);
    ~unpack_spir_gss6450_samples() = default;
    void decode_word(uint32_t input_uint32, gr_complex *out) const;
    int work(int noutput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(int adc_nbit_, bool gss6425_compatible);
    static int32_t sign_extend(uint32_t value, int bits);
    static int32_t decode_gss6425_2bit(uint32_t value);
    static int samples_per_gss6450_word(int adc_bits);

    int adc_bits_;
    int samples_per_int_;
    bool gss6425_compatible_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

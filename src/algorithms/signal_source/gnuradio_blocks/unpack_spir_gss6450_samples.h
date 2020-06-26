/*!
 * \file unpack_spir_gss6450_samples.h
 *
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos, antonio.ramos(at)cttc.es
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is not part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H
#define GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

#include <gnuradio/sync_interpolator.h>
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

class unpack_spir_gss6450_samples;

#if GNURADIO_USES_STD_POINTERS
using unpack_spir_gss6450_samples_sptr = std::shared_ptr<unpack_spir_gss6450_samples>;
#else
using unpack_spir_gss6450_samples_sptr = boost::shared_ptr<unpack_spir_gss6450_samples>;
#endif

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(unsigned int adc_nbit_);


class unpack_spir_gss6450_samples : public gr::sync_interpolator
{
public:
    explicit unpack_spir_gss6450_samples(unsigned int adc_nbit);
    ~unpack_spir_gss6450_samples() = default;
    void decode_4bits_word(uint32_t input_uint32, gr_complex *out, int adc_bits_);
    int work(int noutput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples_sptr(unsigned int adc_nbit);
    unsigned int adc_bits;
    unsigned int samples_per_int;
};

#endif  // GNSS_SDR_UNPACK_SPIR_GSS6450_SAMPLES_H

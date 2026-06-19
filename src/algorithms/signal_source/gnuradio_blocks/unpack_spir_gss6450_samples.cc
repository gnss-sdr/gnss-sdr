/*!
 * \file unpack_spir_gss6450_samples.cc
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos,  antonio(at)cttc.es
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


#include "unpack_spir_gss6450_samples.h"
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <stdexcept>


unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(int adc_nbit_, bool gss6425_compatible)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(adc_nbit_, gss6425_compatible));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(int adc_nbit, bool gss6425_compatible)
    : gr::sync_interpolator("unpack_spir_gss6450_samples",
          gr::io_signature::make(1, 1, sizeof(int32_t)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)), samples_per_gss6450_word(adc_nbit)),
      adc_bits_(adc_nbit),
      samples_per_int_(samples_per_gss6450_word(adc_bits_)),
      gss6425_compatible_(gss6425_compatible && adc_bits_ == 2)
{
}


void unpack_spir_gss6450_samples::decode_word(uint32_t input_uint32, gr_complex* out) const
{
    const uint32_t mask = (uint32_t{1} << adc_bits_) - 1U;
    const int sample_bits = 2 * adc_bits_;

    for (int sample = 0; sample < samples_per_int_; sample++)
        {
            const int shift = 32 - ((sample + 1) * sample_bits);
            const uint32_t q = (input_uint32 >> shift) & mask;
            const uint32_t i = (input_uint32 >> (shift + adc_bits_)) & mask;

            if (gss6425_compatible_)
                {
                    out[sample] = gr_complex(static_cast<float>(decode_gss6425_2bit(i)),
                        static_cast<float>(decode_gss6425_2bit(q)));
                }
            else
                {
                    out[sample] = gr_complex(static_cast<float>(sign_extend(i, adc_bits_)),
                        static_cast<float>(sign_extend(q, adc_bits_)));
                }
        }
}


int32_t unpack_spir_gss6450_samples::sign_extend(uint32_t value, int bits)
{
    const auto sign_bit = uint32_t{1} << (bits - 1);
    const auto full_scale = uint32_t{1} << bits;
    if ((value & sign_bit) != 0)
        {
            return static_cast<int32_t>(value - full_scale);
        }
    return static_cast<int32_t>(value);
}


int32_t unpack_spir_gss6450_samples::decode_gss6425_2bit(uint32_t value)
{
    switch (value & 0x03U)
        {
        case 0x00U:
            return 1;
        case 0x01U:
            return 3;
        case 0x02U:
            return -3;
        default:
            return -1;
        }
}


int unpack_spir_gss6450_samples::samples_per_gss6450_word(int adc_bits)
{
    if (adc_bits != 2 && adc_bits != 4 && adc_bits != 8 && adc_bits != 16)
        {
            throw std::invalid_argument("GSS6450 ADC bits must be one of 2, 4, 8 or 16");
        }
    return 16 / adc_bits;
}


int unpack_spir_gss6450_samples::work(int noutput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
{
    const auto* in = reinterpret_cast<const int32_t*>(input_items[0]);
    auto* out = reinterpret_cast<gr_complex*>(output_items[0]);
    int n_sample = 0;
    int in_counter = 0;
    do
        {
            decode_word(static_cast<uint32_t>(in[in_counter++]), &out[n_sample]);
            n_sample += samples_per_int_;
        }
    while (n_sample < noutput_items);

    return noutput_items;
}

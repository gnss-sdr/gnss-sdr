/*!
 * \file unpack_spir_gss6450_samples.cc
 * \brief Unpacks SPIR int samples
 * \author Antonio Ramos,  antonio(at)cttc.es
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


#include "unpack_spir_gss6450_samples.h"
#include <gnuradio/io_signature.h>
#include <cmath>

unpack_spir_gss6450_samples_sptr make_unpack_spir_gss6450_samples(int adc_nbit_)
{
    return unpack_spir_gss6450_samples_sptr(new unpack_spir_gss6450_samples(adc_nbit_));
}


unpack_spir_gss6450_samples::unpack_spir_gss6450_samples(int adc_nbit)
    : gr::sync_interpolator("unpack_spir_gss6450_samples",
          gr::io_signature::make(1, 1, sizeof(int32_t)),
          gr::io_signature::make(1, 1, sizeof(gr_complex)), 16 / adc_nbit),
      adc_bits(adc_nbit),
      samples_per_int(16 / adc_bits)
{
}


void unpack_spir_gss6450_samples::decode_4bits_word(uint32_t input_uint32, gr_complex* out, int adc_bits_)
{
    int8_t tmp_char;
    float Q;
    float I;
    switch (adc_bits_)
        {
        case 2:
            // four bits per complex sample (2 I + 2 Q), 8 samples per int32[s0,s1,s2,s3,s4,s5,s6,s7]
            for (int i = 0; i < 8; i++)
                {
                    tmp_char = input_uint32 & 3;

                    if (tmp_char >= 2)
                        {
                            I = static_cast<float>(tmp_char - 4);
                        }
                    else
                        {
                            I = static_cast<float>(tmp_char);
                        }
                    input_uint32 = input_uint32 >> 2;
                    tmp_char = input_uint32 & 3;
                    if (tmp_char >= 2)
                        {
                            Q = static_cast<float>(tmp_char - 4);
                        }
                    else
                        {
                            Q = static_cast<float>(tmp_char);
                        }
                    input_uint32 = input_uint32 >> 2;

                    out[7 - i] = gr_complex(I, Q);
                }
            break;
        case 4:
            // eight bits per complex sample (4 I + 4 Q), 4 samples per int32= [s0,s1,s2,s3]
            for (int i = 0; i < 4; i++)
                {
                    tmp_char = input_uint32 & 0x0F;

                    if (tmp_char >= 8)
                        {
                            I = static_cast<float>(tmp_char - 16);
                        }
                    else
                        {
                            I = static_cast<float>(tmp_char);
                        }
                    input_uint32 = input_uint32 >> 4;
                    tmp_char = input_uint32 & 0x0F;
                    if (tmp_char >= 8)
                        {
                            Q = static_cast<float>(tmp_char - 16);
                        }
                    else
                        {
                            Q = static_cast<float>(tmp_char);
                        }
                    input_uint32 = input_uint32 >> 4;

                    out[3 - i] = gr_complex(I, Q);
                }
            break;
        }
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
            decode_4bits_word(in[in_counter++], &out[n_sample], adc_bits);
            n_sample += samples_per_int;
        }
    while (n_sample < noutput_items);

    return noutput_items;
}

/*!
 * \file direct_resampler_conditioner_cc.cc
 * \brief Nearest neighborhood resampler with
 *        gr_complex input and gr_complex output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
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


#include "direct_resampler_conditioner_cc.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>  // for gr_complex
#include <algorithm>    // for min

direct_resampler_conditioner_cc_sptr direct_resampler_make_conditioner_cc(
    double sample_freq_in, double sample_freq_out)
{
    return direct_resampler_conditioner_cc_sptr(
        new direct_resampler_conditioner_cc(sample_freq_in,
            sample_freq_out));
}


direct_resampler_conditioner_cc::direct_resampler_conditioner_cc(
    double sample_freq_in,
    double sample_freq_out)
    : gr::block("direct_resampler_conditioner_cc",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_sample_freq_in(sample_freq_in),
      d_sample_freq_out(sample_freq_out),
      d_phase(0),
      d_lphase(0)
{
    // Computes the phase step multiplying the resampling ratio by 2^32 = 4294967296
    const double two_32 = 4294967296.0;
    if (d_sample_freq_in >= d_sample_freq_out)
        {
            d_phase_step = static_cast<uint32_t>(floor(two_32 * sample_freq_out / sample_freq_in));
        }
    else
        {
            d_phase_step = static_cast<uint32_t>(floor(two_32 * sample_freq_in / sample_freq_out));
        }
#ifdef GR_GREATER_38
    this->set_relative_rate(static_cast<uint64_t>(sample_freq_out), static_cast<uint64_t>(sample_freq_in));
#else
    this->set_relative_rate(sample_freq_out / sample_freq_in);
#endif
}


void direct_resampler_conditioner_cc::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    int nreqd = std::max(static_cast<unsigned>(1), static_cast<int>(static_cast<double>(noutput_items + 1) * sample_freq_in() / sample_freq_out()) + history() - 1);
    unsigned ninputs = ninput_items_required.size();
    for (unsigned i = 0; i < ninputs; i++)
        {
            ninput_items_required[i] = nreqd;
        }
}


int direct_resampler_conditioner_cc::general_work(int noutput_items,
    gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);

    int lcv = 0;
    int count = 0;

    if (d_sample_freq_in >= d_sample_freq_out)
        {
            while ((lcv < noutput_items))
                {
                    if (d_phase <= d_lphase)
                        {
                            out[lcv] = *in;
                            lcv++;
                        }
                    d_lphase = d_phase;
                    d_phase += d_phase_step;
                    in++;
                    count++;
                }
        }
    else
        {
            while ((lcv < noutput_items))
                {
                    d_lphase = d_phase;
                    d_phase += d_phase_step;
                    if (d_phase <= d_lphase)
                        {
                            in++;
                            count++;
                        }
                    out[lcv] = *in;
                    lcv++;
                }
        }

    consume_each(std::min(count, ninput_items[0]));
    return lcv;
}

/*!
 * \file pulse_blanking_cc.cc
 * \brief Implements a pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
 *         Antonio Ramos  (antonio.ramosdet(at)gmail.com)
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017 (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "pulse_blanking_cc.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <cmath>

using google::LogMessage;

pulse_blanking_cc_sptr make_pulse_blanking_cc(float pfa, int length_,
    int n_segments_est, int n_segments_reset)
{
    return pulse_blanking_cc_sptr(new pulse_blanking_cc(pfa, length_, n_segments_est, n_segments_reset));
}


pulse_blanking_cc::pulse_blanking_cc(float pfa, int length_, int n_segments_est, int n_segments_reset) : gr::block("pulse_blanking_cc",
                                                                                                             gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                                                                                             gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    this->pfa = pfa;
    this->length_ = length_;
    last_filtered = false;
    n_segments = 0;
    this->n_segments_est = n_segments_est;
    this->n_segments_reset = n_segments_reset;
    noise_power_estimation = 0.0;
    n_deg_fred = 2 * length_;
    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa));
    zeros_ = static_cast<gr_complex *>(volk_malloc(length_ * sizeof(gr_complex), volk_get_alignment()));
    for (int aux = 0; aux < length_; aux++)
        {
            zeros_[aux] = gr_complex(0, 0);
        }
}


pulse_blanking_cc::~pulse_blanking_cc()
{
    volk_free(zeros_);
}

void pulse_blanking_cc::forecast(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items_required)
{
    for (unsigned int aux = 0; aux < ninput_items_required.size(); aux++)
        {
            ninput_items_required[aux] = length_;
        }
}

int pulse_blanking_cc::general_work(int noutput_items, gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);
    float *magnitude = static_cast<float *>(volk_malloc(noutput_items * sizeof(float), volk_get_alignment()));
    volk_32fc_magnitude_squared_32f(magnitude, in, noutput_items);
    int sample_index = 0;
    float segment_energy;
    while ((sample_index + length_) < noutput_items)
        {
            volk_32f_accumulator_s32f(&segment_energy, (magnitude + sample_index), length_);
            if ((n_segments < n_segments_est) && (last_filtered == false))
                {
                    noise_power_estimation = (static_cast<float>(n_segments) * noise_power_estimation + segment_energy / static_cast<float>(n_deg_fred)) / static_cast<float>(n_segments + 1);
                    memcpy(out, in, sizeof(gr_complex) * length_);
                }
            else
                {
                    if ((segment_energy / noise_power_estimation) > thres_)
                        {
                            memcpy(out, zeros_, sizeof(gr_complex) * length_);
                            last_filtered = true;
                        }
                    else
                        {
                            memcpy(out, in, sizeof(gr_complex) * length_);
                            last_filtered = false;
                            if (n_segments > n_segments_reset)
                                {
                                    n_segments = 0;
                                }
                        }
                }
            in += length_;
            out += length_;
            sample_index += length_;
            n_segments++;
        }
    volk_free(magnitude);
    consume_each(sample_index);
    return sample_index;
}

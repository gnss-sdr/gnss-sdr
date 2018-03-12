/*!
 * \file notch_lite_cc.cc
 * \brief Implements a multi state notch filter algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
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

#include "notch_lite_cc.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <cstring>
#include <cmath>

using google::LogMessage;

notch_lite_sptr make_notch_filter_lite(float p_c_factor, float pfa, int length_, int n_segments_est, int n_segments_reset, int n_segments_coeff)
{
    return notch_lite_sptr(new NotchLite(p_c_factor, pfa, length_, n_segments_est, n_segments_reset, n_segments_coeff));
}

NotchLite::NotchLite(float p_c_factor, float pfa, int length_, int n_segments_est, int n_segments_reset, int n_segments_coeff) : gr::block("NotchLite",
                                                                                                                                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                                                                                                                     gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    set_history(2);
    this->p_c_factor = gr_complex(p_c_factor, 0);
    this->n_segments_est = n_segments_est;
    this->n_segments_reset = n_segments_reset;
    this->n_segments_coeff_reset = n_segments_coeff;
    this->n_segments_coeff = 0;
    this->length_ = length_;
    set_output_multiple(length_);
    this->pfa = pfa;
    n_segments = 0;
    n_deg_fred = 2 * length_;
    noise_pow_est = 0.0;
    filter_state_ = false;
    z_0 = gr_complex(0, 0);
    last_out = gr_complex(0, 0);
    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa));
    c_samples1 = gr_complex(0, 0);
    c_samples2 = gr_complex(0, 0);
    angle1 = 0.0;
    angle2 = 0.0;
    power_spect = static_cast<float *>(volk_malloc(length_ * sizeof(float), volk_get_alignment()));
    d_fft = std::unique_ptr<gr::fft::fft_complex>(new gr::fft::fft_complex(length_, true));
}

NotchLite::~NotchLite()
{
    volk_free(power_spect);
}

void NotchLite::forecast(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items_required)
{
    for (unsigned int aux = 0; aux < ninput_items_required.size(); aux++)
        {
            ninput_items_required[aux] = length_;
        }
}

int NotchLite::general_work(int noutput_items, gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int index_out = 0;
    float sig2dB = 0.0;
    float sig2lin = 0.0;
    lv_32fc_t dot_prod_;
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);
    in++;
    while ((index_out + length_) < noutput_items)
        {
            if ((n_segments < n_segments_est) && (filter_state_ == false))
                {
                    memcpy(d_fft->get_inbuf(), in, sizeof(gr_complex) * length_);
                    d_fft->execute();
                    volk_32fc_s32f_power_spectrum_32f(power_spect, d_fft->get_outbuf(), 1.0, length_);
                    volk_32f_s32f_calc_spectral_noise_floor_32f(&sig2dB, power_spect, 15.0, length_);
                    sig2lin = std::pow(10.0, (sig2dB / 10.0)) / static_cast<float>(n_deg_fred);
                    noise_pow_est = (static_cast<float>(n_segments) * noise_pow_est + sig2lin) / static_cast<float>(n_segments + 1);
                    memcpy(out, in, sizeof(gr_complex) * length_);
                }
            else
                {
                    volk_32fc_x2_conjugate_dot_prod_32fc(&dot_prod_, in, in, length_);
                    if ((lv_creal(dot_prod_) / noise_pow_est) > thres_)
                        {
                            if (filter_state_ == false)
                                {
                                    filter_state_ = true;
                                    last_out = gr_complex(0, 0);
                                    n_segments_coeff = 0;
                                }
                            if (n_segments_coeff == 0)
                                {
                                    volk_32fc_x2_multiply_conjugate_32fc(&c_samples1, (in + 1), in, 1);
                                    volk_32fc_s32f_atan2_32f(&angle1, &c_samples1, static_cast<float>(1.0), 1);
                                    volk_32fc_x2_multiply_conjugate_32fc(&c_samples2, (in + length_ - 1), (in + length_ - 2), 1);
                                    volk_32fc_s32f_atan2_32f(&angle2, &c_samples2, static_cast<float>(1.0), 1);
                                    float angle_ = (angle1 + angle2) / 2.0;
                                    z_0 = std::exp(gr_complex(0, 1) * angle_);
                                }
                            for (int aux = 0; aux < length_; aux++)
                                {
                                    *(out + aux) = *(in + aux) - z_0 * (*(in + aux - 1)) + p_c_factor * z_0 * last_out;
                                    last_out = *(out + aux);
                                }
                            n_segments_coeff++;
                            n_segments_coeff = n_segments_coeff % n_segments_coeff_reset;
                        }
                    else
                        {
                            if (n_segments > n_segments_reset)
                                {
                                    n_segments = 0;
                                }
                            filter_state_ = false;
                            memcpy(out, in, sizeof(gr_complex) * length_);
                        }
                }
            index_out += length_;
            n_segments++;
            in += length_;
            out += length_;
        }
    consume_each(index_out);
    return index_out;
}

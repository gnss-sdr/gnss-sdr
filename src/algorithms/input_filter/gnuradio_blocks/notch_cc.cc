/*!
 * \file notch_cc.cc
 * \brief Implements a multi state notch filter algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "notch_cc.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>
#include <cmath>


notch_sptr make_notch_filter(float pfa, float p_c_factor,
    int32_t length, int32_t n_segments_est, int32_t n_segments_reset)
{
    return notch_sptr(new Notch(pfa, p_c_factor, length, n_segments_est, n_segments_reset));
}


Notch::Notch(float pfa,
    float p_c_factor,
    int32_t length,
    int32_t n_segments_est,
    int32_t n_segments_reset)
    : gr::block("Notch",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      last_out_(gr_complex(0.0, 0.0)),
      z_0_(gr_complex(0.0, 0.0)),
      p_c_factor_(gr_complex(p_c_factor, 0.0)),
      pfa_(pfa),
      noise_pow_est_(0.0),
      length_(length),          // Set the number of samples per segment
      n_deg_fred_(2 * length),  // Number of dregrees of freedom,
      n_segments_(0),
      n_segments_est_(n_segments_est),      // Set the number of segments for noise power estimation
      n_segments_reset_(n_segments_reset),  // Set the period (in segments) when the noise power is estimated
      filter_state_(false)
{
    const int32_t alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred_);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa_));
    c_samples_ = volk_gnsssdr::vector<gr_complex>(length_);
    angle_ = volk_gnsssdr::vector<float>(length_);
    power_spect_ = volk_gnsssdr::vector<float>(length_);
    d_fft_ = gnss_fft_fwd_make_unique(length_);
}


int Notch::general_work(int noutput_items, gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t index_out = 0;
    float sig2dB = 0.0;
    float sig2lin = 0.0;
    lv_32fc_t dot_prod_;
    const auto *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    in++;
    while ((index_out + length_) < noutput_items)
        {
            if ((n_segments_ < n_segments_est_) && (filter_state_ == false))
                {
                    std::copy(in, in + length_, d_fft_->get_inbuf());
                    d_fft_->execute();
                    volk_32fc_s32f_power_spectrum_32f(power_spect_.data(), d_fft_->get_outbuf(), 1.0, length_);
                    volk_32f_s32f_calc_spectral_noise_floor_32f(&sig2dB, power_spect_.data(), 15.0, length_);
                    sig2lin = std::pow(10.0F, (sig2dB / 10.0F)) / (static_cast<float>(n_deg_fred_));
                    noise_pow_est_ = (static_cast<float>(n_segments_) * noise_pow_est_ + sig2lin) / (static_cast<float>(n_segments_ + 1));
                    std::copy(in, in + length_, out);
                }
            else
                {
                    volk_32fc_x2_conjugate_dot_prod_32fc(&dot_prod_, in, in, length_);
                    if ((lv_creal(dot_prod_) / noise_pow_est_) > thres_)
                        {
                            if (filter_state_ == false)
                                {
                                    filter_state_ = true;
                                    last_out_ = gr_complex(0.0, 0.0);
                                }
                            volk_32fc_x2_multiply_conjugate_32fc(c_samples_.data(), in, (in - 1), length_);
                            volk_32fc_s32f_atan2_32f(angle_.data(), c_samples_.data(), static_cast<float>(1.0), length_);
                            for (int32_t aux = 0; aux < length_; aux++)
                                {
                                    z_0_ = std::exp(gr_complex(0.0, 1.0) * (*(angle_.data() + aux)));
                                    *(out + aux) = *(in + aux) - z_0_ * (*(in + aux - 1)) + p_c_factor_ * z_0_ * last_out_;
                                    last_out_ = *(out + aux);
                                }
                        }
                    else
                        {
                            if (n_segments_ > n_segments_reset_)
                                {
                                    n_segments_ = 0;
                                }
                            filter_state_ = false;
                            std::copy(in, in + length_, out);
                        }
                }
            index_out += length_;
            n_segments_++;
            in += length_;
            out += length_;
        }
    consume_each(index_out);
    return index_out;
}

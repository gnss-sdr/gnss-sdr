/*!
 * \file notch_lite_cc.cc
 * \brief Implements a multi state notch filter algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
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
 *
 */

#include "notch_lite_cc.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>
#include <cmath>


notch_lite_sptr make_notch_filter_lite(float p_c_factor, float pfa, int32_t length, int32_t n_segments_est, int32_t n_segments_reset, int32_t n_segments_coeff)
{
    return notch_lite_sptr(new NotchLite(p_c_factor, pfa, length, n_segments_est, n_segments_reset, n_segments_coeff));
}


NotchLite::NotchLite(float p_c_factor,
    float pfa,
    int32_t length,
    int32_t n_segments_est,
    int32_t n_segments_reset,
    int32_t n_segments_coeff)
    : gr::block("NotchLite",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      last_out_(gr_complex(0.0, 0.0)),
      z_0_(gr_complex(0.0, 0.0)),
      p_c_factor_(gr_complex(p_c_factor, 0.0)),
      c_samples1_(gr_complex(0.0, 0.0)),
      c_samples2_(gr_complex(0.0, 0.0)),
      pfa_(pfa),
      noise_pow_est_(0.0),
      angle1_(0.0),
      angle2_(0.0),
      length_(length),
      n_segments_(0),
      n_segments_est_(n_segments_est),
      n_segments_reset_(n_segments_reset),
      n_segments_coeff_reset_(n_segments_coeff),
      n_segments_coeff_(0),
      n_deg_fred_(2 * length),
      filter_state_(false)
{
    const int32_t alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    set_history(2);

    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred_);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa_));

    power_spect_ = volk_gnsssdr::vector<float>(length_);
    d_fft_ = gnss_fft_fwd_make_unique(length_);
}


int NotchLite::general_work(int noutput_items, gr_vector_int &ninput_items __attribute__((unused)),
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
                    sig2lin = std::pow(10.0F, (sig2dB / 10.0F)) / static_cast<float>(n_deg_fred_);
                    noise_pow_est_ = (static_cast<float>(n_segments_) * noise_pow_est_ + sig2lin) / static_cast<float>(n_segments_ + 1);
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
                                    last_out_ = gr_complex(0, 0);
                                    n_segments_coeff_ = 0;
                                }
                            if (n_segments_coeff_ == 0)
                                {
                                    volk_32fc_x2_multiply_conjugate_32fc(&c_samples1_, (in + 1), in, 1);
                                    volk_32fc_s32f_atan2_32f(&angle1_, &c_samples1_, static_cast<float>(1.0), 1);
                                    volk_32fc_x2_multiply_conjugate_32fc(&c_samples2_, (in + length_ - 1), (in + length_ - 2), 1);
                                    volk_32fc_s32f_atan2_32f(&angle2_, &c_samples2_, static_cast<float>(1.0), 1);
                                    float angle_ = (angle1_ + angle2_) / 2.0F;
                                    z_0_ = std::exp(gr_complex(0, 1) * angle_);
                                }
                            for (int32_t aux = 0; aux < length_; aux++)
                                {
                                    *(out + aux) = *(in + aux) - z_0_ * (*(in + aux - 1)) + p_c_factor_ * z_0_ * last_out_;
                                    last_out_ = *(out + aux);
                                }
                            n_segments_coeff_++;
                            n_segments_coeff_ = n_segments_coeff_ % n_segments_coeff_reset_;
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

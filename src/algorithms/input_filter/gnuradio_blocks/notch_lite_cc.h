/*!
 * \file notch_lite_cc.h
 * \brief Implements a notch filter light algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NOTCH_LITE_H
#define GNSS_SDR_NOTCH_LITE_H

#if GNURADIO_USES_STD_POINTERS
#else
#include <boost/shared_ptr.hpp>
#endif
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstdint>
#include <memory>

class NotchLite;

#if GNURADIO_USES_STD_POINTERS
using notch_lite_sptr = std::shared_ptr<NotchLite>;
#else
using notch_lite_sptr = boost::shared_ptr<NotchLite>;
#endif

notch_lite_sptr make_notch_filter_lite(
    float p_c_factor,
    float pfa,
    int32_t length_,
    int32_t n_segments_est,
    int32_t n_segments_reset,
    int32_t n_segments_coeff);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter light version
 */
class NotchLite : public gr::block
{
public:
    ~NotchLite() = default;

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend notch_lite_sptr make_notch_filter_lite(float p_c_factor, float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset, int32_t n_segments_coeff);
    NotchLite(float p_c_factor, float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset, int32_t n_segments_coeff);
    std::unique_ptr<gr::fft::fft_complex> d_fft;
    volk_gnsssdr::vector<float> power_spect;
    gr_complex last_out;
    gr_complex z_0;
    gr_complex p_c_factor;
    gr_complex c_samples1;
    gr_complex c_samples2;
    float pfa;
    float thres_;
    float noise_pow_est;
    float angle1;
    float angle2;
    int32_t length_;
    int32_t n_segments;
    int32_t n_segments_est;
    int32_t n_segments_reset;
    int32_t n_segments_coeff_reset;
    int32_t n_segments_coeff;
    int32_t n_deg_fred;
    bool filter_state_;
};

#endif  // GNSS_SDR_NOTCH_LITE_H

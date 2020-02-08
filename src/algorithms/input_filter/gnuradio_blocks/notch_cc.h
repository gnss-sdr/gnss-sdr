/*!
 * \file notch_cc.h
 * \brief Implements a notch filter algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
 * -------------------------------------------------------------------------
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
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NOTCH_H
#define GNSS_SDR_NOTCH_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <cstdint>
#include <memory>

class Notch;

using notch_sptr = boost::shared_ptr<Notch>;

notch_sptr make_notch_filter(
    float pfa,
    float p_c_factor,
    int32_t length_,
    int32_t n_segments_est,
    int32_t n_segments_reset);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter
 */
class Notch : public gr::block
{
public:
    ~Notch();

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend notch_sptr make_notch_filter(float pfa, float p_c_factor, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);
    Notch(float pfa, float p_c_factor, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);
    float pfa;
    float noise_pow_est;
    float thres_;
    int32_t length_;
    int32_t n_deg_fred;
    uint32_t n_segments;
    uint32_t n_segments_est;
    uint32_t n_segments_reset;
    bool filter_state_;
    gr_complex last_out;
    gr_complex z_0;
    gr_complex p_c_factor;
    gr_complex *c_samples;
    float *angle_;
    float *power_spect;
    std::unique_ptr<gr::fft::fft_complex> d_fft;
};

#endif  // GNSS_SDR_NOTCH_H

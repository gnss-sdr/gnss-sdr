/*!
 * \file pulse_blanking_cc.h
 * \brief Implements a pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
 *         Antonio Ramos  (antonio.ramosdet(at)gmail.com)
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

#ifndef GNSS_SDR_PULSE_BLANKING_H_
#define GNSS_SDR_PULSE_BLANKING_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <cstdint>

class pulse_blanking_cc;

using pulse_blanking_cc_sptr = boost::shared_ptr<pulse_blanking_cc>;

pulse_blanking_cc_sptr make_pulse_blanking_cc(
    float pfa,
    int32_t length_,
    int32_t n_segments_est,
    int32_t n_segments_reset);

class pulse_blanking_cc : public gr::block
{
public:
    ~pulse_blanking_cc();

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend pulse_blanking_cc_sptr make_pulse_blanking_cc(float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);
    pulse_blanking_cc(float pfa, int32_t length_, int32_t n_segments_est, int32_t n_segments_reset);
    int32_t length_;
    int32_t n_segments;
    int32_t n_segments_est;
    int32_t n_segments_reset;
    int32_t n_deg_fred;
    bool last_filtered;
    float noise_power_estimation;
    float thres_;
    float pfa;
    gr_complex *zeros_;
};

#endif  // GNSS_SDR_PULSE_BLANKING_H_

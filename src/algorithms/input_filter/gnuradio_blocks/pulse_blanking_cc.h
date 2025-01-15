/*!
 * \file pulse_blanking_cc.h
 * \brief Implements a pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
 *         Antonio Ramos  (antonio.ramosdet(at)gmail.com)
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

#ifndef GNSS_SDR_PULSE_BLANKING_CC_H
#define GNSS_SDR_PULSE_BLANKING_CC_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstdint>

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_gnuradio_blocks input_filter_gr_blocks
 * GNU Radio blocks implementing input filters,
 * \{ */


class pulse_blanking_cc;

using pulse_blanking_cc_sptr = gnss_shared_ptr<pulse_blanking_cc>;

pulse_blanking_cc_sptr make_pulse_blanking_cc(
    float pfa,
    int32_t length,
    int32_t n_segments_est,
    int32_t n_segments_reset);

class pulse_blanking_cc : public gr::block
{
public:
    ~pulse_blanking_cc() = default;

    int general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend pulse_blanking_cc_sptr make_pulse_blanking_cc(float pfa, int32_t length, int32_t n_segments_est, int32_t n_segments_reset);
    pulse_blanking_cc(float pfa, int32_t length, int32_t n_segments_est, int32_t n_segments_reset);
    volk_gnsssdr::vector<gr_complex> zeros_;
    float noise_power_estimation_;
    float thres_;
    float pfa_;
    int32_t length_;
    int32_t n_segments_;
    int32_t n_segments_est_;
    int32_t n_segments_reset_;
    int32_t n_deg_fred_;
    bool last_filtered_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PULSE_BLANKING_CC_H

/*!
 * \file notch_cc.h
 * \brief Implements a notch filter algorithm
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

#ifndef GNSS_SDR_NOTCH_CC_H
#define GNSS_SDR_NOTCH_CC_H

#include "gnss_block_interface.h"
#include "gnss_sdr_fft.h"
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstdint>
#include <memory>

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_gnuradio_blocks
 * \{ */


class Notch;

using notch_sptr = gnss_shared_ptr<Notch>;

notch_sptr make_notch_filter(
    float pfa,
    float p_c_factor,
    int32_t length,
    int32_t n_segments_est,
    int32_t n_segments_reset);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter
 */
class Notch : public gr::block
{
public:
    ~Notch() = default;

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend notch_sptr make_notch_filter(float pfa, float p_c_factor, int32_t length, int32_t n_segments_est, int32_t n_segments_reset);
    Notch(float pfa, float p_c_factor, int32_t length, int32_t n_segments_est, int32_t n_segments_reset);

    std::unique_ptr<gnss_fft_complex_fwd> d_fft_;
    volk_gnsssdr::vector<gr_complex> c_samples_;
    volk_gnsssdr::vector<float> angle_;
    volk_gnsssdr::vector<float> power_spect_;
    gr_complex last_out_;
    gr_complex z_0_;
    gr_complex p_c_factor_;
    float pfa_;
    float noise_pow_est_;
    float thres_;
    int32_t length_;
    int32_t n_deg_fred_;
    uint32_t n_segments_;
    uint32_t n_segments_est_;
    uint32_t n_segments_reset_;
    bool filter_state_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NOTCH_CC_H

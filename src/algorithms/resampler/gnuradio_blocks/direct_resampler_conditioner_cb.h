/*!
 * \file direct_resampler_conditioner_cb.h
 * \brief Nearest neighborhood resampler with
 *        std::complex<signed char> input and std::complex<signed char> output
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

#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CB_H
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CB_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <cstdint>

/** \addtogroup Resampler
 * \{ */
/** \addtogroup Resampler_gnuradio_blocks
 * GNU Radio blocks for input signal resampling
 * \{ */


class direct_resampler_conditioner_cb;

using direct_resampler_conditioner_cb_sptr = gnss_shared_ptr<direct_resampler_conditioner_cb>;

direct_resampler_conditioner_cb_sptr direct_resampler_make_conditioner_cb(
    double sample_freq_in,
    double sample_freq_out);

/*!
 * \brief This class implements a direct resampler conditioner for std::complex<signed char>
 *
 * Direct resampling without interpolation
 */
class direct_resampler_conditioner_cb : public gr::block
{
public:
    ~direct_resampler_conditioner_cb() = default;

    inline unsigned int sample_freq_in() const
    {
        return d_sample_freq_in;
    }

    inline unsigned int sample_freq_out() const
    {
        return d_sample_freq_out;
    }

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend direct_resampler_conditioner_cb_sptr direct_resampler_make_conditioner_cb(
        double sample_freq_in,
        double sample_freq_out);

    direct_resampler_conditioner_cb(
        double sample_freq_in,
        double sample_freq_out);

    double d_sample_freq_in;
    double d_sample_freq_out;
    uint32_t d_phase;
    uint32_t d_lphase;
    uint32_t d_phase_step;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CB_H

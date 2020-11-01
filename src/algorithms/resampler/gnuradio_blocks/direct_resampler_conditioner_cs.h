/*!
 * \file direct_resampler_conditioner_cs.h
 * \brief Nearest neighborhood resampler with
 *        std::complex<short> input and std::complex<short> output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H

#include <gnuradio/block.h>
#include <cstdint>
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

/** \addtogroup Resampler
 * \{ */
/** \addtogroup Resampler_gnuradio_blocks
 * \{ */


class direct_resampler_conditioner_cs;
#if GNURADIO_USES_STD_POINTERS
using direct_resampler_conditioner_cs_sptr = std::shared_ptr<direct_resampler_conditioner_cs>;
#else
using direct_resampler_conditioner_cs_sptr = boost::shared_ptr<direct_resampler_conditioner_cs>;
#endif

direct_resampler_conditioner_cs_sptr direct_resampler_make_conditioner_cs(
    double sample_freq_in,
    double sample_freq_out);

/*!
 * \brief This class implements a direct resampler conditioner for std::complex<short>
 *
 * Direct resampling without interpolation
 */
class direct_resampler_conditioner_cs : public gr::block
{
public:
    ~direct_resampler_conditioner_cs() = default;

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
    friend direct_resampler_conditioner_cs_sptr direct_resampler_make_conditioner_cs(
        double sample_freq_in,
        double sample_freq_out);

    direct_resampler_conditioner_cs(
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
#endif  // GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H

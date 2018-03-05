/*!
 * \file direct_resampler_conditioner_cc.h
 *
 * \brief Nearest neighborhood resampler with
 *        gr_complex input and gr_complex output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * This block takes in a signal stream and performs direct
 * resampling.
 * The theory behind this block can be found in Chapter 7.5 of
 * the following book.
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CC_H
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CC_H

#include <gnuradio/block.h>
#include <volk/volk.h>

class direct_resampler_conditioner_cc;
typedef boost::shared_ptr<direct_resampler_conditioner_cc> direct_resampler_conditioner_cc_sptr;
direct_resampler_conditioner_cc_sptr
direct_resampler_make_conditioner_cc(double sample_freq_in,
    double sample_freq_out);

/*!
 * \brief This class implements a direct resampler conditioner for complex data
 *
 * Direct resampling without interpolation
 */
class direct_resampler_conditioner_cc : public gr::block
{
private:
    friend direct_resampler_conditioner_cc_sptr
    direct_resampler_make_conditioner_cc(double sample_freq_in,
        double sample_freq_out);
    double d_sample_freq_in;   //! Specifies the sampling frequency of the input signal
    double d_sample_freq_out;  //! Specifies the sampling frequency of the output signal
    uint32_t d_phase;
    uint32_t d_lphase;
    uint32_t d_phase_step;
    unsigned int d_history;
    direct_resampler_conditioner_cc(double sample_freq_in,
        double sample_freq_out);

public:
    ~direct_resampler_conditioner_cc();
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
};

#endif /* GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CC_H */

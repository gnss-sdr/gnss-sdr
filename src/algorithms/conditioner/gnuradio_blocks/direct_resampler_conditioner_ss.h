/*!
 * \file direct_resampler_conditioner_ss.h
 * \brief Direct resampler with
 *        short input and short output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef INCLUDED_DIRECT_RESAMPLER_CONDITIONER_SS_H
#define	INCLUDED_DIRECT_RESAMPLER_CONDITIONER_SS_H

#include <gnuradio/gr_block.h>

class direct_resampler_conditioner_ss;
typedef boost::shared_ptr<direct_resampler_conditioner_ss>
        direct_resampler_conditioner_ss_sptr;
direct_resampler_conditioner_ss_sptr
direct_resampler_make_conditioner_ss(double sample_freq_in,
        double sample_freq_out);

class direct_resampler_conditioner_ss: public gr_block
{

private:

    friend direct_resampler_conditioner_ss_sptr
    direct_resampler_make_conditioner_ss(double sample_freq_in,
            double sample_freq_out);

    double d_sample_freq_in;
    double d_sample_freq_out;
    unsigned int d_phase;
    unsigned int d_lphase;
    unsigned int d_phase_step;
    unsigned int d_history;

    direct_resampler_conditioner_ss(double sample_freq_in,
            double sample_freq_out);

public:

    ~direct_resampler_conditioner_ss();

    unsigned int sample_freq_in() const
    {
        return d_sample_freq_in;
    }
    unsigned int sample_freq_out() const
    {
        return d_sample_freq_out;
    }
    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    int general_work(int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif /* INCLUDED_DIRECT_RESAMPLER_CONDITIONER_SS_H */

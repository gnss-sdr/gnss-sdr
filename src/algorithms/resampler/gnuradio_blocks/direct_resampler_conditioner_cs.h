/*!
 * \file direct_resampler_conditioner_cs.h
 * \brief Nearest neighborhood resampler with
 *        std::complex<short> input and std::complex<short> output
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H
#define GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H

#include <gnuradio/block.h>
#include <volk/volk.h>

class direct_resampler_conditioner_cs;
typedef boost::shared_ptr<direct_resampler_conditioner_cs>
    direct_resampler_conditioner_cs_sptr;

direct_resampler_conditioner_cs_sptr
direct_resampler_make_conditioner_cs(double sample_freq_in,
    double sample_freq_out);
/*!
 * \brief This class implements a direct resampler conditioner for std::complex<short>
 *
 * Direct resampling without interpolation
 */
class direct_resampler_conditioner_cs : public gr::block
{
private:
    friend direct_resampler_conditioner_cs_sptr
    direct_resampler_make_conditioner_cs(double sample_freq_in,
        double sample_freq_out);

    double d_sample_freq_in;
    double d_sample_freq_out;
    uint32_t d_phase;
    uint32_t d_lphase;
    uint32_t d_phase_step;

    direct_resampler_conditioner_cs(double sample_freq_in,
        double sample_freq_out);

public:
    ~direct_resampler_conditioner_cs();

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

#endif /* GNSS_SDR_DIRECT_RESAMPLER_CONDITIONER_CS_H */

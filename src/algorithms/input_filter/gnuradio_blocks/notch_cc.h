/*!
 * \file notch_cc.h
 * \brief Implements a notch filter algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_NOTCH_H_
#define GNSS_SDR_NOTCH_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <memory>

class Notch;

typedef boost::shared_ptr<Notch> notch_sptr;

notch_sptr make_notch_filter(float pfa, float p_c_factor,
    int length_, int n_segments_est, int n_segments_reset);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter
 */

class Notch : public gr::block
{
private:
    float pfa;
    float noise_pow_est;
    float thres_;
    int length_;
    int n_deg_fred;
    unsigned int n_segments;
    unsigned int n_segments_est;
    unsigned int n_segments_reset;
    bool filter_state_;
    gr_complex last_out;
    gr_complex z_0;
    gr_complex p_c_factor;
    gr_complex *c_samples;
    float *angle_;
    float *power_spect;
    std::unique_ptr<gr::fft::fft_complex> d_fft;

public:
    Notch(float pfa, float p_c_factor, int length_, int n_segments_est, int n_segments_reset);

    ~Notch();

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif  //GNSS_SDR_NOTCH_H_

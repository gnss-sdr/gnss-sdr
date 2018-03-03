/*!
 * \file notch_lite_cc.h
 * \brief Implements a notch filter ligth algorithm
 * \author Antonio Ramos (antonio.ramosdet(at)gmail.com)
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017 (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_NOTCH_LITE_H_
#define GNSS_SDR_NOTCH_LITE_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <memory>

class NotchLite;

typedef boost::shared_ptr<NotchLite> notch_lite_sptr;

notch_lite_sptr make_notch_filter_lite(float p_c_factor, float pfa, int length_, int n_segments_est, int n_segments_reset, int n_segments_coeff);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter ligth version
 */

class NotchLite : public gr::block
{
private:
    int length_;
    int n_segments;
    int n_segments_est;
    int n_segments_reset;
    int n_segments_coeff_reset;
    int n_segments_coeff;
    int n_deg_fred;
    float pfa;
    float thres_;
    float noise_pow_est;
    bool filter_state_;
    gr_complex last_out;
    gr_complex z_0;
    gr_complex p_c_factor;
    gr_complex c_samples1;
    gr_complex c_samples2;
    float angle1;
    float angle2;
    float *power_spect;
    std::unique_ptr<gr::fft::fft_complex> d_fft;

public:
    NotchLite(float p_c_factor, float pfa, int length_, int n_segments_est, int n_segments_reset, int n_segments_coeff);

    ~NotchLite();

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif  //GNSS_SDR_NOTCH_LITE_H_

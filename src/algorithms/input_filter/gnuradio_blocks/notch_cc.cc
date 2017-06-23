/*!
 * \file notch_cc.cc
 * \brief Implements a multi state notch filter algorithm
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

#include "notch_cc.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <complex>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

notch_sptr make_notch_filter(float pfa, float p_c_factor, 
                             int length_)
{
    return notch_sptr(new Notch(pfa, p_c_factor, length_));
}

Notch::Notch(float pfa, float p_c_factor, int length_) : gr::block("Notch",
                                               gr::io_signature::make (1, 1, sizeof(gr_complex)),
                                               gr::io_signature::make (1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    set_history(2);
    this->pfa = pfa;
    noise_pow_est = 0.0;
    this->p_c_factor = p_c_factor;
    this->length_ = length_; //Set the number of samples per segment
    set_output_multiple(length_);
    filter_state_ = false; //Initial state of the filter
    n_deg_fred = 2 * length_; //Number of dregrees of freedom
    n_segments = 0; 
    n_segments_est = 8; // Set the number of segments for noise power estimation
    n_segments_reset = 1000000; // Set the period (in segments) when the noise power is estimated
    z_0 = gr_complex(0 , 0);
    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa));
    in = NULL;
    out = NULL;
    paux = NULL;
    c_samples = static_cast<gr_complex *>(volk_malloc(length_ * sizeof(gr_complex), volk_get_alignment()));
    angle_ = static_cast<float *>(volk_malloc(length_ * sizeof(float), volk_get_alignment()));
    last_out = gr_complex(0,0);
}

Notch::~Notch()
{
    volk_free(c_samples);
    volk_free(angle_);
}

int Notch::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int index_in = 1;
    int index_out = 0;
    in = (gr_complex *) input_items[index_in];
    out = (gr_complex *) output_items[index_out];
    int aux = 0;
    gr_complex magnitude;
    float sig2 = 0.0;
    while(((index_out + length_) < noutput_items) && (n_segments < n_segments_est) && (filter_state_ == false))
    {
        volk_32fc_x2_conjugate_dot_prod_32fc(&magnitude, in, in, length_);
        sig2 = magnitude.real() / ((float) n_deg_fred);
        noise_pow_est = (((float) n_segments) * noise_pow_est + sig2) / ((float)(n_segments + 1));
        index_out = index_out + length_;
        index_in = index_in +length_;
        n_segments++;
        memcpy(out, in, sizeof(gr_complex) * length_);
        in = (gr_complex *) input_items[index_in];
        out = (gr_complex *) output_items[index_out];
    }
    while((index_out + length_) < noutput_items)
    {
        n_segments++;
        volk_32fc_x2_conjugate_dot_prod_32fc(&magnitude, in, in, length_);
        if( (magnitude.real() / noise_pow_est) > thres_)
        {
            if(filter_state_ == false)
            {
                filter_state_ = true;
                last_out = gr_complex(0,0);
            }
            paux = (gr_complex *) input_items[index_in-1];
            volk_32fc_x2_multiply_conjugate_32fc(c_samples, in, paux, length_);
            volk_32fc_s32f_atan2_32f(angle_, c_samples, (float)1.0, length_);
            for(aux = 0; aux < length_; aux++)
            {
                z_0 = std::exp(gr_complex(0,1) * angle_[aux]);
                out[index_out] = in[index_in] - z_0 * in[index_in - 1]
                                 + gr_complex(p_c_factor,0) * z_0 * last_out;
                last_out = out[index_out];
                index_out++;
                index_in++;
                in = (gr_complex *) input_items[index_in];
                out = (gr_complex *) output_items[index_out];
            }
            
        }
        else
        {
            if (n_segments > n_segments_reset)
            {
                n_segments = 0;
            }
            filter_state_ = false;
            index_out = index_out + length_;
            index_in = index_in +length_;
            memcpy(out, in, sizeof(gr_complex) * length_);
            in = (gr_complex *) input_items[index_in];
            out = (gr_complex *) output_items[index_out];
        }
    }
    consume_each(index_out);
    return index_out;
}

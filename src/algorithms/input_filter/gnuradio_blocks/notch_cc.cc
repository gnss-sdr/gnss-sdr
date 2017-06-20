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
    this->pfa = pfa;
    noise_pow_est = 0.0;
    this->p_c_factor = p_c_factor;
    this->length_ = length_;
    filter_state_ = false;
    n_deg_fred = 2 * length_;
    n_segments_est = 5;
    n_segments = 0;
    z_0 = gr_complex(0 , 0);
    boost::math::chi_squared_distribution<float> my_dist_(n_deg_fred);
    thres_ = boost::math::quantile(boost::math::complement(my_dist_, pfa));
    
}


int Notch::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    gr_complex * in = (gr_complex *) input_items[0];
    gr_complex *out = (gr_complex *) output_items[0];
    gr_complex * paux;
    int samples_proc = 0;
    int aux = 0;
    gr_complex magnitude;
    float sig2 = 0.0;
    float* angle_;
    gr_complex * c_samples;
    c_samples = static_cast<gr_complex *>(volk_malloc(length_ * sizeof(gr_complex), volk_get_alignment()));
    angle_ = static_cast<float *>(volk_malloc(length_ * sizeof(float), volk_get_alignment()));
    while(((samples_proc + length_) < noutput_items) && (n_segments < n_segments_est))
    {
        volk_32fc_x2_conjugate_dot_prod_32fc(&magnitude, in, in, length_);
        sig2 = magnitude.real() / ((float) n_deg_fred);
        noise_pow_est = (((float) n_segments) * noise_pow_est + sig2) / ((float)(n_segments + 1));
        samples_proc = samples_proc + length_;
        n_segments++;
        memcpy(out, in, sizeof(gr_complex)*length_);
        in = (gr_complex *) input_items[samples_proc];
        out = (gr_complex *) output_items[samples_proc];
    }
    while((samples_proc + length_) < noutput_items)
    {
        volk_32fc_x2_conjugate_dot_prod_32fc(&magnitude, in, in, length_);
        if( (magnitude.real() / noise_pow_est) > thres_)
        {
            filter_state_ = true;
            paux = (gr_complex *) input_items[samples_proc-1];
            volk_32fc_x2_multiply_conjugate_32fc(c_samples, in, paux, length_);
            volk_32fc_s32f_atan2_32f(angle_, c_samples, (float)1.0, length_);
            for(aux = 0; aux < length_; aux++)
            {
                z_0 = std::exp(gr_complex(0,1) *angle_[aux]);
                out[samples_proc] = in[samples_proc] - z_0 * in[samples_proc - 1]
                                    + gr_complex(p_c_factor,0) * z_0 * out[samples_proc -1];
                samples_proc++;
                in = (gr_complex *) input_items[samples_proc];
                out = (gr_complex *) output_items[samples_proc];
            }
            
        }
        else
        {
            filter_state_ = false;
            samples_proc = samples_proc + length_;
            memcpy(out, in, sizeof(gr_complex)*length_);
            in = (gr_complex *) input_items[samples_proc];
            out = (gr_complex *) output_items[samples_proc];
        }
    }
    volk_free(c_samples);
    volk_free(angle_);
    consume_each(samples_proc);
    return samples_proc;
}

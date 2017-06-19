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
#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>

notch_sptr make_notch_filter(double pfa, double p_c_factor, 
                             unsigned int length_)
{
    return notch_sptr(new Notch(pfa, p_c_factor, length_));
}

Notch::Notch(double pfa, double p_c_factor, unsigned int length_) : gr::block("Notch",
                                               gr::io_signature::make (1, 1, sizeof(gr_complex)),
                                               gr::io_signature::make (1, 1, sizeof(gr_complex)))
{
    this->pfa = pfa;
    this->noise_pow_est = 0.0;
    this->p_c_factor = p_c_factor;
    this->length_ = length_;
    filter_state_ = 0;
    n_deg_fred = 2 * length_;
    z_0 = gr_complex(0 , 0);
    thres_ = 
    
}


int Notch::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    
}

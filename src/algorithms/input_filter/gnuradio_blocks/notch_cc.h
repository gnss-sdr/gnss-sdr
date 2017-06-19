/*!
 * \file notch_cc.h
 * \brief Implements a notch filter algorithm
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

#ifndef GNSS_SDR_NOTCH_H_
#define GNSS_SDR_NOTCH_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>

class Notch;

typedef boost::shared_ptr<Notch> notch_sptr;

notch_sptr make_notch_filter(double pfa, double p_c_factor, 
                             unsigned int length_);

/*!
 * \brief This class implements a real-time software-defined multi state notch filter
 */

class Notch: public gr::block
{
private:
    
    friend notch_sptr make_notch_filter(double pfa, double p_c_factor,
                                        unsigned int length_);
    double pfa;
    double noise_pow_est;
    double p_c_factor;
    double thres_;
    unsigned int length_;
    unsigned int n_deg_fred;
    unsigned int filter_state_;
    gr_complex z_0;
    
    
public:
    
    Notch(double pfa, double p_c_factor, unsigned int length_);
    
    ~Notch();
    
    int work (int noutput_items, gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items);
};

#endif //GNSS_SDR_NOTCH_H_

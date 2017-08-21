/*!
 * \file notch_lite_cc.h
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

#ifndef GNSS_SDR_NOTCH_LITE_H_
#define GNSS_SDR_NOTCH_LITE_H_

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>

class NotchLite;

typedef boost::shared_ptr<NotchLite> notch_lite_sptr;

notch_lite_sptr make_notch_filter_lite(float p_c_factor);

/*!
 * \brief This class implements a real-time software-defined single state notch filter
 */

class NotchLite : public gr::block
{
private:
    
    gr_complex last_out;
    gr_complex z_0;
    gr_complex p_c_factor;
    gr_complex* c_samples;
    float* angle_;
        
public:
        
    NotchLite(float p_c_factor);
    
    int general_work (int noutput_items, gr_vector_int &ninput_items, 
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items);
};

#endif //GNSS_SDR_NOTCH_LITE_H_

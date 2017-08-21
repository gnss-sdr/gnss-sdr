/*!
 * \file notch_lite_cc.cc
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

#include "notch_lite_cc.h"
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <iostream>
#include <glog/logging.h>

using google::LogMessage;

notch_lite_sptr make_notch_filter_lite(float p_c_factor)
{
    return notch_lite_sptr(new NotchLite(p_c_factor));
}

NotchLite::NotchLite(float p_c_factor) : gr::block("NotchLite",
                                                   gr::io_signature::make (1, 1, sizeof(gr_complex)),
                                                   gr::io_signature::make (1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    set_history(2);
    this->p_c_factor = gr_complex(p_c_factor , 0);
    z_0 = gr_complex(0 , 0);
    last_out = gr_complex(0,0);
}

int NotchLite::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    gr_complex* in = (gr_complex *) input_items[0];
    gr_complex* out = (gr_complex *) output_items[0];
    c_samples = static_cast<gr_complex *>(volk_malloc(noutput_items * sizeof(gr_complex), volk_get_alignment()));
    angle_ = static_cast<float *>(volk_malloc(noutput_items * sizeof(float), volk_get_alignment()));
        
    volk_32fc_x2_multiply_conjugate_32fc(c_samples, (in + 1), in, noutput_items);
    volk_32fc_s32f_atan2_32f(angle_, c_samples, ((float)1.0), noutput_items);
    for (int aux = 0; aux < noutput_items; aux++)
    {
        z_0 = std::exp(gr_complex(0,1) * (*(angle_ + aux)));
        *(out + aux) = *(in + aux + 1) - z_0 * (*(in + aux)) + p_c_factor * z_0 * last_out;
        last_out = *(out + aux);
    }
    volk_free(c_samples);
    volk_free(angle_);
    consume_each(noutput_items);
    return noutput_items;
}

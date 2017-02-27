/*!
 * \file pulse_blanking_cc.cc
 * \brief Implements a simple pulse blanking algorithm
 * \author Javier Arribas (jarribas(at)cttc.es)
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

#include "pulse_blanking_cc.h"
#include <cmath>
#include <complex>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


pulse_blanking_cc_sptr make_pulse_blanking_cc(double Pfa)
{
    return pulse_blanking_cc_sptr(new pulse_blanking_cc(Pfa));
}



pulse_blanking_cc::pulse_blanking_cc(double Pfa) : gr::block("pulse_blanking_cc",
                        gr::io_signature::make (1, 1, sizeof(gr_complex)),
                        gr::io_signature::make (1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
    d_Pfa = Pfa;
}


int pulse_blanking_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];
    gr_complex *out = (gr_complex *) output_items[0];

    // 1- (optional) Compute the input signal power estimation
    //float mean;
    //float stddev;
    //volk_32f_stddev_and_mean_32f_x2(&stddev, &mean, in, noutput_items);

    float* magnitude;
    magnitude = static_cast<float*>(volk_gnsssdr_malloc(noutput_items * sizeof(float), volk_gnsssdr_get_alignment()));

    float var;
    volk_32fc_magnitude_squared_32f(magnitude, in, noutput_items);
    volk_32f_accumulator_s32f(&var, magnitude, noutput_items);
    var /= static_cast<float>(noutput_items);
    // compute pulse blanking threshold (Paper Borio 2016)

    float Th = sqrt(-2.0 * var * log10(d_Pfa));

    //apply the pulse blanking
    //todo: write volk kernel to optimize the blanking
    memcpy(out,in, sizeof(gr_complex)*noutput_items);
    for (int n = 0; n < noutput_items; n++)
        {
            if (std::abs(out[n]) > Th)
                {
                    out[n] = gr_complex(0,0);
                }
        }
    consume_each(noutput_items);
    return noutput_items;
}

/*!
 * \file correlator.cc
 * \brief Highly optimized vector correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * Class that implements a high optimized vector correlator class.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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


#include "correlator.h"
#include <iostream>
#if USING_VOLK_CW_EPL_CORR_CUSTOM
  #define LV_HAVE_SSE3
  #include "volk_cw_epl_corr.h"
#endif


unsigned long Correlator::next_power_2(unsigned long v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}




void Correlator::Carrier_wipeoff_and_EPL_generic(int signal_length_samples, const gr_complex* input, gr_complex* carrier, gr_complex* E_code, gr_complex* P_code, gr_complex* L_code,gr_complex* E_out, gr_complex* P_out, gr_complex* L_out)
{
    gr_complex bb_signal_sample(0,0);

    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    // perform Early, Prompt and Late correlation
    for(int i=0; i < signal_length_samples; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = input[i] * carrier[i];
            // Now get early, late, and prompt values for each
            *E_out += bb_signal_sample * E_code[i];
            *P_out += bb_signal_sample * P_code[i];
            *L_out += bb_signal_sample * L_code[i];
        }
}




void Correlator::Carrier_wipeoff_and_EPL_volk(int signal_length_samples, const gr_complex* input, gr_complex* carrier, gr_complex* E_code, gr_complex* P_code, gr_complex* L_code, gr_complex* E_out, gr_complex* P_out, gr_complex* L_out)
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_x2_multiply_32fc(bb_signal, input, carrier, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, signal_length_samples);

    volk_free(bb_signal);
}

//void Correlator::Carrier_wipeoff_and_EPL_volk_IQ(int prn_length_samples,int integration_time ,const gr_complex* input, gr_complex* carrier, gr_complex* E_code, gr_complex* P_code, gr_complex* L_code, gr_complex* P_data_code, gr_complex* E_out, gr_complex* P_out, gr_complex* L_out, gr_complex* P_data_out)
//{
//    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
//    volk_32fc_x2_multiply_32fc(bb_signal, input, carrier, integration_time * prn_length_samples);
//    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, integration_time * prn_length_samples);
//    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, integration_time * prn_length_samples);
//    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, integration_time * prn_length_samples);
//    // Vector of Prompts of I code
//    for (int i = 0; i < integration_time; i++)
//	{
//	    volk_32fc_x2_dot_prod_32fc(&P_data_out[i], &bb_signal[i*prn_length_samples], P_data_code, prn_length_samples);
//	}
//
//    volk_free(bb_signal);
//}

void Correlator::Carrier_wipeoff_and_EPL_volk_IQ(int signal_length_samples ,const gr_complex* input, gr_complex* carrier, gr_complex* E_code, gr_complex* P_code, gr_complex* L_code, gr_complex* P_data_code, gr_complex* E_out, gr_complex* P_out, gr_complex* L_out, gr_complex* P_data_out)
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_x2_multiply_32fc(bb_signal, input, carrier, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_data_out, bb_signal, P_data_code, signal_length_samples);

    volk_free(bb_signal);
}


void Correlator::Carrier_wipeoff_and_VEPL_volk(int signal_length_samples, const gr_complex* input, gr_complex* carrier, gr_complex* VE_code, gr_complex* E_code, gr_complex* P_code, gr_complex* L_code, gr_complex* VL_code, gr_complex* VE_out, gr_complex* E_out, gr_complex* P_out, gr_complex* L_out, gr_complex* VL_out)
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_x2_multiply_32fc(bb_signal, input, carrier, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(VE_out, bb_signal, VE_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(VL_out, bb_signal, VL_code, signal_length_samples);

    volk_free(bb_signal);
}



Correlator::Correlator ()
{}


Correlator::~Correlator ()
{}


#if USING_VOLK_CW_EPL_CORR_CUSTOM
void Correlator::Carrier_wipeoff_and_EPL_volk_custom(int signal_length_samples, const gr_complex* input, gr_complex* carrier,gr_complex* E_code, gr_complex* P_code, gr_complex* L_code, gr_complex* E_out, gr_complex* P_out, gr_complex* L_out)
{
    volk_cw_epl_corr_u(input, carrier, E_code, P_code, L_code, E_out, P_out, L_out, signal_length_samples);
}
#endif

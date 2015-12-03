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
#include "accumulate_array.h"


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


void Correlator::Carrier_wipeoff_and_DE_volk(int signal_length_samples,
            const gr_complex* input,
            gr_complex* carrier,
            gr_complex* E_code, gr_complex* P_code, gr_complex* L_code,
            gr_complex* E_subcarrier, gr_complex* P_subcarrier, gr_complex *L_subcarrier,
            gr_complex* P_subcarrier_E_code_out,
            gr_complex* P_subcarrier_P_code_out,
            gr_complex* P_subcarrier_L_code_out,
            gr_complex* P_code_E_subcarrier_out,
            gr_complex* P_code_L_subcarrier_out )
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* code_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_x2_multiply_32fc(bb_signal, input, carrier, signal_length_samples);
    volk_32fc_x2_multiply_32fc(subcarrier_wipeoff, bb_signal, P_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc(code_wipeoff, bb_signal, P_code, signal_length_samples );

    volk_32fc_x2_dot_prod_32fc(P_subcarrier_E_code_out, subcarrier_wipeoff, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_P_code_out, subcarrier_wipeoff, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_L_code_out, subcarrier_wipeoff, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_code_E_subcarrier_out, code_wipeoff, E_subcarrier, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_code_L_subcarrier_out, code_wipeoff, L_subcarrier, signal_length_samples);

    volk_free(bb_signal);
    volk_free(subcarrier_wipeoff);
    volk_free(code_wipeoff);
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
void Correlator::Carrier_rotate_and_EPL_volk(int signal_length_samples,
                                             const gr_complex* input,
                                             gr_complex *phase_as_complex,
                                             gr_complex phase_inc_as_complex,
                                             const gr_complex* E_code,
                                             const gr_complex* P_code,
                                             const gr_complex* L_code,
                                             gr_complex* E_out,
                                             gr_complex* P_out,
                                             gr_complex* L_out )
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, signal_length_samples);

    volk_free(bb_signal);
}

void Correlator::Carrier_rotate_and_VEPL_volk(int signal_length_samples,
                                              const gr_complex* input,
                                              gr_complex *phase_as_complex,
                                              gr_complex phase_inc_as_complex,
                                              const gr_complex* VE_code,
                                              const gr_complex* E_code,
                                              const gr_complex* P_code,
                                              const gr_complex* L_code,
                                              const gr_complex* VL_code,
                                              gr_complex* VE_out,
                                              gr_complex* E_out,
                                              gr_complex* P_out,
                                              gr_complex* L_out,
                                              gr_complex* VL_out )
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(VE_out, bb_signal, VE_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(E_out, bb_signal, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_out, bb_signal, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(L_out, bb_signal, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(VL_out, bb_signal, VL_code, signal_length_samples);

    volk_free(bb_signal);
}

void Correlator::Carrier_rotate_and_DE_volk(int signal_length_samples,
            const gr_complex* input,
            gr_complex *phase_as_complex,
            gr_complex phase_inc_as_complex,
            const gr_complex* E_code,
            const gr_complex* P_code,
            const gr_complex* L_code,
            const gr_complex* E_subcarrier,
            const gr_complex* P_subcarrier,
            const gr_complex *L_subcarrier,
            gr_complex* P_subcarrier_E_code_out,
            gr_complex* P_subcarrier_P_code_out,
            gr_complex* P_subcarrier_L_code_out,
            gr_complex* P_code_E_subcarrier_out,
            gr_complex* P_code_L_subcarrier_out )
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* code_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);
    volk_32fc_x2_multiply_32fc(subcarrier_wipeoff, bb_signal, P_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc(code_wipeoff, bb_signal, P_code, signal_length_samples );

    volk_32fc_x2_dot_prod_32fc(P_subcarrier_E_code_out, subcarrier_wipeoff, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_P_code_out, subcarrier_wipeoff, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_L_code_out, subcarrier_wipeoff, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_code_E_subcarrier_out, code_wipeoff, E_subcarrier, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_code_L_subcarrier_out, code_wipeoff, L_subcarrier, signal_length_samples);

    volk_free(bb_signal);
    volk_free(subcarrier_wipeoff);
    volk_free(code_wipeoff);
}

void Correlator::Carrier_rotate_and_DPE_volk(int signal_length_samples,
            const gr_complex* input,
            gr_complex *phase_as_complex,
            gr_complex phase_inc_as_complex,
            const gr_complex* E_code,
            const gr_complex* P_code,
            const gr_complex* L_code,
            const gr_complex* P_subcarrier,
            const gr_complex* PQ_subcarrier,
            gr_complex* P_subcarrier_E_code_out,
            gr_complex* P_subcarrier_P_code_out,
            gr_complex* P_subcarrier_L_code_out,
            gr_complex* P_code_PQ_subcarrier_out )
{
    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* code_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);
    volk_32fc_x2_multiply_32fc(subcarrier_wipeoff, bb_signal, P_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc(code_wipeoff, bb_signal, P_code, signal_length_samples );

    volk_32fc_x2_dot_prod_32fc(P_subcarrier_E_code_out, subcarrier_wipeoff, E_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_P_code_out, subcarrier_wipeoff, P_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_subcarrier_L_code_out, subcarrier_wipeoff, L_code, signal_length_samples);
    volk_32fc_x2_dot_prod_32fc(P_code_PQ_subcarrier_out, code_wipeoff, PQ_subcarrier, signal_length_samples);

    volk_free(bb_signal);
    volk_free(subcarrier_wipeoff);
    volk_free(code_wipeoff);
}

void Correlator::Carrier_rotate_and_EPL_codeless(int signal_length_samples,
            const gr_complex* input,
            gr_complex *phase_as_complex,
            gr_complex phase_inc_as_complex,
            const int *E_code_phases,
            const int *P_code_phases,
            const int *L_code_phases,
            gr_complex* E_out,
            gr_complex* P_out,
            gr_complex* L_out,
            int code_length )
{

    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));


    gr_complex* accum_early = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_early, accum_early+code_length, 0.0 );

    gr_complex* accum_prompt = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_prompt, accum_prompt+code_length, 0.0 );

    gr_complex* accum_late = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_late, accum_late+code_length, 0.0 );

    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);

    // Now we do an accumulation of the input by code phase bins:
    accumulate_array< gr_complex >( bb_signal, E_code_phases, accum_early, signal_length_samples );
    accumulate_array< gr_complex >( bb_signal, P_code_phases, accum_prompt, signal_length_samples );
    accumulate_array< gr_complex >( bb_signal, L_code_phases, accum_late, signal_length_samples );

    // Now sum the squares of the partial sums: this is implemented as a simple
    // dot product
    volk_32fc_x2_dot_prod_32fc(E_out, accum_early, accum_early, code_length);
    volk_32fc_x2_dot_prod_32fc(P_out, accum_prompt, accum_prompt, code_length);
    volk_32fc_x2_dot_prod_32fc(L_out, accum_late, accum_late, code_length);

    volk_free(bb_signal);
    volk_free(accum_early);
    volk_free(accum_prompt);
    volk_free(accum_late);
}

void Correlator::Carrier_rotate_and_VEPL_codeless(int signal_length_samples,
            const gr_complex* input,
            gr_complex *phase_as_complex,
            gr_complex phase_inc_as_complex,
            const int *VE_code_phases,
            const int *E_code_phases,
            const int *P_code_phases,
            const int *L_code_phases,
            const int *VL_code_phases,
            const gr_complex *VE_subcarrier,
            const gr_complex *E_subcarrier,
            const gr_complex *P_subcarrier,
            const gr_complex *L_subcarrier,
            const gr_complex *VL_subcarrier,
            gr_complex* VE_out,
            gr_complex* E_out,
            gr_complex* P_out,
            gr_complex* L_out,
            gr_complex* VL_out,
            int code_length)
{

    gr_complex* bb_signal = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* ve_subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* e_subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* p_subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* l_subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));
    gr_complex* vl_subcarrier_wipeoff = static_cast<gr_complex*>(volk_malloc(signal_length_samples * sizeof(gr_complex), volk_get_alignment()));

    gr_complex* accum_very_early = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_very_early, accum_very_early+code_length, 0.0 );

    gr_complex* accum_early = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_early, accum_early+code_length, 0.0 );

    gr_complex* accum_prompt = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_prompt, accum_prompt+code_length, 0.0 );

    gr_complex* accum_late = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_late, accum_late+code_length, 0.0 );

    gr_complex* accum_very_late = static_cast<gr_complex*>(volk_malloc(code_length* sizeof(gr_complex), volk_get_alignment()));
    std::fill( accum_very_late, accum_very_late+code_length, 0.0 );

    // 1) Carrier wipe-off
    volk_32fc_s32fc_x2_rotator_32fc(bb_signal, input, phase_inc_as_complex, phase_as_complex, signal_length_samples);

    // 2) Subcarrier wipe-off
    volk_32fc_x2_multiply_32fc( ve_subcarrier_wipeoff, bb_signal, VE_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc( e_subcarrier_wipeoff, bb_signal, E_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc( p_subcarrier_wipeoff, bb_signal, P_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc( l_subcarrier_wipeoff, bb_signal, L_subcarrier, signal_length_samples );
    volk_32fc_x2_multiply_32fc( vl_subcarrier_wipeoff, bb_signal, VL_subcarrier, signal_length_samples );

    // 3) Do an accumulation of the input by code phase bins:
    accumulate_array< gr_complex >( ve_subcarrier_wipeoff, VE_code_phases, accum_very_early, signal_length_samples );
    accumulate_array< gr_complex >( e_subcarrier_wipeoff, E_code_phases, accum_early, signal_length_samples );
    accumulate_array< gr_complex >( p_subcarrier_wipeoff, P_code_phases, accum_prompt, signal_length_samples );
    accumulate_array< gr_complex >( l_subcarrier_wipeoff, L_code_phases, accum_late, signal_length_samples );
    accumulate_array< gr_complex >( vl_subcarrier_wipeoff, VL_code_phases, accum_very_late, signal_length_samples );

    // 4) Now sum the squares of the partial sums: this is implemented as a simple
    // dot product
    volk_32fc_x2_dot_prod_32fc(VE_out, accum_very_early, accum_very_early, code_length);
    volk_32fc_x2_dot_prod_32fc(E_out, accum_early, accum_early, code_length);
    volk_32fc_x2_dot_prod_32fc(P_out, accum_prompt, accum_prompt, code_length);
    volk_32fc_x2_dot_prod_32fc(L_out, accum_late, accum_late, code_length);
    volk_32fc_x2_dot_prod_32fc(VL_out, accum_very_late, accum_very_late, code_length);

    volk_free(bb_signal);
    volk_free(ve_subcarrier_wipeoff);
    volk_free(e_subcarrier_wipeoff);
    volk_free(p_subcarrier_wipeoff);
    volk_free(l_subcarrier_wipeoff);
    volk_free(vl_subcarrier_wipeoff);
    volk_free(accum_very_early);
    volk_free(accum_early);
    volk_free(accum_prompt);
    volk_free(accum_late);
    volk_free(accum_very_late);

}

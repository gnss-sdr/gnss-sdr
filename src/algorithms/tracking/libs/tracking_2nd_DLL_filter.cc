/*!
 * \file tracking_2nd_DLL_filter.cc
 * \brief Implementation of a 2nd order DLL filter for code tracking loop.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements 2 order PLL filter for code tracking loop.
 * The algorithm is described in :
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.~H.~Jensen, A Software-Defined
 * GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007, Applied and Numerical Harmonic Analysis.
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


#include "tracking_2nd_DLL_filter.h"


void Tracking_2nd_DLL_filter::calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k)
{
    // Solve natural frequency
    float Wn;
    Wn = lbw * 8 * zeta / (4 * zeta * zeta + 1);
    // solve for t1 & t2
    *tau1 = k / (Wn * Wn);
    *tau2 = (2.0 * zeta) / Wn;
}


void Tracking_2nd_DLL_filter::set_DLL_BW(float dll_bw_hz)
{
    //Calculate filter coefficient values
    d_dllnoisebandwidth = dll_bw_hz;
    calculate_lopp_coef(&d_tau1_code, &d_tau2_code, d_dllnoisebandwidth, d_dlldampingratio, 1.0);  // Calculate filter coefficient values
}


void Tracking_2nd_DLL_filter::initialize()
{
    // code tracking loop parameters
    d_old_code_nco = 0.0;
    d_old_code_error = 0.0;
}


float Tracking_2nd_DLL_filter::get_code_nco(float DLL_discriminator)
{
    float code_nco;
    code_nco = d_old_code_nco + (d_tau2_code / d_tau1_code) * (DLL_discriminator - d_old_code_error) + (DLL_discriminator + d_old_code_error) * (d_pdi_code / (2 * d_tau1_code));
    //code_nco = d_old_code_nco + (d_tau2_code/d_tau1_code)*(DLL_discriminator - d_old_code_error) + DLL_discriminator * (d_pdi_code/d_tau1_code);
    d_old_code_nco = code_nco;
    d_old_code_error = DLL_discriminator;  //[chips]
    return code_nco;
}

Tracking_2nd_DLL_filter::Tracking_2nd_DLL_filter(float pdi_code)
{
    d_pdi_code = pdi_code;  // Summation interval for code
    d_dlldampingratio = 0.7;
}

Tracking_2nd_DLL_filter::Tracking_2nd_DLL_filter()
{
    d_pdi_code = 0.001;  // Summation interval for code
    d_dlldampingratio = 0.7;
}

Tracking_2nd_DLL_filter::~Tracking_2nd_DLL_filter()
{
}

void Tracking_2nd_DLL_filter::set_pdi(float pdi_code)
{
    d_pdi_code = pdi_code;  // Summation interval for code
}

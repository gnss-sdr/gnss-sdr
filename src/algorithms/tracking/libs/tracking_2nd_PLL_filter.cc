/*!
 * \file tracking_2nd_PLL_filter.cc
 * \brief Implementation of a 2nd order PLL filter for tracking carrier loop.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements 2 order PLL filter for tracking carrier loop. The algorithm
 * is described in:
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

#include "tracking_2nd_PLL_filter.h"


void Tracking_2nd_PLL_filter::calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k)
{
    // Solve natural frequency
    float Wn;
    Wn = lbw * 8 * zeta / (4 * zeta * zeta + 1);
    // solve for t1 & t2
    *tau1 = k / (Wn * Wn);
    *tau2 = (2.0 * zeta) / Wn;
}


void Tracking_2nd_PLL_filter::set_PLL_BW(float pll_bw_hz)
{
    //Calculate filter coefficient values
    d_pllnoisebandwidth = pll_bw_hz;
    calculate_lopp_coef(&d_tau1_carr, &d_tau2_carr, d_pllnoisebandwidth, d_plldampingratio, 0.25);  // Calculate filter coefficient values
}


void Tracking_2nd_PLL_filter::initialize()
{
    // carrier/Costas loop parameters
    d_old_carr_nco = 0.0;
    d_old_carr_error = 0.0;
}


/*
 * PLL second order FIR filter
 * Req Input in [Hz/Ti]
 * The output is in [Hz/s].
 */
float Tracking_2nd_PLL_filter::get_carrier_nco(float PLL_discriminator)
{
    float carr_nco;
    carr_nco = d_old_carr_nco + (d_tau2_carr / d_tau1_carr) * (PLL_discriminator - d_old_carr_error) + (PLL_discriminator + d_old_carr_error) * (d_pdi_carr / (2 * d_tau1_carr));
    //carr_nco = d_old_carr_nco + (d_tau2_carr/d_tau1_carr)*(PLL_discriminator - d_old_carr_error) + PLL_discriminator * (d_pdi_carr/d_tau1_carr);
    d_old_carr_nco = carr_nco;
    d_old_carr_error = PLL_discriminator;
    return carr_nco;
}

Tracking_2nd_PLL_filter::Tracking_2nd_PLL_filter(float pdi_carr)
{
    //--- PLL variables --------------------------------------------------------
    d_pdi_carr = pdi_carr;  // Summation interval for carrier
    //d_plldampingratio = 0.65;
    d_plldampingratio = 0.7;
}


Tracking_2nd_PLL_filter::Tracking_2nd_PLL_filter()
{
    //--- PLL variables --------------------------------------------------------
    d_pdi_carr = 0.001;  // Summation interval for carrier
    d_plldampingratio = 0.7;
}


Tracking_2nd_PLL_filter::~Tracking_2nd_PLL_filter()
{
}

void Tracking_2nd_PLL_filter::set_pdi(float pdi_carr)
{
    d_pdi_carr = pdi_carr;  // Summation interval for code
}

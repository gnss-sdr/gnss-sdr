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
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "tracking_2nd_PLL_filter.h"


void Tracking_2nd_PLL_filter::calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k)
{
    // Solve natural frequency
    const float Wn = lbw * 8.0F * zeta / (4.0F * zeta * zeta + 1.0F);
    // solve for t1 & t2
    *tau1 = k / (Wn * Wn);
    *tau2 = 2.0F * zeta / Wn;
}


void Tracking_2nd_PLL_filter::set_PLL_BW(float pll_bw_hz)
{
    // Calculate filter coefficient values
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
 * PLL second order IIR filter
 * Req Input in [Hz/Ti]
 * The output is in [Hz/s].
 */
float Tracking_2nd_PLL_filter::get_carrier_nco(float PLL_discriminator)
{
    float carr_nco = d_old_carr_nco + (d_tau2_carr / d_tau1_carr) * (PLL_discriminator - d_old_carr_error) + (PLL_discriminator + d_old_carr_error) * (d_pdi_carr / (2.0F * d_tau1_carr));
    d_old_carr_nco = carr_nco;
    d_old_carr_error = PLL_discriminator;
    return carr_nco;
}


Tracking_2nd_PLL_filter::Tracking_2nd_PLL_filter(float pdi_carr)
{
    // PLL variables
    d_pdi_carr = pdi_carr;  // Summation interval for carrier
    d_plldampingratio = 0.7;
}


Tracking_2nd_PLL_filter::Tracking_2nd_PLL_filter()
{
    // PLL variables
    d_pdi_carr = 0.001;  // Summation interval for carrier
    d_plldampingratio = 0.7;
}


void Tracking_2nd_PLL_filter::set_pdi(float pdi_carr)
{
    d_pdi_carr = pdi_carr;  // Summation interval for code
}

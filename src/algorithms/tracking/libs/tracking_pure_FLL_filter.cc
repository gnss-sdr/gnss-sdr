/*!
 * \file tracking_pure_FLL_filter.cc
 * \brief Implementation a pure FLL filter for tracking carrier loop
 * \author Ricardo Amorim, 2024. amorim(at)ita.br 
 * \author Felix Antreich, 2024. antreich(at)ieee.org 
 *
 * Class that implements hybrid FLL and PLL filter for tracking carrier loop
 * Filter design (Kaplan 2nd ed., Pag. 181 Fig. 181)
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

#include "tracking_pure_FLL_filter.h"
#include <iostream>

void Tracking_Pure_FLL_filter::set_params(float fll_bw_hz, float pll_bw_hz, int order)
{
    /*
     * Filter design (Kaplan 2nd ed., Pag. 181 Fig. 181)
     */
    d_order = order;
    if (d_order == 3)
        {
            /*
             *  3rd order PLL with 2nd order FLL assist
             */
            k1 = 0.002903;
            k2 = 0.000002812;
            k3 = 0.0000000009084;
            // k1 = 0.002603;
            // k2 = 0.000003016;
            // k3 = 0.000000001311;
            a0 = k1 + k2 + k3;
            a1 = k2 + 2 * k3;
            a2 = k3;
            ap0 = 0.438 + 0.00626;
            ap1 = 0.00626;
        }
    else
        {
            /*
             * 2nd order PLL with 1st order FLL assist
             */

        }
}


void Tracking_Pure_FLL_filter::initialize(float d_acq_carrier_doppler_hz)
{
    if (d_order == 3)
        {
            d_fll_filt_e = 0.0F;
            d_fll_filt_e1 = d_acq_carrier_doppler_hz;
            d_fll_filt_e2 = d_acq_carrier_doppler_hz;
            d_fll_e2 = 0.0F;
            d_fll_e1 = 0.0F;
            d_pll_filt_e = 0.0F;            
            d_pll_filt_e1 = 0.0F;                        
            d_pll_filt_e2 = 0.0F;
            d_pll_e2 = 0.0F;
            d_pll_e1 = 0.0F;
            k = 0;
        }
    else
        {
        }
}


float Tracking_Pure_FLL_filter::get_carrier_error(float discriminator, int type, float correlation_time_s)
{
    float carrier_error_hz;
    if (d_order == 3)
        {
            if (type == 0)
            {
                d_fll_filt_e = 2 * d_fll_filt_e1 - d_fll_filt_e2 + (a0 - a1 + a2) * d_fll_e2 + (a1 - 2 * a0) * d_fll_e1 + a0 * discriminator;
                d_fll_filt_e2 = d_fll_filt_e1;
                d_fll_filt_e1 = d_fll_filt_e;
                d_fll_e2 = d_fll_e1;
                d_fll_e1 = discriminator;
                carrier_error_hz = d_fll_filt_e;
            }
            else
            {
                d_pll_filt_e = d_pll_filt_e1 + (ap1 - ap0) * d_pll_e1 + ap0 * discriminator;
                d_pll_filt_e1 = d_pll_filt_e;
                d_pll_e1 = discriminator;
                carrier_error_hz = d_pll_filt_e;
            }
        }

    return carrier_error_hz;
}

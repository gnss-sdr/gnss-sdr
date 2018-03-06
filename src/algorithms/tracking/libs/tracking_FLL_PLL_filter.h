/*!
 * \file tracking_FLL_PLL_filter.h
 * \brief Interface of a hybrid FLL and PLL filter for tracking carrier loop
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TRACKING_FLL_PLL_FILTER_H_
#define GNSS_SDR_TRACKING_FLL_PLL_FILTER_H_

/*!
 * \brief This class implements a hybrid FLL and PLL filter for tracking carrier loop
 */
class Tracking_FLL_PLL_filter
{
private:
    // FLL + PLL filter parameters
    int d_order;
    float d_pll_w;
    float d_pll_w0p3;
    float d_pll_w0f2;
    float d_pll_x;
    float d_pll_a2;
    float d_pll_w0f;
    float d_pll_a3;
    float d_pll_w0p2;
    float d_pll_b3;
    float d_pll_w0p;

public:
    void set_params(float fll_bw_hz, float pll_bw_hz, int order);
    void initialize(float d_acq_carrier_doppler_hz);
    float get_carrier_error(float FLL_discriminator, float PLL_discriminator, float correlation_time_s);
    Tracking_FLL_PLL_filter();
    ~Tracking_FLL_PLL_filter();
};

#endif

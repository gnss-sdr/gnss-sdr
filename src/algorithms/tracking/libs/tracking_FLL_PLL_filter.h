/*!
 * \file tracking_FLL_PLL_filter.h
 * \brief Interface of a hybrid FLL and PLL filter for tracking carrier loop
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TRACKING_FLL_PLL_FILTER_H
#define GNSS_SDR_TRACKING_FLL_PLL_FILTER_H

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief This class implements a hybrid FLL and PLL filter for tracking carrier loop
 */
class Tracking_FLL_PLL_filter
{
public:
    Tracking_FLL_PLL_filter();
    ~Tracking_FLL_PLL_filter() = default;
    void set_params(float fll_bw_hz, float pll_bw_hz, int order);
    void initialize(float d_acq_carrier_doppler_hz);
    float get_carrier_error(float FLL_discriminator, float PLL_discriminator, float correlation_time_s);

private:
    // FLL + PLL filter parameters
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
    int d_order;
};


/** \} */
/** \} */
#endif

/*!
 * \file tracking_FLL_PLL_filter.h
 * \brief Interface of pure FLL filter for tracking carrier loop
 * \author Ricardo Amorim, 2024. amorim(at)ita.br 
 * \author Felix Antreich, 2024. antreich(at)ieee.org 
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
    Tracking_FLL_PLL_filter() = default;
    ~Tracking_FLL_PLL_filter() = default;
    void set_params(float fll_bw_hz, float pll_bw_hz, int order);
    void initialize(float d_acq_carrier_doppler_hz);
    float get_carrier_error(float discriminator, int type, float correlation_time_s);

private:
    // FLL + PLL filter parameters
    float k1{0.0};
    float k2{0.0};
    float k3{0.0};
    float a0{0.0};
    float a1{0.0};
    float a2{0.0};
    float ap0{0.0};
    float ap1{0.0};
    float d_fll_filt_e{0.0};
    float d_fll_filt_e1{0.0};
    float d_fll_filt_e2{0.0};
    float d_fll_e1{0.0};
    float d_fll_e2{0.0};
    float d_pll_filt_e{0.0};
    float d_pll_filt_e1{0.0};
    float d_pll_filt_e2{0.0};
    float d_pll_e1{0.0};
    float d_pll_e2{0.0};
    float f_acq{0.0};
    int k{0};
    int d_order{0};
};


/** \} */
/** \} */
#endif

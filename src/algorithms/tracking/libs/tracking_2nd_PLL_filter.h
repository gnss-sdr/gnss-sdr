/*!
 * \file tracking_2nd_PLL_filter.h
 * \brief Interface of a 2nd order PLL filter for carrier tracking loop
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements 2 order PLL filter for tracking carrier loop.
 * The algorithm is described in
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H. Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007, Applied and Numerical Harmonic Analysis.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TRACKING_2ND_PLL_FILTER_H
#define GNSS_SDR_TRACKING_2ND_PLL_FILTER_H

/*!
 * \brief This class implements a 2nd order PLL filter for carrier tracking loop.
 *
 * The algorithm is described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S. H. Jensen, A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007, Applied and Numerical Harmonic Analysis.
 */
class Tracking_2nd_PLL_filter
{
public:
    void set_PLL_BW(float pll_bw_hz);  //!< Set PLL loop bandwidth [Hz]
    void set_pdi(float pdi_carr);      //!< Set Summation interval for code [s]
    void initialize();
    float get_carrier_nco(float PLL_discriminator);
    explicit Tracking_2nd_PLL_filter(float pdi_carr);
    Tracking_2nd_PLL_filter();
    ~Tracking_2nd_PLL_filter() = default;

private:
    // PLL filter parameters
    float d_tau1_carr = 0.0;
    float d_tau2_carr = 0.0;
    float d_pdi_carr = 0.0;
    float d_pllnoisebandwidth = 0.0;
    float d_plldampingratio = 0.0;
    float d_old_carr_error = 0.0;
    float d_old_carr_nco = 0.0;
    void calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k);
};

#endif

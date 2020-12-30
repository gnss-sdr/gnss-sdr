/*!
 * \file tracking_2nd_DLL_filter.h
 * \brief Interface of a 2nd order DLL filter for code tracking loop.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements a 2nd order PLL filter for code tracking loop.
 * The algorithm is described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S. H. Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
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

#ifndef GNSS_SDR_TRACKING_2ND_DLL_FILTER_H
#define GNSS_SDR_TRACKING_2ND_DLL_FILTER_H

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief This class implements a 2nd order DLL filter for code tracking loop.
 *
 * The algorithm is described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S. H. Jensen, A Software-Defined GPS
 * and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007, Applied and Numerical Harmonic Analysis.
 */
class Tracking_2nd_DLL_filter
{
public:
    Tracking_2nd_DLL_filter();
    ~Tracking_2nd_DLL_filter() = default;
    explicit Tracking_2nd_DLL_filter(float pdi_code);

    void set_DLL_BW(float dll_bw_hz);             //!< Set DLL filter bandwidth [Hz]
    void set_pdi(float pdi_code);                 //!< Set Summation interval for code [s]
    void initialize();                            //!< Start tracking with acquisition information
    float get_code_nco(float DLL_discriminator);  //!< Numerically controlled oscillator

private:
    void calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k);

    // PLL filter parameters
    float d_tau1_code = 0.0;
    float d_tau2_code = 0.0;
    float d_pdi_code = 0.0;
    float d_dllnoisebandwidth = 0.0;
    float d_dlldampingratio = 0.0;
    float d_old_code_error = 0.0;
    float d_old_code_nco = 0.0;
};


/** \} */
/** \} */
#endif

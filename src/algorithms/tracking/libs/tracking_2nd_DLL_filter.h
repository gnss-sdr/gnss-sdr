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
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TRACKING_2ND_DLL_FILTER_H_
#define GNSS_SDR_TRACKING_2ND_DLL_FILTER_H_

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
private:
    // PLL filter parameters
    float d_tau1_code = 0.0;
    float d_tau2_code = 0.0;
    float d_pdi_code = 0.0;
    float d_dllnoisebandwidth = 0.0;
    float d_dlldampingratio = 0.0;
    float d_old_code_error = 0.0;
    float d_old_code_nco = 0.0;
    void calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta, float k);

public:
    void set_DLL_BW(float dll_bw_hz);             //! Set DLL filter bandwidth [Hz]
    void set_pdi(float pdi_code);                 //! Set Summation interval for code [s]
    void initialize();                            //! Start tracking with acquisition information
    float get_code_nco(float DLL_discriminator);  //! Numerically controlled oscillator
    Tracking_2nd_DLL_filter(float pdi_code);
    Tracking_2nd_DLL_filter();
    ~Tracking_2nd_DLL_filter();
};

#endif

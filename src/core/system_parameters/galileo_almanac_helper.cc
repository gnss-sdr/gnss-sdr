/*!
 * \file galileo_almanac_helper.cc
 * \brief  Implementation of a Galileo ALMANAC storage helper
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_almanac_helper.h"

Galileo_Almanac_Helper::Galileo_Almanac_Helper()
{
    // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
    IOD_a_7 = 0;
    WN_a_7 = 0.0;
    t0a_7 = 0.0;
    SVID1_7 = 0;
    DELTA_A_7 = 0.0;
    e_7 = 0.0;
    omega_7 = 0.0;
    delta_i_7 = 0.0;
    Omega0_7 = 0.0;
    Omega_dot_7 = 0.0;
    M0_7 = 0.0;

    // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
    IOD_a_8 = 0;
    af0_8 = 0.0;
    af1_8 = 0.0;
    E5b_HS_8 = 0.0;
    E1B_HS_8 = 0.0;
    E5a_HS_8 = 0.0;
    SVID2_8 = 0;
    DELTA_A_8 = 0.0;
    e_8 = 0.0;
    omega_8 = 0.0;
    delta_i_8 = 0.0;
    Omega0_8 = 0.0;
    Omega_dot_8 = 0.0;

    // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
    IOD_a_9 = 0;
    WN_a_9 = 0.0;
    t0a_9 = 0.0;
    M0_9 = 0.0;
    af0_9 = 0.0;
    af1_9 = 0.0;
    E5b_HS_9 = 0.0;
    E1B_HS_9 = 0.0;
    E5a_HS_9 = 0.0;
    SVID3_9 = 0;
    DELTA_A_9 = 0.0;
    e_9 = 0.0;
    omega_9 = 0.0;
    delta_i_9 = 0.0;

    // Word type 10: Almanac for SVID3 (2/2)
    IOD_a_10 = 0;
    Omega0_10 = 0.0;
    Omega_dot_10 = 0.0;
    M0_10 = 0.0;
    af0_10 = 0.0;
    af1_10 = 0.0;
    E5b_HS_10 = 0.0;
    E1B_HS_10 = 0.0;
    E5a_HS_10 = 0.0;
}

Galileo_Almanac Galileo_Almanac_Helper::get_almanac(int i)
{
    Galileo_Almanac galileo_almanac;
    switch (i)
        {
        case 1:
            galileo_almanac.i_satellite_PRN = this->SVID1_7;
            galileo_almanac.d_Toa = this->t0a_7;
            galileo_almanac.d_WNa = this->WN_a_7;
            galileo_almanac.d_IODa = this->IOD_a_7;
            galileo_almanac.d_Delta_i = this->delta_i_7;
            galileo_almanac.d_M_0 = this->M0_7;
            galileo_almanac.d_e_eccentricity = this->e_7;
            galileo_almanac.d_Delta_sqrt_A = this->DELTA_A_7;
            galileo_almanac.d_OMEGA0 = this->Omega0_7;
            galileo_almanac.d_OMEGA = this->omega_7;
            galileo_almanac.d_OMEGA_DOT = this->Omega_dot_7;
            galileo_almanac.d_A_f0 = this->af0_8;
            galileo_almanac.d_A_f1 = this->af1_8;
            galileo_almanac.E5b_HS = this->E5b_HS_8;
            galileo_almanac.E1B_HS = this->E1B_HS_8;
            galileo_almanac.E5a_HS = this->E5a_HS_8;
            break;

        case 2:
            galileo_almanac.i_satellite_PRN = this->SVID2_8;
            galileo_almanac.d_Toa = this->t0a_9;
            galileo_almanac.d_WNa = this->WN_a_9;
            galileo_almanac.d_IODa = this->IOD_a_9;
            galileo_almanac.d_Delta_i = this->delta_i_8;
            galileo_almanac.d_M_0 = this->M0_9;
            galileo_almanac.d_e_eccentricity = this->e_8;
            galileo_almanac.d_Delta_sqrt_A = this->DELTA_A_8;
            galileo_almanac.d_OMEGA0 = this->Omega0_8;
            galileo_almanac.d_OMEGA = this->omega_8;
            galileo_almanac.d_OMEGA_DOT = this->Omega_dot_8;
            galileo_almanac.d_A_f0 = this->af0_9;
            galileo_almanac.d_A_f1 = this->af1_9;
            galileo_almanac.E5b_HS = this->E5b_HS_9;
            galileo_almanac.E1B_HS = this->E1B_HS_9;
            galileo_almanac.E5a_HS = this->E5a_HS_9;
            break;

        case 3:
            galileo_almanac.i_satellite_PRN = this->SVID3_9;
            galileo_almanac.d_Toa = this->t0a_9;
            galileo_almanac.d_WNa = this->WN_a_9;
            galileo_almanac.d_IODa = this->IOD_a_10;
            galileo_almanac.d_Delta_i = this->delta_i_9;
            galileo_almanac.d_M_0 = this->M0_10;
            galileo_almanac.d_e_eccentricity = this->e_9;
            galileo_almanac.d_Delta_sqrt_A = this->DELTA_A_9;
            galileo_almanac.d_OMEGA0 = this->Omega0_10;
            galileo_almanac.d_OMEGA = this->omega_9;
            galileo_almanac.d_OMEGA_DOT = this->Omega_dot_10;
            galileo_almanac.d_A_f0 = this->af0_10;
            galileo_almanac.d_A_f1 = this->af1_10;
            galileo_almanac.E5b_HS = this->E5b_HS_10;
            galileo_almanac.E1B_HS = this->E1B_HS_10;
            galileo_almanac.E5a_HS = this->E5a_HS_10;
            break;

        default:
            break;
        }
    return galileo_almanac;
}

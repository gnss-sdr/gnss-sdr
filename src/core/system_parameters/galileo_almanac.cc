/*!
 * \file galileo_almanac.cc
 * \brief  Implementation of a Galileo ALMANAC storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_almanac.h"

Galileo_Almanac::Galileo_Almanac()
{
    /*Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/
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

    /*Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
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

    /*Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/
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

    /*Word type 10: Almanac for SVID3 (2/2)*/
    IOD_a_10 = 0;
    Omega0_10 = 0.0;
    Omega_dot_10 = 0.0;
    M0_10 = 0.0;
    af0_10 = 0.0;
    af1_10 = 0.0;
    E5b_HS_10 = 0.0;
    E1B_HS_10 = 0.0;
    E5a_HS_10 = 0.0;

    /*GPS to Galileo GST conversion parameters*/
    A_0G_10 = 0.0;
    A_1G_10 = 0.0;
    t_0G_10 = 0.0;
    WN_0G_10 = 0.0;
}

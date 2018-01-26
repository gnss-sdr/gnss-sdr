/*!
 * \file glonass_gnav_almanac.cc
 * \brief  Interface of a GLONASS GNAV ALMANAC storage as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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

#include "glonass_gnav_almanac.h"

Glonass_Gnav_Almanac::Glonass_Gnav_Almanac()
{
    i_satellite_freq_channel = 0;
    i_satellite_PRN = 0;
    i_satellite_slot_number = 0;

    d_n_A = 0.0;
    d_H_n_A = 0.0;
    d_lambda_n_A = 0.0;
    d_t_lambda_n_A = 0.0;
    d_Delta_i_n_A = 0.0;
    d_Delta_T_n_A = 0.0;
    d_Delta_T_n_A_dot = 0.0;
    d_epsilon_n_A = 0.0;
    d_omega_n_A = 0.0;
    d_M_n_A = 0.0;
    d_KP = 0.0;
    d_tau_n_A = 0.0;
    d_C_n = false;
    d_l_n = false;
}

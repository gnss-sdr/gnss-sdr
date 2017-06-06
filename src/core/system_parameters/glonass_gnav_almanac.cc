/*!
 * \file glonass_gnav_almanac.cc
 * \brief  Interface of a GLONASS GNAV ALMANAC storage
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Damian Miralles , 2017. dmiralles2009(at)gmail.com
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
  int i_satellite_freq_channel = 0;
  double d_tau_c = 0.0;
  double d_tau_gps = 0.0;
  double d_N_4 = 0.0;
  double d_N_A = 0.0;
  double d_n_A = 0.0;
  double d_H_n_A = 0.0;
  double d_lambda_n_A = 0.0;
  double d_t_lambda_n_A = 0.0;
  double d_Delta_i_n_A = 0.0;
  double d_Delta_T_n_A = 0.0;
  double d_Delta_T_n_A_dot = 0.0;
  double d_epsilon_n_A = 0.0;
  double d_omega_n_A = 0.0;
  double d_M_n_A = 0.0;
  double d_B1 = 0.0;
  double d_B2 = 0.0;
  double d_KP = 0.0;
  double d_tau_n_A = 0.0;
  double d_C_n_A = 0.0;
}

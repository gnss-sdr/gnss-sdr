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
#include <cmath>
#include "GLONASS_L1_CA.h"
#include "gnss_satellite.h"

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
  d_C_n = 0.0;
  d_l_n = 0.0;
}

void Glonass_Gnav_Almanac::satellite_position(double N_A, double N_i, double t_i)
{
  double T_nom = 43200;     // [seconds]
  double i_nom = D2R*63.0;  // [rad]

  double Delta_t = 0.0;
  double i = 0.0;
  double T = 0.0;
  double n = 0.0;
  double a = 0.0;
  double lambda_dot = 0.0;
  double omega_dot = 0.0;
  double lambda = 0.0;
  double omega = 0.0;
  double E_P = 0.0;
  double Delta_T = 0.0;
  double M = 0.0;
  double E = 0.0;
  double E_old = 0.0;
  double dE = 0.0;

  double e1_x = 0.0;
  double e1_y = 0.0;
  double e1_z = 0.0;

  double e2_x = 0.0;
  double e2_y = 0.0;
  double e2_z = 0.0;
  // Compute time difference to reference time
  Delta_t = (N_i - N_A) * 86400 + (t_i + d_t_lambda_n_A);

  // Compute the actual inclination
  i = i_nom + d_Delta_i_n_A;

  // Compute the actual orbital period:
  T = T_nom + d_Delta_T_n_A;

  // Compute the mean motion
  n = 2*GLONASS_PI/T;

  // Compute the semi-major axis:
  a = cbrt(GLONASS_GM/(n*n));

  // Compute correction to longitude of ascending node
  lambda_dot = -10*pow(GLONASS_SEMI_MAJOR_AXIS / a, 7/2)*D2R*cos(i)/86400;

  // Compute correction to argument of perigee
  omega_dot = 5*pow(GLONASS_SEMI_MAJOR_AXIS / a, 7/2)*D2R*(5*cos(i)*cos(i) - 1)/86400;

  // Compute corrected longitude of ascending node:
  lambda = d_lambda_n_A + (lambda_dot - GLONASS_OMEGA_EARTH_DOT)*Delta_t;

  // Compute corrected argument of perigee:
  omega = d_omega_n_A + omega_dot*Delta_t;

  // Compute eccentric anomaly at point P: Note: P is that point of the orbit the true anomaly of which is identical to the argument of perigee.
  E_P = 2*atan(tan((omega/2)*(sqrt((1 - d_epsilon_n_A)*(1 + d_epsilon_n_A)))));

  // Compute time difference to perigee passing
  if (omega < GLONASS_PI)
    {
        Delta_T = (E_P - d_epsilon_n_A*sin(E_P))/n;
    }
  else
    {
      Delta_T = (E_P - d_epsilon_n_A*sin(E_P))/n + T;
    }

  // Compute mean anomaly at epoch t_i:
  M = n * (Delta_t - Delta_T);

  // Compute eccentric anomaly at epoch t_i. Note: Kepler’s equation has to be solved iteratively

  // Initial guess of eccentric anomaly
  E = M;

  // --- Iteratively compute eccentric anomaly ----------------------------
  for (int ii = 1; ii < 20; ii++)
      {
          E_old   = E;
          E       = M + d_epsilon_n_A * sin(E);
          dE      = fmod(E - E_old, 2.0 * GLONASS_PI);
          if (fabs(dE) < 1e-12)
              {
                  //Necessary precision is reached, exit from the loop
                  break;
              }
      }

  // Compute position in orbital coordinate system
  d_satpos_Xo = a*cos(E) - d_epsilon_n_A;
  d_satpos_Yo = a*sqrt(1 - d_epsilon_n_A*d_epsilon_n_A)*sin(E);
  d_satpos_Zo = a*0;

  // Compute velocity in orbital coordinate system
  d_satvel_Xo = a/(1-d_epsilon_n_A*cos(E))*(-n*sin(E));
  d_satvel_Yo = a/(1-d_epsilon_n_A*cos(E))*(n*sqrt(1 - d_epsilon_n_A*d_epsilon_n_A)*cos(E));
  d_satvel_Zo = a/(1-d_epsilon_n_A*cos(E))*(0);

  // Determine orientation vectors of orbital coordinate system in ECEF system
  e1_x = cos(omega)*cos(lambda) - sin(omega)*sin(lambda);
  e1_y = cos(omega)*sin(lambda) + sin(omega)*cos(lambda)*cos(i);
  e1_z = sin(omega)*sin(i);

  e2_x = -sin(omega)*cos(lambda) - sin(omega)*sin(lambda)*cos(i);
  e2_y = -sin(omega)*sin(lambda) + cos(omega)*cos(lambda)*cos(i);
  e2_z = cos(omega)*sin(i);

  // Convert position from orbital to ECEF system
  d_satpos_X = d_satpos_Xo*e1_x + d_satpos_Xo*e2_x;
  d_satpos_Y = d_satpos_Yo*e1_z + d_satpos_Yo*e2_y;
  d_satpos_Z = d_satpos_Zo*e1_z + d_satpos_Zo*e2_z;

  // Convert position from orbital to ECEF system
  d_satvel_X = d_satvel_Xo*e1_x + d_satvel_Xo*e2_x + GLONASS_OMEGA_EARTH_DOT*d_satpos_Y;
  d_satvel_Y = d_satvel_Yo*e1_z + d_satvel_Yo*e2_y - GLONASS_OMEGA_EARTH_DOT*d_satpos_X;
  d_satvel_Z = d_satvel_Zo*e1_z + d_satvel_Zo*e2_z;

}

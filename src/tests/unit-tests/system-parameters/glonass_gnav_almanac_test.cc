/*!
 * \file code_generation_test.cc
 * \brief  This file implements tests for the generation of complex exponentials.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
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


#include <complex>
#include <ctime>
#include "gps_sdr_signal_processing.h"
#include "gnss_signal_processing.h"



#include <complex>
#include <ctime>
#include "gnss_signal_processing.h"
#include "glonass_gnav_almanac.h"

// See A 3.2.3
TEST(GlonassGnavAlmanacTest, SatellitePosition)
{
    double N_i = 615;           // [days]
    double t_i = 33300.0;        // [seconds]
    double Xoi = 10947.021572;  // [km]
    double Yoi = 13078.978287;  // [km]
    double Zoi = 18922.063362;  // [km]
    double Vxoi = -3.375497;     // [m/s]
    double Vyoi = -0.161453;    // [Кm/s]
    double Vzoi = 2.060844;     // [Кm/s]
    double N_A  = 615;          // [days]

    Glonass_Gnav_Almanac gnav_almanac;


    gnav_almanac.d_lambda_n_A       = -0.189986229;     // [half cycles]
    gnav_almanac.d_t_lambda_n_A     = 27122.09375;      // [second]
    gnav_almanac.d_Delta_i_n_A      = 0.011929512;      // [half cycle]
    gnav_almanac.d_Delta_T_n_A      = -2655.76171875;   // [seconds]
    gnav_almanac.d_Delta_T_n_A_dot  = 0.000549316;      // [Secjnds/cycle2]
    gnav_almanac.d_epsilon_n_A      = 0.001482010;      // [unitless]
    gnav_almanac.d_omega_n_A        = 0.440277100;      // [Half cycle]

    gnav_almanac.satellite_position(N_A, N_i, t_i);

    ASSERT_TRUE(gnav_almanac.d_satpos_Xo - Xoi < DBL_EPSILON );
    ASSERT_TRUE(gnav_almanac.d_satpos_Yo - Yoi < DBL_EPSILON );
    ASSERT_TRUE(gnav_almanac.d_satpos_Zo - Zoi < DBL_EPSILON );
    ASSERT_TRUE(gnav_almanac.d_satvel_Xo - Vxoi < DBL_EPSILON );
    ASSERT_TRUE(gnav_almanac.d_satvel_Yo - Vyoi < DBL_EPSILON );
    ASSERT_TRUE(gnav_almanac.d_satvel_Zo - Vzoi < DBL_EPSILON );
}

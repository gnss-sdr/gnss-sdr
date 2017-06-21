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
#include "gnss_signal_processing.h"
#include "glonass_gnav_ephemeris.h"


TEST(GlonassGnavEphemerisTest, SatellitePosition)
{
    Glonass_Gnav_Ephemeris gnav_eph;

    gnav_eph.d_Xn = 12317.934082000;
    gnav_eph.d_Yn = -2245.13232422;
    gnav_eph.d_Zn = 22212.8173828;
    gnav_eph.d_VXn = -1.25356674194;
    gnav_eph.d_VYn = 2.774200439450;
    gnav_eph.d_VZn = 0.9808206558230000;
    gnav_eph.d_AXn = -0.931322574616e-9;
    gnav_eph.d_AYn = 0.0000000000000000;
    gnav_eph.d_AZn =  -0.186264514923e-8;

    gnav_eph.simplified_satellite_position(60);
}

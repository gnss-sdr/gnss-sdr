/*!
 * \file gps_almanac.cc
 * \brief  Interface of a BEIDOU ALMANAC storage
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_almanac.h"

Beidou_Almanac::Beidou_Almanac()
{
    i_satellite_PRN = 0;
    d_Delta_i = 0.0;
    d_Toa = 0.0;
    d_M_0 = 0.0;
    d_e_eccentricity = 0.0;
    d_sqrt_A = 0.0;
    d_OMEGA0 = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    i_SV_health = 0;
    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
}

/*!
 * \file beidou_cnav2_almanac.cc
 * \brief  Interface of a Beidou CNAV2 ALMANAC storage as described in Beidou ICD
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">Beidou ICD</a>
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

#include "beidou_cnav2_almanac.h"

Beidou_Cnav2_Almanac::Beidou_Cnav2_Almanac()
{
	// Midi Almanac Parameters

	i_satellite_PRN = 0;					//!< PRN number of the corresponding almanac data [dimensionless]
	SatType = 0.0;				//!< Satellite orbit type [dimensionless]
	i_BDS_week = 0.0;					//!< Almanac reference week number [week]
	t_oa = 0.0;					//!< Almanac reference time [s]
	e = 0.0;					//!< Eccentricity [dimensionless]
	delta_i = 0.0;				//!< Correction of inclination angle relative to reference value at reference time [pi]
	sqrt_A = 0.0;				//!< Square root of semi-major axis [m^1/2]
	Omega_0 = 0.0;				//!< Longitude of ascending node of orbital plane at weekly epoch [pi]
	Omega_dot = 0.0;			//!< Rate of right ascension [pi/s]
	omega = 0.0;				//!< Argument of perigee [pi]
	M_0 = 0.0;					//!< Mean anomaly at reference time [pi]
	a_f0 = 0.0;					//!< Satellite clock time bias correction coefficient [s]
	a_f1 = 0.0;					//!< Satellite clock time drift correction coefficient [s/s]
	Health = 0.0;				//!< Satellite health information [dimensionless]

	// Reduced Almanac Parameters

	delta_A = 0.0;				//!< Correction of semi-major axis relative to reference value at reference time
	Phi_0 = 0.0;				//!< Argument of latitude at reference time
	//Health = 0.0;				//!< Satellite health information
}

double Beidou_Cnav2_Almanac::BDS_time_of_transmission(double t_sv)
{
	double t;
	double dt_sv;

	dt_sv = a_f0 + a_f1 * (t-t_oa);

	t = t_sv - dt_sv;
	return t;
}
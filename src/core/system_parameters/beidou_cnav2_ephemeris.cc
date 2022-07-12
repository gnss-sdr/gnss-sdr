/*!
 * \file beidou_cnav2_ephemeris.cc
 * \brief  Interface of a BEIDOU CNAV2 EPHEMERIS storage and orbital model functions
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
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

#include "beidou_cnav2_ephemeris.h"
#include "gnss_satellite.h"
#include <cmath>


Beidou_Cnav2_Ephemeris::Beidou_Cnav2_Ephemeris()
{
	// Other values
	i_satellite_PRN = 0;
	SOW = 0.0;
	i_BDS_week = 0;

	// Satellite Health Satus
	HS = 0.0;			//0:Satellite is healthy/provides services, 1:Satellite is unhealthy or in test/does not provide services, 2:reserved/reserved, 3:reserved/reserved

	// Issue of Data, Ephemeris
	IODE = 0.0;

	// Issue of Data, Clock
	IODC = 0.0;

	// Ephemeris
    t_oe = 0.0;				//!< Ephemeris reference time [s]
    SatType = 0.0;			//!< Satellite orbit type [dimensionless]
    dA = 0.0;				//!< Semi-major axis difference at reference time [m]
    A_dot = 0.0;			//!< Change rate in semi-major axis [m/s]
    dn_0 = 0.0;				//!< Mean motion difference from computed value at reference time [pi/s]
    dn_0_dot = 0.0;			//!< Rate of mean motion difference from computed value at reference time [pi/s^2]
    M_0 = 0.0;				//!< Mean anomaly at reference time [pi]
    e = 0.0;				//!< Eccenticity [dimensionless]
    omega = 0.0;			//!< Argument of perigee [pi]
    Omega_0 = 0.0;			//!< Longitude of ascending node of orbital plane at weekly epoch [pi]
    i_0 = 0.0;				//!< Inclination angle at reference time [pi]
    Omega_dot = 0.0;		//!< Rate of right ascension [pi/s]
    i_0_dot = 0.0;			//!< Rate of inclination angle [pi/s]
    C_IS = 0.0;				//!< Amplitude of sine harmonic correction term to the angle of inclination [rad]
    C_IC = 0.0;				//!< Amplitude of cosine harmonic correction term to the angle of inclination [rad]
    C_RS = 0.0;				//!< Amplitude of sine harmonic correction term to the orbit radius [m]
    C_RC = 0.0;				//!< Amplitude of cosine harmonic correction term to the orbit radius [m]
    C_US = 0.0;				//!< Amplitude of sine harmonic correction to the argument of latitude [rad]
    C_UC = 0.0;				//!< Amplitude of cosine harmonic correction to the argument of latitude [rad]

	// Earth Orientation Parameters
	t_EOP = 0.0;			//EOP data reference time [s]
	PM_X = 0.0;				//X Axis polar motion value at reference time [arc s]
	PM_X_dot = 0.0;			//X Axis polar motion drift at reference time [arc s/day]
	PM_Y = 0.0;				//Y Axis polar motion value at reference time [arc s]
	PM_Y_dot = 0.0;			//Y Axis polar motion drift at reference time [arc s/day]
	dUT1 = 0.0;				//UT1-UTC difference at reference time [s]
	dUT1_dot = 0.0;			//Rate of UT1-UTC difference at reference time [s/day]

	// Satellite Integrity Flag
	DIF = 0.0;
	SIF = 0.0;
	AIF = 0.0;
	DIF_B1C = 0.0;
	SIF_B1C = 0.0;
	AIF_B1C = 0.0;
	SISMAI = 0.0;

	// Signal In Space Accuracy Index
	SISAI_OE = 0.0;		//Satellite orbit along-track and cross-track accuracy index
	t_op = 0.0;			//Time of week for data prediction
	SISAI_ocb = 0.0;	//Satellite orbit radius and fixed satellite clock bias accuracy index
	SISAI_oc1 = 0.0;	//Satellite clock bias accuracy index
	SISAI_oc2 = 0.0;	//Satellite clock drift accuracy index

	// Group Delay Differential Parameters
	T_GDB1Cp = 0.0;	//Group delay differential of the B1C pilot component [s]
	T_GDB2ap = 0.0;	//Group delay differential of the B2a pilot component [s]
	ISC_B2ad = 0.0;	//Group delay differential between the B2a data and pilot components [s]

	// Clock Correction Parameters
	t_oc = 0.0;			//Clock correction parameters reference time [s] effective range 0~604500
	a_0 = 0.0;			//Satellite clock time bias correction coefficient [s]
	a_1 = 0.0;			//Satellite clock time drift correction coefficient [s/s]
	a_2 = 0.0;			//Satellite clock time drift rate correction coefficient [s/s^2]

}

double Beidou_Cnav2_Ephemeris::B2a_ranging_code_phase_correction_w_pilot(double dt_sv)
{
	double dt_sv_B2ap;
	dt_sv_B2ap = dt_sv - T_GDB2ap;
	return dt_sv_B2ap;
}

double Beidou_Cnav2_Ephemeris::B2a_ranging_code_phase_correction_w_data(double dt_sv)
{
	double dt_sv_B2ad;
	dt_sv_B2ad = dt_sv - T_GDB2ap - ISC_B2ad;
	return dt_sv_B2ad;
}

double Beidou_Cnav2_Ephemeris::B2a_UT1_UTC_difference(double t)
{
	return dUT1 -dUT1_dot*(t-t_EOP);
}

double Beidou_Cnav2_Ephemeris::B2a_Polar_motion_x(double t)
{
	double x_p;

	x_p = PM_X + PM_X_dot * (t-t_EOP);
	return x_p;
}

double Beidou_Cnav2_Ephemeris::B2a_Polar_motion_y(double t)
{
	double y_p;

	y_p = PM_Y + PM_Y_dot * (t-t_EOP);
	return y_p;
}
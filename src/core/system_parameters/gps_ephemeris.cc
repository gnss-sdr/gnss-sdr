/*!
 * \file gps_ephemeris.cc
 * \brief  Interface of a GPS EPHEMERIS storage and orbital model functions
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include "gps_ephemeris.h"

Gps_Ephemeris::Gps_Ephemeris()
{

	//Plane A (info from http://www.navcen.uscg.gov/?Do=constellationStatus)
	satelliteBlock[9] = "IIA";
	satelliteBlock[31] = "IIR-M";
	satelliteBlock[8] = "IIA";
	satelliteBlock[7] = "IIR-M";
	satelliteBlock[27] = "IIA";
	//Plane B
	satelliteBlock[16] = "IIR";
	satelliteBlock[25] = "IIF";
	satelliteBlock[28] = "IIR";
	satelliteBlock[12] = "IIR-M";
	satelliteBlock[30] = "IIA";
	//Plane C
	satelliteBlock[29] = "IIR-M";
	satelliteBlock[3] = "IIA";
	satelliteBlock[19] = "IIR";
	satelliteBlock[17] = "IIR-M";
	satelliteBlock[6] = "IIA";
	//Plane D
	satelliteBlock[2] = "IIR";
	satelliteBlock[1] = "IIF";
	satelliteBlock[21] = "IIR";
	satelliteBlock[4] = "IIA";
	satelliteBlock[11] = "IIR";
	satelliteBlock[24] = "IIA"; // Decommissioned from active service on 04 Nov 2011
	//Plane E
	satelliteBlock[20] = "IIR";
	satelliteBlock[22] = "IIR";
	satelliteBlock[5] = "IIR-M";
	satelliteBlock[18] = "IIR";
	satelliteBlock[32] = "IIA";
	satelliteBlock[10] = "IIA";
	//Plane F
	satelliteBlock[14] = "IIR";
	satelliteBlock[15] = "IIR-M";
	satelliteBlock[13] = "IIR";
	satelliteBlock[23] = "IIR";
	satelliteBlock[26] = "IIA";

}


double Gps_Ephemeris::check_t(double time)
{
	double corrTime;
	double half_week = 302400;     // seconds
	corrTime = time;
	if (time > half_week)
	{
		corrTime = time - 2*half_week;
	}
	else if (time < -half_week)
	{
		corrTime = time + 2*half_week;
	}
	return corrTime;
}


// 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
double Gps_Ephemeris::sv_clock_correction(double transmitTime)
{
	double dt;
	dt = check_t(transmitTime - d_Toc);
	d_satClkCorr = (d_A_f2 * dt + d_A_f1) * dt + d_A_f0 + d_dtr;
	double correctedTime = transmitTime - d_satClkCorr;
	return correctedTime;
}



void Gps_Ephemeris::satellitePosition(double transmitTime)
{
	double tk;
	double a;
	double n;
	double n0;
	double M;
	double E;
	double E_old;
	double dE;
	double nu;
	double phi;
	double u;
	double r;
	double i;
	double Omega;

	// Find satellite's position ----------------------------------------------

	// Restore semi-major axis
	a = d_sqrt_A*d_sqrt_A;

	// Time from ephemeris reference epoch
	tk = check_t(transmitTime - d_Toe);

	// Computed mean motion
	n0 = sqrt(GM / (a*a*a));

	// Corrected mean motion
	n = n0 + d_Delta_n;

	// Mean anomaly
	M = d_M_0 + n * tk;

	// Reduce mean anomaly to between 0 and 2pi
	M = fmod((M + 2*GPS_PI), (2*GPS_PI));

	// Initial guess of eccentric anomaly
	E = M;

	// --- Iteratively compute eccentric anomaly ----------------------------
	for (int ii = 1; ii<20; ii++)
	{
		E_old   = E;
		E       = M + d_e_eccentricity * sin(E);
		dE      = fmod(E - E_old, 2*GPS_PI);
		if (fabs(dE) < 1e-12)
		{
			//Necessary precision is reached, exit from the loop
			break;
		}
	}

	// Compute relativistic correction term
	d_dtr = F * d_e_eccentricity * d_sqrt_A * sin(E);

	// Compute the true anomaly
	double tmp_Y = sqrt(1.0 - d_e_eccentricity * d_e_eccentricity) * sin(E);
	double tmp_X = cos(E) - d_e_eccentricity;
	nu = atan2(tmp_Y, tmp_X);

	// Compute angle phi (argument of Latitude)
	phi = nu + d_OMEGA;

	// Reduce phi to between 0 and 2*pi rad
	phi = fmod((phi), (2*GPS_PI));

	// Correct argument of latitude
	u = phi + d_Cuc * cos(2*phi) +  d_Cus * sin(2*phi);

	// Correct radius
	r = a * (1 - d_e_eccentricity*cos(E)) +  d_Crc * cos(2*phi) +  d_Crs * sin(2*phi);

	// Correct inclination
	i = d_i_0 + d_IDOT * tk + d_Cic * cos(2*phi) + d_Cis * sin(2*phi);

	// Compute the angle between the ascending node and the Greenwich meridian
	Omega = d_OMEGA0 + (d_OMEGA_DOT - OMEGA_EARTH_DOT)*tk - OMEGA_EARTH_DOT * d_Toe;

	// Reduce to between 0 and 2*pi rad
	Omega = fmod((Omega + 2*GPS_PI), (2*GPS_PI));

	// --- Compute satellite coordinates in Earth-fixed coordinates
	d_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
	d_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
	d_satpos_Z = sin(u) * r * sin(i);

	// Satellite's velocity. Can be useful for Vector Tracking loops
	double Omega_dot = d_OMEGA_DOT - OMEGA_EARTH_DOT;
	d_satvel_X = - Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + d_satpos_X * cos(Omega) - d_satpos_Y * cos(i) * sin(Omega);
	d_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + d_satpos_X * sin(Omega) + d_satpos_Y * cos(i) * cos(Omega);
	d_satvel_Z = d_satpos_Y * sin(i);
}

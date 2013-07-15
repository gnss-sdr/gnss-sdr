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

#include "galileo_ephemeris.h"

Galileo_Ephemeris::Galileo_Ephemeris()
{
    i_satellite_PRN = 0;
    d_TOW = 0;
    d_Crs = 0;
    d_Delta_n = 0;
    d_M_0 = 0;
    d_Cuc = 0;
    d_e_eccentricity = 0;
    d_Cus = 0;
    d_sqrt_A = 0;
    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0;
    d_OMEGA0 = 0;
    d_Cis = 0;
    d_i_0 = 0;
    d_Crc = 0;
    d_OMEGA = 0;
    d_OMEGA_DOT = 0;
    d_IDOT = 0;
    i_code_on_L2 = 0;
    i_GPS_week = 0;
    b_L2_P_data_flag = false;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD = 0;            //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    d_IODC = 0;           //!< Issue of Data, Clock
    i_AODO = 0;              //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    b_fit_interval_flag = false;//!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0  =  4 hours, 1  =  greater than 4 hours.
    d_spare1 = 0;
    d_spare2 = 0;

    d_A_f0 = 0;          //!< Coefficient 0 of code phase offset model [s]
    d_A_f1 = 0;          //!< Coefficient 1 of code phase offset model [s/s]
    d_A_f2 = 0;          //!< Coefficient 2 of code phase offset model [s/s^2]

    b_integrity_status_flag = false;
    b_alert_flag = false;      //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    b_antispoofing_flag = false;  //!<  If true, the AntiSpoofing mode is ON in that SV

}

void Galileo_Ephemeris::satellitePosition(double transmitTime)
{
	/*
	 * ToDo: Compute satellite position at transmit Time
	 */

    // --- Compute satellite coordinates in Earth-fixed coordinates
    d_satpos_X = 0;
    d_satpos_Y = 0;
    d_satpos_Z = 0;

    // Satellite's velocity.

    d_satvel_X =0;
    d_satvel_Y = 0;
    d_satvel_Z = 0;
}

/*!
 * \file Gps_CNAV_Ephemeris.cc
 * \brief  Interface of a GPS CNAV EPHEMERIS storage and orbital model functions
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "gps_cnav_ephemeris.h"

Gps_CNAV_Ephemeris::Gps_CNAV_Ephemeris()
{
    i_satellite_PRN = 0;

    d_TOW = 0;
    d_Crs = 0;
    d_M_0 = 0;
    d_Cuc = 0;
    d_e_eccentricity = 0;
    d_Cus = 0;

    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0;
    d_OMEGA0 = 0;
    d_Cis = 0;
    d_i_0 = 0;
    d_Crc = 0;
    d_OMEGA = 0;
    d_IDOT = 0;

    i_GPS_week = 0;

    d_TGD = 0;            // Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]

    d_A_f0 = 0;          // Coefficient 0 of code phase offset model [s]
    d_A_f1 = 0;          // Coefficient 1 of code phase offset model [s/s]
    d_A_f2 = 0;          // Coefficient 2 of code phase offset model [s/s^2]

    b_integrity_status_flag = false;
    b_alert_flag = false;         // If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    b_antispoofing_flag = false;  //  If true, the AntiSpoofing mode is ON in that SV

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

    d_satClkDrift = 0.0;
    d_dtr = 0.0;
    d_satpos_X = 0.0;
    d_satpos_Y = 0.0;
    d_satpos_Z = 0.0;
    d_satvel_X = 0.0;
    d_satvel_Y = 0.0;
    d_satvel_Z = 0.0;
}

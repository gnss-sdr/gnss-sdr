/*!
 * \file rtklib_conversions.cc
 * \brief GNSS-SDR to RTKLIB data structures conversion functions
 * \authors <ul>
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          <li> 2007-2013, T. Takasu
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/
#include "rtklib_conversions.h"


obsd_t obs_to_rtklib(Gnss_Synchro gnss_synchro, int week)
{
    obsd_t rtklib_obs;
    rtklib_obs.D[0] = gnss_synchro.Carrier_Doppler_hz;
    rtklib_obs.P[0] = gnss_synchro.Pseudorange_m;
    rtklib_obs.L[0] = gnss_synchro.Carrier_phase_rads;//todo: check units
    //rtklib_obs.SNR = gnss_synchro.CN0_dB_hz;
    rtklib_obs.sat = gnss_synchro.PRN;
    rtklib_obs.time = gpst2time(adjgpsweek(week),gnss_synchro.RX_time);
    //printf("OBS RX TIME [%i]: %s,%f\n\r",rtklib_obs.sat,time_str(rtklib_obs.time,3),rtklib_obs.time.sec);
    return rtklib_obs;
}


eph_t eph_to_rtklib(Galileo_Ephemeris gal_eph)
{
    eph_t rtklib_sat;
    rtklib_sat.sat = gal_eph.i_satellite_PRN;
    rtklib_sat.A = gal_eph.A_1*gal_eph.A_1;
    rtklib_sat.M0 = gal_eph.M0_1;
    rtklib_sat.deln = gal_eph.delta_n_3;
    rtklib_sat.OMG0 = gal_eph.OMEGA_0_2;
    rtklib_sat.OMGd = gal_eph.OMEGA_dot_3;
    rtklib_sat.omg = gal_eph.omega_2;
    rtklib_sat.i0 = gal_eph.i_0_2;
    rtklib_sat.idot = gal_eph.iDot_2;
    rtklib_sat.e = gal_eph.e_1;
    rtklib_sat.Adot = 0; //only in CNAV;
    rtklib_sat.ndot = 0; //only in CNAV;

    rtklib_sat.week = adjgpsweek(gal_eph.WN_5); /* week of tow */
    rtklib_sat.cic = gal_eph.C_ic_4;
    rtklib_sat.cis = gal_eph.C_is_4;
    rtklib_sat.cuc = gal_eph.C_uc_3;
    rtklib_sat.cus = gal_eph.C_us_3;
    rtklib_sat.crc = gal_eph.C_rc_3;
    rtklib_sat.crs = gal_eph.C_rs_3;
    rtklib_sat.f0 = gal_eph.af0_4;
    rtklib_sat.f1 = gal_eph.af1_4;
    rtklib_sat.f2 = gal_eph.af2_4;
    rtklib_sat.tgd[0] = 0;
    rtklib_sat.tgd[1] = 0;
    rtklib_sat.tgd[2] = 0;
    rtklib_sat.tgd[3] = 0;
    rtklib_sat.toes = gal_eph.t0e_1;
    rtklib_sat.toc = gpst2time(rtklib_sat.week,gal_eph.t0c_4);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,gal_eph.TOW_5);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr,&rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc,NULL);
    if      (rtklib_sat.toes<tow-302400.0) {rtklib_sat.week++; tow-=604800.0;}
    else if (rtklib_sat.toes>tow+302400.0) {rtklib_sat.week--; tow+=604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week,rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week,toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,tow);

    return rtklib_sat;
}


eph_t eph_to_rtklib(Gps_Ephemeris gps_eph)
{
    eph_t rtklib_sat;
    rtklib_sat.sat = gps_eph.i_satellite_PRN;
    rtklib_sat.A = gps_eph.d_sqrt_A*gps_eph.d_sqrt_A;
    rtklib_sat.M0 = gps_eph.d_M_0;
    rtklib_sat.deln = gps_eph.d_Delta_n;
    rtklib_sat.OMG0 = gps_eph.d_OMEGA0;
    rtklib_sat.OMGd = gps_eph.d_OMEGA_DOT;
    rtklib_sat.omg = gps_eph.d_OMEGA;
    rtklib_sat.i0 = gps_eph.d_i_0;
    rtklib_sat.idot = gps_eph.d_IDOT;
    rtklib_sat.e = gps_eph.d_e_eccentricity;
    rtklib_sat.Adot = 0; //only in CNAV;
    rtklib_sat.ndot = 0; //only in CNAV;

    rtklib_sat.week = adjgpsweek(gps_eph.i_GPS_week); /* week of tow */
    rtklib_sat.cic = gps_eph.d_Cic;
    rtklib_sat.cis = gps_eph.d_Cis;
    rtklib_sat.cuc = gps_eph.d_Cuc;
    rtklib_sat.cus = gps_eph.d_Cus;
    rtklib_sat.crc = gps_eph.d_Crc;
    rtklib_sat.crs = gps_eph.d_Crs;
    rtklib_sat.f0 = gps_eph.d_A_f0;
    rtklib_sat.f1 = gps_eph.d_A_f1;
    rtklib_sat.f2 = gps_eph.d_A_f2;
    rtklib_sat.tgd[0] = gps_eph.d_TGD;
    rtklib_sat.tgd[1] = 0;
    rtklib_sat.tgd[2] = 0;
    rtklib_sat.tgd[3] = 0;
    rtklib_sat.toes = gps_eph.d_Toe;
    rtklib_sat.toc = gpst2time(rtklib_sat.week,gps_eph.d_Toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,gps_eph.d_TOW);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr,&rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc,NULL);
    if      (rtklib_sat.toes<tow-302400.0) {rtklib_sat.week++; tow-=604800.0;}
    else if (rtklib_sat.toes>tow+302400.0) {rtklib_sat.week--; tow+=604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week,rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week,toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,tow);

    //printf("EPHEMERIS TIME [%i]: %s,%f\n\r",rtklib_sat.sat,time_str(rtklib_sat.toe,3),rtklib_sat.toe.sec);

    return rtklib_sat;
}


eph_t eph_to_rtklib(Gps_CNAV_Ephemeris gps_cnav_eph)
{
    eph_t rtklib_sat;
    rtklib_sat.sat = gps_cnav_eph.i_satellite_PRN;
    const double A_REF = 26559710.0; // See IS-GPS-200H,  pp. 170
    rtklib_sat.A = A_REF + gps_cnav_eph.d_DELTA_A;
    rtklib_sat.M0 = gps_cnav_eph.d_M_0;
    rtklib_sat.deln = gps_cnav_eph.d_Delta_n;
    rtklib_sat.OMG0 = gps_cnav_eph.d_OMEGA0;
    // Compute the angle between the ascending node and the Greenwich meridian
    const double OMEGA_DOT_REF = -2.6e-9; // semicircles / s, see IS-GPS-200H pp. 164
    double d_OMEGA_DOT = OMEGA_DOT_REF*GPS_L2_PI + gps_cnav_eph.d_DELTA_OMEGA_DOT;
    rtklib_sat.OMGd = d_OMEGA_DOT;
    rtklib_sat.omg = gps_cnav_eph.d_OMEGA;
    rtklib_sat.i0 = gps_cnav_eph.d_i_0;
    rtklib_sat.idot = gps_cnav_eph.d_IDOT;
    rtklib_sat.e = gps_cnav_eph.d_e_eccentricity;
    rtklib_sat.Adot = gps_cnav_eph.d_A_DOT; //only in CNAV;
    rtklib_sat.ndot = gps_cnav_eph.d_DELTA_DOT_N; //only in CNAV;

    rtklib_sat.week = adjgpsweek(gps_cnav_eph.i_GPS_week); /* week of tow */
    rtklib_sat.cic = gps_cnav_eph.d_Cic;
    rtklib_sat.cis = gps_cnav_eph.d_Cis;
    rtklib_sat.cuc = gps_cnav_eph.d_Cuc;
    rtklib_sat.cus = gps_cnav_eph.d_Cus;
    rtklib_sat.crc = gps_cnav_eph.d_Crc;
    rtklib_sat.crs = gps_cnav_eph.d_Crs;
    rtklib_sat.f0 = gps_cnav_eph.d_A_f0;
    rtklib_sat.f1 = gps_cnav_eph.d_A_f1;
    rtklib_sat.f2 = gps_cnav_eph.d_A_f2;
    rtklib_sat.tgd[0] = gps_cnav_eph.d_TGD;
    rtklib_sat.tgd[1] = 0;
    rtklib_sat.tgd[2] = 0;
    rtklib_sat.tgd[3] = 0;
    rtklib_sat.toes = gps_cnav_eph.d_Toe1;
    rtklib_sat.toc = gpst2time(rtklib_sat.week,gps_cnav_eph.d_Toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,gps_cnav_eph.d_TOW);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr,&rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc,NULL);
    if      (rtklib_sat.toes<tow-302400.0) {rtklib_sat.week++; tow-=604800.0;}
    else if (rtklib_sat.toes>tow+302400.0) {rtklib_sat.week--; tow+=604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week,rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week,toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week,tow);

    return rtklib_sat;
}

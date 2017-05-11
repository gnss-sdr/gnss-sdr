/*!
 * \file rtklib_conversions.cc
 * \brief GNSS-SDR to RTKLIB data structures conversion functions
 * \author 2017, Javier Arribas
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "rtklib_conversions.h"
#include "rtklib_rtkcmn.h"

obsd_t insert_obs_to_rtklib(obsd_t & rtklib_obs, const Gnss_Synchro & gnss_synchro, int week, int band)
{
    rtklib_obs.D[band] = gnss_synchro.Carrier_Doppler_hz;
    rtklib_obs.P[band] = gnss_synchro.Pseudorange_m;
    rtklib_obs.L[band] = gnss_synchro.Carrier_phase_rads / (2.0 * PI);

    double CN0_dB_Hz_est = gnss_synchro.CN0_dB_hz;
    if (CN0_dB_Hz_est > 63.75) CN0_dB_Hz_est = 63.75;
    if (CN0_dB_Hz_est < 0.0) CN0_dB_Hz_est = 0.0;
    unsigned char CN0_dB_Hz = static_cast<unsigned char>(std::round(CN0_dB_Hz_est / 0.25 ));
    rtklib_obs.SNR[band] = CN0_dB_Hz;
    //Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    switch(gnss_synchro.System)
    {
        case 'G':
            rtklib_obs.sat = gnss_synchro.PRN;
            break;
        case 'E':
            rtklib_obs.sat = gnss_synchro.PRN+NSATGPS+NSATGLO;
            break;
        default:
            rtklib_obs.sat = gnss_synchro.PRN;
    }
    rtklib_obs.time = gpst2time(adjgpsweek(week), gnss_synchro.RX_time);
    rtklib_obs.rcv = 1;
    //printf("OBS RX TIME [%i]: %s,%f\n\r",rtklib_obs.sat,time_str(rtklib_obs.time,3),rtklib_obs.time.sec);
    return rtklib_obs;
}

eph_t eph_to_rtklib(const Galileo_Ephemeris & gal_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0 };
    //Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    rtklib_sat.sat = gal_eph.i_satellite_PRN+NSATGPS+NSATGLO;
    rtklib_sat.A = gal_eph.A_1 * gal_eph.A_1;
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
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gal_eph.t0c_4);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gal_eph.TOW_5);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, NULL);
    if      (rtklib_sat.toes < tow - 302400.0) {rtklib_sat.week++; tow -= 604800.0;}
    else if (rtklib_sat.toes > tow + 302400.0) {rtklib_sat.week--; tow += 604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_Ephemeris & gps_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0 };
    rtklib_sat.sat = gps_eph.i_satellite_PRN;
    rtklib_sat.A = gps_eph.d_sqrt_A * gps_eph.d_sqrt_A;
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
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gps_eph.d_Toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gps_eph.d_TOW);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, NULL);
    if      (rtklib_sat.toes < tow - 302400.0) {rtklib_sat.week++; tow -= 604800.0;}
    else if (rtklib_sat.toes > tow + 302400.0) {rtklib_sat.week--; tow += 604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    //printf("EPHEMERIS TIME [%i]: %s,%f\n\r",rtklib_sat.sat,time_str(rtklib_sat.toe,3),rtklib_sat.toe.sec);

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris & gps_cnav_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0 };
    rtklib_sat.sat = gps_cnav_eph.i_satellite_PRN;
    const double A_REF = 26559710.0; // See IS-GPS-200H,  pp. 170
    rtklib_sat.A = A_REF + gps_cnav_eph.d_DELTA_A;
    rtklib_sat.M0 = gps_cnav_eph.d_M_0;
    rtklib_sat.deln = gps_cnav_eph.d_Delta_n;
    rtklib_sat.OMG0 = gps_cnav_eph.d_OMEGA0;
    // Compute the angle between the ascending node and the Greenwich meridian
    const double OMEGA_DOT_REF = -2.6e-9; // semicircles / s, see IS-GPS-200H pp. 164
    double d_OMEGA_DOT = OMEGA_DOT_REF * GPS_L2_PI + gps_cnav_eph.d_DELTA_OMEGA_DOT;
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
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, NULL);
    if      (rtklib_sat.toes < tow - 302400.0) {rtklib_sat.week++; tow -= 604800.0;}
    else if (rtklib_sat.toes > tow + 302400.0) {rtklib_sat.week--; tow += 604800.0;}
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    return rtklib_sat;
}

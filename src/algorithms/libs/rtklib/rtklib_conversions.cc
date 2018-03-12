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

obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs, const Gnss_Synchro& gnss_synchro, int week, int band)
{
    rtklib_obs.D[band] = gnss_synchro.Carrier_Doppler_hz;
    rtklib_obs.P[band] = gnss_synchro.Pseudorange_m;
    rtklib_obs.L[band] = gnss_synchro.Carrier_phase_rads / PI_2;
    switch (band)
        {
        case 0:
            rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L1C);
            break;
        case 1:
            rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L2S);
            break;
        case 2:
            rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L5X);
            break;
        }
    double CN0_dB_Hz_est = gnss_synchro.CN0_dB_hz;
    if (CN0_dB_Hz_est > 63.75) CN0_dB_Hz_est = 63.75;
    if (CN0_dB_Hz_est < 0.0) CN0_dB_Hz_est = 0.0;
    unsigned char CN0_dB_Hz = static_cast<unsigned char>(std::round(CN0_dB_Hz_est / 0.25));
    rtklib_obs.SNR[band] = CN0_dB_Hz;
    //Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    switch (gnss_synchro.System)
        {
        case 'G':
            rtklib_obs.sat = gnss_synchro.PRN;
            break;
        case 'E':
            rtklib_obs.sat = gnss_synchro.PRN + NSATGPS + NSATGLO;
            break;
        case 'R':
            rtklib_obs.sat = gnss_synchro.PRN + NSATGPS;
            break;

        default:
            rtklib_obs.sat = gnss_synchro.PRN;
        }
    rtklib_obs.time = gpst2time(adjgpsweek(week), gnss_synchro.RX_time);
    rtklib_obs.rcv = 1;
    return rtklib_obs;
}

geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model)
{
    double week, sec;
    int adj_week;
    geph_t rtklib_sat = {0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0};

    rtklib_sat.sat = glonass_gnav_eph.i_satellite_slot_number + NSATGPS; /* satellite number */
    rtklib_sat.iode = static_cast<int>(glonass_gnav_eph.d_t_b);          /* IODE (0-6 bit of tb field) */
    rtklib_sat.frq = glonass_gnav_eph.i_satellite_freq_channel;          /* satellite frequency number */
    rtklib_sat.svh = glonass_gnav_eph.d_l3rd_n;                          /* satellite health*/
    rtklib_sat.sva = static_cast<int>(glonass_gnav_eph.d_F_T);           /* satellite accuracy*/
    rtklib_sat.age = static_cast<int>(glonass_gnav_eph.d_E_n);           /* satellite age*/
    rtklib_sat.pos[0] = glonass_gnav_eph.d_Xn * 1000;                    /* satellite position (ecef) (m) */
    rtklib_sat.pos[1] = glonass_gnav_eph.d_Yn * 1000;                    /* satellite position (ecef) (m) */
    rtklib_sat.pos[2] = glonass_gnav_eph.d_Zn * 1000;                    /* satellite position (ecef) (m) */
    rtklib_sat.vel[0] = glonass_gnav_eph.d_VXn * 1000;                   /* satellite velocity (ecef) (m/s) */
    rtklib_sat.vel[1] = glonass_gnav_eph.d_VYn * 1000;                   /* satellite velocity (ecef) (m/s) */
    rtklib_sat.vel[2] = glonass_gnav_eph.d_VZn * 1000;                   /* satellite velocity (ecef) (m/s) */
    rtklib_sat.acc[0] = glonass_gnav_eph.d_AXn * 1000;                   /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.acc[1] = glonass_gnav_eph.d_AYn * 1000;                   /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.acc[2] = glonass_gnav_eph.d_AZn * 1000;                   /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.taun = glonass_gnav_eph.d_tau_n;                          /* SV clock bias (s) */
    rtklib_sat.gamn = glonass_gnav_eph.d_gamma_n;                        /* SV relative freq bias */
    rtklib_sat.age = static_cast<int>(glonass_gnav_eph.d_Delta_tau_n);   /* delay between L1 and L2 (s) */

    // Time expressed in GPS Time but using RTKLib format
    glonass_gnav_eph.glot_to_gpst(glonass_gnav_eph.d_t_b, gnav_clock_model.d_tau_c, gnav_clock_model.d_tau_gps, &week, &sec);
    adj_week = adjgpsweek(static_cast<int>(week));
    rtklib_sat.toe = gpst2time(adj_week, sec);

    // Time expressed in GPS Time but using RTKLib format
    glonass_gnav_eph.glot_to_gpst(glonass_gnav_eph.d_t_k, gnav_clock_model.d_tau_c, gnav_clock_model.d_tau_gps, &week, &sec);
    adj_week = adjgpsweek(static_cast<int>(week));
    rtklib_sat.tof = gpst2time(adj_week, sec);

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0};
    //Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    rtklib_sat.sat = gal_eph.i_satellite_PRN + NSATGPS + NSATGLO;
    rtklib_sat.A = gal_eph.A_1 * gal_eph.A_1;
    rtklib_sat.M0 = gal_eph.M0_1;
    rtklib_sat.deln = gal_eph.delta_n_3;
    rtklib_sat.OMG0 = gal_eph.OMEGA_0_2;
    rtklib_sat.OMGd = gal_eph.OMEGA_dot_3;
    rtklib_sat.omg = gal_eph.omega_2;
    rtklib_sat.i0 = gal_eph.i_0_2;
    rtklib_sat.idot = gal_eph.iDot_2;
    rtklib_sat.e = gal_eph.e_1;
    rtklib_sat.Adot = 0;  //only in CNAV;
    rtklib_sat.ndot = 0;  //only in CNAV;

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
    rtklib_sat.tgd[0] = gal_eph.BGD_E1E5a_5;
    rtklib_sat.tgd[1] = gal_eph.BGD_E1E5b_5;
    rtklib_sat.tgd[2] = 0;
    rtklib_sat.tgd[3] = 0;
    rtklib_sat.toes = gal_eph.t0e_1;
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gal_eph.t0c_4);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gal_eph.TOW_5);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, NULL);
    if (rtklib_sat.toes < tow - 302400.0)
        {
            rtklib_sat.week++;
            tow -= 604800.0;
        }
    else if (rtklib_sat.toes > tow + 302400.0)
        {
            rtklib_sat.week--;
            tow += 604800.0;
        }
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0};
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
    rtklib_sat.Adot = 0;  //only in CNAV;
    rtklib_sat.ndot = 0;  //only in CNAV;

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
    if (rtklib_sat.toes < tow - 302400.0)
        {
            rtklib_sat.week++;
            tow -= 604800.0;
        }
    else if (rtklib_sat.toes > tow + 302400.0)
        {
            rtklib_sat.week--;
            tow += 604800.0;
        }
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris& gps_cnav_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, 0.0, 0.0};
    rtklib_sat.sat = gps_cnav_eph.i_satellite_PRN;
    const double A_REF = 26559710.0;  // See IS-GPS-200H,  pp. 170
    rtklib_sat.A = A_REF + gps_cnav_eph.d_DELTA_A;
    rtklib_sat.M0 = gps_cnav_eph.d_M_0;
    rtklib_sat.deln = gps_cnav_eph.d_Delta_n;
    rtklib_sat.OMG0 = gps_cnav_eph.d_OMEGA0;
    // Compute the angle between the ascending node and the Greenwich meridian
    const double OMEGA_DOT_REF = -2.6e-9;  // semicircles / s, see IS-GPS-200H pp. 164
    double d_OMEGA_DOT = OMEGA_DOT_REF * PI + gps_cnav_eph.d_DELTA_OMEGA_DOT;
    rtklib_sat.OMGd = d_OMEGA_DOT;
    rtklib_sat.omg = gps_cnav_eph.d_OMEGA;
    rtklib_sat.i0 = gps_cnav_eph.d_i_0;
    rtklib_sat.idot = gps_cnav_eph.d_IDOT;
    rtklib_sat.e = gps_cnav_eph.d_e_eccentricity;
    rtklib_sat.Adot = gps_cnav_eph.d_A_DOT;        //only in CNAV;
    rtklib_sat.ndot = gps_cnav_eph.d_DELTA_DOT_N;  //only in CNAV;

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
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gps_cnav_eph.d_Toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gps_cnav_eph.d_TOW);

    /* adjustment for week handover */
    double tow, toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, NULL);
    if (rtklib_sat.toes < tow - 302400.0)
        {
            rtklib_sat.week++;
            tow -= 604800.0;
        }
    else if (rtklib_sat.toes > tow + 302400.0)
        {
            rtklib_sat.week--;
            tow += 604800.0;
        }
    rtklib_sat.toe = gpst2time(rtklib_sat.week, rtklib_sat.toes);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    return rtklib_sat;
}

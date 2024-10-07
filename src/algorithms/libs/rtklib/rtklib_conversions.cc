/*!
 * \file rtklib_conversions.cc
 * \brief GNSS-SDR to RTKLIB data structures conversion functions
 * \author 2017, Javier Arribas
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_conversions.h"
#include "MATH_CONSTANTS.h"          // for GNSS_PI, TWO_PI
#include "beidou_dnav_ephemeris.h"   // for Beidou_Dnav_Ephemeris
#include "galileo_almanac.h"         // for Galileo_Almanac
#include "galileo_ephemeris.h"       // for Galileo_Ephemeris
#include "glonass_gnav_ephemeris.h"  // for Glonass_Gnav_Ephemeris
#include "glonass_gnav_utc_model.h"  // for Glonass_Gnav_Utc_Model
#include "gnss_obs_codes.h"          // for CODE_L1C, CODE_L2S, CODE_L5X
#include "gnss_synchro.h"            // for Gnss_Synchro
#include "gps_almanac.h"             // for Gps_Almanac
#include "gps_cnav_ephemeris.h"      // for Gps_CNAV_Ephemeris
#include "gps_ephemeris.h"           // for Gps_Ephemeris
#include "rtklib_rtkcmn.h"
#include <cmath>


obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    int week,
    int band,
    bool pre_2009_file)
{
    // Get signal type info to adjust code type based on constellation
    const std::string sig_(gnss_synchro.Signal, 2);

    rtklib_obs.D[band] = gnss_synchro.Carrier_Doppler_hz;
    rtklib_obs.P[band] = gnss_synchro.Pseudorange_m;
    rtklib_obs.L[band] = gnss_synchro.Carrier_phase_rads / TWO_PI;

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
    if (CN0_dB_Hz_est > 63.75)
        {
            CN0_dB_Hz_est = 63.75;
        }
    if (CN0_dB_Hz_est < 0.0)
        {
            CN0_dB_Hz_est = 0.0;
        }
    auto CN0_dB_Hz = static_cast<unsigned char>(std::round(CN0_dB_Hz_est / 0.25));
    rtklib_obs.SNR[band] = CN0_dB_Hz;
    // Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    switch (gnss_synchro.System)
        {
        case 'G':
            rtklib_obs.sat = gnss_synchro.PRN;
            break;
        case 'E':
            rtklib_obs.sat = gnss_synchro.PRN + NSATGPS + NSATGLO;
            if (sig_ == "7X")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L7X);
                }
            if (sig_ == "E6")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L6B);
                }
            break;
        case 'R':
            rtklib_obs.sat = gnss_synchro.PRN + NSATGPS;
            break;
        case 'C':
            rtklib_obs.sat = gnss_synchro.PRN + NSATGPS + NSATGLO + NSATGAL + NSATQZS;
            // Update signal code
            if (sig_ == "B1")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L2I);
                }
            else if (sig_ == "B3")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L6I);
                }

            break;

        default:
            rtklib_obs.sat = gnss_synchro.PRN;
        }

    // Note that BeiDou week numbers do not need adjustment for foreseeable future. Consider change
    // to more elegant solution
    //    if(gnss_synchro.System == 'C')
    //       {
    //           rtklib_obs.time = bdt2gpst(bdt2time(week, gnss_synchro.RX_time));
    //       }
    //    else
    //       {
    //           rtklib_obs.time = gpst2time(adjgpsweek(week), gnss_synchro.RX_time);
    //       }
    //
    if (gnss_synchro.System == 'E')
        {
            rtklib_obs.time = gst2time(week, gnss_synchro.RX_time);
        }
    else
        {
            rtklib_obs.time = gpst2time(adjgpsweek(week, pre_2009_file), gnss_synchro.RX_time);
        }
    // account for the TOW crossover transitory in the first 18 seconds where the week is not yet updated!
    if (gnss_synchro.RX_time < 18.0)
        {
            rtklib_obs.time = timeadd(rtklib_obs.time, 604800);
        }

    rtklib_obs.rcv = 1;

    if (!has_obs_corr.empty())
        {
            float has_pseudorange_correction_m = 0.0;
            float has_bias_correction_cycle = 0.0;
            switch (gnss_synchro.System)
                {
                case 'G':
                    {
                        if (sig_ == "1C")
                            {
                                const auto it = has_obs_corr.find("L1 C/A");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                            }
                        else if (sig_ == "2S")
                            {
                                const auto it = has_obs_corr.find("L2 CM");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                            }
                        else if (sig_ == "L5")
                            {
                                //  TODO: determine which one
                                const auto it = has_obs_corr.find("L5 I");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                                else
                                    {
                                        const auto it_2nd_attempt = has_obs_corr.find("L5 Q");
                                        if (it_2nd_attempt != has_obs_corr.cend())
                                            {
                                                const auto it2 = it_2nd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                if (it2 != it_2nd_attempt->second.cend())
                                                    {
                                                        has_pseudorange_correction_m = it2->second.code_bias_m;
                                                        has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                    }
                                            }
                                        else
                                            {
                                                const auto it_3rd_attempt = has_obs_corr.find("L5 I + L5 Q");
                                                if (it_3rd_attempt != has_obs_corr.cend())
                                                    {
                                                        const auto it2 = it_3rd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                        if (it2 != it_3rd_attempt->second.cend())
                                                            {
                                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                            }
                                                    }
                                            }
                                    }
                            }
                    }
                    break;
                case 'E':
                    {
                        if (sig_ == "1B")
                            {
                                //  TODO: determine which one
                                const auto it = has_obs_corr.find("E1-B I/NAV OS");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                                else
                                    {
                                        const auto it_2nd_attempt = has_obs_corr.find("E1-C");
                                        if (it_2nd_attempt != has_obs_corr.cend())
                                            {
                                                const auto it2 = it_2nd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                if (it2 != it_2nd_attempt->second.cend())
                                                    {
                                                        has_pseudorange_correction_m = it2->second.code_bias_m;
                                                        has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                    }
                                            }
                                        else
                                            {
                                                const auto it_3rd_attempt = has_obs_corr.find("E1-B + E1-C");
                                                if (it_3rd_attempt != has_obs_corr.cend())
                                                    {
                                                        const auto it2 = it_3rd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                        if (it2 != it_3rd_attempt->second.cend())
                                                            {
                                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                            }
                                                    }
                                            }
                                    }
                            }
                        else if (sig_ == "5X")
                            {
                                //  TODO: determine which one
                                const auto it = has_obs_corr.find("E5a-I F/NAV OS");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                                else
                                    {
                                        const auto it_2nd_attempt = has_obs_corr.find("E5a-Q");
                                        if (it_2nd_attempt != has_obs_corr.cend())
                                            {
                                                const auto it2 = it_2nd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                if (it2 != it_2nd_attempt->second.cend())
                                                    {
                                                        has_pseudorange_correction_m = it2->second.code_bias_m;
                                                        has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                    }
                                            }
                                        else
                                            {
                                                const auto it_3rd_attempt = has_obs_corr.find("E5a-I+E5a-Q");
                                                if (it_3rd_attempt != has_obs_corr.cend())
                                                    {
                                                        const auto it2 = it_3rd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                        if (it2 != it_3rd_attempt->second.cend())
                                                            {
                                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                            }
                                                    }
                                            }
                                    }
                            }

                        else if (sig_ == "7X")
                            {
                                //  TODO: determine which one
                                const auto it = has_obs_corr.find("E5bI I/NAV OS");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                                else
                                    {
                                        const auto it_2nd_attempt = has_obs_corr.find("E5b-Q");
                                        if (it_2nd_attempt != has_obs_corr.cend())
                                            {
                                                const auto it2 = it_2nd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                if (it2 != it_2nd_attempt->second.cend())
                                                    {
                                                        has_pseudorange_correction_m = it2->second.code_bias_m;
                                                        has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                    }
                                            }
                                        else
                                            {
                                                const auto it_3rd_attempt = has_obs_corr.find("E5b-I+E5b-Q");
                                                if (it_3rd_attempt != has_obs_corr.cend())
                                                    {
                                                        const auto it2 = it_3rd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                        if (it2 != it_3rd_attempt->second.cend())
                                                            {
                                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                            }
                                                    }
                                            }
                                    }
                            }
                        else if (sig_ == "6B")
                            {
                                //  TODO: determine which one
                                const auto it = has_obs_corr.find("E6-B C/NAV HAS");
                                if (it != has_obs_corr.cend())
                                    {
                                        const auto it2 = it->second.find(static_cast<int>(gnss_synchro.PRN));
                                        if (it2 != it->second.cend())
                                            {
                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                            }
                                    }
                                else
                                    {
                                        const auto it_2nd_attempt = has_obs_corr.find("E6-C");
                                        if (it_2nd_attempt != has_obs_corr.cend())
                                            {
                                                const auto it2 = it_2nd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                if (it2 != it_2nd_attempt->second.cend())
                                                    {
                                                        has_pseudorange_correction_m = it2->second.code_bias_m;
                                                        has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                    }
                                            }
                                        else
                                            {
                                                const auto it_3rd_attempt = has_obs_corr.find("E6-B + E6-C");
                                                if (it_3rd_attempt != has_obs_corr.cend())
                                                    {
                                                        const auto it2 = it_3rd_attempt->second.find(static_cast<int>(gnss_synchro.PRN));
                                                        if (it2 != it_3rd_attempt->second.cend())
                                                            {
                                                                has_pseudorange_correction_m = it2->second.code_bias_m;
                                                                has_bias_correction_cycle = it2->second.phase_bias_cycle;
                                                            }
                                                    }
                                            }
                                    }
                            }
                    }
                    break;
                default:
                    break;
                }

            rtklib_obs.P[band] += has_pseudorange_correction_m;
            rtklib_obs.L[band] += has_bias_correction_cycle;
        }
    return rtklib_obs;
}


obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    int week,
    int band,
    bool pre_2009_file)
{
    std::map<std::string, std::map<int, HAS_obs_corrections>> empty_map;
    return insert_obs_to_rtklib(rtklib_obs,
        gnss_synchro,
        empty_map,
        week,
        band,
        pre_2009_file);
}


geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model)
{
    int week;
    double sec;
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
    rtklib_sat.dtaun = static_cast<int>(glonass_gnav_eph.d_Delta_tau_n); /* delay between L1 and L2 (s) */

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
    std::map<int, HAS_orbit_corrections> empty_orbit_map;
    std::map<int, HAS_clock_corrections> empty_clock_map;
    return eph_to_rtklib(gal_eph, empty_orbit_map, empty_clock_map);
}


eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph,
    const std::map<int, HAS_orbit_corrections>& orbit_correction_map,
    const std::map<int, HAS_clock_corrections>& clock_correction_map)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    // Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    rtklib_sat.sat = gal_eph.PRN + NSATGPS + NSATGLO;
    rtklib_sat.A = gal_eph.sqrtA * gal_eph.sqrtA;
    rtklib_sat.M0 = gal_eph.M_0;
    rtklib_sat.deln = gal_eph.delta_n;
    rtklib_sat.OMG0 = gal_eph.OMEGA_0;
    rtklib_sat.OMGd = gal_eph.OMEGAdot;
    rtklib_sat.omg = gal_eph.omega;
    rtklib_sat.i0 = gal_eph.i_0;
    rtklib_sat.idot = gal_eph.idot;
    rtklib_sat.e = gal_eph.ecc;
    rtklib_sat.Adot = 0;  // only in CNAV;
    rtklib_sat.ndot = 0;  // only in CNAV;

    rtklib_sat.week = gal_eph.WN + 1024; /* week of tow in GPS (not mod-1024) week scale */
    rtklib_sat.cic = gal_eph.Cic;
    rtklib_sat.cis = gal_eph.Cis;
    rtklib_sat.cuc = gal_eph.Cuc;
    rtklib_sat.cus = gal_eph.Cus;
    rtklib_sat.crc = gal_eph.Crc;
    rtklib_sat.crs = gal_eph.Crs;
    rtklib_sat.f0 = gal_eph.af0;
    rtklib_sat.f1 = gal_eph.af1;
    rtklib_sat.f2 = gal_eph.af2;
    rtklib_sat.tgd[0] = gal_eph.BGD_E1E5a;
    rtklib_sat.tgd[1] = gal_eph.BGD_E1E5b;
    rtklib_sat.tgd[2] = 0;
    rtklib_sat.tgd[3] = 0;
    rtklib_sat.toes = gal_eph.toe;
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gal_eph.toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gal_eph.tow);

    /* adjustment for week handover */
    double tow;
    double toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, nullptr);
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

    if (!orbit_correction_map.empty() && !clock_correction_map.empty())
        {
            int count_has_corrections = 0;
            const auto it_orbit = orbit_correction_map.find(static_cast<int>(gal_eph.PRN));
            if (it_orbit != orbit_correction_map.cend())
                {
                    rtklib_sat.has_orbit_radial_correction_m = it_orbit->second.radial_m;
                    rtklib_sat.has_orbit_in_track_correction_m = it_orbit->second.in_track_m;
                    rtklib_sat.has_orbit_cross_track_correction_m = it_orbit->second.cross_track_m;
                    count_has_corrections++;
                }

            const auto it_clock = clock_correction_map.find(static_cast<int>(gal_eph.PRN));
            if (it_clock != clock_correction_map.cend())
                {
                    rtklib_sat.has_clock_correction_m = it_clock->second.clock_correction_m;
                    count_has_corrections++;
                }
            rtklib_sat.apply_has_corrections = (count_has_corrections == 2) ? true : false;
            if (rtklib_sat.apply_has_corrections)
                {
                    rtklib_sat.tgd[0] = 0.0;
                    rtklib_sat.tgd[1] = 0.0;
                }
        }
    else
        {
            rtklib_sat.apply_has_corrections = false;
        }
    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph, bool pre_2009_file)
{
    std::map<int, HAS_orbit_corrections> empty_orbit_map;
    std::map<int, HAS_clock_corrections> empty_clock_map;
    return eph_to_rtklib(gps_eph, empty_orbit_map, empty_clock_map, pre_2009_file);
}


eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph,
    const std::map<int, HAS_orbit_corrections>& orbit_correction_map,
    const std::map<int, HAS_clock_corrections>& clock_correction_map,
    bool pre_2009_file)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    rtklib_sat.sat = gps_eph.PRN;
    rtklib_sat.A = gps_eph.sqrtA * gps_eph.sqrtA;
    rtklib_sat.M0 = gps_eph.M_0;
    rtklib_sat.deln = gps_eph.delta_n;
    rtklib_sat.OMG0 = gps_eph.OMEGA_0;
    rtklib_sat.OMGd = gps_eph.OMEGAdot;
    rtklib_sat.omg = gps_eph.omega;
    rtklib_sat.i0 = gps_eph.i_0;
    rtklib_sat.idot = gps_eph.idot;
    rtklib_sat.e = gps_eph.ecc;
    rtklib_sat.Adot = 0;  // only in CNAV;
    rtklib_sat.ndot = 0;  // only in CNAV;

    rtklib_sat.week = adjgpsweek(gps_eph.WN, pre_2009_file); /* week of tow */
    rtklib_sat.cic = gps_eph.Cic;
    rtklib_sat.cis = gps_eph.Cis;
    rtklib_sat.cuc = gps_eph.Cuc;
    rtklib_sat.cus = gps_eph.Cus;
    rtklib_sat.crc = gps_eph.Crc;
    rtklib_sat.crs = gps_eph.Crs;
    rtklib_sat.f0 = gps_eph.af0;
    rtklib_sat.f1 = gps_eph.af1;
    rtklib_sat.f2 = gps_eph.af2;
    rtklib_sat.tgd[0] = gps_eph.TGD;
    rtklib_sat.tgd[1] = 0.0;
    rtklib_sat.tgd[2] = 0.0;
    rtklib_sat.tgd[3] = 0.0;
    rtklib_sat.toes = gps_eph.toe;
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gps_eph.toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gps_eph.tow);

    /* adjustment for week handover */
    double tow;
    double toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, nullptr);
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

    if (!orbit_correction_map.empty() && !clock_correction_map.empty())
        {
            int count_has_corrections = 0;
            const auto it_orbit = orbit_correction_map.find(static_cast<int>(gps_eph.PRN));
            if (it_orbit != orbit_correction_map.cend())
                {
                    rtklib_sat.has_orbit_radial_correction_m = it_orbit->second.radial_m;
                    rtklib_sat.has_orbit_in_track_correction_m = it_orbit->second.in_track_m;
                    rtklib_sat.has_orbit_cross_track_correction_m = it_orbit->second.cross_track_m;
                    count_has_corrections++;
                }

            const auto it_clock = clock_correction_map.find(static_cast<int>(gps_eph.PRN));
            if (it_clock != clock_correction_map.cend())
                {
                    rtklib_sat.has_clock_correction_m = it_clock->second.clock_correction_m;
                    count_has_corrections++;
                }
            rtklib_sat.apply_has_corrections = (count_has_corrections == 2) ? true : false;
            if (rtklib_sat.apply_has_corrections)
                {
                    rtklib_sat.tgd[0] = 0.0;
                    rtklib_sat.tgd[1] = 0.0;
                }
        }
    else
        {
            rtklib_sat.has_orbit_radial_correction_m = 0.0;
            rtklib_sat.has_orbit_in_track_correction_m = 0.0;
            rtklib_sat.has_orbit_cross_track_correction_m = 0.0;
            rtklib_sat.has_clock_correction_m = 0.0;
            rtklib_sat.apply_has_corrections = false;
        }

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Beidou_Dnav_Ephemeris& bei_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    rtklib_sat.sat = bei_eph.PRN + NSATGPS + NSATGLO + NSATGAL + NSATQZS;
    rtklib_sat.A = bei_eph.sqrtA * bei_eph.sqrtA;
    rtklib_sat.M0 = bei_eph.M_0;
    rtklib_sat.deln = bei_eph.delta_n;
    rtklib_sat.OMG0 = bei_eph.OMEGA_0;
    rtklib_sat.OMGd = bei_eph.OMEGAdot;
    rtklib_sat.omg = bei_eph.omega;
    rtklib_sat.i0 = bei_eph.i_0;
    rtklib_sat.idot = bei_eph.idot;
    rtklib_sat.e = bei_eph.ecc;
    rtklib_sat.Adot = 0;  // only in CNAV;
    rtklib_sat.ndot = 0;  // only in CNAV;

    rtklib_sat.svh = bei_eph.SV_health;
    rtklib_sat.sva = bei_eph.SV_accuracy;

    rtklib_sat.code = bei_eph.sig_type;                   /* B1I data */
    rtklib_sat.flag = bei_eph.nav_type;                   /* MEO/IGSO satellite */
    rtklib_sat.iode = static_cast<int32_t>(bei_eph.AODE); /* AODE */
    rtklib_sat.iodc = static_cast<int32_t>(bei_eph.AODC); /* AODC */

    rtklib_sat.week = bei_eph.WN; /* week of tow */
    rtklib_sat.cic = bei_eph.Cic;
    rtklib_sat.cis = bei_eph.Cis;
    rtklib_sat.cuc = bei_eph.Cuc;
    rtklib_sat.cus = bei_eph.Cus;
    rtklib_sat.crc = bei_eph.Crc;
    rtklib_sat.crs = bei_eph.Crs;
    rtklib_sat.f0 = bei_eph.af0;
    rtklib_sat.f1 = bei_eph.af1;
    rtklib_sat.f2 = bei_eph.af2;
    rtklib_sat.tgd[0] = bei_eph.TGD1;
    rtklib_sat.tgd[1] = bei_eph.TGD2;
    rtklib_sat.tgd[2] = 0.0;
    rtklib_sat.tgd[3] = 0.0;
    rtklib_sat.toes = bei_eph.toe;
    rtklib_sat.toe = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.toe));
    rtklib_sat.toc = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.toc));
    rtklib_sat.ttr = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.tow));
    /* adjustment for week handover */
    double tow;
    double toc;
    double toe;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, nullptr);
    toe = time2gpst(rtklib_sat.toe, nullptr);

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
    rtklib_sat.toe = gpst2time(rtklib_sat.week, toe);
    rtklib_sat.toc = gpst2time(rtklib_sat.week, toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, tow);

    rtklib_sat.has_orbit_radial_correction_m = 0.0;
    rtklib_sat.has_orbit_in_track_correction_m = 0.0;
    rtklib_sat.has_orbit_cross_track_correction_m = 0.0;
    rtklib_sat.has_clock_correction_m = 0.0;
    rtklib_sat.apply_has_corrections = false;

    return rtklib_sat;
}


eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris& gps_cnav_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    rtklib_sat.sat = gps_cnav_eph.PRN;
    rtklib_sat.A = gps_cnav_eph.sqrtA * gps_cnav_eph.sqrtA;
    rtklib_sat.M0 = gps_cnav_eph.M_0;
    rtklib_sat.deln = gps_cnav_eph.delta_n;
    rtklib_sat.OMG0 = gps_cnav_eph.OMEGA_0;
    rtklib_sat.OMGd = gps_cnav_eph.OMEGAdot;
    rtklib_sat.omg = gps_cnav_eph.omega;
    rtklib_sat.i0 = gps_cnav_eph.i_0;
    rtklib_sat.idot = gps_cnav_eph.idot;
    rtklib_sat.e = gps_cnav_eph.ecc;
    rtklib_sat.Adot = gps_cnav_eph.Adot;        // only in CNAV;
    rtklib_sat.ndot = gps_cnav_eph.delta_ndot;  // only in CNAV;

    rtklib_sat.week = adjgpsweek(gps_cnav_eph.WN); /* week of tow */
    rtklib_sat.cic = gps_cnav_eph.Cic;
    rtklib_sat.cis = gps_cnav_eph.Cis;
    rtklib_sat.cuc = gps_cnav_eph.Cuc;
    rtklib_sat.cus = gps_cnav_eph.Cus;
    rtklib_sat.crc = gps_cnav_eph.Crc;
    rtklib_sat.crs = gps_cnav_eph.Crs;
    rtklib_sat.f0 = gps_cnav_eph.af0;
    rtklib_sat.f1 = gps_cnav_eph.af1;
    rtklib_sat.f2 = gps_cnav_eph.af2;
    rtklib_sat.tgd[0] = gps_cnav_eph.TGD;
    rtklib_sat.tgd[1] = 0.0;
    rtklib_sat.tgd[2] = 0.0;
    rtklib_sat.tgd[3] = 0.0;
    rtklib_sat.isc[0] = gps_cnav_eph.ISCL1;
    rtklib_sat.isc[1] = gps_cnav_eph.ISCL2;
    rtklib_sat.isc[2] = gps_cnav_eph.ISCL5I;
    rtklib_sat.isc[3] = gps_cnav_eph.ISCL5Q;
    rtklib_sat.toes = gps_cnav_eph.toe1;
    rtklib_sat.toc = gpst2time(rtklib_sat.week, gps_cnav_eph.toc);
    rtklib_sat.ttr = gpst2time(rtklib_sat.week, gps_cnav_eph.tow);

    /* adjustment for week handover */
    double tow;
    double toc;
    tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    toc = time2gpst(rtklib_sat.toc, nullptr);
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

    rtklib_sat.has_orbit_radial_correction_m = 0.0;
    rtklib_sat.has_orbit_in_track_correction_m = 0.0;
    rtklib_sat.has_orbit_cross_track_correction_m = 0.0;
    rtklib_sat.has_clock_correction_m = 0.0;
    rtklib_sat.apply_has_corrections = false;

    return rtklib_sat;
}


alm_t alm_to_rtklib(const Gps_Almanac& gps_alm)
{
    alm_t rtklib_alm;

    rtklib_alm = {0, 0, 0, 0, {0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    rtklib_alm.sat = gps_alm.PRN;
    rtklib_alm.svh = gps_alm.SV_health;
    rtklib_alm.svconf = gps_alm.AS_status;
    rtklib_alm.week = gps_alm.WNa;
    gtime_t toa;
    toa.time = gps_alm.toa;
    toa.sec = 0.0;
    rtklib_alm.toa = toa;
    rtklib_alm.A = gps_alm.sqrtA * gps_alm.sqrtA;
    rtklib_alm.e = gps_alm.ecc;
    rtklib_alm.i0 = (gps_alm.delta_i + 0.3) * GNSS_PI;
    rtklib_alm.OMG0 = gps_alm.OMEGA_0 * GNSS_PI;
    rtklib_alm.OMGd = gps_alm.OMEGAdot * GNSS_PI;
    rtklib_alm.omg = gps_alm.omega * GNSS_PI;
    rtklib_alm.M0 = gps_alm.M_0 * GNSS_PI;
    rtklib_alm.f0 = gps_alm.af0;
    rtklib_alm.f1 = gps_alm.af1;
    rtklib_alm.toas = gps_alm.toa;

    return rtklib_alm;
}


alm_t alm_to_rtklib(const Galileo_Almanac& gal_alm)
{
    alm_t rtklib_alm;

    rtklib_alm = {0, 0, 0, 0, {0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    rtklib_alm.sat = gal_alm.PRN + NSATGPS + NSATGLO;
    rtklib_alm.svh = gal_alm.E1B_HS;
    rtklib_alm.svconf = gal_alm.E1B_HS;
    rtklib_alm.week = gal_alm.WNa;
    gtime_t toa;
    toa.time = gal_alm.toa;
    toa.sec = 0.0;
    rtklib_alm.toa = toa;
    rtklib_alm.A = gal_alm.sqrtA * gal_alm.sqrtA;
    rtklib_alm.e = gal_alm.ecc;
    rtklib_alm.i0 = (gal_alm.delta_i + 56.0 / 180.0) * GNSS_PI;
    rtklib_alm.OMG0 = gal_alm.OMEGA_0 * GNSS_PI;
    rtklib_alm.OMGd = gal_alm.OMEGAdot * GNSS_PI;
    rtklib_alm.omg = gal_alm.omega * GNSS_PI;
    rtklib_alm.M0 = gal_alm.M_0 * GNSS_PI;
    rtklib_alm.f0 = gal_alm.af0;
    rtklib_alm.f1 = gal_alm.af1;
    rtklib_alm.toas = gal_alm.toa;

    return rtklib_alm;
}

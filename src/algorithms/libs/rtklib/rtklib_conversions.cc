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
#include "Beidou_DNAV.h"             // for BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET
#include "MATH_CONSTANTS.h"          // for GNSS_PI, TWO_PI
#include "beidou_cnav1_ephemeris.h"  // for Beidou_Cnav1_Ephemeris
#include "beidou_dnav_almanac.h"     // for Beidou_Dnav_Almanac
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
#include <vector>

namespace
{
const HAS_obs_corrections* find_has_obs_correction(const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    const std::vector<std::string>& signal_candidates,
    int prn)
{
    for (const auto& signal : signal_candidates)
        {
            const auto signal_it = has_obs_corr.find(signal);
            if (signal_it == has_obs_corr.cend())
                {
                    continue;
                }

            const auto prn_it = signal_it->second.find(prn);
            if (prn_it != signal_it->second.cend())
                {
                    return &prn_it->second;
                }
        }
    return nullptr;
}
}  // namespace


static double gps_cnav_ura_upper_bound_m(int32_t ura_index)
{
    switch (ura_index)
        {
        case -15:
            return 0.01;
        case -14:
            return 0.02;
        case -13:
            return 0.03;
        case -12:
            return 0.04;
        case -11:
            return 0.06;
        case -10:
            return 0.08;
        case -9:
            return 0.11;
        case -8:
            return 0.15;
        case -7:
            return 0.21;
        case -6:
            return 0.30;
        case -5:
            return 0.43;
        case -4:
            return 0.60;
        case -3:
            return 0.85;
        case -2:
            return 1.20;
        case -1:
            return 1.70;
        case 0:
            return 2.40;
        case 1:
            return 3.40;
        case 2:
            return 4.85;
        case 3:
            return 6.85;
        case 4:
            return 9.65;
        case 5:
            return 13.65;
        case 6:
            return 24.0;
        case 7:
            return 48.0;
        case 8:
            return 96.0;
        case 9:
            return 192.0;
        case 10:
            return 384.0;
        case 11:
            return 768.0;
        case 12:
            return 1536.0;
        case 13:
            return 3072.0;
        case 14:
            return 6144.0;
        default:
            return 6144.0;
        }
}


static int32_t gps_cnav_ura_to_rtklib_sva(int32_t uraed, int32_t uraned0)
{
    if (uraed == 15 || uraed == -16 || uraned0 == 15 || uraned0 == -16)
        {
            return 15;
        }

    const double composite_ura_m = std::hypot(gps_cnav_ura_upper_bound_m(uraed), gps_cnav_ura_upper_bound_m(uraned0));
    const double rtklib_ura_bound_m[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};

    for (int32_t i = 0; i < 15; ++i)
        {
            if (composite_ura_m <= rtklib_ura_bound_m[i])
                {
                    return i;
                }
        }

    return 15;
}


obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    int week,
    int band,
    const HAS_obs_corrections** applied_has_correction,
    int ref_week)
{
    if (applied_has_correction != nullptr)
        {
            *applied_has_correction = nullptr;
        }

    // Get signal type info to adjust code type based on constellation
    const std::string sig_(gnss_synchro.Signal, 2);

    rtklib_obs.D[band] = gnss_synchro.Carrier_Doppler_hz;
    rtklib_obs.P[band] = gnss_synchro.Pseudorange_m;
    rtklib_obs.L[band] = gnss_synchro.Carrier_phase_rads / TWO_PI;
    // bit 0: loss of lock or cycle slip. A half-cycle re-resolution steps the
    // reported phase by half a cycle at this single epoch, i.e. it is a slip
    // event, not a persistent "half-cycle unresolved" condition: LLI bit 1 is
    // a state indicator whose every transition counts as a slip downstream
    // (detslp_ll), so mapping the one-epoch event flag there would reset the
    // phase bias a second, spurious time when the flag drops
    rtklib_obs.LLI[band] = static_cast<unsigned char>(
        (gnss_synchro.Flag_cycle_slip || gnss_synchro.Flag_half_cycle_slip) ? 1U : 0U);

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
            rtklib_obs.sat = satno(SYS_GLO, gnss_synchro.PRN);
            if (sig_ == "2G")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L2C);
                }
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
            else if (sig_ == "1D")
                {
                    rtklib_obs.code[band] = static_cast<unsigned char>(CODE_L1P);
                }

            break;
        case 'J':
            rtklib_obs.sat = satno(SYS_QZS, gnss_synchro.PRN);
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
            rtklib_obs.time = gpst2time(adjgpsweek(week, ref_week), gnss_synchro.RX_time);
        }
    // account for the TOW crossover transitory in the first 18 seconds where the week is not yet updated!
    if (gnss_synchro.RX_time < 18.0)
        {
            rtklib_obs.time = timeadd(rtklib_obs.time, 604800);
        }

    rtklib_obs.rcv = 1;

    if (!has_obs_corr.empty())
        {
            const HAS_obs_corrections* has_correction = nullptr;
            const int prn = static_cast<int>(gnss_synchro.PRN);
            switch (gnss_synchro.System)
                {
                case 'G':
                    {
                        if (sig_ == "1C")
                            {
                                const std::vector<std::string> signal_candidates = {"L1 C/A"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                        else if (sig_ == "2S")
                            {
                                const std::vector<std::string> signal_candidates = {"L2 CM", "L2 CL", "L2 CM+CL", "L2 P"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                        else if (sig_ == "L5")
                            {
                                const std::vector<std::string> signal_candidates = {"L5 I", "L5 Q", "L5 I + L5 Q"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                    }
                    break;
                case 'E':
                    {
                        if (sig_ == "1B")
                            {
                                const std::vector<std::string> signal_candidates = {"E1-B I/NAV OS", "E1-C", "E1-B + E1-C"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                        else if (sig_ == "5X")
                            {
                                const std::vector<std::string> signal_candidates = {"E5a-I F/NAV OS", "E5a-Q", "E5a-I+E5a-Q"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }

                        else if (sig_ == "7X")
                            {
                                const std::vector<std::string> signal_candidates = {"E5b-I I/NAV OS", "E5b-Q", "E5b-I+E5b-Q"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                        else if (sig_ == "E6")
                            {
                                const std::vector<std::string> signal_candidates = {"E6-B C/NAV HAS", "E6-C", "E6-B + E6-C"};
                                has_correction = find_has_obs_correction(has_obs_corr, signal_candidates, prn);
                            }
                    }
                    break;
                default:
                    break;
                }

            if (has_correction != nullptr)
                {
                    rtklib_obs.P[band] += has_correction->code_bias_m;
                    rtklib_obs.L[band] += has_correction->phase_bias_cycle;
                    if (has_correction->phase_bias_discontinuity)
                        {
                            rtklib_obs.LLI[band] |= 1U;
                        }
                    if (applied_has_correction != nullptr)
                        {
                            *applied_has_correction = has_correction;
                        }
                }
        }
    return rtklib_obs;
}


obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    int week,
    int band,
    int ref_week)
{
    return insert_obs_to_rtklib(rtklib_obs,
        gnss_synchro,
        has_obs_corr,
        week,
        band,
        static_cast<const HAS_obs_corrections**>(nullptr),
        ref_week);
}


obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    int week,
    int band,
    int ref_week)
{
    std::map<std::string, std::map<int, HAS_obs_corrections>> empty_map;
    return insert_obs_to_rtklib(rtklib_obs,
        gnss_synchro,
        empty_map,
        week,
        band,
        ref_week);
}


geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model, bool glonass_strict_health)
{
    int week;
    double sec;
    int adj_week;
    geph_t rtklib_sat = {0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0};

    // The ln flag (string 3) always marks a satellite malfunction. The MSB of
    // the Bn word is a stricter health indicator that is only applied when
    // glonass_strict_health is enabled
    const bool bn_msb_unhealthy = glonass_strict_health && ((static_cast<int32_t>(glonass_gnav_eph.d_B_n) & 4) != 0);

    rtklib_sat.sat = satno(SYS_GLO, glonass_gnav_eph.i_satellite_slot_number);       /* satellite number */
    rtklib_sat.iode = static_cast<int>(std::lround(glonass_gnav_eph.d_t_b / 900.0)); /* IODE (tb interval index) */
    rtklib_sat.frq = glonass_gnav_eph.i_satellite_freq_channel;                      /* satellite frequency number */
    rtklib_sat.svh = (bn_msb_unhealthy || glonass_gnav_eph.d_l3rd_n) ? 1 : 0;        /* satellite health from the ln flag (and optionally the Bn MSB) */
    rtklib_sat.sva = static_cast<int>(glonass_gnav_eph.d_F_T);                       /* satellite accuracy*/
    rtklib_sat.age = static_cast<int>(glonass_gnav_eph.d_E_n);                       /* satellite age*/
    rtklib_sat.pos[0] = glonass_gnav_eph.d_Xn * 1000;                                /* satellite position (ecef) (m) */
    rtklib_sat.pos[1] = glonass_gnav_eph.d_Yn * 1000;                                /* satellite position (ecef) (m) */
    rtklib_sat.pos[2] = glonass_gnav_eph.d_Zn * 1000;                                /* satellite position (ecef) (m) */
    rtklib_sat.vel[0] = glonass_gnav_eph.d_VXn * 1000;                               /* satellite velocity (ecef) (m/s) */
    rtklib_sat.vel[1] = glonass_gnav_eph.d_VYn * 1000;                               /* satellite velocity (ecef) (m/s) */
    rtklib_sat.vel[2] = glonass_gnav_eph.d_VZn * 1000;                               /* satellite velocity (ecef) (m/s) */
    rtklib_sat.acc[0] = glonass_gnav_eph.d_AXn * 1000;                               /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.acc[1] = glonass_gnav_eph.d_AYn * 1000;                               /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.acc[2] = glonass_gnav_eph.d_AZn * 1000;                               /* satellite acceleration (ecef) (m/s^2) */
    rtklib_sat.taun = glonass_gnav_eph.d_tau_n;                                      /* SV clock bias (s) */
    rtklib_sat.gamn = glonass_gnav_eph.d_gamma_n;                                    /* SV relative freq bias */
    rtklib_sat.dtaun = glonass_gnav_eph.d_Delta_tau_n;                               /* delay between L1 and L2 (s) */

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
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0, 0.0, -1, 0};
    // Galileo is the third satellite system for RTKLIB, so, add the required offset to discriminate Galileo ephemeris
    rtklib_sat.sat = gal_eph.PRN + NSATGPS + NSATGLO;
    rtklib_sat.code = gal_eph.nav_message_type == Galileo_Nav_Message_Type::FNAV ? 2 : 1;
    rtklib_sat.sva = gal_eph.SISA;
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
            const auto sis_iod = static_cast<uint16_t>(gal_eph.IOD_ephemeris);
            bool orbit_correction_applied = false;
            if (it_orbit != orbit_correction_map.cend() &&
                it_orbit->second.iod == sis_iod)
                {
                    rtklib_sat.has_orbit_radial_correction_m = it_orbit->second.radial_m;
                    rtklib_sat.has_orbit_in_track_correction_m = it_orbit->second.in_track_m;
                    rtklib_sat.has_orbit_cross_track_correction_m = it_orbit->second.cross_track_m;
                    orbit_correction_applied = true;
                    count_has_corrections++;
                }

            const auto it_clock = clock_correction_map.find(static_cast<int>(gal_eph.PRN));
            if (it_clock != clock_correction_map.cend() &&
                it_clock->second.iod == sis_iod &&
                orbit_correction_applied &&
                it_clock->second.mask_id == it_orbit->second.mask_id &&
                it_clock->second.iod_set_id == it_orbit->second.iod_set_id)
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


eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph, int ref_week)
{
    std::map<int, HAS_orbit_corrections> empty_orbit_map;
    std::map<int, HAS_clock_corrections> empty_clock_map;
    return eph_to_rtklib(gps_eph, empty_orbit_map, empty_clock_map, ref_week);
}


eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph,
    const std::map<int, HAS_orbit_corrections>& orbit_correction_map,
    const std::map<int, HAS_clock_corrections>& clock_correction_map,
    int ref_week)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0, 0.0, -1, 0};
    const int gps_sys = (MINPRNQZS <= gps_eph.PRN && gps_eph.PRN <= MAXPRNQZS) ? SYS_QZS : SYS_GPS;
    rtklib_sat.sat = satno(gps_sys, gps_eph.PRN);
    rtklib_sat.iode = gps_eph.IODE_SF3;
    rtklib_sat.iodc = gps_eph.IODC;
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

    rtklib_sat.week = adjgpsweek(gps_eph.WN, ref_week); /* week of tow */
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
            const auto sis_iod = static_cast<uint16_t>(gps_eph.IODE_SF3);
            bool orbit_correction_applied = false;
            if (it_orbit != orbit_correction_map.cend() &&
                it_orbit->second.iod == sis_iod)
                {
                    rtklib_sat.has_orbit_radial_correction_m = it_orbit->second.radial_m;
                    rtklib_sat.has_orbit_in_track_correction_m = it_orbit->second.in_track_m;
                    rtklib_sat.has_orbit_cross_track_correction_m = it_orbit->second.cross_track_m;
                    orbit_correction_applied = true;
                    count_has_corrections++;
                }

            const auto it_clock = clock_correction_map.find(static_cast<int>(gps_eph.PRN));
            if (it_clock != clock_correction_map.cend() &&
                it_clock->second.iod == sis_iod &&
                orbit_correction_applied &&
                it_clock->second.mask_id == it_orbit->second.mask_id &&
                it_clock->second.iod_set_id == it_orbit->second.iod_set_id)
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
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0, 0.0, -1, 0};
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


eph_t eph_to_rtklib(const Beidou_Cnav1_Ephemeris& bei_eph)
{
    eph_t rtklib_sat = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}, {0, 0}, {0, 0}, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0, 0.0, -1, 0};
    rtklib_sat.sat = bei_eph.PRN + NSATGPS + NSATGLO + NSATGAL + NSATQZS;
    rtklib_sat.A = bei_eph.A0;
    rtklib_sat.M0 = bei_eph.M_0;
    rtklib_sat.deln = bei_eph.delta_n;
    rtklib_sat.OMG0 = bei_eph.OMEGA_0;
    rtklib_sat.OMGd = bei_eph.OMEGAdot;
    rtklib_sat.omg = bei_eph.omega;
    rtklib_sat.i0 = bei_eph.i_0;
    rtklib_sat.idot = bei_eph.idot;
    rtklib_sat.e = bei_eph.ecc;
    rtklib_sat.Adot = bei_eph.Adot;
    rtklib_sat.ndot = bei_eph.delta_ndot;
    rtklib_sat.svh = bei_eph.hs;
    rtklib_sat.sva = 0;
    rtklib_sat.code = bei_eph.sig_type;
    rtklib_sat.flag = bei_eph.nav_type;
    rtklib_sat.iode = static_cast<int32_t>(bei_eph.IODE);
    rtklib_sat.iodc = static_cast<int32_t>(bei_eph.IODC);
    rtklib_sat.week = bei_eph.WN;
    rtklib_sat.cic = bei_eph.Cic;
    rtklib_sat.cis = bei_eph.Cis;
    rtklib_sat.cuc = bei_eph.Cuc;
    rtklib_sat.cus = bei_eph.Cus;
    rtklib_sat.crc = bei_eph.Crc;
    rtklib_sat.crs = bei_eph.Crs;
    rtklib_sat.f0 = bei_eph.af0;
    rtklib_sat.f1 = bei_eph.af1;
    rtklib_sat.f2 = bei_eph.af2;
    /* ICD §7.6 → gettgd(): tgd[0]=TGD_B1Cp, tgd[1]=TGD_B2ap, tgd[2]=ISC_B1Cd */
    rtklib_sat.tgd[0] = bei_eph.TGD_B1Cp;
    rtklib_sat.tgd[1] = bei_eph.TGD_B2ap;
    rtklib_sat.tgd[2] = bei_eph.ISC_B1Cd;
    rtklib_sat.toes = bei_eph.toe;
    rtklib_sat.toe = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.toe));
    rtklib_sat.toc = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.toc));
    rtklib_sat.ttr = bdt2gpst(bdt2time(rtklib_sat.week, bei_eph.tow));

    double tow = time2gpst(rtklib_sat.ttr, &rtklib_sat.week);
    const double toc = time2gpst(rtklib_sat.toc, nullptr);
    const double toe = time2gpst(rtklib_sat.toe, nullptr);
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
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {}, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0, 0, 0.0, -1, 0};
    const int gps_sys = (MINPRNQZS <= gps_cnav_eph.PRN && gps_cnav_eph.PRN <= MAXPRNQZS) ? SYS_QZS : SYS_GPS;
    rtklib_sat.sat = satno(gps_sys, gps_cnav_eph.PRN);
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
    rtklib_sat.cnav_uraed = gps_cnav_eph.URAED;
    rtklib_sat.cnav_uraned0 = gps_cnav_eph.URANED0;
    rtklib_sat.cnav_uraned1 = gps_cnav_eph.URANED1;
    rtklib_sat.cnav_uraned2 = gps_cnav_eph.URANED2;
    rtklib_sat.cnav_top = gps_cnav_eph.top;
    rtklib_sat.cnav_wnop = gps_cnav_eph.WNop;
    rtklib_sat.cnav_ura_valid = 1;
    rtklib_sat.sva = gps_cnav_ura_to_rtklib_sva(gps_cnav_eph.URAED, gps_cnav_eph.URANED0);

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

    const int gps_sys = (MINPRNQZS <= gps_alm.PRN && gps_alm.PRN <= MAXPRNQZS) ? SYS_QZS : SYS_GPS;

    rtklib_alm.sat = satno(gps_sys, gps_alm.PRN);
    rtklib_alm.svh = gps_alm.SV_health;
    rtklib_alm.svconf = gps_alm.AS_status;
    rtklib_alm.week = gps_alm.WNa;
    gtime_t toa;
    toa.time = gps_alm.toa;
    toa.sec = 0.0;
    rtklib_alm.toa = toa;
    rtklib_alm.A = gps_alm.sqrtA * gps_alm.sqrtA;
    rtklib_alm.e = gps_alm.ecc;
    rtklib_alm.i0 = ((gps_alm.get_system() == 'J') ? gps_alm.delta_i : (gps_alm.delta_i + 0.3)) * GNSS_PI;
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


alm_t alm_to_rtklib(const Beidou_Dnav_Almanac& bei_alm)
{
    alm_t rtklib_alm = {0, 0, 0, 0, {0, 0}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    rtklib_alm.sat = satno(SYS_BDS, static_cast<int32_t>(bei_alm.PRN));
    rtklib_alm.svh = bei_alm.SV_health;
    rtklib_alm.svconf = 0;
    rtklib_alm.week = bei_alm.WNa + BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET;
    rtklib_alm.toa = bdt2gpst(bdt2time(bei_alm.WNa, static_cast<double>(bei_alm.toa)));
    rtklib_alm.A = bei_alm.sqrtA * bei_alm.sqrtA;
    rtklib_alm.e = bei_alm.ecc;
    const bool geo = (bei_alm.PRN >= 1 && bei_alm.PRN <= 5) || (bei_alm.PRN >= 59 && bei_alm.PRN <= 63);
    rtklib_alm.i0 = bei_alm.delta_i + (geo ? 0.0 : 0.3 * GNSS_PI);
    rtklib_alm.OMG0 = bei_alm.OMEGA_0;
    rtklib_alm.OMGd = bei_alm.OMEGAdot;
    rtklib_alm.omg = bei_alm.omega;
    rtklib_alm.M0 = bei_alm.M_0;
    rtklib_alm.f0 = bei_alm.af0;
    rtklib_alm.f1 = bei_alm.af1;
    rtklib_alm.toas = static_cast<double>(bei_alm.toa);

    return rtklib_alm;
}

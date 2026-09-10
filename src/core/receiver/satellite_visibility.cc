/*!
 * \file satellite_visibility.cc
 * \brief Runtime satellite-visibility tracking for acquisition search
 * prioritization.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "satellite_visibility.h"
#include "agnss_ref_location.h"  // for parse_agnss_ref_location
#include "agnss_ref_time.h"      // for parse_agnss_ref_utc_time
#include "configuration_interface.h"
#include "geofunctions.h"
#include "monitor_pvt.h"
#include "pvt_interface.h"
#include "rtklib_conversions.h"  // for alm_to_rtklib, eph_to_rtklib
#include "rtklib_ephemeris.h"    // for alm2pos, eph2pos
#include "rtklib_rtkcmn.h"       // for utc2gpst, gpst2time
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
// Signed age (seconds) of data referenced at TOW ref_tow, relative to the
// current TOW now_tow (both seconds-of-week, [0, 604800)) -- positive means
// the reference is in the past, negative means it's still ahead (normal for
// ephemeris broadcast near the start of its fit interval). Handles the
// week-boundary wrap so data referenced just before a Saturday/Sunday
// midnight rollover isn't misjudged as ~604800 s stale the moment the week
// ticks over. Only unambiguous for |age| well under half a week (302400 s);
// callers must keep their max-age thresholds under that.
double DataAgeS(double now_tow, int32_t ref_tow)
{
    double age = now_tow - static_cast<double>(ref_tow);
    if (age > 302400.0)
        {
            age -= 604800.0;
        }
    else if (age < -302400.0)
        {
            age += 604800.0;
        }
    return age;
}
}  // namespace


std::vector<std::pair<int, Gnss_Satellite>> compute_visible_satellites(
    const std::shared_ptr<PvtInterface>& pvt_ptr,
    const gtime_t& gps_gtime,
    const arma::vec& r_eb_e,
    double elevation_mask_deg,
    std::vector<std::pair<int, Gnss_Satellite>>* below_mask_out,
    double almanac_max_age_s,
    double* seconds_until_next_expiry_out,
    const std::set<std::pair<std::string, uint32_t>>* only_prns)
{
    std::vector<std::pair<int, Gnss_Satellite>> available_satellites;
    // Almanac WNa is truncated (2 bits for Galileo, 8 for GPS) and needs a
    // nearby full week to resolve against -- gps_gtime (the query time this
    // whole computation is FOR) is always a valid, always-current source for
    // that, unlike a config-supplied reference week (GNSS-SDR.observation_
    // date), which is easy to leave unset. An unset config value used to
    // flow through as ref_gps_week<=0, which resolve_truncated_week() (see
    // its doc comment) treated as "no reference -- return WNa unresolved",
    // silently reintroducing the ~1024/~16-week rollover error the WN fix
    // was supposed to eliminate. Deriving it here instead means a valid
    // reference is guaranteed every time this function runs at all.
    int ref_gps_week = 0;
    const double now_tow = time2gpst(gps_gtime, &ref_gps_week);
    if (seconds_until_next_expiry_out != nullptr)
        {
            *seconds_until_next_expiry_out = std::numeric_limits<double>::infinity();
        }
    auto note_freshness = [&](double age, double max_age_s) {
        if (seconds_until_next_expiry_out != nullptr && std::isfinite(max_age_s))
            {
                *seconds_until_next_expiry_out = std::min(*seconds_until_next_expiry_out, max_age_s - age);
            }
    };
    // PRNs already classified (visible or not) via ephemeris -- the almanac
    // loops below skip any PRN in here entirely, rather than only skipping
    // ones already added as visible. Ephemeris is materially more accurate
    // than almanac, so once it has classified a PRN one way or the other,
    // almanac must not be allowed to reclassify it the other way; without
    // this, the same PRN could end up in both available_satellites (visible)
    // and *below_mask_out (elevation known, not visible) depending on which
    // data source is inspected first, which callers rely on never happening.
    std::vector<unsigned int> handled_gps;
    std::vector<unsigned int> handled_gal;
    std::vector<unsigned int> handled_bds;

    const std::map<int, Gps_Ephemeris> gps_eph_map = pvt_ptr->get_gps_ephemeris();
    for (const auto& it : gps_eph_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("GPS"), it.second.PRN)) == 0)
                {
                    continue;
                }
            const double age = DataAgeS(now_tow, it.second.toe);
            if (std::abs(age) > MAXDTOE)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE);
            const eph_t rtklib_eph = eph_to_rtklib(it.second, ref_gps_week);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            double sat_pos_variance_m2;
            eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s,
                &sat_pos_variance_m2);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            handled_gps.push_back(it.second.PRN);
            // Broadcast health (0 == OK) is stronger evidence than geometry
            // alone -- a satellite explicitly flagged unhealthy is known
            // unusable regardless of what its computed elevation says, same
            // as an elevation at/below the mask.
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    std::cout << "Using GPS Ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("GPS"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using GPS Ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("GPS"), it.second.PRN));
                }
        }

    const std::map<int, Galileo_Ephemeris> gal_eph_map = pvt_ptr->get_galileo_ephemeris();
    for (const auto& it : gal_eph_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Galileo"), it.second.PRN)) == 0)
                {
                    continue;
                }
            const double age = DataAgeS(now_tow, it.second.toe);
            if (std::abs(age) > MAXDTOE_GAL)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE_GAL);
            const eph_t rtklib_eph = eph_to_rtklib(it.second);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            double sat_pos_variance_m2;
            eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s,
                &sat_pos_variance_m2);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            handled_gal.push_back(it.second.PRN);
            // E1B_HS (0 == "signal OK" per the Galileo ICD) gates this --
            // 1B is the primary/search-initiating signal, and 5X only ever
            // gets searched once 1B is confirmed tracked (assist path), so
            // an unhealthy 1B makes the whole satellite unusable for search
            // regardless of E5a_HS/E5b_HS or computed elevation.
            if (El > elevation_mask_deg && it.second.E1B_HS == 0)
                {
                    std::cout << "Using Galileo Ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Galileo"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using Galileo Ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Galileo"), it.second.PRN));
                }
        }

    const std::map<int, Beidou_Dnav_Ephemeris> bds_eph_map = pvt_ptr->get_beidou_dnav_ephemeris();
    for (const auto& it : bds_eph_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Beidou"), it.second.PRN)) == 0)
                {
                    continue;
                }
            const double age = DataAgeS(now_tow, it.second.toe);
            if (std::abs(age) > MAXDTOE_BDS)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE_BDS);
            const eph_t rtklib_eph = eph_to_rtklib(it.second);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            double sat_pos_variance_m2;
            eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s,
                &sat_pos_variance_m2);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            handled_bds.push_back(it.second.PRN);
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    std::cout << "Using BeiDou Ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Beidou"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using BeiDou Ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Beidou"), it.second.PRN));
                }
        }

    const std::map<int, Gps_Almanac> gps_alm_map = pvt_ptr->get_gps_almanac();
    for (const auto& it : gps_alm_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("GPS"), it.second.PRN)) == 0)
                {
                    continue;
                }
            if (std::find(handled_gps.begin(), handled_gps.end(), it.second.PRN) != handled_gps.end())
                {
                    continue;  // ephemeris already classified this PRN
                }
            const double age = DataAgeS(now_tow, it.second.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
            const alm_t rtklib_alm = alm_to_rtklib(it.second, ref_gps_week);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            // gps_gtime is a proper absolute epoch (see its own construction
            // above), matching what alm_to_rtklib() now builds rtklib_alm.toa
            // as (gpst2time(week, toa) -- also absolute, since the WN fix).
            // This used to instead build a throwaway gtime_t truncated down
            // to a bare seconds-of-week value via fmod(..., 604800), which
            // was only self-consistent with alm_to_rtklib()'s pre-fix
            // toa.time = raw_toa (also un-resolved/small) -- once toa became
            // a real absolute epoch, subtracting it from that truncated
            // query time in alm2pos() produced a ~56-year-scale tk, the same
            // class of error the WN fix was supposed to eliminate, just
            // relocated to the other operand. BeiDou's almanac loop below
            // never had this truncation and has always passed gps_gtime
            // directly -- matching that here.
            alm2pos(gps_gtime, &rtklib_alm, r_sat.data(), &clock_bias_s);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    std::cout << "Using GPS Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("GPS"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using GPS Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("GPS"), it.second.PRN));
                }
        }

    const std::map<int, Galileo_Almanac> gal_alm_map = pvt_ptr->get_galileo_almanac();
    for (const auto& it : gal_alm_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Galileo"), it.second.PRN)) == 0)
                {
                    continue;
                }
            if (std::find(handled_gal.begin(), handled_gal.end(), it.second.PRN) != handled_gal.end())
                {
                    continue;  // ephemeris already classified this PRN
                }
            const double age = DataAgeS(now_tow, it.second.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
            const alm_t rtklib_alm = alm_to_rtklib(it.second, ref_gps_week);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            // See the matching comment in the GPS almanac loop above -- same
            // fix, same reason.
            alm2pos(gps_gtime, &rtklib_alm, r_sat.data(), &clock_bias_s);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            // A handful of single-tick,
            // self-correcting extreme-elevation readings have been observed
            // (e.g. E11 at -89.8 deg for exactly one Tick(), -16/-15 deg
            // immediately before and after) with no almanac decode/overwrite
            // for that PRN anywhere near the event, and every other
            // satellite in the same Tick() (same gps_gtime, same r_eb_e)
            // computing a normal elevation. That rules out a shared
            // time/position corruption and points at either the stored
            // record for just this PRN or the alm2pos() Kepler solve itself.
            // Dump every field alm2pos() actually consumes so the next
            // occurrence is caught with enough data to tell those apart.
            if (std::abs(El) > 85.0)
                {
                    LOG(WARNING) << "[visibility] extreme-El diag: Galileo Almanac Sat " << it.second.PRN
                                 << " El=" << El << " week=" << rtklib_alm.week
                                 << " toa.time=" << rtklib_alm.toa.time << " toa.sec=" << rtklib_alm.toa.sec
                                 << " gps_gtime.time=" << gps_gtime.time << " gps_gtime.sec=" << gps_gtime.sec
                                 << " M_0=" << it.second.M_0 << " ecc=" << it.second.ecc
                                 << " sqrtA=" << it.second.sqrtA << " OMEGA_0=" << it.second.OMEGA_0
                                 << " omega=" << it.second.omega << " OMEGAdot=" << it.second.OMEGAdot
                                 << " delta_i=" << it.second.delta_i << " af0=" << it.second.af0
                                 << " af1=" << it.second.af1 << " r_sat=(" << r_sat[0] << "," << r_sat[1] << "," << r_sat[2] << ")"
                                 << " r_eb_e=(" << r_eb_e[0] << "," << r_eb_e[1] << "," << r_eb_e[2] << ")";
                }
            if (El > elevation_mask_deg && it.second.E1B_HS == 0)
                {
                    std::cout << "Using Galileo Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Galileo"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using Galileo Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Galileo"), it.second.PRN));
                }
        }

    const std::map<int, Beidou_Dnav_Almanac> bds_alm_map = pvt_ptr->get_beidou_dnav_almanac();
    for (const auto& it : bds_alm_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Beidou"), it.second.PRN)) == 0)
                {
                    continue;
                }
            if (std::find(handled_bds.begin(), handled_bds.end(), it.second.PRN) != handled_bds.end())
                {
                    continue;  // ephemeris already classified this PRN
                }
            const double age = DataAgeS(now_tow, it.second.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
            const alm_t rtklib_alm = alm_to_rtklib(it.second);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            alm2pos(gps_gtime, &rtklib_alm, r_sat.data(), &clock_bias_s);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    std::cout << "Using BeiDou Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Beidou"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    std::cout << "Using BeiDou Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El << '\n';
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Beidou"), it.second.PRN));
                }
        }

    std::sort(available_satellites.begin(), available_satellites.end(), [](const std::pair<int, Gnss_Satellite>& a, const std::pair<int, Gnss_Satellite>& b) {
        return a.first < b.first;
    });
    std::reverse(available_satellites.begin(), available_satellites.end());
    return available_satellites;
}


namespace
{
// Keeps (search_ratio_ + 1) comfortably clear of uint32_t overflow in
// GNSSFlowgraph::pop_by_visibility()'s counter modulo, no matter what a
// misconfigured .conf sets.
constexpr uint32_t kMaxSearchRatio = 1000000U;

// See ticks_since_data_check_'s doc comment.
constexpr int kDataCheckEveryNTicks = 20;
}  // namespace


SatelliteVisibility::SatelliteVisibility(const std::shared_ptr<ConfigurationInterface>& configuration)
    : enabled_(configuration->property("GNSS-SDR.enable_visibility_aware_search", false)),
      search_ratio_(std::min(configuration->property("GNSS-SDR.visible_vs_mayvisible_search_ratio", 3U), kMaxSearchRatio)),
      // 120 s (not the old 5 s): satellites move negligibly over a couple of
      // minutes, and the tighter reactivity that actually matters now comes
      // from the event-driven triggers below (data arrival/update, receiver
      // movement, data expiry), not from polling this interval hard.
      recompute_interval_s_(configuration->property("GNSS-SDR.visibility_recompute_interval_s", 120.0)),
      elevation_mask_deg_(configuration->property("GNSS-SDR.search_elevation_mask", 0.0)),
      position_threshold_m_(configuration->property("GNSS-SDR.visibility_recompute_position_threshold_m", 1000.0)),
      // Half-window around toa -- see compute_visible_satellites()'s
      // matching parameter. Must stay well under 302400 s (half a week) for
      // the toa-only staleness math to remain unambiguous. Default: 3 days.
      // (Ephemeris staleness has no equivalent config parameter -- see
      // almanac_max_age_s_'s doc comment.)
      almanac_max_age_s_(configuration->property("GNSS-SDR.visibility_almanac_max_age_s", 259200.0)),
      have_agnss_reference_(false),
      agnss_ref_lat_deg_(0.0),
      agnss_ref_lon_deg_(0.0),
      agnss_ref_utc_time_(0),
      last_fix_valid_(false),
      first_data_check_(true),
      last_recompute_rx_time_s_(-1.0e9),
      last_recompute_r_eb_e_(arma::vec{0.0, 0.0, 0.0}),
      next_expiry_deadline_rx_time_(std::numeric_limits<double>::infinity()),
      ticks_since_data_check_(kDataCheckEveryNTicks)
{
    // Shared with ControlThread::init() (see agnss_ref_location.h/
    // agnss_ref_time.h) so this parsing -- including the AGNSS_ref_utc_time
    // empty-string-means-wall-clock-time fallback -- lives in exactly one
    // place rather than being duplicated per caller.
    const Agnss_Ref_Location parsed_location = parse_agnss_ref_location(
        configuration->property("GNSS-SDR.AGNSS_ref_location", std::string("")));
    have_agnss_reference_ = parsed_location.valid;
    agnss_ref_lat_deg_ = parsed_location.lat;
    agnss_ref_lon_deg_ = parsed_location.lon;

    const Agnss_Ref_Time parsed_time = parse_agnss_ref_utc_time(
        configuration->property("GNSS-SDR.AGNSS_ref_utc_time", std::string("")));
    // parsed_time.valid is only false for a non-empty, malformed string --
    // fall back to wall-clock time there too rather than an unusable zero.
    agnss_ref_utc_time_ = parsed_time.valid ? static_cast<time_t>(parsed_time.seconds) : std::time(nullptr);

    // Same GNSS-SDR.<System>_banned_prns parsing as
    // GNSSFlowgraph::set_signals_list(), for every system
    // compute_visible_satellites() covers.
    for (const std::string system : {"GPS", "Galileo", "Beidou"})
        {
            const auto sv_banned = configuration->property("GNSS-SDR." + system + "_banned_prns", std::string(""));
            if (sv_banned.empty())
                {
                    continue;
                }
            std::stringstream ss(sv_banned);
            while (ss.good())
                {
                    std::string substr;
                    std::getline(ss, substr, ',');
                    try
                        {
                            banned_.emplace(system, static_cast<uint32_t>(std::stoi(substr)));
                        }
                    catch (const std::invalid_argument& ia)
                        {
                            std::cerr << "Invalid argument at GNSS-SDR." << system << "_banned_prns configuration parameter: " << ia.what() << '\n';
                        }
                    catch (const std::out_of_range& oor)
                        {
                            std::cerr << "Out of range at GNSS-SDR." << system << "_banned_prns configuration parameter: " << oor.what() << '\n';
                        }
                }
        }

    // Which systems actually have a channel configured, same signal-code
    // list as gnss_block_factory.cc's authoritative signal_mapping (only
    // the entries belonging to a system compute_visible_satellites()
    // covers).
    static const std::map<std::string, std::vector<std::string>> system_signals{
        {"GPS", {"1C", "2S", "L5"}},
        {"Galileo", {"1B", "5X", "E6", "7X"}},
        {"Beidou", {"B1", "1D", "B3"}}};
    for (const auto& entry : system_signals)
        {
            for (const auto& signal : entry.second)
                {
                    if (configuration->property("Channels_" + signal + ".count", 0) > 0)
                        {
                            configured_systems_.insert(entry.first);
                            break;
                        }
                }
        }
}


bool SatelliteVisibility::DataChanged(const std::shared_ptr<PvtInterface>& pvt_ptr,
    std::set<std::pair<std::string, uint32_t>>* changed_prns_out)
{
    // Per-(system, type, PRN) fingerprint keyed on toe/toa -- finer than a
    // map-size count: catches an existing PRN's data being *replaced* (new
    // toe/toa broadcast, same PRN, same map size), not just brand new PRNs
    // appearing. See last_data_fingerprints_'s doc comment.
    std::map<std::tuple<std::string, std::string, uint32_t>, int32_t> current;
    auto add = [&current](const char* system, const char* type, uint32_t prn, int32_t ref_time) {
        current.emplace(std::make_tuple(std::string(system), std::string(type), prn), ref_time);
    };
    for (const auto& it : pvt_ptr->get_gps_ephemeris())
        {
            add("GPS", "EPH", it.second.PRN, it.second.toe);
        }
    for (const auto& it : pvt_ptr->get_galileo_ephemeris())
        {
            add("Galileo", "EPH", it.second.PRN, it.second.toe);
        }
    for (const auto& it : pvt_ptr->get_beidou_dnav_ephemeris())
        {
            add("Beidou", "EPH", it.second.PRN, it.second.toe);
        }
    for (const auto& it : pvt_ptr->get_gps_almanac())
        {
            add("GPS", "ALM", it.second.PRN, it.second.toa);
        }
    for (const auto& it : pvt_ptr->get_galileo_almanac())
        {
            add("Galileo", "ALM", it.second.PRN, it.second.toa);
        }
    for (const auto& it : pvt_ptr->get_beidou_dnav_almanac())
        {
            add("Beidou", "ALM", it.second.PRN, it.second.toa);
        }

    const bool changed = first_data_check_ || (current != last_data_fingerprints_);
    if (changed && !first_data_check_)
        {
            size_t added = 0;
            size_t updated = 0;
            size_t removed = 0;
            for (const auto& entry : current)
                {
                    const auto it = last_data_fingerprints_.find(entry.first);
                    if (it == last_data_fingerprints_.end())
                        {
                            ++added;
                            if (changed_prns_out != nullptr)
                                {
                                    changed_prns_out->emplace(std::get<0>(entry.first), std::get<2>(entry.first));
                                }
                        }
                    else if (it->second != entry.second)
                        {
                            ++updated;
                            if (changed_prns_out != nullptr)
                                {
                                    changed_prns_out->emplace(std::get<0>(entry.first), std::get<2>(entry.first));
                                }
                        }
                }
            for (const auto& entry : last_data_fingerprints_)
                {
                    if (current.find(entry.first) == current.end())
                        {
                            ++removed;
                            // A PRN whose data disappeared entirely has nothing
                            // left to recompute Az/El from, but Tick()'s merge
                            // still needs to know about it so it can drop back
                            // to "maybe visible" if it was previously classified
                            // -- see the merge logic in Tick().
                            if (changed_prns_out != nullptr)
                                {
                                    changed_prns_out->emplace(std::get<0>(entry.first), std::get<2>(entry.first));
                                }
                        }
                }
            LOG(INFO) << "[visibility] data_changed: " << added << " new, " << updated
                      << " updated, " << removed << " removed (ephemeris/almanac entries, by PRN)";
        }
    first_data_check_ = false;
    last_data_fingerprints_ = std::move(current);
    return changed;
}


bool SatelliteVisibility::Tick(const std::shared_ptr<PvtInterface>& pvt_ptr, const Monitor_Pvt& fix_status)
{
    if (!enabled_ || !pvt_ptr)
        {
            return false;
        }

    const bool fix_valid = (fix_status.RX_time >= 0.0);
    const bool fix_became_valid = fix_valid && !last_fix_valid_;
    last_fix_valid_ = fix_valid;

    std::array<float, 3> LLH{};
    gtime_t gps_gtime{};
    if (fix_valid)
        {
            LLH[0] = static_cast<float>(fix_status.latitude);
            LLH[1] = static_cast<float>(fix_status.longitude);
            LLH[2] = static_cast<float>(fix_status.height);
            gps_gtime = gpst2time(static_cast<int>(fix_status.week), fix_status.RX_time);
        }
    else if (have_agnss_reference_)
        {
            LLH[0] = static_cast<float>(agnss_ref_lat_deg_);
            LLH[1] = static_cast<float>(agnss_ref_lon_deg_);
            LLH[2] = 0.0F;
            gtime_t utc_gtime{};
            // agnss_ref_utc_time_ prefers the configured AGNSS_ref_utc_time
            // over the host's wall-clock time -- critical for offline/replay
            // runs, where the host clock has nothing to do with the GNSS
            // time actually embedded in the samples being processed (using
            // wall-clock time there silently computes elevation for the
            // wrong epoch/orbital configuration entirely). See its
            // construction above.
            utc_gtime.time = agnss_ref_utc_time_;
            utc_gtime.sec = 0.0;
            gps_gtime = utc2gpst(utc_gtime);
        }
    else
        {
            return false;  // no position/time source available yet
        }

    const arma::vec LLH_rad = arma::vec{degtorad(LLH[0]), degtorad(LLH[1]), LLH[2]};
    arma::mat C_tmp = arma::zeros(3, 3);
    arma::vec r_eb_e = arma::zeros(3, 1);
    arma::vec v_eb_e = arma::zeros(3, 1);
    Geo_to_ECEF(LLH_rad, arma::vec{0, 0, 0}, C_tmp, r_eb_e, v_eb_e, C_tmp);

    // Throttle DataChanged() itself (see ticks_since_data_check_'s doc
    // comment) -- it's only worth paying its full-map-copy cost once every
    // few Tick() calls, not on every idle tick.
    bool data_changed = false;
    std::set<std::pair<std::string, uint32_t>> changed_prns;
    if (++ticks_since_data_check_ >= kDataCheckEveryNTicks)
        {
            ticks_since_data_check_ = 0;
            data_changed = DataChanged(pvt_ptr, &changed_prns);
        }

    // Recompute cadence is governed by receiver time (fix_status.RX_time),
    // not wall-clock time -- see last_recompute_rx_time_s_'s doc comment.
    // Not meaningful before the first fix (RX_time isn't ticking yet);
    // fix_became_valid/data_changed cover that phase instead.
    const bool interval_elapsed = fix_valid && ((fix_status.RX_time - last_recompute_rx_time_s_) >= recompute_interval_s_);

    // See position_threshold_m_'s doc comment.
    const bool moved_significantly = arma::norm(r_eb_e - last_recompute_r_eb_e_, 2) >= position_threshold_m_;

    // See next_expiry_deadline_rx_time_'s doc comment.
    const bool expired = fix_valid && (fix_status.RX_time >= next_expiry_deadline_rx_time_);

    if (!fix_became_valid && !data_changed && !interval_elapsed && !moved_significantly && !expired)
        {
            return false;
        }

    // fix_became_valid/interval_elapsed/moved_significantly/expired can all
    // affect *every* satellite's elevation (receiver position or time moved,
    // or cached data anywhere went stale) -- those need a full sweep. Only
    // data_changed firing alone means exactly one thing changed (one PRN's
    // ephemeris/almanac), so scoping the recompute to just that PRN (via
    // only_prns below) is safe and correct, and avoids redoing eph2pos/
    // topocent for the whole constellation on every single ephemeris page
    // that happens to arrive during startup.
    const bool needs_full_recompute = fix_became_valid || interval_elapsed || moved_significantly || expired;

    // Both of these gate a *full-sweep* trigger (interval_elapsed,
    // moved_significantly) against drift since the last full sweep -- they
    // must only advance when a full sweep actually just ran. Updating them
    // on every qualifying Tick(), including a data_changed-only (targeted)
    // one, would keep resetting both clocks before the interval/displacement
    // threshold is ever reached whenever ephemeris/almanac data is arriving
    // often enough (routine with a dozen+ tracked satellites) -- silently
    // starving the periodic/position-triggered full recompute forever, even
    // though a satellite could be rising above the horizon the whole time
    // with no ephemeris/almanac change of its own to trigger a targeted one.
    if (needs_full_recompute)
        {
            if (fix_valid)
                {
                    last_recompute_rx_time_s_ = fix_status.RX_time;
                }
            last_recompute_r_eb_e_ = r_eb_e;
        }

    std::vector<std::pair<int, Gnss_Satellite>> below_mask;
    double seconds_until_next_expiry = std::numeric_limits<double>::infinity();
    const auto elevations = compute_visible_satellites(pvt_ptr, gps_gtime, r_eb_e, elevation_mask_deg_,
        &below_mask, almanac_max_age_s_,
        needs_full_recompute ? &seconds_until_next_expiry : nullptr,
        needs_full_recompute ? nullptr : &changed_prns);
    // Only a full sweep sees every currently-classified satellite's
    // freshness, so only a full sweep can correctly narrow this deadline --
    // updating it from a targeted sweep's much smaller view could push it
    // later than the true soonest expiry among satellites not touched this
    // tick, silently delaying a real expired-data recompute.
    if (needs_full_recompute)
        {
            next_expiry_deadline_rx_time_ = (fix_valid && std::isfinite(seconds_until_next_expiry))
                                                ? fix_status.RX_time + seconds_until_next_expiry
                                                : std::numeric_limits<double>::infinity();
        }

    // Classify every freshly computed elevation (visible or excluded --
    // both mean elevation was computable) using the raw reading itself,
    // always -- decoding correctness is enforced at its actual sources
    // (week-number resolution in alm_to_rtklib(), almanac freshness/
    // overwrite priority in update_almanac_if_fresher(), the ephemeris/
    // almanac map mutex), not by second-guessing a live reading here.
    // reference_elevation_deg_ is updated with every raw reading as it's
    // accepted -- see its own doc comment for what it's actually for.
    std::vector<std::pair<int, Gnss_Satellite>> accepted_entries;
    accepted_entries.reserve(elevations.size() + below_mask.size());
    // Health-check failures from below_mask -- see the loop below. Always
    // excluded, unconditionally, regardless of elevation.
    std::vector<std::pair<int, Gnss_Satellite>> forced_excluded_entries;
    {
        auto record_elevation = [&](const std::pair<int, Gnss_Satellite>& entry) {
            const auto key = std::make_pair(entry.second.get_system(), entry.second.get_PRN());
            const int raw = entry.first;
            reference_elevation_deg_[key] = raw;
            accepted_entries.emplace_back(raw, entry.second);
        };
        for (const auto& entry : elevations)
            {
                record_elevation(entry);
            }
        for (const auto& entry : below_mask)
            {
                reference_elevation_deg_[std::make_pair(entry.second.get_system(), entry.second.get_PRN())] = entry.first;
                // A below_mask entry with a raw elevation *above* the mask
                // can only mean one thing: it failed the broadcast health
                // check in compute_visible_satellites(), not the elevation
                // check (that path's condition is `El > mask && healthy`,
                // so anything landing here with El > mask must have failed
                // on health). This is a real, unconditional classification
                // rule (a health flag is definitive, not a heuristic) --
                // route straight to forced-excluded instead of the
                // elevation-mask comparison the loop below applies to
                // accepted_entries.
                if (static_cast<double>(entry.first) > elevation_mask_deg_)
                    {
                        forced_excluded_entries.push_back(entry);
                        continue;
                    }
                accepted_entries.emplace_back(entry.first, entry.second);
            }
    }

    // A full recompute starts classification from scratch, exactly as
    // before. A targeted recompute starts from the *current* membership
    // instead, with only the recomputed PRNs (changed_prns) first erased
    // from both sets -- they may no longer belong in either (e.g. their
    // data was removed entirely and compute_visible_satellites() has
    // nothing to classify them with this tick) -- so every other
    // satellite's classification is left completely untouched, and the loop
    // below only re-adds the PRNs that were actually recomputed.
    std::set<std::pair<std::string, uint32_t>> new_visible = needs_full_recompute ? std::set<std::pair<std::string, uint32_t>>{} : visible_;
    std::set<std::pair<std::string, uint32_t>> new_excluded = needs_full_recompute ? std::set<std::pair<std::string, uint32_t>>{} : excluded_;
    if (!needs_full_recompute)
        {
            for (const auto& key : changed_prns)
                {
                    new_visible.erase(key);
                    new_excluded.erase(key);
                }
        }
    for (const auto& entry : accepted_entries)
        {
            const auto key = std::make_pair(entry.second.get_system(), entry.second.get_PRN());
            if (static_cast<double>(entry.first) > elevation_mask_deg_)
                {
                    if (banned_.count(key) > 0)
                        {
                            continue;
                        }
                    new_visible.emplace(key);
                }
            else
                {
                    new_excluded.emplace(key);
                }
        }
    for (const auto& entry : forced_excluded_entries)
        {
            new_excluded.emplace(entry.second.get_system(), entry.second.get_PRN());
        }

    const bool changed = (new_visible != visible_) || (new_excluded != excluded_);
    visible_ = std::move(new_visible);
    excluded_ = std::move(new_excluded);

    // Always-on diagnostic (LOG, not DLOG -- DLOG is compiled out entirely in
    // release/NDEBUG builds, so it would be invisible in exactly the builds
    // this is most useful for). Reports what SatelliteVisibility currently
    // believes is visible and excluded (elevation known, at/below the mask),
    // WITH the computed elevation for each, plus the maybe-visible set (no
    // ephemeris/almanac yet -- genuinely unknown, still searched at lower
    // priority) by PRN only, since it has no elevation to show. Compact,
    // sorted-by-PRN groups (E03(+31), not "Galileo 3 (31 deg)") read far
    // faster at a glance than the old prose form -- this is what a manually
    // reconstructed comparison across recompute snapshots looked like
    // before this was built directly into the log line.
    // See GNSSFlowgraph::pop_by_visibility() for what actually gets
    // *searched* from each bucket, which is the more direct signal for
    // diagnosing CPU load.
    {
        auto short_id = [](const Gnss_Satellite& sat) {
            std::ostringstream os;
            os << sat.get_system_short() << std::setfill('0') << std::setw(2) << sat.get_PRN();
            return os.str();
        };
        auto sort_by_prn = [](std::vector<std::pair<int, Gnss_Satellite>> entries) {
            std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
                return a.second.get_PRN() < b.second.get_PRN();
            });
            return entries;
        };
        auto append_with_el = [&](std::ostringstream& os, const std::vector<std::pair<int, Gnss_Satellite>>& entries) {
            bool first = true;
            for (const auto& entry : sort_by_prn(entries))
                {
                    if (!first)
                        {
                            os << ", ";
                        }
                    os << short_id(entry.second) << "(" << std::showpos << entry.first << std::noshowpos << ")";
                    first = false;
                }
        };
        // Built from the final, post-merge visible_/excluded_ rather than
        // accumulated during the classify loop above, so a targeted
        // recompute's report is just as complete as a full one: every
        // entry's elevation comes from reference_elevation_deg_, which was
        // just refreshed above for whichever PRN(s) were actually
        // recomputed this tick and still holds the last known value for
        // everyone else, untouched.
        auto elevation_for = [&](const std::pair<std::string, uint32_t>& key) -> int {
            const auto it = reference_elevation_deg_.find(key);
            return it != reference_elevation_deg_.end() ? static_cast<int>(it->second) : 0;
        };
        std::vector<std::pair<int, Gnss_Satellite>> visible_entries;
        visible_entries.reserve(visible_.size());
        for (const auto& key : visible_)
            {
                visible_entries.emplace_back(elevation_for(key), Gnss_Satellite(key.first, key.second));
            }
        std::vector<std::pair<int, Gnss_Satellite>> excluded_entries;
        excluded_entries.reserve(excluded_.size());
        for (const auto& key : excluded_)
            {
                excluded_entries.emplace_back(elevation_for(key), Gnss_Satellite(key.first, key.second));
            }
        // Maybe-visible has no elevation to report -- listing by PRN only
        // needs the full non-banned PRN range of every *configured* system
        // compute_visible_satellites() covers (same {"GPS", "Galileo",
        // "Beidou"} list as the banned_prns parsing above), not just
        // whatever happened to be in visible_/excluded_ this tick. This
        // used to be hardcoded to Galileo's own range (1-36) only, from
        // when the feature was Galileo-only -- silently dropping every
        // other system's not-yet-classified satellites from this specific
        // report line once multi-constellation support was added, even
        // though visible_/excluded_ themselves were already tracking them
        // correctly (that's why GPS satellites show up fine in VISIBLE/
        // EXCLUDED once classified -- only this maybe-visible enumeration
        // was ever GPS-blind). Scoped to configured_systems_ (systems with
        // at least one channel actually configured) so an unconfigured
        // system's whole PRN range doesn't sit here as permanent noise --
        // nothing will ever decode ephemeris/almanac for it in this run.
        std::vector<Gnss_Satellite> maybe_visible;
        for (const auto& system_range : {std::make_tuple(std::string("GPS"), static_cast<uint32_t>(MINPRNGPS), static_cast<uint32_t>(MAXPRNGPS)),
                 std::make_tuple(std::string("Galileo"), static_cast<uint32_t>(MINPRNGAL), static_cast<uint32_t>(MAXPRNGAL)),
                 std::make_tuple(std::string("Beidou"), static_cast<uint32_t>(MINPRNBDS), static_cast<uint32_t>(MAXPRNBDS))})
            {
                const auto& system = std::get<0>(system_range);
                if (configured_systems_.count(system) == 0)
                    {
                        continue;
                    }
                for (uint32_t prn = std::get<1>(system_range); prn <= std::get<2>(system_range); prn++)
                    {
                        const auto key = std::make_pair(system, prn);
                        if (banned_.count(key) > 0 || visible_.count(key) > 0 || excluded_.count(key) > 0)
                            {
                                continue;
                            }
                        maybe_visible.emplace_back(system, prn);
                    }
            }
        std::sort(maybe_visible.begin(), maybe_visible.end(), [](const auto& a, const auto& b) { return a.get_PRN() < b.get_PRN(); });

        std::ostringstream oss;
        oss << "[visibility] recompute (triggered by:"
            << (fix_became_valid ? " fix_became_valid" : "")
            << (data_changed ? " data_changed" : "")
            << (interval_elapsed ? " interval_elapsed" : "")
            << (moved_significantly ? " moved_significantly" : "")
            << (expired ? " data_expired" : "")
            << "; " << (needs_full_recompute ? "full" : "targeted (" + std::to_string(changed_prns.size()) + " sat)")
            << "; fix " << (fix_valid ? "valid" : "not valid, using AGNSS_ref_location")
            << ", LLH " << LLH[0] << " deg, " << LLH[1] << " deg, " << LLH[2] << " m, GPS time " << gps_gtime.time
            << ", mask " << elevation_mask_deg_ << " deg)\n"
            << "VISIBLE (" << visible_entries.size() << "): ";
        append_with_el(oss, visible_entries);
        oss << "\nEXCLUDED (" << excluded_entries.size() << "): ";
        append_with_el(oss, excluded_entries);
        oss << "\nMAYBE VISIBLE (" << maybe_visible.size() << "): ";
        bool first = true;
        for (const auto& sat : maybe_visible)
            {
                if (!first)
                    {
                        oss << ", ";
                    }
                oss << short_id(sat);
                first = false;
            }
        LOG(INFO) << oss.str();
        std::cout << oss.str() << '\n';
    }

    return changed;
}


bool SatelliteVisibility::IsVisible(const Gnss_Satellite& sat) const
{
    return visible_.find(std::make_pair(sat.get_system(), sat.get_PRN())) != visible_.end();
}


bool SatelliteVisibility::IsExcluded(const Gnss_Satellite& sat) const
{
    return excluded_.find(std::make_pair(sat.get_system(), sat.get_PRN())) != excluded_.end();
}

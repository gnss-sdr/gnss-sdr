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
#include "Beidou_CNAV2.h"
#include "GLONASS_L1_L2_CA.h"    // for GLONASS_PRN
#include "agnss_ref_location.h"  // for parse_agnss_ref_location
#include "agnss_ref_time.h"      // for parse_agnss_ref_utc_time
#include "configuration_interface.h"
#include "geofunctions.h"
#include "monitor_pvt.h"
#include "pvt_interface.h"
#include "qzss.h"
#include "rtklib_conversions.h"  // for alm_to_rtklib, eph_to_rtklib
#include "rtklib_ephemeris.h"    // for alm2pos, eph2pos
#include "rtklib_rtkcmn.h"       // for utc2gpst, gpst2time
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
// Keeps (search_ratio_ + 1) clear of uint32_t overflow in
// GNSSFlowgraph::pop_by_visibility()'s counter modulo.
constexpr uint32_t kMaxSearchRatio = 1000000U;

// Tick() calls between DataChanged() runs, which copy every ephemeris/almanac map.
constexpr int kDataCheckEveryNTicks = 20;
}  // namespace


SatelliteVisibility::SatelliteVisibility(const std::shared_ptr<ConfigurationInterface>& configuration)
    : enabled_(configuration->property("GNSS-SDR.enable_visibility_aware_search", false)),
      search_ratio_(std::min(configuration->property("GNSS-SDR.visible_vs_mayvisible_search_ratio", 3U), kMaxSearchRatio)),
      // Satellites move negligibly over minutes; fast reaction comes from the
      // event-driven triggers (data arrival, movement, expiry), not polling.
      recompute_interval_s_(configuration->property("GNSS-SDR.visibility_recompute_interval_s", 120.0)),
      elevation_mask_deg_(configuration->property("GNSS-SDR.search_elevation_mask", 0.0)),
      position_threshold_m_(configuration->property("GNSS-SDR.visibility_recompute_position_threshold_m", 1000.0)),
      // Maximum absolute age of the resolved almanac epoch. Default: 3 days.
      almanac_max_age_s_(configuration->property("GNSS-SDR.visibility_almanac_max_age_s", 259200.0)),
      glonass_strict_health_(configuration->property("PVT.glonass_strict_health", true)),
      have_agnss_reference_(false),
      agnss_ref_lat_deg_(0.0),
      agnss_ref_lon_deg_(0.0),
      agnss_ref_utc_time_(0),
      last_recompute_rx_time_s_(-1.0e9),
      last_recompute_r_eb_e_(arma::vec{0.0, 0.0, 0.0}),
      next_expiry_deadline_rx_time_(std::numeric_limits<double>::infinity()),
      ticks_since_data_check_(kDataCheckEveryNTicks),
      last_fix_valid_(false),
      first_data_check_(true)
{
    // Parsing shared with ControlThread::init(); an empty AGNSS_ref_utc_time
    // means wall-clock time.
    const Agnss_Ref_Location parsed_location = parse_agnss_ref_location(
        configuration->property("GNSS-SDR.AGNSS_ref_location", std::string("")));
    have_agnss_reference_ = parsed_location.valid;
    agnss_ref_lat_deg_ = parsed_location.lat;
    agnss_ref_lon_deg_ = parsed_location.lon;

    const Agnss_Ref_Time parsed_time = parse_agnss_ref_utc_time(
        configuration->property("GNSS-SDR.AGNSS_ref_utc_time", std::string("")));
    // A non-empty malformed string also falls back to wall-clock time.
    agnss_ref_utc_time_ = parsed_time.valid ? static_cast<time_t>(parsed_time.seconds) : std::time(nullptr);

    // Same GNSS-SDR.<System>_banned_prns parsing as GNSSFlowgraph::set_signals_list().
    for (const std::string system : {"GPS", "Galileo", "Beidou", "Glonass", "QZSS"})
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
                            LOG(WARNING) << "Invalid argument at GNSS-SDR." << system << "_banned_prns configuration parameter: " << ia.what();
                        }
                    catch (const std::out_of_range& oor)
                        {
                            LOG(WARNING) << "Out of range at GNSS-SDR." << system << "_banned_prns configuration parameter: " << oor.what();
                        }
                }
        }

    // Signal codes per system, as in gnss_block_factory.cc's signal_mapping.
    static const std::map<std::string, std::vector<std::string>> system_signals{
        {"GPS", {"1C", "2S", "L5"}},
        {"Galileo", {"1B", "5X", "E6", "7X"}},
        {"Beidou", {"B1", "1D", "B3", "5D"}},
        {"Glonass", {"1G", "2G"}},
        {"QZSS", {"J1", "J5"}}};
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


std::vector<std::pair<int, Gnss_Satellite>> compute_visible_satellites(
    const std::shared_ptr<PvtInterface>& pvt_ptr,
    const gtime_t& gps_gtime,
    const arma::vec& r_eb_e,
    double elevation_mask_deg,
    std::vector<std::pair<int, Gnss_Satellite>>* below_mask_out,
    double almanac_max_age_s,
    double* seconds_until_next_expiry_out,
    const std::set<std::pair<std::string, uint32_t>>* only_prns,
    bool glonass_strict_health)
{
    std::vector<std::pair<int, Gnss_Satellite>> available_satellites;
    // Truncated week fields (GPS ephemeris WN mod 1024, GPS/Galileo almanac
    // WNa) are resolved against the query epoch's own week: always available
    // and near-correct, unlike a config-supplied reference week.
    int ref_gps_week = 0;
    time2gpst(gps_gtime, &ref_gps_week);
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
    // PRNs already classified by ephemeris (either way). Almanac loops skip
    // them, so a satellite never lands in both available_satellites and
    // *below_mask_out.
    std::vector<unsigned int> handled_gps;
    std::vector<unsigned int> handled_gal;
    std::vector<unsigned int> handled_bds;

    const std::map<int, Gps_Ephemeris> gps_eph_map = pvt_ptr->get_gps_ephemeris();
    for (const auto& it : gps_eph_map)
        {
            const std::string system = (it.second.PRN >= MINPRNQZS && it.second.PRN <= MAXPRNQZS) ? "QZSS" : "GPS";
            if (only_prns != nullptr && only_prns->count(std::make_pair(system, it.second.PRN)) == 0)
                {
                    continue;
                }
            const eph_t rtklib_eph = eph_to_rtklib(it.second, ref_gps_week);
            const double age = timediff(gps_gtime, rtklib_eph.toe);
            if (std::abs(age) > MAXDTOE)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE);
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
            // Unhealthy is treated like below-mask, not like "no data".
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    LOG(INFO) << "Using " << system << " Ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(system, it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using " << system << " Ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(system, it.second.PRN));
                }
        }

    const std::map<int, Gps_CNAV_Ephemeris> qzss_cnav_map = pvt_ptr->get_gps_cnav_ephemeris();
    for (const auto& it : qzss_cnav_map)
        {
            if (it.second.PRN < MINPRNQZS || it.second.PRN > MAXPRNQZS ||
                std::find(handled_gps.begin(), handled_gps.end(), it.second.PRN) != handled_gps.end())
                {
                    continue;
                }
            const std::string system("QZSS");
            if (only_prns != nullptr && only_prns->count(std::make_pair(system, it.second.PRN)) == 0)
                {
                    continue;
                }
            const eph_t rtklib_eph = eph_to_rtklib(it.second);
            const double age = timediff(gps_gtime, rtklib_eph.toe);
            if (std::abs(age) > MAXDTOE_QZS)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE_QZS);
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
            // Unhealthy is treated like below-mask, not like "no data".
            if (El > elevation_mask_deg && it.second.signal_health == 0)
                {
                    LOG(INFO) << "Using " << system << " CNAV ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(system, it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using " << system << " CNAV ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(system, it.second.PRN));
                }
        }

    const std::map<int, Galileo_Ephemeris> gal_eph_map = pvt_ptr->get_galileo_ephemeris();
    for (const auto& it : gal_eph_map)
        {
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Galileo"), it.second.PRN)) == 0)
                {
                    continue;
                }
            const eph_t rtklib_eph = eph_to_rtklib(it.second);
            const double age = timediff(gps_gtime, rtklib_eph.toe);
            if (std::abs(age) > MAXDTOE_GAL)
                {
                    continue;  // stale -- treat as if no ephemeris exists; almanac may still classify it
                }
            note_freshness(age, MAXDTOE_GAL);
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
            // E1B_HS gates the whole satellite: 1B initiates the search and
            // 5X is only searched once 1B is tracked.
            if (El > elevation_mask_deg && it.second.E1B_HS == 0)
                {
                    LOG(INFO) << "Using Galileo Ephemeris: Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Galileo"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using Galileo Ephemeris (excluded): Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Galileo"), it.second.PRN));
                }
        }

    // Classify each BeiDou satellite once, from the usable orbit closest to
    // the query epoch. Equal-age records retain DNAV, then CNAV1, then CNAV2
    // priority, independent of telemetry arrival order. Almanac remains the
    // fallback when no fresh ephemeris is available.
    std::map<uint32_t, eph_t> bds_eph_map;
    const auto add_bds_ephemeris = [&](uint32_t prn, const eph_t& ephemeris) {
        if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Beidou"), prn)) == 0)
            {
                return;
            }
        const double age = timediff(gps_gtime, ephemeris.toe);
        if (!std::isfinite(age) || std::abs(age) > MAXDTOE_BDS ||
            !std::isfinite(ephemeris.A) || ephemeris.A <= 0.0 ||
            !std::isfinite(ephemeris.e) || ephemeris.e < 0.0 || ephemeris.e >= 1.0)
            {
                return;
            }
        const auto previous = bds_eph_map.find(prn);
        if (previous == bds_eph_map.cend() ||
            std::abs(age) < std::abs(timediff(gps_gtime, previous->second.toe)))
            {
                bds_eph_map[prn] = ephemeris;
            }
    };
    for (const auto& entry : pvt_ptr->get_beidou_dnav_ephemeris())
        {
            add_bds_ephemeris(entry.second.PRN, eph_to_rtklib(entry.second));
        }
    const std::array<std::map<int, Beidou_Cnav1_Ephemeris>, 2> bds_cnav_maps{
        {pvt_ptr->get_beidou_cnav1_ephemeris(), pvt_ptr->get_beidou_cnav2_ephemeris()}};
    for (size_t source = 0; source < bds_cnav_maps.size(); ++source)
        {
            const int expected_source = source == 0 ? BDS_EPH_SOURCE_CNAV1 : BDS_EPH_SOURCE_CNAV2;
            for (const auto& entry : bds_cnav_maps[source])
                {
                    const auto& ephemeris = entry.second;
                    if (ephemeris.sig_type == expected_source &&
                        (ephemeris.sat_type == 2 || ephemeris.sat_type == 3))
                        {
                            add_bds_ephemeris(ephemeris.PRN, eph_to_rtklib(ephemeris));
                        }
                }
        }
    for (const auto& it : bds_eph_map)
        {
            const eph_t& rtklib_eph = it.second;
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            double sat_pos_variance_m2;
            eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s,
                &sat_pos_variance_m2);
            if (!std::all_of(r_sat.cbegin(), r_sat.cend(), [](double value) { return std::isfinite(value); }))
                {
                    continue;
                }
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            if (!std::isfinite(El))
                {
                    continue;
                }
            note_freshness(timediff(gps_gtime, rtklib_eph.toe), MAXDTOE_BDS);
            handled_bds.push_back(it.first);
            if (El > elevation_mask_deg && rtklib_eph.svh == 0)
                {
                    LOG(INFO) << "Using BeiDou Ephemeris (source " << rtklib_eph.code << "): Sat " << it.first << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El), Gnss_Satellite(std::string("Beidou"), it.first));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using BeiDou Ephemeris (source " << rtklib_eph.code << ", excluded): Sat " << it.first << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Beidou"), it.first));
                }
        }

    const auto glonass_utc = pvt_ptr->get_glonass_utc_model();
    std::set<uint32_t> handled_glonass;
    for (const auto& it : pvt_ptr->get_glonass_ephemeris())
        {
            const auto& ephemeris = it.second;
            if (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Glonass"), ephemeris.PRN)) == 0)
                {
                    continue;
                }
            if (ephemeris.d_N_T < 1.0 || ephemeris.d_N_T > 1461.0 || ephemeris.d_yr < 1996.0)
                {
                    continue;
                }
            const auto geph = eph_to_rtklib(ephemeris, glonass_utc, glonass_strict_health);
            const double age = timediff(gps_gtime, geph.toe);
            if (std::abs(age) > MAXDTOE_GLO)
                {
                    continue;
                }
            std::array<double, 3> position{};
            double clock_bias;
            double variance;
            geph2pos(gps_gtime, &geph, position.data(), &clock_bias, &variance);
            if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]) ||
                std::hypot(std::hypot(position[0], position[1]), position[2]) < 6378136.0)
                {
                    continue;
                }
            note_freshness(age, MAXDTOE_GLO);
            handled_glonass.insert(ephemeris.PRN);
            double azimuth;
            double elevation;
            double distance;
            topocent(&azimuth, &elevation, &distance, r_eb_e, arma::vec{position[0], position[1], position[2]} - r_eb_e);
            const auto entry = std::make_pair(static_cast<int>(std::floor(elevation)), Gnss_Satellite("Glonass", ephemeris.PRN));
            if (elevation > elevation_mask_deg && geph.svh == 0)
                {
                    available_satellites.push_back(entry);
                }
            else if (below_mask_out != nullptr)
                {
                    below_mask_out->push_back(entry);
                }
        }
    for (const auto& it : pvt_ptr->get_glonass_almanac())
        {
            const auto& almanac = it.second;
            if (handled_glonass.count(almanac.PRN) != 0 ||
                (only_prns != nullptr && only_prns->count(std::make_pair(std::string("Glonass"), almanac.PRN)) == 0))
                {
                    continue;
                }
            const auto epoch = glonass_almanac_epoch(almanac);
            const double age = timediff(gps_gtime, epoch);
            if (epoch.time == 0 || std::abs(age) > almanac_max_age_s)
                {
                    continue;
                }
            std::array<double, 3> position{};
            if (!almanac.satellite_position(timediff(gpst2utc(gps_gtime), gpst2utc(epoch)), position))
                {
                    continue;
                }
            note_freshness(age, almanac_max_age_s);
            double azimuth;
            double elevation;
            double distance;
            topocent(&azimuth, &elevation, &distance, r_eb_e, arma::vec{position[0], position[1], position[2]} - r_eb_e);
            const auto entry = std::make_pair(static_cast<int>(std::floor(elevation)), Gnss_Satellite("Glonass", almanac.PRN));
            // ICD Table 5.1: Cn=1 is operational; ln=1 indicates malfunction.
            if (elevation > elevation_mask_deg && almanac.d_C_n && !almanac.d_l_n)
                {
                    available_satellites.push_back(entry);
                }
            else if (below_mask_out != nullptr)
                {
                    below_mask_out->push_back(entry);
                }
        }

    const std::map<int, Gps_Almanac> gps_alm_map = pvt_ptr->get_gps_almanac();
    for (const auto& it : gps_alm_map)
        {
            const std::string system = (it.second.PRN >= MINPRNQZS && it.second.PRN <= MAXPRNQZS) ? "QZSS" : "GPS";
            if (only_prns != nullptr && only_prns->count(std::make_pair(system, it.second.PRN)) == 0)
                {
                    continue;
                }
            if (std::find(handled_gps.begin(), handled_gps.end(), it.second.PRN) != handled_gps.end())
                {
                    continue;  // ephemeris already classified this PRN
                }
            const alm_t rtklib_alm = alm_to_rtklib(it.second, ref_gps_week);
            const double age = timediff(gps_gtime, rtklib_alm.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            // gps_gtime and rtklib_alm.toa are both absolute epochs; a
            // seconds-of-week query time here would corrupt tk in alm2pos().
            alm2pos(gps_gtime, &rtklib_alm, r_sat.data(), &clock_bias_s);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            if (El > elevation_mask_deg && it.second.SV_health == 0)
                {
                    LOG(INFO) << "Using " << system << " Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(system, it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using " << system << " Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(system, it.second.PRN));
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
            const alm_t rtklib_alm = alm_to_rtklib(it.second, ref_gps_week);
            const double age = timediff(gps_gtime, rtklib_alm.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
            std::array<double, 3> r_sat{};
            double clock_bias_s;
            alm2pos(gps_gtime, &rtklib_alm, r_sat.data(), &clock_bias_s);
            double Az;
            double El;
            double dist_m;
            const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
            const arma::vec dx = r_sat_eb_e - r_eb_e;
            topocent(&Az, &El, &dist_m, r_eb_e, dx);
            // Sporadic single-tick extreme elevations have been seen from a
            // Galileo almanac while every other satellite in the same tick
            // was normal. Dump every alm2pos() input to diagnose the next one.
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
                    LOG(INFO) << "Using Galileo Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Galileo"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using Galileo Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
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
            const alm_t rtklib_alm = alm_to_rtklib(it.second);
            const double age = timediff(gps_gtime, rtklib_alm.toa);
            if (std::abs(age) > almanac_max_age_s)
                {
                    continue;  // stale -- treat as if no almanac exists for this PRN either
                }
            note_freshness(age, almanac_max_age_s);
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
                    LOG(INFO) << "Using BeiDou Almanac:  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    available_satellites.emplace_back(floor(El),
                        (Gnss_Satellite(std::string("Beidou"), it.second.PRN)));
                }
            else if (below_mask_out != nullptr)
                {
                    LOG(INFO) << "Using BeiDou Almanac (excluded):  Sat " << it.second.PRN << " Az: " << Az << " El: " << El;
                    below_mask_out->emplace_back(floor(El), Gnss_Satellite(std::string("Beidou"), it.second.PRN));
                }
        }

    std::sort(available_satellites.begin(), available_satellites.end(), [](const std::pair<int, Gnss_Satellite>& a, const std::pair<int, Gnss_Satellite>& b) {
        return a.first < b.first;
    });
    std::reverse(available_satellites.begin(), available_satellites.end());
    return available_satellites;
}


bool SatelliteVisibility::DataChanged(const std::shared_ptr<PvtInterface>& pvt_ptr,
    std::set<std::pair<std::string, uint32_t>>* changed_prns_out)
{
    // Fingerprint on (toe/toa, health) so a replaced entry counts as a
    // change, not only a new PRN.
    std::map<std::tuple<std::string, std::string, uint32_t>, NavigationFingerprint> current;
    auto add = [&current](const char* system, const char* type, uint32_t prn, double ref_time, int32_t health,
                   uint32_t iode = 0, uint32_t iodc = 0, int32_t sat_type = 0, int32_t signal_type = 0) {
        current.emplace(std::make_tuple(std::string(system), std::string(type), prn),
            std::make_tuple(ref_time, health, iode, iodc, sat_type, signal_type));
    };
    for (const auto& it : pvt_ptr->get_gps_ephemeris())
        {
            add(it.second.PRN >= MINPRNQZS && it.second.PRN <= MAXPRNQZS ? "QZSS" : "GPS", "EPH", it.second.PRN, it.second.WN * 604800.0 + it.second.toe, it.second.SV_health);
        }
    for (const auto& it : pvt_ptr->get_galileo_ephemeris())
        {
            add("Galileo", "EPH", it.second.PRN, it.second.WN * 604800.0 + it.second.toe, it.second.E1B_HS);
        }
    for (const auto& it : pvt_ptr->get_beidou_dnav_ephemeris())
        {
            add("Beidou", "EPH", it.second.PRN, it.second.WN * 604800.0 + it.second.toe, it.second.SV_health);
        }
    const std::array<std::map<int, Beidou_Cnav1_Ephemeris>, 2> bds_cnav_maps{
        {pvt_ptr->get_beidou_cnav1_ephemeris(), pvt_ptr->get_beidou_cnav2_ephemeris()}};
    for (size_t source = 0; source < bds_cnav_maps.size(); ++source)
        {
            for (const auto& it : bds_cnav_maps[source])
                {
                    const auto& ephemeris = it.second;
                    add("Beidou", source == 0 ? "CNAV1" : "CNAV2", ephemeris.PRN,
                        ephemeris.WN * 604800.0 + ephemeris.toe, ephemeris.hs,
                        ephemeris.IODE, ephemeris.IODC, ephemeris.sat_type, ephemeris.sig_type);
                }
        }
    for (const auto& it : pvt_ptr->get_gps_almanac())
        {
            add(it.second.PRN >= MINPRNQZS && it.second.PRN <= MAXPRNQZS ? "QZSS" : "GPS", "ALM", it.second.PRN, it.second.WNa * 604800.0 + it.second.toa, it.second.SV_health);
        }
    for (const auto& it : pvt_ptr->get_galileo_almanac())
        {
            add("Galileo", "ALM", it.second.PRN, it.second.WNa * 604800.0 + it.second.toa, it.second.E1B_HS);
        }
    for (const auto& it : pvt_ptr->get_beidou_dnav_almanac())
        {
            add("Beidou", "ALM", it.second.PRN, it.second.WNa * 604800.0 + it.second.toa, it.second.SV_health);
        }

    for (const auto& it : pvt_ptr->get_gps_cnav_ephemeris())
        {
            if (it.second.PRN >= MINPRNQZS && it.second.PRN <= MAXPRNQZS)
                {
                    add("QZSS", "CNAV", it.second.PRN, it.second.WN * 604800.0 + it.second.toe1, it.second.signal_health);
                }
        }
    for (const auto& it : pvt_ptr->get_glonass_ephemeris())
        {
            add("Glonass", "EPH", it.second.PRN, (it.second.d_yr * 1461.0 + it.second.d_N_T) * 86400.0 + it.second.d_t_b,
                (static_cast<int32_t>(it.second.d_B_n) & 4) | static_cast<int32_t>(it.second.d_l3rd_n));
        }
    for (const auto& it : pvt_ptr->get_glonass_almanac())
        {
            add("Glonass", "ALM", it.second.PRN, (it.second.d_N_4 * 1461.0 + it.second.d_N_A) * 86400.0 + it.second.d_t_lambda_n_A,
                (it.second.d_C_n ? 0 : 2) | static_cast<int32_t>(it.second.d_l_n));
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
                            // A removed PRN has nothing to recompute from, but
                            // Tick() must still drop it from visible_/excluded_.
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


void SatelliteVisibility::SetCommandReference(time_t utc_time, const std::array<float, 3>& LLH,
    const Monitor_Pvt& current_fix, double receiver_time_s)
{
    command_reference_utc_time_ = utc_time;
    command_reference_llh_ = LLH;
    command_reference_receiver_time_s_ = receiver_time_s;
    command_previous_fix_time_s_ = -1.0;
    if (current_fix.RX_time >= 0.0)
        {
            const auto epoch = gpst2time(static_cast<int>(current_fix.week), current_fix.RX_time);
            command_previous_fix_time_s_ = static_cast<double>(epoch.time) + epoch.sec;
        }
    have_command_reference_ = true;
    command_reference_changed_ = true;
    ticks_since_data_check_ = kDataCheckEveryNTicks;
}


bool SatelliteVisibility::Tick(const std::shared_ptr<PvtInterface>& pvt_ptr, const Monitor_Pvt& fix_status,
    double receiver_time_s)
{
    if (!enabled_ || !pvt_ptr)
        {
            return false;
        }

    bool fix_valid = (fix_status.RX_time >= 0.0);
    if (have_command_reference_ && fix_valid)
        {
            const auto epoch = gpst2time(static_cast<int>(fix_status.week), fix_status.RX_time);
            // The status receiver keeps publishing the pre-command fix during
            // an outage. Only a new PVT epoch may replace the supplied reference.
            have_command_reference_ = (static_cast<double>(epoch.time) + epoch.sec == command_previous_fix_time_s_);
        }
    fix_valid = fix_valid && !have_command_reference_;
    const bool fix_became_valid = fix_valid && !last_fix_valid_;
    last_fix_valid_ = fix_valid;

    std::array<float, 3> LLH{};
    gtime_t gps_gtime{};
    if (have_command_reference_)
        {
            LLH = command_reference_llh_;
            gtime_t utc_gtime{};
            utc_gtime.time = command_reference_utc_time_;
            gps_gtime = timeadd(utc2gpst(utc_gtime), std::max(0.0, receiver_time_s - command_reference_receiver_time_s_));
        }
    else if (fix_valid)
        {
            LLH[0] = static_cast<float>(fix_status.latitude);
            LLH[1] = static_cast<float>(fix_status.longitude);
            LLH[2] = static_cast<float>(fix_status.height);
            gps_gtime = gpst2time(static_cast<int>(fix_status.week), fix_status.RX_time);
            const double fix_time_s = static_cast<double>(gps_gtime.time) + gps_gtime.sec;
            if (fix_became_valid || fix_time_s != last_fix_time_s_)
                {
                    last_fix_time_s_ = fix_time_s;
                    last_fix_receiver_time_s_ = receiver_time_s;
                }
            // The status receiver retains the last successful fix during an
            // outage. Keep its position, but advance its epoch with samples so
            // rising satellites and expired navigation data can be reconsidered.
            gps_gtime = timeadd(gps_gtime, std::max(0.0, receiver_time_s - last_fix_receiver_time_s_));
        }
    else if (have_agnss_reference_)
        {
            LLH[0] = static_cast<float>(agnss_ref_lat_deg_);
            LLH[1] = static_cast<float>(agnss_ref_lon_deg_);
            LLH[2] = 0.0F;
            gtime_t utc_gtime{};
            // Configured AGNSS_ref_utc_time (wall clock if unset): in replay
            // runs the host clock is unrelated to the samples' GNSS time.
            utc_gtime.time = agnss_ref_utc_time_;
            utc_gtime.sec = 0.0;
            gps_gtime = timeadd(utc2gpst(utc_gtime), receiver_time_s);
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

    // DataChanged() copies every ephemeris/almanac map; pay that only every
    // few ticks.
    bool data_changed = false;
    std::set<std::pair<std::string, uint32_t>> changed_prns;
    if (++ticks_since_data_check_ >= kDataCheckEveryNTicks)
        {
            ticks_since_data_check_ = 0;
            data_changed = DataChanged(pvt_ptr, &changed_prns);
        }

    // Include the GPS week so cadence and expiry survive the TOW rollover.
    const double rx_time_s = static_cast<double>(gps_gtime.time) + gps_gtime.sec;
    const bool interval_elapsed = (rx_time_s - last_recompute_rx_time_s_) >= recompute_interval_s_;

    const bool moved_significantly = arma::norm(r_eb_e - last_recompute_r_eb_e_, 2) >= position_threshold_m_;

    const bool expired = rx_time_s >= next_expiry_deadline_rx_time_;

    const bool reference_changed = command_reference_changed_;
    command_reference_changed_ = false;
    if (!reference_changed && !fix_became_valid && !data_changed && !interval_elapsed && !moved_significantly && !expired)
        {
            return false;
        }

    // Position/time/expiry triggers can change every satellite's elevation,
    // so they need a full sweep. data_changed alone only affects the PRNs in
    // changed_prns, so the recompute is scoped to them via only_prns.
    const bool needs_full_recompute = reference_changed || fix_became_valid || interval_elapsed || moved_significantly || expired;

    // Advance the full-sweep baselines only after a full sweep: resetting
    // them on targeted recomputes would starve the interval/displacement
    // triggers whenever data arrives faster than either threshold.
    if (needs_full_recompute)
        {
            last_recompute_rx_time_s_ = rx_time_s;
            last_recompute_r_eb_e_ = r_eb_e;
        }

    std::vector<std::pair<int, Gnss_Satellite>> below_mask;
    double seconds_until_next_expiry = std::numeric_limits<double>::infinity();
    const auto elevations = compute_visible_satellites(pvt_ptr, gps_gtime, r_eb_e, elevation_mask_deg_,
        &below_mask, almanac_max_age_s_,
        needs_full_recompute ? &seconds_until_next_expiry : nullptr,
        needs_full_recompute ? nullptr : &changed_prns, glonass_strict_health_);
    // Only a full sweep sees every classified satellite's freshness; a
    // targeted sweep could push the deadline past the true soonest expiry.
    if (needs_full_recompute)
        {
            next_expiry_deadline_rx_time_ = std::isfinite(seconds_until_next_expiry)
                                                ? rx_time_s + seconds_until_next_expiry
                                                : std::numeric_limits<double>::infinity();
        }

    // Full recompute: classify from scratch. Targeted: keep the current
    // membership, erase only changed_prns (their data may be gone) and
    // re-add whatever was recomputed.
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
    // The computation already applied the mask and health check using the
    // full-precision elevation. Floored elevations are only for diagnostics.
    for (const auto& entry : elevations)
        {
            const auto key = std::make_pair(entry.second.get_system(), entry.second.get_PRN());
            reference_elevation_deg_[key] = entry.first;
            if (banned_.count(key) == 0)
                {
                    new_visible.emplace(key);
                }
        }
    for (const auto& entry : below_mask)
        {
            const auto key = std::make_pair(entry.second.get_system(), entry.second.get_PRN());
            reference_elevation_deg_[key] = entry.first;
            new_excluded.emplace(key);
        }

    const bool changed = (new_visible != visible_) || (new_excluded != excluded_);
    visible_ = std::move(new_visible);
    excluded_ = std::move(new_excluded);

    // Always-on diagnostic (LOG, not DLOG, which is compiled out in release
    // builds): VISIBLE/EXCLUDED with floored elevation, MAYBE VISIBLE by PRN
    // only. GNSSFlowgraph::pop_by_visibility() logs what is actually searched.
    {
        auto short_id = [](const Gnss_Satellite& sat) {
            std::ostringstream os;
            os << sat.get_system_short() << std::setfill('0') << std::setw(2) << sat.get_PRN();
            return os.str();
        };
        auto sort_by_prn = [](std::vector<std::pair<int, Gnss_Satellite>> entries) {
            std::sort(entries.begin(), entries.end(), [](const std::pair<int, Gnss_Satellite>& a, const std::pair<int, Gnss_Satellite>& b) {
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
        // Built from the post-merge sets so a targeted recompute reports every
        // satellite; reference_elevation_deg_ keeps the last value of PRNs not
        // recomputed this tick.
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
        // Maybe-visible = full PRN range of each configured system minus
        // banned/visible/excluded. Scoped to configured_systems_ so an
        // unconfigured system's range is not listed as permanent noise.
        std::vector<Gnss_Satellite> maybe_visible;
        for (const auto& system_range : {std::make_tuple(std::string("GPS"), static_cast<uint32_t>(MINPRNGPS), static_cast<uint32_t>(MAXPRNGPS)),
                 std::make_tuple(std::string("Galileo"), static_cast<uint32_t>(MINPRNGAL), static_cast<uint32_t>(MAXPRNGAL)),
                 std::make_tuple(std::string("Beidou"), static_cast<uint32_t>(MINPRNBDS), static_cast<uint32_t>(MAXPRNBDS)),
                 std::make_tuple(std::string("Glonass"), static_cast<uint32_t>(MINPRNGLO), static_cast<uint32_t>(MAXPRNGLO)),
                 std::make_tuple(std::string("QZSS"), static_cast<uint32_t>(MINPRNQZS), static_cast<uint32_t>(QZSS_L5_MAX_PRN))})
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
        std::sort(maybe_visible.begin(), maybe_visible.end(), [](const Gnss_Satellite& a, const Gnss_Satellite& b) { return a.get_PRN() < b.get_PRN(); });

        std::ostringstream oss;
        oss << "[visibility] recompute (triggered by:"
            << (reference_changed ? " command_reference_changed" : "")
            << (fix_became_valid ? " fix_became_valid" : "")
            << (data_changed ? " data_changed" : "")
            << (interval_elapsed ? " interval_elapsed" : "")
            << (moved_significantly ? " moved_significantly" : "")
            << (expired ? " data_expired" : "")
            << "; " << (needs_full_recompute ? "full" : "targeted (" + std::to_string(changed_prns.size()) + " sat)")
            << "; fix " << (have_command_reference_ ? "using telecommand reference" : (fix_valid ? "valid" : "not valid, using AGNSS_ref_location"))
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
    }

    return changed;
}


bool SatelliteVisibility::IsVisible(const Gnss_Satellite& sat) const
{
    return visible_.find(std::make_pair(sat.get_system(), sat.get_system() == "QZSS" ? qzss_l1cb_prn_to_nominal_prn(sat.get_PRN()) : sat.get_PRN())) != visible_.end();
}


bool SatelliteVisibility::IsExcluded(const Gnss_Satellite& sat) const
{
    return excluded_.find(std::make_pair(sat.get_system(), sat.get_system() == "QZSS" ? qzss_l1cb_prn_to_nominal_prn(sat.get_PRN()) : sat.get_PRN())) != excluded_.end();
}


SatelliteVisibility::SearchVisibility SatelliteVisibility::GetSearchVisibility(const Gnss_Satellite& sat) const
{
    if (sat.get_system() != "Glonass")
        {
            return IsVisible(sat) ? SearchVisibility::Visible : (IsExcluded(sat) ? SearchVisibility::Excluded : SearchVisibility::Unknown);
        }

    const auto frequency = GLONASS_PRN.find(sat.get_PRN());
    if (sat.get_PRN() == 0 || frequency == GLONASS_PRN.cend())
        {
            return SearchVisibility::Unknown;
        }

    // The pool keeps one representative per FDMA frequency, but acquisition
    // can lock onto any slot using it. A missing partner's navigation data must
    // therefore keep the frequency searchable even if the representative is
    // below the mask or unhealthy.
    bool all_excluded = true;
    for (const auto& slot : GLONASS_PRN)
        {
            if (slot.first == 0 || slot.second != frequency->second)
                {
                    continue;
                }
            const auto key = std::make_pair(std::string("Glonass"), slot.first);
            if (visible_.count(key) != 0)
                {
                    return SearchVisibility::Visible;
                }
            all_excluded = all_excluded && excluded_.count(key) != 0;
        }
    return all_excluded ? SearchVisibility::Excluded : SearchVisibility::Unknown;
}


bool SatelliteVisibility::IsSearchVisible(const Gnss_Satellite& sat) const
{
    return GetSearchVisibility(sat) == SearchVisibility::Visible;
}


bool SatelliteVisibility::IsSearchExcluded(const Gnss_Satellite& sat) const
{
    return GetSearchVisibility(sat) == SearchVisibility::Excluded;
}

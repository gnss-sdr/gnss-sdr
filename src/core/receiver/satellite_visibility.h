/*!
 * \file satellite_visibility.h
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

#ifndef GNSS_SDR_SATELLITE_VISIBILITY_H
#define GNSS_SDR_SATELLITE_VISIBILITY_H

#include "gnss_satellite.h"
#include "rtklib.h"  // for gtime_t
#include <armadillo>
#include <cstdint>
#include <ctime>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

class ConfigurationInterface;
class Monitor_Pvt;
class PvtInterface;

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver
 * \{ */

/*!
 * \brief Computes the elevation of every satellite with usable ephemeris or
 * almanac data in pvt_ptr (GPS, Galileo, BeiDou, GLONASS, QZSS) and returns
 * those strictly above elevation_mask_deg that broadcast a healthy status.
 * Ephemeris is preferred over almanac, so each satellite is classified by
 * exactly one data source. An unhealthy satellite (SV_health != 0; Galileo
 * E1B_HS != 0, since 1B initiates the search) is reported as below-mask, not
 * as "no data": it is known unusable whatever its geometry.
 *
 * Shared elevation core of ControlThread::get_visible_sats() and
 * SatelliteVisibility::Tick().
 *
 * \param gps_gtime query epoch (GPST). Its week also resolves the truncated
 * week fields (GPS ephemeris WN mod 1024, GPS/Galileo almanac WNa), so no
 * config-supplied reference week is needed.
 * \param r_eb_e receiver ECEF position (m).
 * \param elevation_mask_deg GNSS-SDR.search_elevation_mask (deg); distinct
 * from PVT.elevation_mask, which gates the solution, not the search order.
 * \param below_mask_out when non-null, receives every satellite whose
 * elevation was computable but at/below the mask (or unhealthy). Never
 * overlaps the returned list.
 * \param almanac_max_age_s max |gps_gtime - toa| (s) before an almanac entry
 * is ignored as stale. Ephemeris staleness is not configurable: RTKLIB's
 * per-system MAXDTOE* constants apply. Infinity disables the check.
 * \param seconds_until_next_expiry_out when non-null, set to the seconds
 * until the soonest classified entry crosses its staleness threshold;
 * +infinity if nothing was classified.
 * \param only_prns when non-null, restricts the computation to these
 * (system, PRN) pairs; valid only when nothing that affects the other
 * satellites (position, time) has changed.
 * \param glonass_strict_health if true, the Bn MSB also marks a GLONASS
 * ephemeris unhealthy (the ln flag always does).
 * \return (floor(El) in deg, satellite) pairs, sorted by descending elevation.
 */
std::vector<std::pair<int, Gnss_Satellite>> compute_visible_satellites(
    const std::shared_ptr<PvtInterface>& pvt_ptr,
    const gtime_t& gps_gtime,
    const arma::vec& r_eb_e,
    double elevation_mask_deg,
    std::vector<std::pair<int, Gnss_Satellite>>* below_mask_out = nullptr,
    double almanac_max_age_s = std::numeric_limits<double>::infinity(),
    double* seconds_until_next_expiry_out = nullptr,
    const std::set<std::pair<std::string, uint32_t>>* only_prns = nullptr,
    bool glonass_strict_health = true);


/*!
 * \brief Runtime classification of the searchable satellites, refreshed from
 * the control-thread tick, into two sets:
 *
 *   - visible_: elevation computable and above GNSS-SDR.search_elevation_mask,
 *     healthy, and not in GNSS-SDR.<System>_banned_prns. Searched first.
 *   - excluded_: elevation computable but at/below the mask (or unhealthy).
 *     Not searched while this holds, which is what lets a channel go idle
 *     once nothing plausible remains.
 *
 * Everything else is "maybe visible" (no usable data yet), searched at lower
 * priority per GNSS-SDR.visible_vs_mayvisible_search_ratio. The two sets are
 * disjoint by construction (one data source per satellite). A satellite
 * already tracked on another signal is exempt from exclusion in
 * GNSSFlowgraph::pop_by_visibility(): almanac and ephemeris can disagree at
 * the mask boundary, and tracking is stronger evidence than either.
 *
 * Tick() recomputes when: the fix just became valid; any PRN's ephemeris/
 * almanac fingerprint (toe/toa, health) changed; visibility_recompute_interval_s
 * of receiver time elapsed; the receiver moved more than
 * visibility_recompute_position_threshold_m; or the freshest classified data
 * reached its staleness threshold (RTKLIB MAXDTOE* for ephemeris,
 * visibility_almanac_max_age_s for almanac). Expiry is enforced inside
 * compute_visible_satellites(), so an expired PRN reverts to maybe-visible.
 *
 * Inert unless GNSS-SDR.enable_visibility_aware_search=true (default false):
 * Tick() is a no-op and IsVisible()/IsExcluded() return false, so every
 * satellite is treated as maybe-visible.
 */
class SatelliteVisibility
{
public:
    explicit SatelliteVisibility(const std::shared_ptr<ConfigurationInterface>& configuration);

    bool enabled() const { return enabled_; }
    uint32_t search_ratio() const { return search_ratio_; }

    /*!
     * \brief Re-evaluates visibility when one of the triggers listed in the
     * class description fires. Uses the latest fix position, else
     * GNSS-SDR.AGNSS_ref_location/AGNSS_ref_utc_time if configured, else
     * does nothing. Elapsed sample time advances the epoch between fixes
     * and before the first fix, including during recorded-data playback.
     *
     * \param receiver_time_s elapsed sample time, independent of valid fixes.
     * \returns true iff visible_/excluded_ membership changed, i.e. the
     * caller must rebuild its per-signal search queues.
     */
    bool Tick(const std::shared_ptr<PvtInterface>& pvt_ptr, const Monitor_Pvt& fix_status,
        double receiver_time_s = 0.0);

    bool IsVisible(const Gnss_Satellite& sat) const;
    // Elevation computable and at/below the mask (or unhealthy). Not the
    // negation of IsVisible(): a satellite with no data is in neither set.
    bool IsExcluded(const Gnss_Satellite& sat) const;

    // GLONASS acquisition searches an FDMA frequency shared by orbital slots:
    // visible if any slot is visible, excluded only if every slot is excluded.
    // Other constellations retain their per-satellite classification.
    bool IsSearchVisible(const Gnss_Satellite& sat) const;
    bool IsSearchExcluded(const Gnss_Satellite& sat) const;

private:
    enum class SearchVisibility
    {
        Unknown,
        Visible,
        Excluded
    };

    SearchVisibility GetSearchVisibility(const Gnss_Satellite& sat) const;

    // changed_prns_out, when non-null, receives every (system, PRN) added,
    // updated, or removed since the last check, so Tick() can recompute only
    // those. Left untouched on the first call, which forces a full recompute
    // anyway.
    bool DataChanged(const std::shared_ptr<PvtInterface>& pvt_ptr,
        std::set<std::pair<std::string, uint32_t>>* changed_prns_out = nullptr);

    bool enabled_;
    uint32_t search_ratio_;
    double recompute_interval_s_;
    double elevation_mask_deg_;

    // GNSS-SDR.visibility_recompute_position_threshold_m: ECEF displacement
    // (m) since the last full recompute that forces a new one, independently
    // of recompute_interval_s_.
    double position_threshold_m_;

    // GNSS-SDR.visibility_almanac_max_age_s (s). Ephemeris staleness is not
    // configurable: RTKLIB's per-system MAXDTOE* constants apply.
    double almanac_max_age_s_;

    bool glonass_strict_health_;
    bool have_agnss_reference_;
    double agnss_ref_lat_deg_;
    double agnss_ref_lon_deg_;

    // GNSS-SDR.AGNSS_ref_utc_time, falling back to wall-clock time if unset.
    // Initial epoch for the no-fix fallback; in replay runs the wall clock is
    // unrelated to the GNSS time in the samples, so configure it explicitly.
    time_t agnss_ref_utc_time_;

    // GNSS-SDR.<System>_banned_prns, parsed as in GNSSFlowgraph::set_signals_list().
    // A banned PRN is never classified visible, keeping the diagnostics
    // consistent with its removal from the search pool.
    std::set<std::pair<std::string, uint32_t>> banned_;

    // Systems with at least one Channels_<signal>.count > 0. Scopes the
    // maybe-visible report in Tick() so an unconfigured system's PRN range
    // is not listed as permanently unclassified.
    std::set<std::string> configured_systems_;

    std::set<std::pair<std::string, uint32_t>> visible_;
    std::set<std::pair<std::string, uint32_t>> excluded_;  // elevation computable, at/below the mask

    // Floored elevations for diagnostics, including unchanged PRNs during
    // targeted recomputes. Classification uses the computation's full-precision
    // mask and health decisions, never these rounded values.
    std::map<std::pair<std::string, uint32_t>, double> reference_elevation_deg_;

    // Anchor the last successful fix to the elapsed sample clock. Repeated
    // status snapshots must not reset this anchor during a solution outage.
    double last_fix_time_s_{-1.0};
    double last_fix_receiver_time_s_{0.0};

    // (system, "EPH"/"ALM"/"CNAV", PRN) -> (toe/toa as absolute seconds,
    // health). Catches a PRN's data being replaced, not only new PRNs.
    // first_data_check_ tells "never checked" from "checked, still empty".
    std::map<std::tuple<std::string, std::string, uint32_t>, std::pair<double, int32_t>> last_data_fingerprints_;

    // Absolute GPST seconds (gtime_t.time + sec) of the last full sweep:
    // monotonic across week rollover, and receiver time keeps the interval
    // independent of replay speed.
    double last_recompute_rx_time_s_;

    // ECEF position (m) of the last full sweep. Seeded at the origin so the
    // first displacement check trivially exceeds position_threshold_m_.
    arma::vec last_recompute_r_eb_e_;

    // Receiver time (same scale as last_recompute_rx_time_s_) at which the
    // freshest classified data goes stale. +infinity until a full sweep with
    // data has run; only full sweeps update it.
    double next_expiry_deadline_rx_time_;

    // Runs DataChanged() (which copies every ephemeris/almanac map) only
    // every kDataCheckEveryNTicks calls; a call count rather than a time
    // throttle so it scales with the tick rate. Seeded at the threshold so
    // the first Tick() checks immediately.
    int ticks_since_data_check_;

    bool last_fix_valid_;
    bool first_data_check_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SATELLITE_VISIBILITY_H

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
 * \brief Computes elevation (deg, floored) for every satellite with usable
 * GPS/Galileo/BeiDou ephemeris or almanac data in pvt_ptr, at the given GPS
 * time and ECEF receiver position, keeping only those at or above
 * elevation_mask_deg AND broadcasting a healthy status (SV_health == 0 for
 * GPS/BeiDou; Galileo's E1B_HS == 0, since 1B is the search-initiating
 * signal and 5X only ever gets searched once 1B is tracked). A satellite
 * marked unhealthy is known unusable regardless of its geometric elevation
 * -- that broadcast declaration is stronger evidence than geometry alone,
 * so it's treated the same as an elevation at/below the mask (below_mask_out
 * below), not folded into "no data" -- otherwise a satellite whose almanac
 * genuinely places it above the mask but marks it unhealthy (an old/retired
 * satellite still broadcasting plausible-looking orbital elements is the
 * common case) would be classified visible and searched forever despite
 * never being acquirable. Sorted descending by elevation.
 *
 * This is the elevation-computation core factored out of
 * ControlThread::get_visible_sats() so it can be reused by
 * SatelliteVisibility's runtime recompute below without duplicating the
 * rtklib eph2pos/alm2pos/topocent logic.
 *
 * \param elevation_mask_deg minimum elevation (deg) to be considered
 * visible -- see GNSS-SDR.search_elevation_mask. Distinct from (and not
 * necessarily equal to) PVT.elevation_mask, which gates the navigation
 * solution rather than the acquisition search order.
 * \param below_mask_out when non-null, appended with every satellite whose
 * elevation *could* be computed (has usable ephemeris/almanac data) but
 * came out at or below elevation_mask_deg -- i.e. we know for sure it's not
 * visible, as opposed to a satellite with no data at all (we don't know
 * either way). Each satellite is classified by exactly one data source
 * (ephemeris preferred over almanac when both exist), so the returned list
 * and *below_mask_out never overlap. ControlThread::get_visible_sats()
 * passes nullptr; SatelliteVisibility's runtime recompute uses it to tell
 * "computed, not visible" apart from "can't tell" (see the class doc
 * comment below).
 * \param ephemeris_max_age_s / almanac_max_age_s maximum |current_time -
 * toe| / |current_time - toa|, in seconds, before that entry is treated as
 * stale -- i.e. as if the PRN had no ephemeris/almanac at all (falls through
 * to almanac if ephemeris is stale but almanac isn't; reverts to
 * "maybe visible" if both are, or neither exists). toe/toa are seconds-of-
 * week, so ages beyond roughly half a week (302400 s) can't be represented
 * unambiguously by this check alone -- keep both well under that. Default
 * (infinity) disables staleness checking entirely, matching this function's
 * behavior before ages were tracked; ControlThread::get_visible_sats() relies
 * on that default.
 * \param seconds_until_next_expiry_out when non-null, set to the number of
 * receiver-time seconds from now until the *soonest* currently-classified
 * (visible or below-mask) satellite's data would cross its staleness
 * threshold -- i.e. how long the caller can wait before an expiry-driven
 * recompute is needed. Left at +infinity if nothing was classified.
 * \param only_prns when non-null, every ephemeris/almanac loop skips any
 * (system, PRN) not in this set -- lets a caller recompute just the
 * satellite(s) whose data actually changed instead of the whole
 * constellation, when nothing else (receiver position, elapsed time) that
 * could affect every other satellite's elevation has changed. Left null,
 * every satellite with usable data is computed, as before.
 */
// The truncated week fields (ephemeris' mod-1024 rollover, almanac's WNa --
// 2 bits for Galileo, 8 for GPS) are resolved against gps_gtime's own GPS
// week internally, rather than a caller-supplied reference: gps_gtime is the
// query time this whole computation is FOR, so it's always available and
// always close to correct, unlike a config-derived reference week (see
// gps_ref_week_from_config()) that can be left unset.
std::vector<std::pair<int, Gnss_Satellite>> compute_visible_satellites(
    const std::shared_ptr<PvtInterface>& pvt_ptr,
    const gtime_t& gps_gtime,
    const arma::vec& r_eb_e,
    double elevation_mask_deg,
    std::vector<std::pair<int, Gnss_Satellite>>* below_mask_out = nullptr,
    double ephemeris_max_age_s = std::numeric_limits<double>::infinity(),
    double almanac_max_age_s = std::numeric_limits<double>::infinity(),
    double* seconds_until_next_expiry_out = nullptr,
    const std::set<std::pair<std::string, uint32_t>>* only_prns = nullptr);


/*!
 * \brief Maintains a runtime-updated classification of the satellites
 * configured for search, recomputed on GNSSFlowgraph's control-thread tick,
 * into exactly two searchable sets:
 *
 *   - visible: elevation computable and above GNSS-SDR.search_elevation_mask
 *     right now (IsVisible() true), and not in that system's
 *     GNSS-SDR.<System>_banned_prns list. Searched with priority.
 *   - maybe visible: elevation NOT computable yet (no ephemeris/almanac) --
 *     the system genuinely cannot tell whether it's visible. Not stored
 *     explicitly; it's whatever is in neither visible_ nor excluded_. Still
 *     searched, at lower priority (see
 *     GNSS-SDR.visible_vs_mayvisible_search_ratio), since it might turn out
 *     to be visible once data arrives.
 *
 * A satellite whose elevation *can* be computed is always removed from
 * maybe-visible: above the mask, it's in visible_; at or below it, it's
 * excluded from both -- computable-and-not-visible is not the same as
 * "don't know", so it isn't searched at all while that holds (this is what
 * lets a channel go idle once nothing plausible remains, instead of
 * grinding through satellites we're confident aren't there). A satellite
 * that's already confirmed tracked on one of its other signals is exempted
 * from this exclusion regardless of what the elevation computation says --
 * see GNSSFlowgraph::pop_by_visibility() -- since a coarse almanac and a
 * precise ephemeris (only decoded once tracking begins) can legitimately
 * disagree right at the mask boundary, and active tracking is stronger
 * evidence than either.
 *
 * visible_ and excluded_ are always disjoint by construction (see
 * compute_visible_satellites()'s single-data-source-per-satellite rule).
 *
 * Tick() recomputes when any of five conditions holds: the fix just became
 * valid; new/updated ephemeris or almanac data arrived for any PRN (a
 * per-PRN toe/toa fingerprint, not just a map-size check -- catches a PRN's
 * data being replaced, not just a brand new PRN appearing);
 * GNSS-SDR.visibility_recompute_interval_s of receiver time has elapsed
 * since the last recompute; the receiver has moved more than
 * GNSS-SDR.visibility_recompute_position_threshold_m since the last
 * recompute; or the freshest data behind the current classification has
 * aged past GNSS-SDR.visibility_ephemeris_max_age_s /
 * _almanac_max_age_s (see compute_visible_satellites()'s age parameters --
 * expiry is enforced there, not just detected here, so an expired PRN
 * actually reverts to maybe-visible rather than staying classified off
 * stale data).
 *
 * Disabled unless GNSS-SDR.enable_visibility_aware_search=true (the default);
 * Tick() is then a no-op and IsVisible()/IsExcluded() always return false,
 * so callers degrade to treating every satellite as maybe-visible (i.e.
 * today's undifferentiated single-queue behavior stays intact upstream).
 */
class SatelliteVisibility
{
public:
    explicit SatelliteVisibility(const std::shared_ptr<ConfigurationInterface>& configuration);

    bool enabled() const { return enabled_; }
    uint32_t search_ratio() const { return search_ratio_; }

    /*!
     * \brief Re-evaluates visibility if warranted: the fix just became
     * valid, more ephemeris/almanac data arrived since the last check, or
     * the recompute interval elapsed. Uses the live PVT fix when valid,
     * else the static GNSS-SDR.AGNSS_ref_location if configured, else does
     * nothing (insufficient information).
     *
     * \returns true iff bucket membership actually changed, i.e. the caller
     * should rebuild its per-signal search queues.
     */
    bool Tick(const std::shared_ptr<PvtInterface>& pvt_ptr, const Monitor_Pvt& fix_status);

    bool IsVisible(const Gnss_Satellite& sat) const;
    // Elevation computable and at/below the mask -- see the class doc
    // comment above for why this isn't the same as "not visible", and why
    // it's a separate query from IsVisible() rather than its negation.
    bool IsExcluded(const Gnss_Satellite& sat) const;

private:
    // changed_prns_out, when non-null, receives the (system, PRN) of every
    // entry that was added, updated, or removed relative to the last check
    // -- lets Tick() recompute just those satellites instead of the whole
    // constellation when nothing else has changed. Left untouched on the
    // very first call (first_data_check_): with nothing to diff against yet,
    // that call is irrelevant anyway, since moved_significantly is also
    // unconditionally true the first time (see last_recompute_r_eb_e_'s doc
    // comment), which always forces a full recompute regardless.
    bool DataChanged(const std::shared_ptr<PvtInterface>& pvt_ptr,
        std::set<std::pair<std::string, uint32_t>>* changed_prns_out = nullptr);

    bool enabled_;
    uint32_t search_ratio_;
    double recompute_interval_s_;
    double elevation_mask_deg_;
    // GNSS-SDR.visibility_recompute_position_threshold_m: forces an
    // immediate recompute once the receiver has moved this far (meters,
    // ECEF) since the last one, independent of recompute_interval_s_ --
    // covers a moving receiver crossing enough ground that elevations could
    // have shifted meaningfully before the next periodic tick.
    double position_threshold_m_;
    // GNSS-SDR.visibility_ephemeris_max_age_s / _almanac_max_age_s: see
    // compute_visible_satellites()'s matching parameters.
    double ephemeris_max_age_s_;
    double almanac_max_age_s_;
    bool have_agnss_reference_;
    double agnss_ref_lat_deg_;
    double agnss_ref_lon_deg_;
    // GNSS-SDR.AGNSS_ref_utc_time (via parse_agnss_ref_utc_time(), which
    // already falls back to wall-clock time if unset) -- the target instant
    // for the no-fix-yet fallback recompute below. Always usable, no
    // separate "have a value" flag needed. This matters for offline/replay
    // runs specifically: without a configured reference, wall-clock time
    // gets used, which has nothing to do with the GNSS time actually
    // embedded in the samples being processed, silently classifying
    // satellites against the wrong orbital configuration entirely.
    time_t agnss_ref_utc_time_;

    // GNSS-SDR.<System>_banned_prns, same parsing as GNSSFlowgraph::set_signals_list(),
    // for GPS/Galileo/Beidou (the systems compute_visible_satellites() covers).
    // A banned PRN is never classified visible, matching how it's removed
    // from the search pool entirely elsewhere -- this just keeps the
    // visibility classification/diagnostics consistent with that, even
    // though a banned PRN can never actually be picked from the pool anyway.
    std::set<std::pair<std::string, uint32_t>> banned_;

    // Which of {"GPS", "Galileo", "Beidou"} have at least one channel
    // actually configured (any of that system's own signals, e.g. GPS's
    // "1C"/"2S"/"L5" -- same Channels_<signal>.count check
    // search_next_signal() already uses to decide whether a signal is in
    // play). Used to scope the maybe-visible report in Tick() to systems
    // this receiver is actually searching -- otherwise an unconfigured
    // system's full PRN range would sit as permanently unclassified noise,
    // since nothing will ever decode ephemeris/almanac for it here.
    std::set<std::string> configured_systems_;

    std::set<std::pair<std::string, uint32_t>> visible_;
    std::set<std::pair<std::string, uint32_t>> excluded_;  // elevation computable, at/below the mask

    // Last elevation (deg) seen for each (system, PRN) -- not part of
    // classification (that always uses the raw, freshly computed elevation
    // directly: decoding correctness is enforced at its actual sources --
    // week-number resolution in alm_to_rtklib(), almanac freshness/
    // overwrite priority in update_almanac_if_fresher(), the ephemeris/
    // almanac map mutex against the cross-thread data race -- not by
    // second-guessing or withholding a live reading here). Kept purely so
    // Tick()'s diagnostic report can show every currently classified
    // satellite's elevation even on a targeted recompute, which only
    // freshly computes the satellite(s) whose data actually changed --
    // this is where every other satellite's last-known value comes from.
    std::map<std::pair<std::string, uint32_t>, double> reference_elevation_deg_;

    bool last_fix_valid_;
    // Per-(system, "EPH"/"ALM", PRN) fingerprint (toe or toa) of the last
    // ephemeris/almanac data seen for that entry. Finer than a map-size
    // count: catches an existing PRN's data being *replaced* (new toe/toa,
    // same map size), not just a brand new PRN appearing. Compared
    // wholesale in DataChanged(); first_data_check_ handles the "nothing to
    // compare against yet" first call explicitly, since an empty map here
    // is otherwise indistinguishable from "already checked, still no data".
    std::map<std::tuple<std::string, std::string, uint32_t>, int32_t> last_data_fingerprints_;
    bool first_data_check_;
    // Receiver time (fix_status.RX_time, same units/epoch as the gps_gtime
    // built in Tick()) at which the last periodic recompute ran -- NOT
    // wall-clock time. This receiver frequently runs far slower than
    // real-time (heavy acquisition load pegs the CPU) or far faster (fast
    // file replay), so a std::chrono::steady_clock-based interval would
    // silently drift from its configured meaning in receiver-time terms:
    // e.g. a "5 second" wall-clock throttle can correspond to as little as
    // ~1 receiver-second under heavy acquisition load, defeating the
    // point of throttling exactly when CPU is most contended. Only
    // meaningful once a fix is valid (RX_time isn't ticking before that);
    // pre-fix recomputes rely on fix_became_valid/data_changed instead.
    // Seeded far in the past so the first valid-fix tick always recomputes.
    double last_recompute_rx_time_s_;
    // ECEF position (meters) used at the last recompute -- see
    // position_threshold_m_. Seeded at the origin (Earth's core, guaranteed
    // far from any real position) so the first recompute's displacement
    // check doesn't need special-casing.
    arma::vec last_recompute_r_eb_e_;
    // Receiver time (same scale as last_recompute_rx_time_s_) at which the
    // freshest data behind the current classification will cross its
    // staleness threshold -- see compute_visible_satellites()'s
    // seconds_until_next_expiry_out. +infinity (never trigger) until a
    // recompute with real data has actually run.
    double next_expiry_deadline_rx_time_;
    // Caps how often DataChanged() -- which copies out every
    // ephemeris/almanac map just to check .size() -- actually runs, as a
    // fixed fraction of Tick() calls (a call-count cap, not a time-based
    // one: unlike a wall-clock throttle, this stays correctly proportional
    // to Tick()'s own call rate regardless of how fast or slow the
    // receiver happens to be progressing relative to real time). Seeded at
    // the threshold (not 0) so the very first Tick() call always runs
    // DataChanged() -- which itself always reports "changed" on its own
    // first call (first_data_check_) -- producing an initial classification
    // as early as possible, rather than waiting N calls in.
    int ticks_since_data_check_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SATELLITE_VISIBILITY_H

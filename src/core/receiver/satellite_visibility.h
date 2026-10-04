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
#include <array>
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
 * \brief Return healthy GPS/Galileo/BeiDou/GLONASS/QZSS satellites above the mask.
 * Prefer fresh ephemeris over almanac. BeiDou selects the closest orbit
 * (ties: DNAV, CNAV1, CNAV2); B-CNAV supports IGSO/MEO only.
 * Unhealthy satellites are excluded, not unknown; Galileo uses E1B health.
 *
 * \param gps_gtime GPST query epoch; also resolves truncated navigation weeks.
 * \param r_eb_e receiver ECEF position (m).
 * \param elevation_mask_deg search mask (deg), independent of PVT.elevation_mask.
 * \param below_mask_out optional output for satellites at/below the mask or unhealthy.
 * \param almanac_max_age_s maximum absolute almanac age (s); infinity disables it.
 * Ephemeris age limits use RTKLIB's per-system MAXDTOE* constants.
 * \param seconds_until_next_expiry_out optional time to earliest classified-data expiry;
 * infinity if nothing was classified.
 * \param only_prns optional (system, PRN) filter; requires unchanged position/time.
 * \param glonass_strict_health also reject the Bn MSB; ln always marks unhealthy.
 * \return (floor(elevation) in degrees, satellite) pairs, highest elevation first.
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
 * \brief Opt-in acquisition search classification: visible, excluded, or unknown.
 * Healthy satellites above the search mask are preferred; unknowns are searched
 * less often. Banned PRNs are omitted, and tracking can override exclusion.
 * Tick() refreshes on reference, fix, navigation-data, position, interval, or
 * expiry changes. Expired data returns satellites to the unknown pool.
 * Disabled by default (GNSS-SDR.enable_visibility_aware_search).
 */
class SatelliteVisibility
{
public:
    explicit SatelliteVisibility(const std::shared_ptr<ConfigurationInterface>& configuration);

    bool enabled() const { return enabled_; }
    uint32_t search_ratio() const { return search_ratio_; }

    /*!
     * \brief Supplies a telecommand position and UTC epoch, anchored to the
     * current sample clock. Forces the next Tick() to recompute and overrides
     * the current fix/configured reference until a different PVT epoch arrives.
     */
    void SetCommandReference(time_t utc_time, const std::array<float, 3>& LLH,
        const Monitor_Pvt& current_fix, double receiver_time_s);

    /*!
     * \brief Refresh visibility using telecommand, fix, then AGNSS reference priority.
     * \param receiver_time_s elapsed sample time; advances epochs between fixes.
     * \returns true when classification changes require rebuilding search queues.
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

    // Predict Doppler (Hz) from usable navigation data and receiver clock drift.
    // Live fixes must be anchored by Tick() within one second of sample time.
    // Pre-fix prediction requires opt-in, an AGNSS reference, and explicit bounds;
    // it assumes zero velocity and GNSS-SDR.clock_frequency_offset_ppm.
    // GLONASS requires exactly one visible slot on the searched FDMA frequency.
    // Returns false without changing doppler_hz if prediction is unavailable.
    bool PredictedDopplerHz(const std::shared_ptr<PvtInterface>& pvt_ptr,
        const Monitor_Pvt& fix_status, double receiver_time_s, const Gnss_Satellite& sat, const std::string& signal,
        double& doppler_hz) const;

    // Doppler uncertainty half-width (Hz): zero for a live fix, otherwise the
    // configured clock-error and speed bounds projected onto this carrier.
    // Missing/invalid bounds return infinity; explicit zero bounds are allowed.
    double PredictedDopplerUncertaintyHz(const Monitor_Pvt& fix_status, const std::string& signal) const;

private:
    enum class SearchVisibility
    {
        Unknown,
        Visible,
        Excluded
    };

    SearchVisibility GetSearchVisibility(const Gnss_Satellite& sat) const;

    // Predict for the sole visible slot on prn's FDMA frequency using ephemeris
    // or almanac. Returns its carrier frequency; band is 1 (L1) or 2 (L2).
    bool GlonassGeometricDopplerHz(const std::shared_ptr<PvtInterface>& pvt_ptr,
        const gtime_t& gps_gtime, uint32_t prn, int band, const std::array<double, 3>& rx_pos_m,
        const std::array<double, 3>& rx_vel_mps, double& geometric_doppler_hz, double& carrier_freq_hz) const;

    // QZSS geometric Doppler from LNAV, CNAV, or almanac, in that order.
    // Maps L1 C/B PRNs to their nominal PRNs.
    bool QzssGeometricDopplerHz(const std::shared_ptr<PvtInterface>& pvt_ptr,
        const gtime_t& gps_gtime, uint32_t prn, const std::array<double, 3>& rx_pos_m,
        const std::array<double, 3>& rx_vel_mps, double carrier_freq_hz, double& geometric_doppler_hz) const;

    // BeiDou part of PredictedDopplerHz(): use the same validated DNAV,
    // CNAV1, CNAV2 or fallback almanac orbit as visibility, evaluated on
    // the requested carrier at the receiver's ECEF position and velocity.
    bool BeidouGeometricDopplerHz(const std::shared_ptr<PvtInterface>& pvt_ptr,
        const gtime_t& gps_gtime, uint32_t prn, const std::array<double, 3>& rx_pos_m,
        const std::array<double, 3>& rx_vel_mps, double carrier_freq_hz, double& geometric_doppler_hz) const;

    // Reports added, changed, or removed PRNs for targeted recomputation.
    // The first call leaves changed_prns_out untouched and forces a full sweep.
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

    // Opt-in pre-fix prediction from the AGNSS reference (default false).
    bool doppler_prediction_before_fix_;

    // Pre-fix clock offset/error (ppm) and speed bound (m/s), from the matching
    // GNSS-SDR configuration keys. Live fixes supply their own clock and velocity.
    double clock_frequency_offset_ppm_;
    double clock_frequency_max_error_ppm_;
    double receiver_max_velocity_m_s_;
    bool have_doppler_uncertainty_budget_;

    bool have_command_reference_{false};
    std::array<float, 3> command_reference_llh_{};
    time_t command_reference_utc_time_{0};
    double command_reference_receiver_time_s_{0.0};
    double command_previous_fix_time_s_{-1.0};
    bool command_reference_changed_{false};

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

    // Per (system, navigation family, PRN): absolute toe/toa, health, IODE, IODC,
    // satellite type, and signal type. Unused legacy fields are zero.
    using NavigationFingerprint = std::tuple<double, int32_t, uint32_t, uint32_t, int32_t, int32_t>;
    std::map<std::tuple<std::string, std::string, uint32_t>, NavigationFingerprint> last_data_fingerprints_;

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

    // Throttle navigation-map copies by tick count; check on the first Tick().
    int ticks_since_data_check_;

    bool last_fix_valid_;
    bool first_data_check_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_SATELLITE_VISIBILITY_H

/*!
 * \file rtklib_conversions.h
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

#ifndef GNSS_SDR_RTKLIB_CONVERSIONS_H
#define GNSS_SDR_RTKLIB_CONVERSIONS_H

#include "rtklib.h"
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup RTKLIB_Library
 * \{ */


class Beidou_Cnav1_Ephemeris;
class Beidou_Dnav_Almanac;
class Beidou_Dnav_Ephemeris;
class Galileo_Almanac;
class Galileo_Ephemeris;
class Glonass_Gnav_Almanac;
class Glonass_Gnav_Ephemeris;
class Glonass_Gnav_Utc_Model;
class Gnss_Synchro;
class Gps_Almanac;
class Gps_CNAV_Ephemeris;
class Gps_CNAV2_Ephemeris;
class Gps_Ephemeris;

class HAS_clock_corrections
{
public:
    HAS_clock_corrections() = default;
    float clock_correction_m{};
    uint32_t valid_from{};   //!< t_MT1 in seconds of week
    uint32_t valid_until{};  //!< t_MT1 + VI, possibly beyond the end of the week
    uint16_t iod{};
    uint8_t mask_id{};
    uint8_t iod_set_id{};
};

class HAS_orbit_corrections
{
public:
    HAS_orbit_corrections() = default;
    float radial_m{};
    float in_track_m{};
    float cross_track_m{};
    uint32_t valid_from{};   //!< t_MT1 in seconds of week
    uint32_t valid_until{};  //!< t_MT1 + VI, possibly beyond the end of the week
    uint16_t iod{};
    uint8_t mask_id{};
    uint8_t iod_set_id{};
};

class HAS_bias_corrections
{
public:
    HAS_bias_corrections() = default;
    float bias{};
    uint32_t valid_from{};   //!< t_MT1 in seconds of week
    uint32_t valid_until{};  //!< t_MT1 + VI, possibly beyond the end of the week
    uint8_t mask_id{};
    uint8_t iod_set_id{};
};

class HAS_obs_corrections
{
public:
    HAS_obs_corrections() = default;
    std::string signal;
    float code_bias_m{};
    float phase_bias_cycle{};
    uint8_t phase_discontinuity_indicator{};
    bool phase_bias_discontinuity{};
    bool code_bias_available{};
    bool phase_bias_available{};
};


/*!
 * \brief Tracks the HAS corrections applied to the carrier phase of each
 * signal. Turning the HAS orbit and clock corrections or a phase bias on or
 * off, or switching the selected HAS signal, steps the carrier phase (by up
 * to 10.23 cycles for a phase bias, HAS SIS ICD 1.0 Section 7.5), which the
 * solver must handle as a cycle slip.
 */
class HAS_carrier_continuity
{
public:
    HAS_carrier_continuity() = default;

    /*!
     * \brief Records the HAS corrections applied to the carrier phase of a
     * signal of a satellite in the current epoch
     * \param sat RTKLIB satellite number
     * \param signal GNSS-SDR two-character signal identifier
     * \param orbit_clock_applied whether the HAS orbit and clock corrections are applied
     * \param phase_bias_signal selected HAS signal name, or empty if no phase bias is applied
     * \return true if they differ from the previous observation of the signal
     */
    bool changed(int sat, const std::string& signal, bool orbit_clock_applied, const std::string& phase_bias_signal);

private:
    std::map<std::pair<int, std::string>, std::pair<bool, std::string>> d_states;
};


eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph);

/*!
 * \brief Galileo ephemeris with HAS orbit and clock corrections. They are
 * applied, and the BGDs dropped, only to an I/NAV ephemeris with the corrected
 * issue of data, when both the orbit and the clock corrections exist for the
 * same Mask ID and IOD Set ID
 */
eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph,
    const std::map<int, HAS_orbit_corrections>& orbit_correction_map,
    const std::map<int, HAS_clock_corrections>& clock_correction_map);

eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph,
    int ref_week = 0);

eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph,
    const std::map<int, HAS_orbit_corrections>& orbit_correction_map,
    const std::map<int, HAS_clock_corrections>& clock_correction_map,
    int ref_week = 0);

eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris& gps_cnav_eph);

/*!
 * \brief Copies the inter-signal corrections of a GPS CNAV ephemeris to the
 * RTKLIB record of a satellite built from another navigation message, when
 * an L2C or L5 observation joins it. An L1C first slot keeps its ISC_L1CP.
 * A record with HAS corrections gets no ISCs: the HAS code biases replace the
 * broadcast group delays (HAS SIS ICD 1.0 Section 7.4).
 * \param eph RTKLIB ephemeris record of the satellite
 * \param gps_cnav_eph CNAV ephemeris of the satellite
 * \param first_slot_code observation code in the first slot of the record
 */
void add_gps_cnav_iscs(eph_t& eph, const Gps_CNAV_Ephemeris& gps_cnav_eph, unsigned char first_slot_code);

/*!
 * \brief GPS L1C CNAV-2 ephemeris. The first ISC slot holds ISC_L1CP, the
 * correction of the L1C pilot observations (IS-GPS-800J, 3.5.3.9.1).
 */
eph_t eph_to_rtklib(const Gps_CNAV2_Ephemeris& gps_cnav2_eph);

eph_t eph_to_rtklib(const Beidou_Dnav_Ephemeris& bei_eph);

eph_t eph_to_rtklib(const Beidou_Cnav1_Ephemeris& bei_eph);

/*!
 * \brief Absolute GPST epoch of a GLONASS almanac (N_4/N_A calendar day plus
 * t_lambda_n_A, which is given in MSK = UTC+3).
 * \return Zero gtime_t when the broadcast date is unavailable or invalid.
 */
gtime_t glonass_almanac_epoch(const Glonass_Gnav_Almanac& almanac);

/*!
 * \brief Converts almanac data to RTKLIB's alm_t format.
 *
 * \param ref_week Current GPS week, used to resolve the truncated almanac
 * week (WNa: 8 bits for GPS, 2 bits for Galileo) to the closest full week so
 * that alm.toa is an absolute epoch, as eph_to_rtklib() does for ephemeris.
 * 0 (default) uses WNa as-is; only valid for callers that never propagate
 * the orbit (e.g. unit tests checking unrelated fields).
 */
alm_t alm_to_rtklib(const Gps_Almanac& gps_alm, int ref_week = 0);
alm_t alm_to_rtklib(const Galileo_Almanac& gal_alm, int ref_week = 0);
alm_t alm_to_rtklib(const Beidou_Dnav_Almanac& bei_alm);

/*!
 * \brief Broadcast health of a GLONASS satellite according to its GNAV ephemeris
 * \param glonass_gnav_eph GLONASS GNAV Ephemeris structure
 * \param glonass_strict_health when true (default), the MSB of the Bn word also marks the
 * satellite as unhealthy; when false, only the ln flag is used
 * \return true if the satellite does not report a malfunction
 */
bool glonass_gnav_is_healthy(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, bool glonass_strict_health = true);

/*!
 * \brief Transforms a Glonass_Gnav_Ephemeris to its RTKLIB counterpart
 * \param glonass_gnav_eph GLONASS GNAV Ephemeris structure
 * \param gnav_clock_model GLONASS GNAV UTC model (for the GLONASST to GPST conversion)
 * \param glonass_strict_health when true (default), the MSB of the Bn word also marks the
 * satellite as unhealthy; when false, only the ln flag is used
 * \return Ephemeris structure for RTKLIB parsing
 */
geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model, bool glonass_strict_health = true);

/*!
 * \brief RTKLIB data-component code used for configured-frequency preflight.
 * Runtime observations use Gnss_Synchro::observation_code().
 * \param system GNSS-SDR system identifier ('G', 'E', 'R', 'C', 'J', 'S')
 * \param signal GNSS-SDR two-character signal identifier (e.g. "1C", "5X", "B3")
 * \param band RTKLIB band slot the signal is stored in
 * \return observation code, CODE_NONE if the signal has no code for that slot
 */
unsigned char rtklib_obs_code(char system, const std::string& signal, int band);

/*!
 * \brief HAS signals (HAS SIS ICD 1.0 Table 20) whose biases can correct an
 * observation, in order of preference: the tracked component of the signal,
 * the other component, and their combination
 * \param system GNSS-SDR system identifier ('G' or 'E')
 * \param signal GNSS-SDR two-character signal identifier (e.g. "1C", "E6")
 * \param tracking_pilot true if the pilot component is tracked
 * \return HAS signal names, empty if HAS does not correct the signal
 */
std::vector<std::string> has_signal_candidates(char system, const std::string& signal, bool tracking_pilot = false);

/*!
 * \brief HAS correction that insert_obs_to_rtklib() applies to an observation
 * \param has_obs_corr HAS corrections, first key is the HAS signal name, second key is the PRN
 * \param system GNSS-SDR system identifier ('G' or 'E')
 * \param signal GNSS-SDR two-character signal identifier
 * \param prn satellite PRN
 * \param tracking_pilot true if the pilot component is tracked
 * \return correction of the first candidate with an available code bias for the PRN, nullptr if none
 */
const HAS_obs_corrections* find_has_obs_correction(const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    char system,
    const std::string& signal,
    int prn,
    bool tracking_pilot = false);

obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    int week,
    int band,
    const HAS_obs_corrections** applied_has_correction,
    int ref_week);

obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs,
    const Gnss_Synchro& gnss_synchro,
    const std::map<std::string, std::map<int, HAS_obs_corrections>>& has_obs_corr,
    int week,
    int band,
    int ref_week = 0);

obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs, const Gnss_Synchro& gnss_synchro, int week, int band, int ref_week = 0);


/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_CONVERSIONS_H

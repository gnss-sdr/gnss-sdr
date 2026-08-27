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

#include "gps_cnav2_ephemeris.h"
#include "rtklib.h"
#include <cstdint>
#include <map>
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup RTKLIB_Library
 * \{ */


class Beidou_Cnav1_Ephemeris;
class Beidou_Dnav_Almanac;
class Beidou_Dnav_Ephemeris;
class Galileo_Almanac;
class Galileo_Ephemeris;
class Glonass_Gnav_Ephemeris;
class Glonass_Gnav_Utc_Model;
class Gnss_Synchro;
class Gps_Almanac;
class Gps_CNAV_Ephemeris;
class Gps_Ephemeris;

class HAS_clock_corrections
{
public:
    HAS_clock_corrections() = default;
    float clock_correction_m{};
    uint32_t valid_until{};
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
    uint32_t valid_until{};
    uint16_t iod{};
    uint8_t mask_id{};
    uint8_t iod_set_id{};
};

class HAS_bias_corrections
{
public:
    HAS_bias_corrections() = default;
    float bias{};
    uint32_t valid_until{};
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
};


eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph);

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
eph_t eph_to_rtklib(const Gps_CNAV2_Ephemeris& gps_cnav2_eph);

eph_t eph_to_rtklib(const Beidou_Dnav_Ephemeris& bei_eph);

eph_t eph_to_rtklib(const Beidou_Cnav1_Ephemeris& bei_eph);

alm_t alm_to_rtklib(const Gps_Almanac& gps_alm);
alm_t alm_to_rtklib(const Galileo_Almanac& gal_alm);
alm_t alm_to_rtklib(const Beidou_Dnav_Almanac& bei_alm);

/*!
 * \brief Transforms a Glonass_Gnav_Ephemeris to its RTKLIB counterpart
 * \param glonass_gnav_eph GLONASS GNAV Ephemeris structure
 * \param gnav_clock_model GLONASS GNAV UTC model (for the GLONASST to GPST conversion)
 * \param glonass_strict_health when true (default), the MSB of the Bn word also marks the
 * satellite as unhealthy; when false, only the ln flag is used
 * \return Ephemeris structure for RTKLIB parsing
 */
geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model, bool glonass_strict_health = true);

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

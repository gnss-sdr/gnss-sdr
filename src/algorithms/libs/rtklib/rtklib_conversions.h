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

/** \addtogroup PVT
 * \{ */
/** \addtogroup RTKLIB_Library
 * \{ */


class Beidou_Dnav_Ephemeris;
class Galileo_Almanac;
class Galileo_Ephemeris;
class Glonass_Gnav_Ephemeris;
class Glonass_Gnav_Utc_Model;
class Gnss_Synchro;
class Gps_Almanac;
class Gps_CNAV_Ephemeris;
class Gps_Ephemeris;

eph_t eph_to_rtklib(const Galileo_Ephemeris& gal_eph);
eph_t eph_to_rtklib(const Gps_Ephemeris& gps_eph, bool pre_2009_file);
eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris& gps_cnav_eph);
eph_t eph_to_rtklib(const Beidou_Dnav_Ephemeris& bei_eph);

alm_t alm_to_rtklib(const Gps_Almanac& gps_alm);
alm_t alm_to_rtklib(const Galileo_Almanac& gal_alm);

/*!
 * \brief Transforms a Glonass_Gnav_Ephemeris to its RTKLIB counterpart
 * \param glonass_gnav_eph GLONASS GNAV Ephemeris structure
 * \return Ephemeris structure for RTKLIB parsing
 */
geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& gnav_clock_model);

obsd_t insert_obs_to_rtklib(obsd_t& rtklib_obs, const Gnss_Synchro& gnss_synchro, int week, int band, bool pre_2009_file = false);


/** \} */
/** \} */
#endif  // GNSS_SDR_RTKLIB_CONVERSIONS_H

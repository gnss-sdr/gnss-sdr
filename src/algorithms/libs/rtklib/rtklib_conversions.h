/*!
 * \file rtklib_conversions.h
 * \brief GNSS-SDR to RTKLIB data structures conversion functions
 * \author 2017, Javier Arribas
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_CONVERSIONS_H_
#define GNSS_SDR_RTKLIB_CONVERSIONS_H_

#include "rtklib.h"
#include "gnss_synchro.h"
#include "galileo_ephemeris.h"
#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"

eph_t eph_to_rtklib(const Galileo_Ephemeris & gal_eph);
eph_t eph_to_rtklib(const Gps_Ephemeris & gps_eph);
eph_t eph_to_rtklib(const Gps_CNAV_Ephemeris & gps_cnav_eph);
/*!
 * \brief Transforms a Glonass_Gnav_Ephemeris to its RTKLIB counterpart
 * \param glonass_gnav_eph GLONASS GNAV Ephemeris structure
 * \return Ephemeris structure for RTKLIB parsing
 */
geph_t eph_to_rtklib(const Glonass_Gnav_Ephemeris & glonass_gnav_eph, const Glonass_Gnav_Utc_Model & gnav_clock_model);

obsd_t insert_obs_to_rtklib(obsd_t & rtklib_obs, const Gnss_Synchro & gnss_synchro, int week, int band);

#endif /* GNSS_SDR_RTKLIB_CONVERSIONS_H_ */

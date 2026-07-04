/*!
 * \file gps_cnav2_navigation_message.h
 * \brief  Interface of a GPS CNAV2 Data message decoder
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_CNAV2_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GPS_CNAV2_NAVIGATION_MESSAGE_H


#include "GPS_CNAV2.h"
#include "gps_cnav2_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

#include "GPS_CNAV.h"
#include "gps_cnav_navigation_message.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

/*!
 * \brief This class decodes a GPS CNAV2 Data message as described in IS-GPS-800J
 *
 *   TODO: For now unstested in QZSS!
 *
 * See https://www.gps.gov/sites/default/files/2025-07/IS-GPS-800J.pdf
 */
class Gps_CNAV2_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    explicit Gps_CNAV2_Navigation_Message(uint32_t prn, CnavSystem system = CnavSystem::GPS);

    void decode_sf2(uint16_t toi, const std::bitset<GPS_L1C_SF_2_DATA_BITS>& data_bits);
    void decode_sf3(uint16_t toi, const std::bitset<GPS_L1C_SF_3_DATA_BITS>& data_bits);

    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Gps_CNAV2_Ephemeris get_ephemeris() const;

    // TODO
    // /*!
    //  * \brief Check if we have a new iono record stored in the GPS ephemeris class
    //  */
    // bool have_new_iono();

    // /*!
    //  * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
    //  */
    // Gps_CNAV_Iono get_iono() const;

    // /*!
    //  * \brief Obtain a GPS UTC model parameters class filled with current SV data
    //  */
    // Gps_CNAV_Utc_Model get_utc_model();

    // /*!
    //  * \briefCheck if we have a new GPS UTC model record stored in the GPS ephemeris class
    //  */
    // bool have_new_utc_model();

    /*!
     * \brief Check if we have a new ephemeris stored in the GPS ephemeris class
     */
    bool have_new_ephemeris();

private:
    Gps_CNAV2_Ephemeris ephemeris_record;
    // TODO
    // Gps_CNAV_Iono iono_record;
    // Gps_CNAV_Utc_Model utc_model_record;

    CnavSystem d_system;

    bool b_flag_ephemeris{};
    // TODO
    // bool b_flag_clock_valid{};
    // bool b_flag_iono_valid{};
    // bool b_flag_utc_valid{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H

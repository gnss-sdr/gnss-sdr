/*!
 * \file gps_cnav_navigation_message.h
 * \brief  Interface of a GPS CNAV Data message decoder
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H


#include "GPS_CNAV.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>


/*!
 * \brief This class decodes a GPS CNAV Data message as described in IS-GPS-200K
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf Appendix III
 */
class Gps_CNAV_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Gps_CNAV_Navigation_Message();

    int32_t d_TOW;
    bool b_flag_ephemeris_1;
    bool b_flag_ephemeris_2;
    bool b_flag_iono_valid;  //!< If set, it indicates that the ionospheric parameters are filled and are not yet read by the get_iono
    bool b_flag_utc_valid;   //!< If set, it indicates that the utc parameters are filled and are not yet read by the get_utc_model

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs https://www.navcen.uscg.gov/?Do=constellationStatus

    // satellite positions
    double d_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // satellite identification info
    int32_t i_channel_ID;
    uint32_t i_satellite_PRN;

    // Satellite velocity
    double d_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]

    // public functions
    void reset();

    void decode_page(std::bitset<GPS_CNAV_DATA_PAGE_BITS> data_bits);

    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Gps_CNAV_Ephemeris get_ephemeris();

    /*!
     * \brief Check if we have a new iono record stored in the GPS ephemeris class
     */
    bool have_new_iono();

    /*!
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Gps_CNAV_Iono get_iono();

    /*!
     * \brief Obtain a GPS UTC model parameters class filled with current SV data
     */
    Gps_CNAV_Utc_Model get_utc_model();

    /*!
     * \briefCheck if we have a new GPS UTC model record stored in the GPS ephemeris class
     */
    bool have_new_utc_model();

    /*!
     * \brief Check if we have a new ephemeris stored in the GPS ephemeris class
     */
    bool have_new_ephemeris();

private:
    uint64_t read_navigation_unsigned(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);
    int64_t read_navigation_signed(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);
    bool read_navigation_bool(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);

    Gps_CNAV_Ephemeris ephemeris_record;
    Gps_CNAV_Iono iono_record;
    Gps_CNAV_Utc_Model utc_model_record;
};

#endif

/*!
 * \file gps_cnav_navigation_message.h
 * \brief  Interface of a GPS CNAV Data message decoder
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes a GPS CNAV Data message as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix III
 */
class Gps_CNAV_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Gps_CNAV_Navigation_Message();

    void decode_page(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& data_bits);

    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Gps_CNAV_Ephemeris get_ephemeris() const;

    /*!
     * \brief Check if we have a new iono record stored in the GPS ephemeris class
     */
    bool have_new_iono();

    /*!
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Gps_CNAV_Iono get_iono() const;

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
    uint64_t read_navigation_unsigned(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    bool read_navigation_bool(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;

    Gps_CNAV_Ephemeris ephemeris_record{};
    Gps_CNAV_Iono iono_record{};
    Gps_CNAV_Utc_Model utc_model_record{};

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs https://www.navcen.uscg.gov/?Do=constellationStatus

    int32_t d_TOW{};

    bool b_flag_ephemeris_1{};
    bool b_flag_ephemeris_2{};
    bool b_flag_iono_valid{};  //!< If set, it indicates that the ionospheric parameters are filled and are not yet read by the get_iono
    bool b_flag_utc_valid{};   //!< If set, it indicates that the utc parameters are filled and are not yet read by the get_utc_model
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_NAVIGATION_MESSAGE_H

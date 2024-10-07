/*!
 * \file glonass_gnav_navigation_message.h
 * \brief  Interface of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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


#ifndef GNSS_SDR_GLONASS_GNAV_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GLONASS_GNAV_NAVIGATION_MESSAGE_H


#include "GLONASS_L1_L2_CA.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <utility>  // for pair
#include <vector>   // for vector

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes a GLONASS GNAV Data message as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Glonass_Gnav_Navigation_Message();

    /*!
     * \brief Compute CRC for GLONASS GNAV strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS>& bits) const;

    /*!
     * \brief Computes the frame number being decoded given the satellite slot number
     * \param satellite_slot_number [in] Satellite slot number identifier
     * \returns Frame number being decoded, 0 if operation was not successful.
     */
    uint32_t get_frame_number(uint32_t satellite_slot_number);

    /*!
     * \brief Obtain a GLONASS GNAV SV Ephemeris class filled with current SV data
     */
    Glonass_Gnav_Ephemeris get_ephemeris() const
    {
        return gnav_ephemeris;
    }

    /*!
     * \brief Obtain a GLONASS GNAV UTC model parameters class filled with current SV data
     */
    inline Glonass_Gnav_Utc_Model get_utc_model() const
    {
        return gnav_utc_model;
    }

    /*!
     * \brief Returns a Glonass_Gnav_Almanac object filled with the latest navigation data received
     * \param satellite_slot_number Slot number identifier for the satellite
     * \returns Returns the Glonass_Gnav_Almanac object for the input slot number
     */
    Glonass_Gnav_Almanac get_almanac(uint32_t satellite_slot_number) const;

    /*!
     * \brief Returns true if a new Glonass_Gnav_Ephemeris object has arrived.
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Glonass_Gnav_Utc_Model object has arrived
     */
    bool have_new_utc_model();

    /*!
     * \brief Returns true if new Glonass_Gnav_Almanac object has arrived.
     */
    bool have_new_almanac();

    /*!
     * \brief Decodes the GLONASS GNAV string
     * \param frame_string [in] is the string message within the parsed frame
     * \returns Returns the ID of the decoded string
     */
    int32_t string_decoder(const std::string& frame_string);

    inline bool get_flag_CRC_test() const
    {
        return flag_CRC_test;
    }

    inline void set_rf_link(int32_t rf_link)
    {
        gnav_ephemeris.i_satellite_freq_channel = rf_link;
    }

    inline uint32_t get_alm_satellite_slot_number() const
    {
        return i_alm_satellite_slot_number;
    }

    inline bool get_flag_update_slot_number() const
    {
        return flag_update_slot_number;
    }

    inline void set_flag_update_slot_number(bool flag_slot)
    {
        flag_update_slot_number = flag_slot;
    }

    inline bool get_flag_TOW_new() const
    {
        return flag_TOW_new;
    }

    inline void set_flag_TOW_new(bool tow_new)
    {
        flag_TOW_new = tow_new;
    }

    inline bool is_flag_TOW_set() const
    {
        return flag_TOW_set;
    }

    inline void set_flag_ephemeris_str_1(bool ephemeris_str_1)
    {
        flag_ephemeris_str_1 = ephemeris_str_1;
    }

    inline void set_flag_ephemeris_str_2(bool ephemeris_str_2)
    {
        flag_ephemeris_str_2 = ephemeris_str_2;
    }

    inline void set_flag_ephemeris_str_3(bool ephemeris_str_3)
    {
        flag_ephemeris_str_3 = ephemeris_str_3;
    }

    inline void set_flag_ephemeris_str_4(bool ephemeris_str_4)
    {
        flag_ephemeris_str_4 = ephemeris_str_4;
    }

private:
    uint64_t read_navigation_unsigned(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    bool read_navigation_bool(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;

    Glonass_Gnav_Ephemeris gnav_ephemeris{};                   // Ephemeris information decoded
    Glonass_Gnav_Utc_Model gnav_utc_model{};                   // UTC model information
    Glonass_Gnav_Almanac gnav_almanac[GLONASS_CA_NBR_SATS]{};  // Almanac information for all 24 satellites

    std::map<int, std::string> satelliteBlock;  // Map that stores to which block the PRN belongs

    double d_previous_tb{};                       // Previous iode for the Glonass_Gnav_Ephemeris object. Used to determine when new data arrives
    double d_previous_Na[GLONASS_CA_NBR_SATS]{};  // Previous time for almanac of the Glonass_Gnav_Almanac object

    uint32_t d_frame_ID{};
    uint32_t d_string_ID{};
    uint32_t i_alm_satellite_slot_number{};  // SV Orbit Slot Number

    bool flag_CRC_test{};
    bool flag_update_slot_number{};

    // Ephemeris Flags and control variables
    bool flag_all_ephemeris{};    // Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_str_1{};  // Flag indicating that ephemeris 1/4 (string 1) have been received
    bool flag_ephemeris_str_2{};  // Flag indicating that ephemeris 2/4 (string 2) have been received
    bool flag_ephemeris_str_3{};  // Flag indicating that ephemeris 3/4 (string 3) have been received
    bool flag_ephemeris_str_4{};  // Flag indicating that ephemeris 4/4 (string 4) have been received

    // Almanac Flags
    bool flag_almanac_str_6{};   // Flag indicating that almanac of string 6 have been received
    bool flag_almanac_str_7{};   // Flag indicating that almanac of string 7 have been received
    bool flag_almanac_str_8{};   // Flag indicating that almanac of string 8 have been received
    bool flag_almanac_str_9{};   // Flag indicating that almanac of string 9 have been received
    bool flag_almanac_str_10{};  // Flag indicating that almanac of string 10 have been received
    bool flag_almanac_str_11{};  // Flag indicating that almanac of string 11 have been received
    bool flag_almanac_str_12{};  // Flag indicating that almanac of string 12 have been received
    bool flag_almanac_str_13{};  // Flag indicating that almanac of string 13 have been received
    bool flag_almanac_str_14{};  // Flag indicating that almanac of string 14 have been received
    bool flag_almanac_str_15{};  // Flag indicating that almanac of string 15 have been received

    // UTC and System Clocks Flags
    bool flag_utc_model_str_5{};  // Clock info send in string 5 of navigation data

    bool flag_TOW_set{};  // Flag indicating when the TOW has been set
    bool flag_TOW_new{};  // Flag indicating when a new TOW has been computed
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_GNAV_NAVIGATION_MESSAGE_H

/*!
 * \file glonass_gnav_navigation_message.h
 * \brief  Interface of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GLONASS_GNAV_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_GLONASS_GNAV_NAVIGATION_MESSAGE_H_


#include <bitset>
#include "GLONASS_L1_CA.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_utc_model.h"



/*!
 * \brief This class decodes a GLONASS GNAV Data message as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Navigation_Message
{
private:
    unsigned long int read_navigation_unsigned(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    signed long int read_navigation_signed(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    bool read_navigation_bool(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);

public:
    bool flag_CRC_test;
    unsigned int d_frame_ID;
    unsigned int d_string_ID;
    bool flag_update_slot_number;

    int i_channel_ID;
    unsigned int i_satellite_PRN;

    Glonass_Gnav_Ephemeris gnav_ephemeris;    //!< Ephemeris information decoded
    Glonass_Gnav_Utc_Model gnav_utc_model;    //!< UTC model information
    Glonass_Gnav_Almanac gnav_almanac[GLONASS_L1_CA_NBR_SATS];  //!< Almanac information for all 24 satellites

    // Ephemeris Flags and control variables
    bool flag_all_ephemeris;      //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_str_1;    //!< Flag indicating that ephemeris 1/4 (string 1) have been received
    bool flag_ephemeris_str_2;    //!< Flag indicating that ephemeris 2/4 (string 2) have been received
    bool flag_ephemeris_str_3;    //!< Flag indicating that ephemeris 3/4 (string 3) have been received
    bool flag_ephemeris_str_4;    //!< Flag indicating that ephemeris 4/4 (string 4) have been received

    // Almanac Flags
    bool flag_all_almanac;        //!< Flag indicating that all almanac have been received
    bool flag_almanac_str_6;      //!< Flag indicating that almanac of string 6 have been received
    bool flag_almanac_str_7;      //!< Flag indicating that almanac of string 7 have been received
    bool flag_almanac_str_8;      //!< Flag indicating that almanac of string 8 have been received
    bool flag_almanac_str_9;      //!< Flag indicating that almanac of string 9 have been received
    bool flag_almanac_str_10;     //!< Flag indicating that almanac of string 10 have been received
    bool flag_almanac_str_11;     //!< Flag indicating that almanac of string 11 have been received
    bool flag_almanac_str_12;     //!< Flag indicating that almanac of string 12 have been received
    bool flag_almanac_str_13;     //!< Flag indicating that almanac of string 13 have been received
    bool flag_almanac_str_14;     //!< Flag indicating that almanac of string 14 have been received
    bool flag_almanac_str_15;     //!< Flag indicating that almanac of string 15 have been received
    unsigned int i_alm_satellite_slot_number; //!< SV Orbit Slot Number

    // UTC and System Clocks Flags
    bool flag_utc_model_valid;    //!< If set, it indicates that the UTC model parameters are filled
    bool flag_utc_model_str_5;    //!< Clock info send in string 5 of navigation data
    bool flag_utc_model_str_15;   //!< Clock info send in string 15 of frame 5 of navigation data

    bool flag_TOW_set;      //!< Flag indicating when the TOW has been set
    bool flag_TOW_new;      //!< Flag indicating when a new TOW has been computed

    double d_satClkCorr;    //!<  Satellite clock error
    double d_dtr;           //!<  Relativistic clock correction term
    double d_satClkDrift;   //!<  Satellite clock drift

    double d_previous_tb; //!< Previous iode for the Glonass_Gnav_Ephemeris object. Used to determine when new data arrives
    double d_previous_Na[GLONASS_L1_CA_NBR_SATS]; //!< Previous time for almanac of the Glonass_Gnav_Almanac object

    /*!
     * \brief Compute CRC for GLONASS GNAV strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS> bits);

    /*!
     * \brief Computes the frame number being decoded given the satellite slot number
     * \param satellite_slot_number [in] Satellite slot number identifier
     * \returns Frame number being decoded, 0 if operation was not successful.
     */
    unsigned int get_frame_number(unsigned int satellite_slot_number);

    /*!
     * \brief Reset GLONASS GNAV Navigation Information
     */
    void reset();

    /*!
     * \brief Obtain a GLONASS GNAV SV Ephemeris class filled with current SV data
     */
    Glonass_Gnav_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a GLONASS GNAV UTC model parameters class filled with current SV data
     */
    Glonass_Gnav_Utc_Model get_utc_model();

    /*!
     * \brief Returns a Glonass_Gnav_Almanac object filled with the latest navigation data received
     * \param satellite_slot_number Slot number identifier for the satellite
     * \returns Returns the Glonass_Gnav_Almanac object for the input slot number
     */
    Glonass_Gnav_Almanac get_almanac(unsigned int satellite_slot_number);

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
    int string_decoder(std::string frame_string);

    /*!
     * Default constructor
     */
    Glonass_Gnav_Navigation_Message();
};

#endif

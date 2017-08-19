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
#include <map>
#include <string>
#include <utility>
#include <vector>
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
    unsigned int frame_ID;

    Glonass_Gnav_Ephemeris gnav_ephemeris;      //!< Ephemeris information decoded
    Glonass_Gnav_Utc_Model gnav_utc_model;      //!< UTC model information
    Glonass_Gnav_Almanac gnav_almanac[24];      //!< Almanac information for all 24 satellites

    //!< Satellite Identification
    int i_channel_ID;               //!< Channel ID assigned by the receiver
    unsigned int i_satellite_freq_channel;   //!< SV Frequency Slot Number
    unsigned int i_satellite_slot_number;    //!< SV Orbit Slot Number

    //!< Ephmeris Flags
    bool flag_all_ephemeris;  //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_str_1;    //!< Flag indicating that ephemeris 1/4 (string 1) have been received
    bool flag_ephemeris_str_2;    //!< Flag indicating that ephemeris 2/4 (string 2) have been received
    bool flag_ephemeris_str_3;    //!< Flag indicating that ephemeris 3/4 (string 3) have been received
    bool flag_ephemeris_str_4;    //!< Flag indicating that ephemeris 4/4 (string 4) have been received

    //!< Almanac Flags
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

    //!< UTC and System Clocks Flags
    bool flag_utc_model_valid;      //!< If set, it indicates that the UTC model parameters are filled
    bool flag_utc_model_str_5;      //!< Clock info send in string 5 of navigation data
    bool flag_utc_model_str_15;     //!< Clock info send in string 15 of frame 5 of navigation data
    bool flag_TOW_5;
    bool flag_TOW_6;
    bool flag_TOW_set;              //!< it is true when page 5 or page 6 arrives

    //broadcast orbit 1
    //TODO Need to send the information regarding the frame number
    double d_TOW;           //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    double d_TOW_F1;        //!< Time of GPS Week from HOW word of Subframe 1 [s]
    double d_TOW_F2;        //!< Time of GPS Week from HOW word of Subframe 2 [s]
    double d_TOW_F3;        //!< Time of GPS Week from HOW word of Subframe 3 [s]
    double d_TOW_F4;        //!< Time of GPS Week from HOW word of Subframe 4 [s]
    double d_TOW_F5;        //!< Time of GPS Week from HOW word of Subframe 5 [s]

    // Clock terms
    double d_satClkCorr;     // Satellite clock error
    double d_dtr;            // Relativistic clock correction term
    double d_satClkDrift;    // Satellite clock drift

    bool CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS> bits);

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

    /*
     * \brief Returns a Galileo_Almanac object filled with the latest navigation data received
     */
    Glonass_Gnav_Almanac get_almanac(int satellite_slot_number);

    /*
     * \brief Returns true if new Ephemeris has arrived. The flag is set to false when the function is executed
     */
    bool have_new_ephemeris();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_utc_model();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_almanac();

    /*!
     * \brief Decodes the GLONASS GNAV string
     */
    int string_decoder(char *string);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and returns it in [s]
     */
    double utc_time(const double glonasstime_corrected) const;

    /*!
     * Default constructor
     */
    Glonass_Gnav_Navigation_Message();
};

#endif

/*!
 * \file glonass_gnav_navigation_message.h
 * \brief  Interface of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
 * See http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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
//#include "gps_iono.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_utc_model.h"



/*!
 * \brief This class decodes a GLONASS GNAV Data message as described in GLONASS ICD (Edition 5.1)
 *
 * See http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdfdescribed in IS-GPS-200E
 *
 */
class Glonass_Gnav_Navigation_Message
{
private:
    unsigned long int read_navigation_unsigned(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    signed long int read_navigation_signed(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    bool read_navigation_bool(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter);
    bool _CRC_test(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits,boost::uint32_t checksum);

    unsigned int get_frame_number(unsigned int satellite_slot_number);

public:
    bool b_valid_ephemeris_set_flag; // flag indicating that this ephemeris set have passed the validation check
    int Page_type_time_stamp;
    int flag_even_word;
    std::string page_Even;
    bool flag_CRC_test;

    Glonass_Gnav_Ephemeris gnav_ephemeris;      //!< Ephemeris information decoded
    Glonass_Gnav_Iono gnav_iono;                //!< Iono corrections information
    Glonass_Gnav_Utc_Model gnav_utc_model;      //!< UTC model information
    Glonass_Gnav_Almanac gnav_almanac[24];      //!< Almanac information for all 24 satellites

    //!< Ephmeris Flags
    bool flag_all_ephemeris;  //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_str_1;    //!< Flag indicating that ephemeris 1/4 (word 1) have been received
    bool flag_ephemeris_str_2;    //!< Flag indicating that ephemeris 2/4 (word 2) have been received
    bool flag_ephemeris_str_3;    //!< Flag indicating that ephemeris 3/4 (word 3) have been received
    bool flag_ephemeris_str_4;    //!< Flag indicating that ephemeris 4/4 (word 4) have been received

    bool flag_iono_and_GST;   //!< Flag indicating that ionospheric and GST parameters (word 5) have been received
    bool flag_TOW_5;
    bool flag_TOW_6;
    bool flag_TOW_set;        //!< it is true when page 5 or page 6 arrives
    bool flag_utc_model;      //!< Flag indicating that utc model parameters (word 6) have been received

    bool flag_all_almanac;        //!< Flag indicating that all almanac have been received
    bool flag_almanac_str_6;      //!< Flag indicating that almanac 1/4 (word 7) have been received
    bool flag_almanac_str_7;      //!< Flag indicating that almanac 2/4 (word 8) have been received
    bool flag_almanac_str_8;      //!< Flag indicating that almanac 3/4 (word 9) have been received
    bool flag_almanac_str_9;      //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_10;     //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_11;     //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_12;     //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_13;     //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_14;     //!< Flag indicating that almanac 4/4 (word 10) have been received
    bool flag_almanac_str_15;     //!< Flag indicating that almanac 4/4 (word 10) have been received

    //broadcast orbit 1
    //TODO Need to send the information regarding the frame number
    double d_TOW; //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    double d_TOW_F1;            //!< Time of GPS Week from HOW word of Subframe 1 [s]
    double d_TOW_F2;            //!< Time of GPS Week from HOW word of Subframe 2 [s]
    double d_TOW_F3;            //!< Time of GPS Week from HOW word of Subframe 3 [s]
    double d_TOW_F4;            //!< Time of GPS Week from HOW word of Subframe 4 [s]
    double d_TOW_F5;            //!< Time of GPS Week from HOW word of Subframe 5 [s]

    // Clock terms
    double d_satClkCorr;     // Satellite clock error
    double d_dtr;            // Relativistic clock correction term
    double d_satClkDrift;    // Satellite clock drift

    // satellite identification info
    int i_channel_ID;
    unsigned int i_satellite_PRN;   //!< SV PRN Number
    int i_satellite_freq_channel;   //!< SV Frequency Slot Number
    int i_satellite_slot_number;    //!< SV Orbit Slot Number

    // UTC parameters
    bool flag_utc_model_valid; //!< If set, it indicates that the UTC model parameters are filled

    // satellite positions
    double d_satpos_X;        //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    double d_satpos_Y;        //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    double d_satpos_Z;        //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    // Satellite velocity
    double d_satvel_X;        //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Y;        //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Z;        //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
    // Satellite acceleration
    double d_satacc_X;        //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
    double d_satacc_Y;        //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
    double d_satacc_Z;        //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]

    /*!
     * \brief Reset GLONASS GNAV Navigation Information
     */
    void reset();

    /*!
     * \brief Obtain a GLONASS GNAV SV Ephemeris class filled with current SV data
     */
    Glonass_Gnav_Ephemeris get_ephemeris();

    /*!
    // TODO Should I keep this function ?
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Glonass_Gnav_Iono get_iono();

    /*!
     * \brief Obtain a GLONASS GNAV UTC model parameters class filled with current SV data
     */
    Glonass_Gnav_Utc_Model get_utc_model();

    /*
     * \brief Returns a Galileo_Almanac object filled with the latest navigation data received
     */
    Glonass_Gnav_Almanac get_almanac();

    /*
     * \brief Returns true if new Ephemeris has arrived. The flag is set to false when the function is executed
     */
    bool have_new_ephemeris();

    /*
     * \brief Returns true if new Iono model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_iono();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_utc_model();

    /*
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_almanac();



    /*!
     * \brief Decodes the GLONASS GNAV frame
     */
    int string_decoder(char *string, int frame_ID);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and returns it in [s]
     */
    double utc_time(const double gpstime_corrected) const;

    /*!
     * Default constructor
     */
    Glonass_Gnav_Navigation_Message();
};

#endif

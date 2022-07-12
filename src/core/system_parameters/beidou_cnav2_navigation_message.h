/*!
 * \file beidou_cnav2_navigation_message.h
 * \brief  Interface of a BEIDOU CNAV2 Data message decoder as described in BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_BEIDOU_CNAV2_NAVIGATION_MESSAGE_H_


#include "Beidou_B2a.h"
#include "beidou_cnav2_almanac.h"
#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_iono.h"
#include "beidou_cnav2_utc_model.h"
#include <bitset>
#include <cstdint>


/*!
 * \brief This class decodes a BEIDOU cnav2 Data message as described in BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
 */
class Beidou_Cnav2_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Beidou_Cnav2_Navigation_Message();

    bool flag_crc_test;         //!< Flag indicating CRC test
    uint32_t i_frame_mes_type;  //!< Flag indicating MesType
    int32_t i_alm_satellite_PRN;
    int32_t i_channel_ID;      //!< PRN of the channel
    uint32_t i_satellite_PRN;  //!< Satellite PRN

    Beidou_Cnav2_Ephemeris cnav2_ephemeris;                     //!< Ephemeris information decoded
    Beidou_Cnav2_Utc_Model cnav2_utc_model;                     //!< UTC model information
    Beidou_Cnav2_Iono cnav2_iono;                               //!< UTC model information
    Beidou_Cnav2_Almanac cnav2_almanac[BEIDOU_CNAV2_NBR_SATS];  //!< Almanac information for all 63 satellites

    // Ephemeris Flags and control variables
    bool flag_all_ephemeris;          //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_mes_type_10;  //!< Flag indicating that ephemeris 1/2 (Type 10) have been received
    bool flag_ephemeris_mes_type_11;  //!< Flag indicating that ephemeris 2/2 (Type 11) have been received
    bool flag_ephemeris_mes_type_30;  //!< Flag indicating that (Type 30) have been received
    bool flag_ephemeris_mes_type_31;  //!< Flag indicating that (Type 31) have been received
    bool flag_ephemeris_mes_type_32;  //!< Flag indicating that (Type 32) have been received
    bool flag_ephemeris_mes_type_34;  //!< Flag indicating that (Type 34) have been received
    bool flag_ephemeris_mes_type_40;  //!< Flag indicating that (Type 40) have been received

    // Almanac Flags
    bool flag_almanac_mes_type_31;  //!< Flag indicating that almanac of Type 31 have been received
    bool flag_almanac_mes_type_33;  //!< Flag indicating that almanac of Type 33 have been received
    bool flag_almanac_mes_type_40;  //!< Flag indicating that almanac of Type 40 have been received

    // UTC and System Clocks Flags
    bool flag_utc_model_valid;        //!< If set, it indicates that the UTC model parameters are filled
    bool flag_utc_model_mes_type_32;  //!< If set, it indicates that the UTC model parameters of Type 32 have been received
    bool flag_utc_model_mes_type_33;  //!< If set, it indicates that the UTC model parameters of Type 33 have been received
    bool flag_utc_model_mes_type_34;  //!< If set, it indicates that the UTC model parameters of Type 34 have been received

    // Iono Flgas
    bool flag_iono_valid;        //!< If set, it indicates that the UTC model parameters are filled
    bool flag_iono_mes_type_30;  //!< If set, it indicates that the UTC model parameters of Type 32 have been received


    bool flag_TOW_set;  //!< Flag indicating when the TOW has been set
    bool flag_TOW_new;  //!< Flag indicating when a new TOW has been computed
    bool flag_TOW_10;
    bool flag_TOW_11;
    bool flag_TOW_30;
    bool flag_TOW_31;
    bool flag_TOW_32;
    bool flag_TOW_33;
    bool flag_TOW_34;
    bool flag_TOW_40;

    double d_satClkCorr;   //!<  Satellite clock error
    double d_dtr;          //!<  Relativistic clock correction term
    double d_satClkDrift;  //!<  Satellite clock drift

    double d_previous_tb;                         //!< Previous iode for the Beidou_Cnav2_Ephemeris object. Used to determine when new data arrives
    double d_previous_Na[BEIDOU_CNAV2_NBR_SATS];  //!< Previous time for almanac of the Beidou_Cnav2_Almanac object

    double temp;  //!< Temporary value

    double crc_compute;  //!< Value of the computed CRC

    /*!
     * \brief Compute CRC for BEIDOU CNAV2 strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool crc_test(std::bitset<BEIDOU_CNAV2_DATA_BITS> bits, uint32_t crc_decoded);

    /*!
     * \brief Computes the frame number being decoded given the satellite slot number
     * \param satellite_slot_number [in] Satellite slot number identifier
     * \returns Frame number being decoded, 0 if operation was not successful.
     */
    uint32_t get_frame_number(uint32_t satellite_slot_number);

    /*!
     * \brief Reset BEIDOU CNAV2 Navigation Information
     */
    void reset();

    /*!
     * \brief Obtain a BEIDOU CNAV2 SV Ephemeris class filled with current SV data
     */
    Beidou_Cnav2_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a BEIDOU CNAV2 UTC model parameters class filled with current SV data
     */
    Beidou_Cnav2_Utc_Model get_utc_model();

    /*!
     * \brief Obtain a BEIDOU CNAV2 Iono model parameters class filled with current SV data
     */
    Beidou_Cnav2_Iono get_iono();

    /*!
     * \brief Returns a Beidou_Cnav2_Almanac object filled with the latest navigation data received
     * \param satellite_slot_number Slot number identifier for the satellite
     * \returns Returns the Beidou_Cnav2_Almanac object for the input slot number
     */
    Beidou_Cnav2_Almanac get_almanac(uint32_t satellite_slot_number);

    /*!
     * \brief Returns true if a new Beidou_Cnav2_Ephemeris object has arrived.
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Beidou_Cnav2_Utc_Model object has arrived
     */
    bool have_new_utc_model();

    /*!
     * \brief Returns true if new Beidou_Cnav2_Iono object has arrived
     */
    bool have_new_iono();

    /*!
     * \brief Returns true if new Beidou_Cnav2_Almanac object has arrived.
     */
    bool have_new_almanac();

    /*!
     * \brief Decodes the BEIDOU CNAV2 string
     * \param frame_string [in] is the string message within the parsed frame
     * \returns Returns the ID of the decoded string
     */
    int32_t frame_decoder(std::string const &frame_string);

private:
    uint64_t read_navigation_unsigned(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
    int64_t read_navigation_signed(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
    bool read_navigation_bool(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
};

#endif
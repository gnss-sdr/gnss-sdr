/*!
 * \file galileo_fnav_message.h
 * \brief  Implementation of a Galileo F/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 1.1 (Sept. 2010)
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *         <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_fnav_message.h"
#include <boost/crc.hpp>  // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include <iostream>


typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_Galileo_FNAV_type;

void Galileo_Fnav_Message::reset()
{
    flag_CRC_test = false;
    flag_all_ephemeris = false;  //!< Flag indicating that all words containing ephemeris have been received
    flag_ephemeris_1 = false;    //!< Flag indicating that ephemeris 1/3 (word 2) have been received
    flag_ephemeris_2 = false;    //!< Flag indicating that ephemeris 2/3 (word 3) have been received
    flag_ephemeris_3 = false;    //!< Flag indicating that ephemeris 3/3 (word 4) have been received

    flag_iono_and_GST = false;  //!< Flag indicating that ionospheric and GST parameters (word 1) have been received
    flag_TOW_1 = false;
    flag_TOW_2 = false;
    flag_TOW_3 = false;
    flag_TOW_4 = false;
    flag_TOW_set = false;    //!< it is true when page 1,2,3 or 4 arrives
    flag_utc_model = false;  //!< Flag indicating that utc model parameters (word 4) have been received

    flag_all_almanac = false;  //!< Flag indicating that all almanac have been received
    flag_almanac_1 = false;    //!< Flag indicating that almanac 1/2 (word 5) have been received
    flag_almanac_2 = false;    //!< Flag indicating that almanac 2/2 (word 6) have been received

    IOD_ephemeris = 0;

    page_type = 0;
    /* WORD 1 SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal
     * health and Data validity status*/
    FNAV_SV_ID_PRN_1 = 0;
    FNAV_IODnav_1 = -1;
    FNAV_t0c_1 = 0;
    FNAV_af0_1 = 0;
    FNAV_af1_1 = 0;
    FNAV_af2_1 = 0;
    FNAV_SISA_1 = 0;
    FNAV_ai0_1 = 0;
    FNAV_ai1_1 = 0;
    FNAV_ai2_1 = 0;
    FNAV_region1_1 = 0;
    FNAV_region2_1 = 0;
    FNAV_region3_1 = 0;
    FNAV_region4_1 = 0;
    FNAV_region5_1 = 0;
    FNAV_BGD_1 = 0;
    FNAV_E5ahs_1 = 0;
    FNAV_WN_1 = 0;
    FNAV_TOW_1 = 0;
    FNAV_E5advs_1 = 0;

    // WORD 2 Ephemeris (1/3) and GST
    FNAV_IODnav_2 = -2;
    FNAV_M0_2 = 0;
    FNAV_omegadot_2 = 0;
    FNAV_e_2 = 0;
    FNAV_a12_2 = 0;
    FNAV_omega0_2 = 0;
    FNAV_idot_2 = 0;
    FNAV_WN_2 = 0;
    FNAV_TOW_2 = 0;

    // WORD 3 Ephemeris (2/3) and GST
    FNAV_IODnav_3 = -3;
    FNAV_i0_3 = 0;
    FNAV_w_3 = 0;
    FNAV_deltan_3 = 0;
    FNAV_Cuc_3 = 0;
    FNAV_Cus_3 = 0;
    FNAV_Crc_3 = 0;
    FNAV_Crs_3 = 0;
    FNAV_t0e_3 = 0;
    FNAV_WN_3 = 0;
    FNAV_TOW_3 = 0;

    /* WORD 4 Ephemeris (3/3), GST-UTC conversion, GST-GPS conversion and TOW.
    Note that the clock is repeated in this page type*/
    FNAV_IODnav_4 = -4;
    FNAV_Cic_4 = 0;
    FNAV_Cis_4 = 0;
    FNAV_A0_4 = 0;
    FNAV_A1_4 = 0;
    FNAV_deltatls_4 = 0;
    FNAV_t0t_4 = 0;
    FNAV_WNot_4 = 0;
    FNAV_WNlsf_4 = 0;
    FNAV_DN_4 = 0;
    FNAV_deltatlsf_4 = 0;
    FNAV_t0g_4 = 0;
    FNAV_A0g_4 = 0;
    FNAV_A1g_4 = 0;
    FNAV_WN0g_4 = 0;
    FNAV_TOW_4 = 0;

    // WORD 5 Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
    FNAV_IODa_5 = 0;
    FNAV_WNa_5 = 0;
    FNAV_t0a_5 = 0;
    FNAV_SVID1_5 = 0;
    FNAV_Deltaa12_1_5 = 0;
    FNAV_e_1_5 = 0;
    FNAV_w_1_5 = 0;
    FNAV_deltai_1_5 = 0;
    FNAV_Omega0_1_5 = 0;
    FNAV_Omegadot_1_5 = 0;
    FNAV_M0_1_5 = 0;
    FNAV_af0_1_5 = 0;
    FNAV_af1_1_5 = 0;
    FNAV_E5ahs_1_5 = 0;
    FNAV_SVID2_5 = 0;
    FNAV_Deltaa12_2_5 = 0;
    FNAV_e_2_5 = 0;
    FNAV_w_2_5 = 0;
    FNAV_deltai_2_5 = 0;

    // WORD 6 Almanac (SVID2(2/2) and SVID3)
    FNAV_IODa_6 = 0;
    FNAV_Omega0_2_6 = 0;
    FNAV_Omegadot_2_6 = 0;
    FNAV_M0_2_6 = 0;
    FNAV_af0_2_6 = 0;
    FNAV_af1_2_6 = 0;
    FNAV_E5ahs_2_6 = 0;
    FNAV_SVID3_6 = 0;
    FNAV_Deltaa12_3_6 = 0;
    FNAV_e_3_6 = 0;
    FNAV_w_3_6 = 0;
    FNAV_deltai_3_6 = 0;
    FNAV_Omega0_3_6 = 0;
    FNAV_Omegadot_3_6 = 0;
    FNAV_M0_3_6 = 0;
    FNAV_af0_3_6 = 0;
    FNAV_af1_3_6 = 0;
    FNAV_E5ahs_3_6 = 0;
}

Galileo_Fnav_Message::Galileo_Fnav_Message()
{
    reset();
}

//int Galileo_Fnav_Message::toInt(std::string bitString)
//{
//    int tempInt;
//    int num=0;
//    int sLength = bitString.length();
//    for(int i=0; i<sLength; i++)
//    {
//        num |= (1 << (sLength-1-i))*tempInt;
//    }
//
//    return num;
//}

void Galileo_Fnav_Message::split_page(std::string page_string)
{
    std::string message_word = page_string.substr(0, 214);
    std::string CRC_data = page_string.substr(214, 24);
    std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> Word_for_CRC_bits(message_word);
    std::bitset<24> checksum(CRC_data);
    if (_CRC_test(Word_for_CRC_bits, checksum.to_ulong()) == true)
        {
            flag_CRC_test = true;
            // CRC correct: Decode word
            decode_page(message_word);
        }
    else
        {
            flag_CRC_test = false;
        }
}


bool Galileo_Fnav_Message::_CRC_test(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, boost::uint32_t checksum)
{
    CRC_Galileo_FNAV_type CRC_Galileo;

    boost::uint32_t crc_computed;
    // Galileo FNAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    // ToDo: Use boost::dynamic_bitset for all the bitset operations in this class

    boost::dynamic_bitset<unsigned char> frame_bits(std::string(bits.to_string()));

    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    CRC_Galileo.process_bytes(bytes.data(), GALILEO_FNAV_DATA_FRAME_BYTES);

    crc_computed = CRC_Galileo.checksum();
    if (checksum == crc_computed)
        {
            return true;
        }
    else
        {
            return false;
        }
}


void Galileo_Fnav_Message::decode_page(std::string data)
{
    std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> data_bits(data);
    page_type = read_navigation_unsigned(data_bits, FNAV_PAGE_TYPE_bit);
    switch (page_type)
        {
        case 1:  // SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal health and Data validity status
            FNAV_SV_ID_PRN_1 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_SV_ID_PRN_1_bit));
            FNAV_IODnav_1 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODnav_1_bit));
            FNAV_t0c_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_t0c_1_bit));
            FNAV_t0c_1 *= FNAV_t0c_1_LSB;
            FNAV_af0_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af0_1_bit));
            FNAV_af0_1 *= FNAV_af0_1_LSB;
            FNAV_af1_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af1_1_bit));
            FNAV_af1_1 *= FNAV_af1_1_LSB;
            FNAV_af2_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af2_1_bit));
            FNAV_af2_1 *= FNAV_af2_1_LSB;
            FNAV_SISA_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_SISA_1_bit));
            FNAV_ai0_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_ai0_1_bit));
            FNAV_ai0_1 *= FNAV_ai0_1_LSB;
            FNAV_ai1_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_ai1_1_bit));
            FNAV_ai1_1 *= FNAV_ai1_1_LSB;
            FNAV_ai2_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_ai2_1_bit));
            FNAV_ai2_1 *= FNAV_ai2_1_LSB;
            FNAV_region1_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_region1_1_bit));
            FNAV_region2_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_region2_1_bit));
            FNAV_region3_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_region3_1_bit));
            FNAV_region4_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_region4_1_bit));
            FNAV_region5_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_region5_1_bit));
            FNAV_BGD_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_BGD_1_bit));
            FNAV_BGD_1 *= FNAV_BGD_1_LSB;
            FNAV_E5ahs_1 = static_cast<unsigned int>(read_navigation_unsigned(data_bits, FNAV_E5ahs_1_bit));
            FNAV_WN_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WN_1_bit));
            FNAV_TOW_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_TOW_1_bit));
            FNAV_E5advs_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_E5advs_1_bit));

            flag_TOW_1 = true;
            flag_TOW_set = true;
            flag_iono_and_GST = true;  //set to false externally
            break;
        case 2:  // Ephemeris (1/3) and GST
            FNAV_IODnav_2 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODnav_2_bit));
            FNAV_M0_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_M0_2_bit));
            FNAV_M0_2 *= FNAV_M0_2_LSB;
            FNAV_omegadot_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_omegadot_2_bit));
            FNAV_omegadot_2 *= FNAV_omegadot_2_LSB;
            FNAV_e_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_e_2_bit));
            FNAV_e_2 *= FNAV_e_2_LSB;
            FNAV_a12_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_a12_2_bit));
            FNAV_a12_2 *= FNAV_a12_2_LSB;
            FNAV_omega0_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_omega0_2_bit));
            FNAV_omega0_2 *= FNAV_omega0_2_LSB;
            FNAV_idot_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_idot_2_bit));
            FNAV_idot_2 *= FNAV_idot_2_LSB;
            FNAV_WN_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WN_2_bit));
            FNAV_TOW_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_TOW_2_bit));

            flag_TOW_2 = true;
            flag_TOW_set = true;
            flag_ephemeris_1 = true;
            break;
        case 3:  // Ephemeris (2/3) and GST
            FNAV_IODnav_3 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODnav_3_bit));
            FNAV_i0_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_i0_3_bit));
            FNAV_i0_3 *= FNAV_i0_3_LSB;
            FNAV_w_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_w_3_bit));
            FNAV_w_3 *= FNAV_w_3_LSB;
            FNAV_deltan_3 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_deltan_3_bit));
            FNAV_deltan_3 *= FNAV_deltan_3_LSB;
            FNAV_Cuc_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Cuc_3_bit));
            FNAV_Cuc_3 *= FNAV_Cuc_3_LSB;
            FNAV_Cus_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Cus_3_bit));
            FNAV_Cus_3 *= FNAV_Cus_3_LSB;
            FNAV_Crc_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Crc_3_bit));
            FNAV_Crc_3 *= FNAV_Crc_3_LSB;
            FNAV_Crs_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Crs_3_bit));
            FNAV_Crs_3 *= FNAV_Crs_3_LSB;
            FNAV_t0e_3 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_t0e_3_bit));
            FNAV_t0e_3 *= FNAV_t0e_3_LSB;
            FNAV_WN_3 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WN_3_bit));
            FNAV_TOW_3 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_TOW_3_bit));

            flag_TOW_3 = true;
            flag_TOW_set = true;
            flag_ephemeris_2 = true;
            break;
        case 4:  // Ephemeris (3/3),  GST-UTC conversion,  GST-GPS conversion and TOW
            FNAV_IODnav_4 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODnav_4_bit));
            FNAV_Cic_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_Cic_4_bit));
            FNAV_Cic_4 *= FNAV_Cic_4_LSB;
            FNAV_Cis_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_Cis_4_bit));
            FNAV_Cis_4 *= FNAV_Cis_4_LSB;
            FNAV_A0_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_A0_4_bit));
            FNAV_A0_4 *= FNAV_A0_4_LSB;
            FNAV_A1_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_A1_4_bit));
            FNAV_A1_4 *= FNAV_A1_4_LSB;
            FNAV_deltatls_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_deltatls_4_bit));
            FNAV_t0t_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_t0t_4_bit));
            FNAV_t0t_4 *= FNAV_t0t_4_LSB;
            FNAV_WNot_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WNot_4_bit));
            FNAV_WNlsf_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WNlsf_4_bit));
            FNAV_DN_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_DN_4_bit));
            FNAV_deltatlsf_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_deltatlsf_4_bit));
            FNAV_t0g_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_t0g_4_bit));
            FNAV_t0g_4 *= FNAV_t0g_4_LSB;
            FNAV_A0g_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A0g_4_bit));
            FNAV_A0g_4 *= FNAV_A0g_4_LSB;
            FNAV_A1g_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A1g_4_bit));
            FNAV_A1g_4 *= FNAV_A1g_4_LSB;
            FNAV_WN0g_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WN0g_4_bit));
            FNAV_TOW_4 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_TOW_4_bit));

            flag_TOW_4 = true;
            flag_TOW_set = true;
            flag_ephemeris_3 = true;
            flag_utc_model = true;  //set to false externally
            break;
        case 5:  // Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
            FNAV_IODa_5 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODa_5_bit));
            FNAV_WNa_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_WNa_5_bit));
            FNAV_t0a_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_t0a_5_bit));
            FNAV_t0a_5 *= FNAV_t0a_5_LSB;
            FNAV_SVID1_5 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_SVID1_5_bit));
            FNAV_Deltaa12_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Deltaa12_1_5_bit));
            FNAV_Deltaa12_1_5 *= FNAV_Deltaa12_5_LSB;
            FNAV_e_1_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_e_1_5_bit));
            FNAV_e_1_5 *= FNAV_e_5_LSB;
            FNAV_w_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_w_1_5_bit));
            FNAV_w_1_5 *= FNAV_w_5_LSB;
            FNAV_deltai_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_deltai_1_5_bit));
            FNAV_deltai_1_5 *= FNAV_deltai_5_LSB;
            FNAV_Omega0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Omega0_1_5_bit));
            FNAV_Omega0_1_5 *= FNAV_Omega0_5_LSB;
            FNAV_Omegadot_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Omegadot_1_5_bit));
            FNAV_Omegadot_1_5 *= FNAV_Omegadot_5_LSB;
            FNAV_M0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_1_5_bit));
            FNAV_M0_1_5 *= FNAV_M0_5_LSB;
            FNAV_af0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af0_1_5_bit));
            FNAV_af0_1_5 *= FNAV_af0_5_LSB;
            FNAV_af1_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af1_1_5_bit));
            FNAV_af1_1_5 *= FNAV_af1_5_LSB;
            FNAV_E5ahs_1_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E5ahs_1_5_bit));
            FNAV_SVID2_5 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_SVID2_5_bit));
            FNAV_Deltaa12_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Deltaa12_2_5_bit));
            FNAV_Deltaa12_2_5 *= FNAV_Deltaa12_5_LSB;
            FNAV_e_2_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_e_2_5_bit));
            FNAV_e_2_5 *= FNAV_e_5_LSB;
            FNAV_w_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_w_2_5_bit));
            FNAV_w_2_5 *= FNAV_w_5_LSB;
            FNAV_deltai_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_deltai_2_5_bit));
            FNAV_deltai_2_5 *= FNAV_deltai_5_LSB;
            //TODO check this
            // Omega0_2 must be decoded when the two pieces are joined
            omega0_1 = data.substr(210, 4);
            //omega_flag=true;
            //
            //FNAV_Omega012_2_5=static_cast<double>(read_navigation_signed(data_bits, FNAV_Omega012_2_5_bit);

            flag_almanac_1 = true;
            break;
        case 6:  // Almanac (SVID2(2/2) and SVID3)
            FNAV_IODa_6 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_IODa_6_bit));

            /* Don't worry about omega pieces. If page 5 has not been received, all_ephemeris
         * flag will be set to false and the data won't be recorded.*/
            std::string omega0_2 = data.substr(10, 12);
            std::string Omega0 = omega0_1 + omega0_2;
            std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> omega_bits(Omega0);
            const std::vector<std::pair<int, int>> om_bit({{0, 12}});
            FNAV_Omega0_2_6 = static_cast<double>(read_navigation_signed(omega_bits, om_bit));
            FNAV_Omega0_2_6 *= FNAV_Omega0_5_LSB;
            //
            FNAV_Omegadot_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Omegadot_2_6_bit));
            FNAV_Omegadot_2_6 *= FNAV_Omegadot_5_LSB;
            FNAV_M0_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_2_6_bit));
            FNAV_M0_2_6 *= FNAV_M0_5_LSB;
            FNAV_af0_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af0_2_6_bit));
            FNAV_af0_2_6 *= FNAV_af0_5_LSB;
            FNAV_af1_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af1_2_6_bit));
            FNAV_af1_2_6 *= FNAV_af1_5_LSB;
            FNAV_E5ahs_2_6 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E5ahs_2_6_bit));
            FNAV_SVID3_6 = static_cast<int>(read_navigation_unsigned(data_bits, FNAV_SVID3_6_bit));
            FNAV_Deltaa12_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Deltaa12_3_6_bit));
            FNAV_Deltaa12_3_6 *= FNAV_Deltaa12_5_LSB;
            FNAV_e_3_6 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_e_3_6_bit));
            FNAV_e_3_6 *= FNAV_e_5_LSB;
            FNAV_w_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_w_3_6_bit));
            FNAV_w_3_6 *= FNAV_w_5_LSB;
            FNAV_deltai_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_deltai_3_6_bit));
            FNAV_deltai_3_6 *= FNAV_deltai_5_LSB;
            FNAV_Omega0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Omega0_3_6_bit));
            FNAV_Omega0_3_6 *= FNAV_Omega0_5_LSB;
            FNAV_Omegadot_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_Omegadot_3_6_bit));
            FNAV_Omegadot_3_6 *= FNAV_Omegadot_5_LSB;
            FNAV_M0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_3_6_bit));
            FNAV_M0_3_6 *= FNAV_M0_5_LSB;
            FNAV_af0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af0_3_6_bit));
            FNAV_af0_3_6 *= FNAV_af0_5_LSB;
            FNAV_af1_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_af1_3_6_bit));
            FNAV_af1_3_6 *= FNAV_af1_5_LSB;
            FNAV_E5ahs_3_6 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E5ahs_3_6_bit));

            flag_almanac_2 = true;
            break;
        }
}


uint64_t Galileo_Fnav_Message::read_navigation_unsigned(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    uint64_t value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Galileo_Fnav_Message::read_navigation_signed(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    int64_t value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(int64_t);
    if (long_int_size_bytes == 8)  // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFFFFFFFFFF;  //64 bits variable
                }
            else
                {
                    value &= 0;
                }

            for (int i = 0; i < num_of_slices; i++)
                {
                    for (int j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1;                  //shift left
                            value &= 0xFFFFFFFFFFFFFFFE;  //reset the corresponding bit (for the 64 bits variable)
                            if (bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFF;
                }
            else
                {
                    value &= 0;
                }

            for (int i = 0; i < num_of_slices; i++)
                {
                    for (int j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1;          //shift left
                            value &= 0xFFFFFFFE;  //reset the corresponding bit
                            if (bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    return value;
}


bool Galileo_Fnav_Message::have_new_ephemeris()  //Check if we have a new ephemeris stored in the galileo navigation class
{
    if ((flag_ephemeris_1 == true) and (flag_ephemeris_2 == true) and (flag_ephemeris_3 == true) and (flag_iono_and_GST == true))
        {
            //if all ephemeris pages have the same IOD, then they belong to the same block
            if ((FNAV_IODnav_1 == FNAV_IODnav_2) and (FNAV_IODnav_3 == FNAV_IODnav_4) and (FNAV_IODnav_1 == FNAV_IODnav_3))
                {
                    std::cout << "Ephemeris (1, 2, 3) have been received and belong to the same batch" << std::endl;
                    flag_ephemeris_1 = false;  // clear the flag
                    flag_ephemeris_2 = false;  // clear the flag
                    flag_ephemeris_3 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    IOD_ephemeris = FNAV_IODnav_1;
                    std::cout << "Batch number: " << IOD_ephemeris << std::endl;
                    return true;
                }
            else
                {
                    return false;
                }
        }
    else
        return false;
}


bool Galileo_Fnav_Message::have_new_iono_and_GST()  //Check if we have a new iono data set stored in the galileo navigation class
{
    if ((flag_iono_and_GST == true) and (flag_utc_model == true))  //the condition on flag_utc_model is added to have a time stamp for iono
        {
            flag_iono_and_GST = false;  // clear the flag
            return true;
        }
    else
        return false;
}


bool Galileo_Fnav_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model == true)
        {
            flag_utc_model = false;  // clear the flag
            return true;
        }
    else
        return false;
}


bool Galileo_Fnav_Message::have_new_almanac()  //Check if we have a new almanac data set stored in the galileo navigation class
{
    if ((flag_almanac_1 == true) and (flag_almanac_2 == true))
        {
            //All almanac have been received
            flag_almanac_1 = false;
            flag_almanac_2 = false;
            flag_all_almanac = true;
            return true;
        }
    else
        return false;
}


Galileo_Ephemeris Galileo_Fnav_Message::get_ephemeris()
{
    Galileo_Ephemeris ephemeris;
    ephemeris.flag_all_ephemeris = flag_all_ephemeris;
    ephemeris.IOD_ephemeris = IOD_ephemeris;
    ephemeris.SV_ID_PRN_4 = FNAV_SV_ID_PRN_1;
    ephemeris.i_satellite_PRN = FNAV_SV_ID_PRN_1;
    ephemeris.M0_1 = FNAV_M0_2;               // Mean anomaly at reference time [semi-circles]
    ephemeris.delta_n_3 = FNAV_deltan_3;      // Mean motion difference from computed value  [semi-circles/sec]
    ephemeris.e_1 = FNAV_e_2;                 // Eccentricity
    ephemeris.A_1 = FNAV_a12_2;               // Square root of the semi-major axis [meters^1/2]
    ephemeris.OMEGA_0_2 = FNAV_omega0_2;      // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    ephemeris.i_0_2 = FNAV_i0_3;              // Inclination angle at reference time  [semi-circles]
    ephemeris.omega_2 = FNAV_w_3;             // Argument of perigee [semi-circles]
    ephemeris.OMEGA_dot_3 = FNAV_omegadot_2;  // Rate of right ascension [semi-circles/sec]
    ephemeris.iDot_2 = FNAV_idot_2;           // Rate of inclination angle [semi-circles/sec]
    ephemeris.C_uc_3 = FNAV_Cuc_3;            // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    ephemeris.C_us_3 = FNAV_Cus_3;            // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    ephemeris.C_rc_3 = FNAV_Crc_3;            // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    ephemeris.C_rs_3 = FNAV_Crs_3;            // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    ephemeris.C_ic_4 = FNAV_Cic_4;            // Amplitude of the cosine harmonic correction     term to the angle of inclination [radians]
    ephemeris.C_is_4 = FNAV_Cis_4;            // Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    ephemeris.t0e_1 = FNAV_t0e_3;             // Ephemeris reference time [s]

    /*Clock correction parameters*/
    ephemeris.t0c_4 = FNAV_t0c_1;  // Clock correction data reference Time of Week [sec]
    ephemeris.af0_4 = FNAV_af0_1;  // SV clock bias correction coefficient [s]
    ephemeris.af1_4 = FNAV_af1_1;  // SV clock drift correction coefficient [s/s]
    ephemeris.af2_4 = FNAV_af2_1;  // SV clock drift rate correction coefficient [s/s^2]

    /*GST*/
    ephemeris.WN_5 = FNAV_WN_3;    // Week number
    ephemeris.TOW_5 = FNAV_TOW_3;  // Time of Week

    /* Health status */
    ephemeris.E5a_HS = FNAV_E5ahs_1;
    ephemeris.E5a_DVS = FNAV_E5advs_1;
    return ephemeris;
}


Galileo_Iono Galileo_Fnav_Message::get_iono()
{
    Galileo_Iono iono;
    /*Ionospheric correction*/
    /*Az*/
    iono.ai0_5 = FNAV_ai0_1;  // Effective Ionisation Level 1st order parameter [sfu]
    iono.ai1_5 = FNAV_ai1_1;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    iono.ai2_5 = FNAV_ai2_1;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    /*Ionospheric disturbance flag*/
    iono.Region1_flag_5 = FNAV_region1_1;  // Ionospheric Disturbance Flag for region 1
    iono.Region2_flag_5 = FNAV_region2_1;  // Ionospheric Disturbance Flag for region 2
    iono.Region3_flag_5 = FNAV_region3_1;  // Ionospheric Disturbance Flag for region 3
    iono.Region4_flag_5 = FNAV_region4_1;  // Ionospheric Disturbance Flag for region 4
    iono.Region5_flag_5 = FNAV_region5_1;  // Ionospheric Disturbance Flag for region 5

    /*GST*/
    iono.TOW_5 = FNAV_TOW_1;
    iono.WN_5 = FNAV_WN_1;
    return iono;
}


Galileo_Utc_Model Galileo_Fnav_Message::get_utc_model()
{
    Galileo_Utc_Model utc_model;
    //Gal_utc_model.valid = flag_utc_model_valid;
    /*Word type 6: GST-UTC conversion parameters*/
    utc_model.A0_6 = FNAV_A0_4;
    utc_model.A1_6 = FNAV_A1_4;
    utc_model.Delta_tLS_6 = FNAV_deltatls_4;
    utc_model.t0t_6 = FNAV_t0t_4;
    utc_model.WNot_6 = FNAV_WNot_4;
    utc_model.WN_LSF_6 = FNAV_WNlsf_4;
    utc_model.DN_6 = FNAV_DN_4;
    utc_model.Delta_tLSF_6 = FNAV_deltatlsf_4;
    utc_model.flag_utc_model = flag_utc_model;
    /*GST*/
    //utc_model.WN_5 = WN_5; //Week number
    //utc_model.TOW_5 = WN_5; //Time of Week
    return utc_model;
}


Galileo_Almanac Galileo_Fnav_Message::get_almanac()
{
    Galileo_Almanac almanac;
    /*FNAV equivalent of INAV Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number*/
    almanac.IOD_a_7 = FNAV_IODa_5;
    almanac.WN_a_7 = FNAV_WNa_5;
    almanac.t0a_7 = FNAV_t0a_5;
    almanac.SVID1_7 = FNAV_SVID1_5;
    almanac.DELTA_A_7 = FNAV_Deltaa12_1_5;
    almanac.e_7 = FNAV_e_1_5;
    almanac.omega_7 = FNAV_w_1_5;
    almanac.delta_i_7 = FNAV_deltai_1_5;
    almanac.Omega0_7 = FNAV_Omega0_1_5;
    almanac.Omega_dot_7 = FNAV_Omegadot_1_5;
    almanac.M0_7 = FNAV_M0_1_5;

    /*FNAV equivalent of INAV Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
    almanac.IOD_a_8 = FNAV_IODa_5;
    almanac.af0_8 = FNAV_af0_1_5;
    almanac.af1_8 = FNAV_af1_1_5;
    almanac.E5a_HS_8 = FNAV_E5ahs_1_5;
    almanac.SVID2_8 = FNAV_SVID2_5;
    almanac.DELTA_A_8 = FNAV_Deltaa12_2_5;
    almanac.e_8 = FNAV_e_2_5;
    almanac.omega_8 = FNAV_w_2_5;
    almanac.delta_i_8 = FNAV_deltai_2_5;
    almanac.Omega0_8 = FNAV_Omega0_2_6;
    almanac.Omega_dot_8 = FNAV_Omegadot_2_6;

    /*FNAV equivalent of INAV Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)*/
    almanac.IOD_a_9 = FNAV_IODa_6;
    almanac.WN_a_9 = FNAV_WNa_5;
    almanac.t0a_9 = FNAV_t0a_5;
    almanac.M0_9 = FNAV_M0_2_6;
    almanac.af0_9 = FNAV_af0_2_6;
    almanac.af1_9 = FNAV_af1_2_6;
    almanac.E5a_HS_9 = FNAV_E5ahs_2_6;
    almanac.SVID3_9 = FNAV_SVID3_6;
    almanac.DELTA_A_9 = FNAV_Deltaa12_3_6;
    almanac.e_9 = FNAV_e_3_6;
    almanac.omega_9 = FNAV_w_3_6;
    almanac.delta_i_9 = FNAV_deltai_3_6;

    /*FNAV equivalent of INAV Word type 10: Almanac for SVID3 (2/2)*/
    almanac.IOD_a_10 = FNAV_IODa_6;
    almanac.Omega0_10 = FNAV_Omega0_3_6;
    almanac.Omega_dot_10 = FNAV_Omegadot_3_6;
    almanac.M0_10 = FNAV_M0_3_6;
    almanac.af0_10 = FNAV_af0_3_6;
    almanac.af1_10 = FNAV_af1_3_6;
    almanac.E5a_HS_10 = FNAV_E5ahs_3_6;
    return almanac;
}

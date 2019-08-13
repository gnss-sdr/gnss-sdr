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
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
#include <algorithm>  // for reverse
#include <iostream>   // for string, operator<<
#include <iterator>   // for back_insert_iterator

using CRC_Galileo_FNAV_type = boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false>;

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
    // WORD 1 SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal
    // health and Data validity status
    FNAV_SV_ID_PRN_1 = 0;
    FNAV_IODnav_1 = -1;
    FNAV_t0c_1 = 0;
    FNAV_af0_1 = 0.0;
    FNAV_af1_1 = 0.0;
    FNAV_af2_1 = 0.0;
    FNAV_SISA_1 = 0;
    FNAV_ai0_1 = 0.0;
    FNAV_ai1_1 = 0.0;
    FNAV_ai2_1 = 0.0;
    FNAV_region1_1 = false;
    FNAV_region2_1 = false;
    FNAV_region3_1 = false;
    FNAV_region4_1 = false;
    FNAV_region5_1 = false;
    FNAV_BGD_1 = 0.0;
    FNAV_E5ahs_1 = 0;
    FNAV_WN_1 = 0;
    FNAV_TOW_1 = 0;
    FNAV_E5advs_1 = false;

    // WORD 2 Ephemeris (1/3) and GST
    FNAV_IODnav_2 = -2;
    FNAV_M0_2 = 0.0;
    FNAV_omegadot_2 = 0.0;
    FNAV_e_2 = 0.0;
    FNAV_a12_2 = 0.0;
    FNAV_omega0_2 = 0.0;
    FNAV_idot_2 = 0.0;
    FNAV_WN_2 = 0;
    FNAV_TOW_2 = 0;

    // WORD 3 Ephemeris (2/3) and GST
    FNAV_IODnav_3 = -3;
    FNAV_i0_3 = 0.0;
    FNAV_w_3 = 0.0;
    FNAV_deltan_3 = 0.0;
    FNAV_Cuc_3 = 0.0;
    FNAV_Cus_3 = 0.0;
    FNAV_Crc_3 = 0.0;
    FNAV_Crs_3 = 0.0;
    FNAV_t0e_3 = 0;
    FNAV_WN_3 = 0;
    FNAV_TOW_3 = 0;

    // WORD 4 Ephemeris (3/3), GST-UTC conversion, GST-GPS conversion and TOW.
    // Note that the clock is repeated in this page type
    FNAV_IODnav_4 = -4;
    FNAV_Cic_4 = 0.0;
    FNAV_Cis_4 = 0.0;
    FNAV_A0_4 = 0.0;
    FNAV_A1_4 = 0.0;
    FNAV_deltatls_4 = 0;
    FNAV_t0t_4 = 0;
    FNAV_WNot_4 = 0;
    FNAV_WNlsf_4 = 0;
    FNAV_DN_4 = 0;
    FNAV_deltatlsf_4 = 0;
    FNAV_t0g_4 = 0;
    FNAV_A0g_4 = 0.0;
    FNAV_A1g_4 = 0.0;
    FNAV_WN0g_4 = 0;
    FNAV_TOW_4 = 0;

    // WORD 5 Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
    FNAV_IODa_5 = 0;
    FNAV_WNa_5 = 0;
    FNAV_t0a_5 = 0;
    FNAV_SVID1_5 = 0;
    FNAV_Deltaa12_1_5 = 0.0;
    FNAV_e_1_5 = 0.0;
    FNAV_w_1_5 = 0.0;
    FNAV_deltai_1_5 = 0.0;
    FNAV_Omega0_1_5 = 0.0;
    FNAV_Omegadot_1_5 = 0.0;
    FNAV_M0_1_5 = 0.0;
    FNAV_af0_1_5 = 0.0;
    FNAV_af1_1_5 = 0.0;
    FNAV_E5ahs_1_5 = 0U;
    FNAV_SVID2_5 = 0;
    FNAV_Deltaa12_2_5 = 0;
    FNAV_e_2_5 = 0.0;
    FNAV_w_2_5 = 0.0;
    FNAV_deltai_2_5 = 0.0;

    // WORD 6 Almanac (SVID2(2/2) and SVID3)
    FNAV_IODa_6 = 0;
    FNAV_Omega0_2_6 = 0.0;
    FNAV_Omegadot_2_6 = 0.0;
    FNAV_M0_2_6 = 0.0;
    FNAV_af0_2_6 = 0.0;
    FNAV_af1_2_6 = 0.0;
    FNAV_E5ahs_2_6 = 0;
    FNAV_SVID3_6 = 0;
    FNAV_Deltaa12_3_6 = 0.0;
    FNAV_e_3_6 = 0.0;
    FNAV_w_3_6 = 0.0;
    FNAV_deltai_3_6 = 0.0;
    FNAV_Omega0_3_6 = 0.0;
    FNAV_Omegadot_3_6 = 0.0;
    FNAV_M0_3_6 = 0.0;
    FNAV_af0_3_6 = 0.0;
    FNAV_af1_3_6 = 0.0;
    FNAV_E5ahs_3_6 = 0;
}


Galileo_Fnav_Message::Galileo_Fnav_Message()
{
    reset();
}


void Galileo_Fnav_Message::split_page(const std::string& page_string)
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


bool Galileo_Fnav_Message::_CRC_test(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, uint32_t checksum)
{
    CRC_Galileo_FNAV_type CRC_Galileo;

    uint32_t crc_computed;
    // Galileo FNAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    // ToDo: Use boost::dynamic_bitset for all the bitset operations in this class

    boost::dynamic_bitset<uint8_t> frame_bits(std::string(bits.to_string()));

    std::vector<uint8_t> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    CRC_Galileo.process_bytes(bytes.data(), GALILEO_FNAV_DATA_FRAME_BYTES);

    crc_computed = CRC_Galileo.checksum();
    if (checksum == crc_computed)
        {
            return true;
        }

    return false;
}


void Galileo_Fnav_Message::decode_page(const std::string& data)
{
    std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> data_bits(data);
    page_type = read_navigation_unsigned(data_bits, FNAV_PAGE_TYPE_BIT);
    switch (page_type)
        {
        case 1:  // SVID, Clock correction, SISA, Ionospheric correction, BGD, GST, Signal health and Data validity status
            FNAV_SV_ID_PRN_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_SV_ID_PRN_1_BIT));
            FNAV_IODnav_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DNAV_1_BIT));
            FNAV_t0c_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_T0C_1_BIT));
            FNAV_t0c_1 *= FNAV_T0C_1_LSB;
            FNAV_af0_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF0_1_BIT));
            FNAV_af0_1 *= FNAV_AF0_1_LSB;
            FNAV_af1_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF1_1_BIT));
            FNAV_af1_1 *= FNAV_AF1_1_LSB;
            FNAV_af2_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF2_1_BIT));
            FNAV_af2_1 *= FNAV_AF2_1_LSB;
            FNAV_SISA_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_SISA_1_BIT));
            FNAV_ai0_1 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_AI0_1_BIT));
            FNAV_ai0_1 *= FNAV_AI0_1_LSB;
            FNAV_ai1_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AI1_1_BIT));
            FNAV_ai1_1 *= FNAV_AI1_1_LSB;
            FNAV_ai2_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AI2_1_BIT));
            FNAV_ai2_1 *= FNAV_AI2_1_LSB;
            FNAV_region1_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_REGION1_1_BIT));
            FNAV_region2_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_REGION2_1_BIT));
            FNAV_region3_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_REGION3_1_BIT));
            FNAV_region4_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_REGION4_1_BIT));
            FNAV_region5_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_REGION5_1_BIT));
            FNAV_BGD_1 = static_cast<double>(read_navigation_signed(data_bits, FNAV_BGD_1_BIT));
            FNAV_BGD_1 *= FNAV_BGD_1_LSB;
            FNAV_E5ahs_1 = static_cast<uint32_t>(read_navigation_unsigned(data_bits, FNAV_E5AHS_1_BIT));
            FNAV_WN_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_WN_1_BIT));
            FNAV_TOW_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_TOW_1_BIT));
            FNAV_E5advs_1 = static_cast<bool>(read_navigation_unsigned(data_bits, FNAV_E5ADVS_1_BIT));
            flag_TOW_1 = true;
            flag_TOW_set = true;
            flag_iono_and_GST = true;  // set to false externally
            break;
        case 2:  // Ephemeris (1/3) and GST
            FNAV_IODnav_2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DNAV_2_BIT));
            FNAV_M0_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_2_BIT));
            FNAV_M0_2 *= FNAV_M0_2_LSB;
            FNAV_omegadot_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGADOT_2_BIT));
            FNAV_omegadot_2 *= FNAV_OMEGADOT_2_LSB;
            FNAV_e_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E_2_BIT));
            FNAV_e_2 *= FNAV_E_2_LSB;
            FNAV_a12_2 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_A12_2_BIT));
            FNAV_a12_2 *= FNAV_A12_2_LSB;
            FNAV_omega0_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGA0_2_BIT));
            FNAV_omega0_2 *= FNAV_OMEGA0_2_LSB;
            FNAV_idot_2 = static_cast<double>(read_navigation_signed(data_bits, FNAV_IDOT_2_BIT));
            FNAV_idot_2 *= FNAV_IDOT_2_LSB;
            FNAV_WN_2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_WN_2_BIT));
            FNAV_TOW_2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_TOW_2_BIT));
            flag_TOW_2 = true;
            flag_TOW_set = true;
            flag_ephemeris_1 = true;
            break;
        case 3:  // Ephemeris (2/3) and GST
            FNAV_IODnav_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DNAV_3_BIT));
            FNAV_i0_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_I0_3_BIT));
            FNAV_i0_3 *= FNAV_I0_3_LSB;
            FNAV_w_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_W_3_BIT));
            FNAV_w_3 *= FNAV_W_3_LSB;
            FNAV_deltan_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAN_3_BIT));
            FNAV_deltan_3 *= FNAV_DELTAN_3_LSB;
            FNAV_Cuc_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CUC_3_BIT));
            FNAV_Cuc_3 *= FNAV_CUC_3_LSB;
            FNAV_Cus_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CUS_3_BIT));
            FNAV_Cus_3 *= FNAV_CUS_3_LSB;
            FNAV_Crc_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CRC_3_BIT));
            FNAV_Crc_3 *= FNAV_CRC_3_LSB;
            FNAV_Crs_3 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CRS_3_BIT));
            FNAV_Crs_3 *= FNAV_CRS_3_LSB;
            FNAV_t0e_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_T0E_3_BIT));
            FNAV_t0e_3 *= FNAV_T0E_3_LSB;
            FNAV_WN_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_WN_3_BIT));
            FNAV_TOW_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_TOW_3_BIT));
            flag_TOW_3 = true;
            flag_TOW_set = true;
            flag_ephemeris_2 = true;
            break;
        case 4:  // Ephemeris (3/3),  GST-UTC conversion,  GST-GPS conversion and TOW
            FNAV_IODnav_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DNAV_4_BIT));
            FNAV_Cic_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CIC_4_BIT));
            FNAV_Cic_4 *= FNAV_CIC_4_LSB;
            FNAV_Cis_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_CIS_4_BIT));
            FNAV_Cis_4 *= FNAV_CIS_4_LSB;
            FNAV_A0_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A0_4_BIT));
            FNAV_A0_4 *= FNAV_A0_4_LSB;
            FNAV_A1_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A1_4_BIT));
            FNAV_A1_4 *= FNAV_A1_4_LSB;
            FNAV_deltatls_4 = static_cast<int32_t>(read_navigation_signed(data_bits, FNAV_DELTATLS_4_BIT));
            FNAV_t0t_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_T0T_4_BIT));
            FNAV_t0t_4 *= FNAV_T0T_4_LSB;
            FNAV_WNot_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_W_NOT_4_BIT));
            FNAV_WNlsf_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_W_NLSF_4_BIT));
            FNAV_DN_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_DN_4_BIT));
            FNAV_deltatlsf_4 = static_cast<int32_t>(read_navigation_signed(data_bits, FNAV_DELTATLSF_4_BIT));
            FNAV_t0g_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_T0G_4_BIT));
            FNAV_t0g_4 *= FNAV_T0G_4_LSB;
            FNAV_A0g_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A0G_4_BIT));
            FNAV_A0g_4 *= FNAV_A0G_4_LSB;
            FNAV_A1g_4 = static_cast<double>(read_navigation_signed(data_bits, FNAV_A1G_4_BIT));
            FNAV_A1g_4 *= FNAV_A1G_4_LSB;
            FNAV_WN0g_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_W_N0G_4_BIT));
            FNAV_TOW_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_TOW_4_BIT));
            flag_TOW_4 = true;
            flag_TOW_set = true;
            flag_ephemeris_3 = true;
            flag_utc_model = true;  // set to false externally
            break;
        case 5:  // Almanac (SVID1 and SVID2(1/2)), Week Number and almanac reference time
            FNAV_IODa_5 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DA_5_BIT));
            FNAV_WNa_5 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_W_NA_5_BIT));
            FNAV_t0a_5 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_T0A_5_BIT));
            FNAV_t0a_5 *= FNAV_T0A_5_LSB;
            FNAV_SVID1_5 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_SVI_D1_5_BIT));
            FNAV_Deltaa12_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAA12_1_5_BIT));
            FNAV_Deltaa12_1_5 *= FNAV_DELTAA12_5_LSB;
            FNAV_e_1_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E_1_5_BIT));
            FNAV_e_1_5 *= FNAV_E_5_LSB;
            FNAV_w_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_W_1_5_BIT));
            FNAV_w_1_5 *= FNAV_W_5_LSB;
            FNAV_deltai_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAI_1_5_BIT));
            FNAV_deltai_1_5 *= FNAV_DELTAI_5_LSB;
            FNAV_Omega0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGA0_1_5_BIT));
            FNAV_Omega0_1_5 *= FNAV_OMEGA0_5_LSB;
            FNAV_Omegadot_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGADOT_1_5_BIT));
            FNAV_Omegadot_1_5 *= FNAV_OMEGADOT_5_LSB;
            FNAV_M0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_1_5_BIT));
            FNAV_M0_1_5 *= FNAV_M0_5_LSB;
            FNAV_af0_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF0_1_5_BIT));
            FNAV_af0_1_5 *= FNAV_AF0_5_LSB;
            FNAV_af1_1_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF1_1_5_BIT));
            FNAV_af1_1_5 *= FNAV_AF1_5_LSB;
            FNAV_E5ahs_1_5 = static_cast<uint32_t>(read_navigation_unsigned(data_bits, FNAV_E5AHS_1_5_BIT));
            FNAV_SVID2_5 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_SVI_D2_5_BIT));
            FNAV_Deltaa12_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAA12_2_5_BIT));
            FNAV_Deltaa12_2_5 *= FNAV_DELTAA12_5_LSB;
            FNAV_e_2_5 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E_2_5_BIT));
            FNAV_e_2_5 *= FNAV_E_5_LSB;
            FNAV_w_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_W_2_5_BIT));
            FNAV_w_2_5 *= FNAV_W_5_LSB;
            FNAV_deltai_2_5 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAI_2_5_BIT));
            FNAV_deltai_2_5 *= FNAV_DELTAI_5_LSB;
            //TODO check this
            // Omega0_2 must be decoded when the two pieces are joined
            omega0_1 = data.substr(210, 4);
            //omega_flag=true;
            //
            //FNAV_Omega012_2_5=static_cast<double>(read_navigation_signed(data_bits, FNAV_Omega012_2_5_bit);
            flag_almanac_1 = true;
            break;
        case 6:  // Almanac (SVID2(2/2) and SVID3)
            FNAV_IODa_6 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_IO_DA_6_BIT));
            // Don't worry about omega pieces. If page 5 has not been received, all_ephemeris
            // flag will be set to false and the data won't be recorded.*/
            std::string omega0_2 = data.substr(10, 12);
            std::string Omega0 = omega0_1 + omega0_2;
            std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> omega_bits(Omega0);
            const std::vector<std::pair<int32_t, int32_t>> om_bit({{0, 12}});
            FNAV_Omega0_2_6 = static_cast<double>(read_navigation_signed(omega_bits, om_bit));
            FNAV_Omega0_2_6 *= FNAV_OMEGA0_5_LSB;
            FNAV_Omegadot_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGADOT_2_6_BIT));
            FNAV_Omegadot_2_6 *= FNAV_OMEGADOT_5_LSB;
            FNAV_M0_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_2_6_BIT));
            FNAV_M0_2_6 *= FNAV_M0_5_LSB;
            FNAV_af0_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF0_2_6_BIT));
            FNAV_af0_2_6 *= FNAV_AF0_5_LSB;
            FNAV_af1_2_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF1_2_6_BIT));
            FNAV_af1_2_6 *= FNAV_AF1_5_LSB;
            FNAV_E5ahs_2_6 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_E5AHS_2_6_BIT));
            FNAV_SVID3_6 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_SVI_D3_6_BIT));
            FNAV_Deltaa12_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAA12_3_6_BIT));
            FNAV_Deltaa12_3_6 *= FNAV_DELTAA12_5_LSB;
            FNAV_e_3_6 = static_cast<double>(read_navigation_unsigned(data_bits, FNAV_E_3_6_BIT));
            FNAV_e_3_6 *= FNAV_E_5_LSB;
            FNAV_w_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_W_3_6_BIT));
            FNAV_w_3_6 *= FNAV_W_5_LSB;
            FNAV_deltai_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_DELTAI_3_6_BIT));
            FNAV_deltai_3_6 *= FNAV_DELTAI_5_LSB;
            FNAV_Omega0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGA0_3_6_BIT));
            FNAV_Omega0_3_6 *= FNAV_OMEGA0_5_LSB;
            FNAV_Omegadot_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_OMEGADOT_3_6_BIT));
            FNAV_Omegadot_3_6 *= FNAV_OMEGADOT_5_LSB;
            FNAV_M0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_M0_3_6_BIT));
            FNAV_M0_3_6 *= FNAV_M0_5_LSB;
            FNAV_af0_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF0_3_6_BIT));
            FNAV_af0_3_6 *= FNAV_AF0_5_LSB;
            FNAV_af1_3_6 = static_cast<double>(read_navigation_signed(data_bits, FNAV_AF1_3_6_BIT));
            FNAV_af1_3_6 *= FNAV_AF1_5_LSB;
            FNAV_E5ahs_3_6 = static_cast<int32_t>(read_navigation_unsigned(data_bits, FNAV_E5AHS_3_6_BIT));

            flag_almanac_2 = true;
            break;
        }
}


uint64_t Galileo_Fnav_Message::read_navigation_unsigned(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    uint64_t value = 0ULL;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1U;  // shift left
                    if (static_cast<int>(bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Galileo_Fnav_Message::read_navigation_signed(std::bitset<GALILEO_FNAV_DATA_FRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    int64_t value = 0LL;
    int num_of_slices = parameter.size();

    // read the MSB and perform the sign extension
    if (static_cast<int>(bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[0].first]) == 1)
        {
            value ^= 0x0FFFFFFFFFFFFFFF;  // 64 bits variable
        }
    else
        {
            value &= 0;
        }

    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;                  // shift left
                    value &= 0xFFFFFFFFFFFFFFFE;  // reset the corresponding bit (for the 64 bits variable)
                    if (static_cast<int>(bits[GALILEO_FNAV_DATA_FRAME_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


bool Galileo_Fnav_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if ((flag_ephemeris_1 == true) and (flag_ephemeris_2 == true) and (flag_ephemeris_3 == true) and (flag_iono_and_GST == true))
        {
            // if all ephemeris pages have the same IOD, then they belong to the same block
            if ((FNAV_IODnav_1 == FNAV_IODnav_2) and (FNAV_IODnav_3 == FNAV_IODnav_4) and (FNAV_IODnav_1 == FNAV_IODnav_3))
                {
                    DLOG(INFO) << "Ephemeris (1, 2, 3) have been received and belong to the same batch";
                    flag_ephemeris_1 = false;  // clear the flag
                    flag_ephemeris_2 = false;  // clear the flag
                    flag_ephemeris_3 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    IOD_ephemeris = FNAV_IODnav_1;
                    DLOG(INFO) << "Batch number: " << IOD_ephemeris;
                    return true;
                }
        }
    return false;
}


bool Galileo_Fnav_Message::have_new_iono_and_GST()  // Check if we have a new iono data set stored in the galileo navigation class
{
    if ((flag_iono_and_GST == true) and (flag_utc_model == true))  // the condition on flag_utc_model is added to have a time stamp for iono
        {
            flag_iono_and_GST = false;  // clear the flag
        }
    else
        {
            return false;
        }
    return true;
}


bool Galileo_Fnav_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model == true)
        {
            flag_utc_model = false;  // clear the flag
        }
    else
        {
            return false;
        }
    return true;
}


bool Galileo_Fnav_Message::have_new_almanac()  // Check if we have a new almanac data set stored in the galileo navigation class
{
    if ((flag_almanac_1 == true) and (flag_almanac_2 == true))
        {
            // All almanac have been received
            flag_almanac_1 = false;
            flag_almanac_2 = false;
            flag_all_almanac = true;
        }
    else
        {
            return false;
        }
    return true;
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

    // Clock correction parameters
    ephemeris.t0c_4 = FNAV_t0c_1;  // Clock correction data reference Time of Week [sec]
    ephemeris.af0_4 = FNAV_af0_1;  // SV clock bias correction coefficient [s]
    ephemeris.af1_4 = FNAV_af1_1;  // SV clock drift correction coefficient [s/s]
    ephemeris.af2_4 = FNAV_af2_1;  // SV clock drift rate correction coefficient [s/s^2]

    // GST
    ephemeris.WN_5 = FNAV_WN_3;    // Week number
    ephemeris.TOW_5 = FNAV_TOW_3;  // Time of Week

    // Health status
    ephemeris.E5a_HS = FNAV_E5ahs_1;
    ephemeris.E5a_DVS = FNAV_E5advs_1;
    return ephemeris;
}


Galileo_Iono Galileo_Fnav_Message::get_iono()
{
    Galileo_Iono iono;
    // Ionospheric correction
    iono.ai0_5 = FNAV_ai0_1;  // Effective Ionisation Level 1st order parameter [sfu]
    iono.ai1_5 = FNAV_ai1_1;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    iono.ai2_5 = FNAV_ai2_1;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // Ionospheric disturbance flag
    iono.Region1_flag_5 = FNAV_region1_1;  // Ionospheric Disturbance Flag for region 1
    iono.Region2_flag_5 = FNAV_region2_1;  // Ionospheric Disturbance Flag for region 2
    iono.Region3_flag_5 = FNAV_region3_1;  // Ionospheric Disturbance Flag for region 3
    iono.Region4_flag_5 = FNAV_region4_1;  // Ionospheric Disturbance Flag for region 4
    iono.Region5_flag_5 = FNAV_region5_1;  // Ionospheric Disturbance Flag for region 5

    // GST
    iono.TOW_5 = FNAV_TOW_1;
    iono.WN_5 = FNAV_WN_1;
    return iono;
}


Galileo_Utc_Model Galileo_Fnav_Message::get_utc_model()
{
    Galileo_Utc_Model utc_model;
    // Word type 6: GST-UTC conversion parameters
    utc_model.A0_6 = FNAV_A0_4;
    utc_model.A1_6 = FNAV_A1_4;
    utc_model.Delta_tLS_6 = FNAV_deltatls_4;
    utc_model.t0t_6 = FNAV_t0t_4;
    utc_model.WNot_6 = FNAV_WNot_4;
    utc_model.WN_LSF_6 = FNAV_WNlsf_4;
    utc_model.DN_6 = FNAV_DN_4;
    utc_model.Delta_tLSF_6 = FNAV_deltatlsf_4;
    utc_model.flag_utc_model = flag_utc_model;
    // GST
    //utc_model.WN_5 = WN_5; //Week number
    //utc_model.TOW_5 = WN_5; //Time of Week
    return utc_model;
}


Galileo_Almanac_Helper Galileo_Fnav_Message::get_almanac()
{
    Galileo_Almanac_Helper almanac;
    // FNAV equivalent of INAV Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
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

    // FNAV equivalent of INAV Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
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

    // FNAV equivalent of INAV Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
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

    // FNAV equivalent of INAV Word type 10: Almanac for SVID3 (2/2)
    almanac.IOD_a_10 = FNAV_IODa_6;
    almanac.Omega0_10 = FNAV_Omega0_3_6;
    almanac.Omega_dot_10 = FNAV_Omegadot_3_6;
    almanac.M0_10 = FNAV_M0_3_6;
    almanac.af0_10 = FNAV_af0_3_6;
    almanac.af1_10 = FNAV_af1_3_6;
    almanac.E5a_HS_10 = FNAV_E5ahs_3_6;
    return almanac;
}

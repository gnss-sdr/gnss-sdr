/*!
 * \file galileo_inav_message.cc
 * \brief  Implementation of a Galileo I/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 1.1 (Sept. 2010)
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_inav_message.h"
#include <boost/crc.hpp>             // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>  // for boost::dynamic_bitset
#include <glog/logging.h>            // for DLOG
#include <algorithm>                 // for reverse
#include <iostream>                  // for operator<<
#include <limits>                    // for std::numeric_limits


using CRC_Galileo_INAV_type = boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false>;


bool Galileo_Inav_Message::CRC_test(std::bitset<GALILEO_DATA_FRAME_BITS> bits, uint32_t checksum) const
{
    CRC_Galileo_INAV_type CRC_Galileo;

    // Galileo INAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    // ToDo: Use boost::dynamic_bitset for all the bitset operations in this class

    boost::dynamic_bitset<unsigned char> frame_bits(std::string(bits.to_string()));

    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    CRC_Galileo.process_bytes(bytes.data(), GALILEO_DATA_FRAME_BYTES);

    const uint32_t crc_computed = CRC_Galileo.checksum();
    if (checksum == crc_computed)
        {
            return true;
        }
    return false;
}


uint64_t Galileo_Inav_Message::read_navigation_unsigned(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> >& parameter) const
{
    uint64_t value = 0ULL;
    const int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1U;  // shift left
                    if (static_cast<int>(bits[GALILEO_DATA_JK_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


uint64_t Galileo_Inav_Message::read_page_type_unsigned(std::bitset<GALILEO_PAGE_TYPE_BITS> bits, const std::vector<std::pair<int32_t, int32_t> >& parameter) const
{
    uint64_t value = 0ULL;
    const int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1U;  // shift left
                    if (static_cast<int>(bits[GALILEO_PAGE_TYPE_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1ULL;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Galileo_Inav_Message::read_navigation_signed(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> >& parameter) const
{
    int64_t value = 0LL;
    const int32_t num_of_slices = parameter.size();

    // read the MSB and perform the sign extension
    if (static_cast<int>(bits[GALILEO_DATA_JK_BITS - parameter[0].first]) == 1)
        {
            value ^= 0xFFFFFFFFFFFFFFFFLL;  // 64 bits variable
        }
    else
        {
            value &= 0LL;
        }

    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value *= 2;                   // shift left the signed integer
                    value &= 0xFFFFFFFFFFFFFFFE;  // reset the corresponding bit (for the 64 bits variable)
                    if (static_cast<int>(bits[GALILEO_DATA_JK_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1LL;  // insert the bit
                        }
                }
        }
    return value;
}


bool Galileo_Inav_Message::read_navigation_bool(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> >& parameter) const
{
    bool value;
    if (static_cast<int>(static_cast<int>(bits[GALILEO_DATA_JK_BITS - parameter[0].first])) == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


void Galileo_Inav_Message::split_page(std::string page_string, int32_t flag_even_word)
{
    int32_t Page_type = 0;

    if (page_string.at(0) == '1')  // if page is odd
        {
            const std::string& page_Odd = page_string;

            if (flag_even_word == 1)  // An odd page has been received but the previous even page is kept in memory and it is considered to join pages
                {
                    const std::string page_INAV_even = page_Even;
                    const std::string page_INAV = page_INAV_even + page_Odd;  // Join pages: Even + Odd = INAV page
                    const std::string Even_bit = page_INAV.substr(0, 1);
                    const std::string Page_type_even = page_INAV.substr(1, 1);
                    const std::string nominal = "0";

                    const std::string Data_k = page_INAV.substr(2, 112);
                    const std::string Odd_bit = page_INAV.substr(114, 1);
                    const std::string Page_type_Odd = page_INAV.substr(115, 1);
                    const std::string Data_j = page_INAV.substr(116, 16);

                    const std::string Reserved_1 = page_INAV.substr(132, 40);
                    const std::string SAR = page_INAV.substr(172, 22);
                    const std::string Spare = page_INAV.substr(194, 2);
                    const std::string CRC_data = page_INAV.substr(196, 24);
                    const std::string Reserved_2 = page_INAV.substr(220, 8);
                    const std::string Tail_odd = page_INAV.substr(228, 6);

                    // ************ CRC checksum control *******/
                    std::stringstream TLM_word_for_CRC_stream;
                    TLM_word_for_CRC_stream << page_INAV;
                    const std::string TLM_word_for_CRC = TLM_word_for_CRC_stream.str().substr(0, GALILEO_DATA_FRAME_BITS);
                    const std::bitset<GALILEO_DATA_FRAME_BITS> TLM_word_for_CRC_bits(TLM_word_for_CRC);
                    const std::bitset<24> checksum(CRC_data);

                    if (CRC_test(TLM_word_for_CRC_bits, checksum.to_ulong()) == true)
                        {
                            flag_CRC_test = true;
                            // CRC correct: Decode word
                            const std::string page_number_bits = Data_k.substr(0, 6);
                            const std::bitset<GALILEO_PAGE_TYPE_BITS> page_type_bits(page_number_bits);  // from string to bitset
                            Page_type = static_cast<int32_t>(read_page_type_unsigned(page_type_bits, TYPE));
                            Page_type_time_stamp = Page_type;
                            const std::string Data_jk_ephemeris = Data_k + Data_j;
                            page_jk_decoder(Data_jk_ephemeris.c_str());
                        }
                    else
                        {
                            // Wrong CRC... discard frame
                            flag_CRC_test = false;
                        }
                }  // end of CRC checksum control
        }          // end if (page_string.at(0)=='1')
    else
        {
            page_Even = page_string.substr(0, 114);
        }
}


bool Galileo_Inav_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if ((flag_ephemeris_1 == true) and (flag_ephemeris_2 == true) and (flag_ephemeris_3 == true) and (flag_ephemeris_4 == true) and (flag_iono_and_GST == true))
        {
            // if all ephemeris pages have the same IOD, then they belong to the same block
            if ((IOD_nav_1 == IOD_nav_2) and (IOD_nav_3 == IOD_nav_4) and (IOD_nav_1 == IOD_nav_3))
                {
                    DLOG(INFO) << "Ephemeris (1, 2, 3, 4) have been received and belong to the same batch";
                    flag_ephemeris_1 = false;  // clear the flag
                    flag_ephemeris_2 = false;  // clear the flag
                    flag_ephemeris_3 = false;  // clear the flag
                    flag_ephemeris_4 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    IOD_ephemeris = IOD_nav_1;
                    DLOG(INFO) << "Batch number: " << IOD_ephemeris;
                    return true;
                }
        }
    return false;
}


bool Galileo_Inav_Message::have_new_iono_and_GST()  // Check if we have a new iono data set stored in the galileo navigation class
{
    if ((flag_iono_and_GST == true) and (flag_utc_model == true))  // the condition on flag_utc_model is added to have a time stamp for iono
        {
            flag_iono_and_GST = false;  // clear the flag
            return true;
        }

    return false;
}


bool Galileo_Inav_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model == true)
        {
            flag_utc_model = false;  // clear the flag
            return true;
        }

    return false;
}


bool Galileo_Inav_Message::have_new_almanac()  // Check if we have a new almanac data set stored in the galileo navigation class
{
    if ((flag_almanac_1 == true) and (flag_almanac_2 == true) and (flag_almanac_3 == true) and (flag_almanac_4 == true))
        {
            // All Almanac data have been received
            flag_almanac_1 = false;
            flag_almanac_2 = false;
            flag_almanac_3 = false;
            flag_almanac_4 = false;
            flag_all_almanac = true;
            return true;
        }

    return false;
}


Galileo_Ephemeris Galileo_Inav_Message::get_ephemeris() const
{
    Galileo_Ephemeris ephemeris;
    ephemeris.flag_all_ephemeris = flag_all_ephemeris;
    ephemeris.IOD_ephemeris = IOD_ephemeris;
    ephemeris.SV_ID_PRN_4 = SV_ID_PRN_4;
    ephemeris.i_satellite_PRN = SV_ID_PRN_4;
    ephemeris.M0_1 = M0_1;                // Mean anomaly at reference time [semi-circles]
    ephemeris.delta_n_3 = delta_n_3;      // Mean motion difference from computed value  [semi-circles/sec]
    ephemeris.e_1 = e_1;                  // Eccentricity
    ephemeris.A_1 = A_1;                  // Square root of the semi-major axis [meters^1/2]
    ephemeris.OMEGA_0_2 = OMEGA_0_2;      // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    ephemeris.i_0_2 = i_0_2;              // Inclination angle at reference time  [semi-circles]
    ephemeris.omega_2 = omega_2;          // Argument of perigee [semi-circles]
    ephemeris.OMEGA_dot_3 = OMEGA_dot_3;  // Rate of right ascension [semi-circles/sec]
    ephemeris.iDot_2 = iDot_2;            // Rate of inclination angle [semi-circles/sec]
    ephemeris.C_uc_3 = C_uc_3;            // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    ephemeris.C_us_3 = C_us_3;            // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    ephemeris.C_rc_3 = C_rc_3;            // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    ephemeris.C_rs_3 = C_rs_3;            // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    ephemeris.C_ic_4 = C_ic_4;            // Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    ephemeris.C_is_4 = C_is_4;            // Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    ephemeris.t0e_1 = t0e_1;              // Ephemeris reference time [s]

    // Clock correction parameters
    ephemeris.t0c_4 = t0c_4;  // Clock correction data reference Time of Week [sec]
    ephemeris.af0_4 = af0_4;  // SV clock bias correction coefficient [s]
    ephemeris.af1_4 = af1_4;  // SV clock drift correction coefficient [s/s]
    ephemeris.af2_4 = af2_4;  // SV clock drift rate correction coefficient [s/s^2]

    // GST
    ephemeris.WN_5 = WN_5;    // Week number
    ephemeris.TOW_5 = TOW_5;  // Time of Week

    ephemeris.SISA_3 = SISA_3;
    ephemeris.E5b_HS_5 = E5b_HS_5;    // E5b Signal Health Status
    ephemeris.E1B_HS_5 = E1B_HS_5;    // E1B Signal Health Status
    ephemeris.E5b_DVS_5 = E5b_DVS_5;  // E5b Data Validity Status
    ephemeris.E1B_DVS_5 = E1B_DVS_5;  // E1B Data Validity Status

    ephemeris.BGD_E1E5a_5 = BGD_E1E5a_5;  // E1-E5a Broadcast Group Delay [s]
    ephemeris.BGD_E1E5b_5 = BGD_E1E5b_5;  // E1-E5b Broadcast Group Delay [s]

    ephemeris.Galileo_satClkDrift = Galileo_satClkDrift;

    return ephemeris;
}


Galileo_Iono Galileo_Inav_Message::get_iono() const
{
    Galileo_Iono iono;
    // Ionospheric correction
    iono.ai0_5 = ai0_5;  // Effective Ionisation Level 1st order parameter [sfu]
    iono.ai1_5 = ai1_5;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    iono.ai2_5 = ai2_5;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // GST
    // This is the ONLY page containing the Week Number (WN)
    iono.TOW_5 = TOW_5;
    iono.WN_5 = WN_5;

    // Ionospheric disturbance flag
    iono.Region1_flag_5 = Region1_flag_5;  // Ionospheric Disturbance Flag for region 1
    iono.Region2_flag_5 = Region2_flag_5;  // Ionospheric Disturbance Flag for region 2
    iono.Region3_flag_5 = Region3_flag_5;  // Ionospheric Disturbance Flag for region 3
    iono.Region4_flag_5 = Region4_flag_5;  // Ionospheric Disturbance Flag for region 4
    iono.Region5_flag_5 = Region5_flag_5;  // Ionospheric Disturbance Flag for region 5

    return iono;
}


Galileo_Utc_Model Galileo_Inav_Message::get_utc_model() const
{
    Galileo_Utc_Model utc_model;
    // Word type 6: GST-UTC conversion parameters
    utc_model.A0_6 = A0_6;
    utc_model.A1_6 = A1_6;
    utc_model.Delta_tLS_6 = Delta_tLS_6;
    utc_model.t0t_6 = t0t_6;
    utc_model.WNot_6 = WNot_6;
    utc_model.WN_LSF_6 = WN_LSF_6;
    utc_model.DN_6 = DN_6;
    utc_model.Delta_tLSF_6 = Delta_tLSF_6;
    utc_model.flag_utc_model = flag_utc_model;
    // GPS to Galileo GST conversion parameters
    utc_model.A_0G_10 = A_0G_10;
    utc_model.A_1G_10 = A_1G_10;
    utc_model.t_0G_10 = t_0G_10;
    utc_model.WN_0G_10 = WN_0G_10;
    return utc_model;
}


Galileo_Almanac_Helper Galileo_Inav_Message::get_almanac() const
{
    Galileo_Almanac_Helper almanac;
    // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
    almanac.IOD_a_7 = IOD_a_7;
    almanac.WN_a_7 = WN_a_7;
    almanac.t0a_7 = t0a_7;
    almanac.SVID1_7 = SVID1_7;
    almanac.DELTA_A_7 = DELTA_A_7;
    almanac.e_7 = e_7;
    almanac.omega_7 = omega_7;
    almanac.delta_i_7 = delta_i_7;
    almanac.Omega0_7 = Omega0_7;
    almanac.Omega_dot_7 = Omega_dot_7;
    almanac.M0_7 = M0_7;

    // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
    almanac.IOD_a_8 = IOD_a_8;
    almanac.af0_8 = af0_8;
    almanac.af1_8 = af1_8;
    almanac.E5b_HS_8 = E5b_HS_8;
    almanac.E1B_HS_8 = E1B_HS_8;
    almanac.SVID2_8 = SVID2_8;
    almanac.DELTA_A_8 = DELTA_A_8;
    almanac.e_8 = e_8;
    almanac.omega_8 = omega_8;
    almanac.delta_i_8 = delta_i_8;
    almanac.Omega0_8 = Omega0_8;
    almanac.Omega_dot_8 = Omega_dot_8;

    // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
    almanac.IOD_a_9 = IOD_a_9;
    almanac.WN_a_9 = WN_a_9;
    almanac.t0a_9 = t0a_9;
    almanac.M0_9 = M0_9;
    almanac.af0_9 = af0_9;
    almanac.af1_9 = af1_9;
    almanac.E5b_HS_9 = E5b_HS_9;
    almanac.E1B_HS_9 = E1B_HS_9;
    almanac.SVID3_9 = SVID3_9;
    almanac.DELTA_A_9 = DELTA_A_9;
    almanac.e_9 = e_9;
    almanac.omega_9 = omega_9;
    almanac.delta_i_9 = delta_i_9;

    // Word type 10: Almanac for SVID3 (2/2)
    almanac.IOD_a_10 = IOD_a_10;
    almanac.Omega0_10 = Omega0_10;
    almanac.Omega_dot_10 = Omega_dot_10;
    almanac.M0_10 = M0_10;
    almanac.af0_10 = af0_10;
    almanac.af1_10 = af1_10;
    almanac.E5b_HS_10 = E5b_HS_10;
    almanac.E1B_HS_10 = E1B_HS_10;

    return almanac;
}


int32_t Galileo_Inav_Message::page_jk_decoder(const char* data_jk)
{
    const std::string data_jk_string = data_jk;
    const std::bitset<GALILEO_DATA_JK_BITS> data_jk_bits(data_jk_string);

    const auto page_number = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, PAGE_TYPE_BIT));
    DLOG(INFO) << "Page number = " << page_number;

    switch (page_number)
        {
        case 1:  // Word type 1: Ephemeris (1/4)
            IOD_nav_1 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_NAV_1_BIT));
            DLOG(INFO) << "IOD_nav_1= " << IOD_nav_1;
            t0e_1 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T0_E_1_BIT));
            t0e_1 = t0e_1 * T0E_1_LSB;
            DLOG(INFO) << "t0e_1= " << t0e_1;
            M0_1 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_1_BIT));
            M0_1 = M0_1 * M0_1_LSB;
            DLOG(INFO) << "M0_1= " << M0_1;
            e_1 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E_1_BIT));
            e_1 = e_1 * E_1_LSB;
            DLOG(INFO) << "e_1= " << e_1;
            A_1 = static_cast<double>(read_navigation_unsigned(data_jk_bits, A_1_BIT));
            A_1 = A_1 * A_1_LSB_GAL;
            DLOG(INFO) << "A_1= " << A_1;
            flag_ephemeris_1 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 2:  // Word type 2: Ephemeris (2/4)
            IOD_nav_2 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_NAV_2_BIT));
            DLOG(INFO) << "IOD_nav_2= " << IOD_nav_2;
            OMEGA_0_2 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_0_2_BIT));
            OMEGA_0_2 = OMEGA_0_2 * OMEGA_0_2_LSB;
            DLOG(INFO) << "OMEGA_0_2= " << OMEGA_0_2;
            i_0_2 = static_cast<double>(read_navigation_signed(data_jk_bits, I_0_2_BIT));
            i_0_2 = i_0_2 * I_0_2_LSB;
            DLOG(INFO) << "i_0_2= " << i_0_2;
            omega_2 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_2_BIT));
            omega_2 = omega_2 * OMEGA_2_LSB;
            DLOG(INFO) << "omega_2= " << omega_2;
            iDot_2 = static_cast<double>(read_navigation_signed(data_jk_bits, I_DOT_2_BIT));
            iDot_2 = iDot_2 * I_DOT_2_LSB;
            DLOG(INFO) << "iDot_2= " << iDot_2;
            flag_ephemeris_2 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 3:  // Word type 3: Ephemeris (3/4) and SISA
            IOD_nav_3 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_NAV_3_BIT));
            DLOG(INFO) << "IOD_nav_3= " << IOD_nav_3;
            OMEGA_dot_3 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_DOT_3_BIT));
            OMEGA_dot_3 = OMEGA_dot_3 * OMEGA_DOT_3_LSB;
            DLOG(INFO) << "OMEGA_dot_3= " << OMEGA_dot_3;
            delta_n_3 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_N_3_BIT));
            delta_n_3 = delta_n_3 * DELTA_N_3_LSB;
            DLOG(INFO) << "delta_n_3= " << delta_n_3;
            C_uc_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_UC_3_BIT));
            C_uc_3 = C_uc_3 * C_UC_3_LSB;
            DLOG(INFO) << "C_uc_3= " << C_uc_3;
            C_us_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_US_3_BIT));
            C_us_3 = C_us_3 * C_US_3_LSB;
            DLOG(INFO) << "C_us_3= " << C_us_3;
            C_rc_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_RC_3_BIT));
            C_rc_3 = C_rc_3 * C_RC_3_LSB;
            DLOG(INFO) << "C_rc_3= " << C_rc_3;
            C_rs_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_RS_3_BIT));
            C_rs_3 = C_rs_3 * C_RS_3_LSB;
            DLOG(INFO) << "C_rs_3= " << C_rs_3;
            SISA_3 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SISA_3_BIT));
            DLOG(INFO) << "SISA_3= " << SISA_3;
            flag_ephemeris_3 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 4:  // Word type 4: Ephemeris (4/4) and Clock correction parameters
            IOD_nav_4 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_NAV_4_BIT));
            DLOG(INFO) << "IOD_nav_4= " << IOD_nav_4;
            SV_ID_PRN_4 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SV_ID_PRN_4_BIT));
            DLOG(INFO) << "SV_ID_PRN_4= " << SV_ID_PRN_4;
            C_ic_4 = static_cast<double>(read_navigation_signed(data_jk_bits, C_IC_4_BIT));
            C_ic_4 = C_ic_4 * C_IC_4_LSB;
            DLOG(INFO) << "C_ic_4= " << C_ic_4;
            C_is_4 = static_cast<double>(read_navigation_signed(data_jk_bits, C_IS_4_BIT));
            C_is_4 = C_is_4 * C_IS_4_LSB;
            DLOG(INFO) << "C_is_4= " << C_is_4;
            // Clock correction parameters
            t0c_4 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T0C_4_BIT));
            t0c_4 = t0c_4 * T0C_4_LSB;
            DLOG(INFO) << "t0c_4= " << t0c_4;
            af0_4 = static_cast<double>(read_navigation_signed(data_jk_bits, AF0_4_BIT));
            af0_4 = af0_4 * AF0_4_LSB;
            DLOG(INFO) << "af0_4 = " << af0_4;
            af1_4 = static_cast<double>(read_navigation_signed(data_jk_bits, AF1_4_BIT));
            af1_4 = af1_4 * AF1_4_LSB;
            DLOG(INFO) << "af1_4 = " << af1_4;
            af2_4 = static_cast<double>(read_navigation_signed(data_jk_bits, AF2_4_BIT));
            af2_4 = af2_4 * AF2_4_LSB;
            DLOG(INFO) << "af2_4 = " << af2_4;
            spare_4 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SPARE_4_BIT));
            DLOG(INFO) << "spare_4 = " << spare_4;
            flag_ephemeris_4 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 5:  // Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST
            // Ionospheric correction
            ai0_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, AI0_5_BIT));
            ai0_5 = ai0_5 * AI0_5_LSB;
            DLOG(INFO) << "ai0_5= " << ai0_5;
            ai1_5 = static_cast<double>(read_navigation_signed(data_jk_bits, AI1_5_BIT));
            ai1_5 = ai1_5 * AI1_5_LSB;
            DLOG(INFO) << "ai1_5= " << ai1_5;
            ai2_5 = static_cast<double>(read_navigation_signed(data_jk_bits, AI2_5_BIT));
            ai2_5 = ai2_5 * AI2_5_LSB;
            DLOG(INFO) << "ai2_5= " << ai2_5;
            // Ionospheric disturbance flag
            Region1_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, REGION1_5_BIT));
            DLOG(INFO) << "Region1_flag_5= " << Region1_flag_5;
            Region2_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, REGION2_5_BIT));
            DLOG(INFO) << "Region2_flag_5= " << Region2_flag_5;
            Region3_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, REGION3_5_BIT));
            DLOG(INFO) << "Region3_flag_5= " << Region3_flag_5;
            Region4_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, REGION4_5_BIT));
            DLOG(INFO) << "Region4_flag_5= " << Region4_flag_5;
            Region5_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, REGION5_5_BIT));
            DLOG(INFO) << "Region5_flag_5= " << Region5_flag_5;
            BGD_E1E5a_5 = static_cast<double>(read_navigation_signed(data_jk_bits, BGD_E1_E5A_5_BIT));
            BGD_E1E5a_5 = BGD_E1E5a_5 * BGD_E1_E5A_5_LSB;
            DLOG(INFO) << "BGD_E1E5a_5= " << BGD_E1E5a_5;
            BGD_E1E5b_5 = static_cast<double>(read_navigation_signed(data_jk_bits, BGD_E1_E5B_5_BIT));
            BGD_E1E5b_5 = BGD_E1E5b_5 * BGD_E1_E5B_5_LSB;
            DLOG(INFO) << "BGD_E1E5b_5= " << BGD_E1E5b_5;
            E5b_HS_5 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E5B_HS_5_BIT));
            DLOG(INFO) << "E5b_HS_5= " << E5b_HS_5;
            E1B_HS_5 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E1_B_HS_5_BIT));
            DLOG(INFO) << "E1B_HS_5= " << E1B_HS_5;
            E5b_DVS_5 = static_cast<bool>(read_navigation_unsigned(data_jk_bits, E5B_DVS_5_BIT));
            DLOG(INFO) << "E5b_DVS_5= " << E5b_DVS_5;
            E1B_DVS_5 = static_cast<bool>(read_navigation_unsigned(data_jk_bits, E1_B_DVS_5_BIT));
            DLOG(INFO) << "E1B_DVS_5= " << E1B_DVS_5;
            // GST
            WN_5 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_5_BIT));
            DLOG(INFO) << "WN_5= " << WN_5;
            TOW_5 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TOW_5_BIT));
            DLOG(INFO) << "TOW_5= " << TOW_5;
            flag_TOW_5 = true;  // set to false externally
            spare_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SPARE_5_BIT));
            DLOG(INFO) << "spare_5= " << spare_5;
            flag_iono_and_GST = true;  // set to false externally
            flag_TOW_set = true;       // set to false externally
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 6:  // Word type 6: GST-UTC conversion parameters
            A0_6 = static_cast<double>(read_navigation_signed(data_jk_bits, A0_6_BIT));
            A0_6 = A0_6 * A0_6_LSB;
            DLOG(INFO) << "A0_6= " << A0_6;
            A1_6 = static_cast<double>(read_navigation_signed(data_jk_bits, A1_6_BIT));
            A1_6 = A1_6 * A1_6_LSB;
            DLOG(INFO) << "A1_6= " << A1_6;
            Delta_tLS_6 = static_cast<int32_t>(read_navigation_signed(data_jk_bits, DELTA_T_LS_6_BIT));
            DLOG(INFO) << "Delta_tLS_6= " << Delta_tLS_6;
            t0t_6 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T0T_6_BIT));
            t0t_6 = t0t_6 * T0T_6_LSB;
            DLOG(INFO) << "t0t_6= " << t0t_6;
            WNot_6 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, W_NOT_6_BIT));
            DLOG(INFO) << "WNot_6= " << WNot_6;
            WN_LSF_6 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_LSF_6_BIT));
            DLOG(INFO) << "WN_LSF_6= " << WN_LSF_6;
            DN_6 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, DN_6_BIT));
            DLOG(INFO) << "DN_6= " << DN_6;
            Delta_tLSF_6 = static_cast<int32_t>(read_navigation_signed(data_jk_bits, DELTA_T_LSF_6_BIT));
            DLOG(INFO) << "Delta_tLSF_6= " << Delta_tLSF_6;
            TOW_6 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TOW_6_BIT));
            DLOG(INFO) << "TOW_6= " << TOW_6;
            flag_TOW_6 = true;      // set to false externally
            flag_utc_model = true;  // set to false externally
            flag_TOW_set = true;    // set to false externally
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 7:  // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
            IOD_a_7 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_A_7_BIT));
            DLOG(INFO) << "IOD_a_7= " << IOD_a_7;
            WN_a_7 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_A_7_BIT));
            DLOG(INFO) << "WN_a_7= " << WN_a_7;
            t0a_7 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T0A_7_BIT));
            t0a_7 = t0a_7 * T0A_7_LSB;
            DLOG(INFO) << "t0a_7= " << t0a_7;
            SVID1_7 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SVI_D1_7_BIT));
            DLOG(INFO) << "SVID1_7= " << SVID1_7;
            DELTA_A_7 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_7_BIT));
            DELTA_A_7 = DELTA_A_7 * DELTA_A_7_LSB;
            DLOG(INFO) << "DELTA_A_7= " << DELTA_A_7;
            e_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E_7_BIT));
            e_7 = e_7 * E_7_LSB;
            DLOG(INFO) << "e_7= " << e_7;
            omega_7 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_7_BIT));
            omega_7 = omega_7 * OMEGA_7_LSB;
            DLOG(INFO) << "omega_7= " << omega_7;
            delta_i_7 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_I_7_BIT));
            delta_i_7 = delta_i_7 * DELTA_I_7_LSB;
            DLOG(INFO) << "delta_i_7= " << delta_i_7;
            Omega0_7 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA0_7_BIT));
            Omega0_7 = Omega0_7 * OMEGA0_7_LSB;
            DLOG(INFO) << "Omega0_7= " << Omega0_7;
            Omega_dot_7 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_DOT_7_BIT));
            Omega_dot_7 = Omega_dot_7 * OMEGA_DOT_7_LSB;
            DLOG(INFO) << "Omega_dot_7= " << Omega_dot_7;
            M0_7 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_7_BIT));
            M0_7 = M0_7 * M0_7_LSB;
            DLOG(INFO) << "M0_7= " << M0_7;
            flag_almanac_1 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 8:  // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
            IOD_a_8 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_A_8_BIT));
            DLOG(INFO) << "IOD_a_8= " << IOD_a_8;
            af0_8 = static_cast<double>(read_navigation_signed(data_jk_bits, AF0_8_BIT));
            af0_8 = af0_8 * AF0_8_LSB;
            DLOG(INFO) << "af0_8= " << af0_8;
            af1_8 = static_cast<double>(read_navigation_signed(data_jk_bits, AF1_8_BIT));
            af1_8 = af1_8 * AF1_8_LSB;
            DLOG(INFO) << "af1_8= " << af1_8;
            E5b_HS_8 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E5B_HS_8_BIT));
            DLOG(INFO) << "E5b_HS_8= " << E5b_HS_8;
            E1B_HS_8 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E1_B_HS_8_BIT));
            DLOG(INFO) << "E1B_HS_8= " << E1B_HS_8;
            SVID2_8 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SVI_D2_8_BIT));
            DLOG(INFO) << "SVID2_8= " << SVID2_8;
            DELTA_A_8 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_8_BIT));
            DELTA_A_8 = DELTA_A_8 * DELTA_A_8_LSB;
            DLOG(INFO) << "DELTA_A_8= " << DELTA_A_8;
            e_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E_8_BIT));
            e_8 = e_8 * E_8_LSB;
            DLOG(INFO) << "e_8= " << e_8;
            omega_8 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_8_BIT));
            omega_8 = omega_8 * OMEGA_8_LSB;
            DLOG(INFO) << "omega_8= " << omega_8;
            delta_i_8 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_I_8_BIT));
            delta_i_8 = delta_i_8 * DELTA_I_8_LSB;
            DLOG(INFO) << "delta_i_8= " << delta_i_8;
            Omega0_8 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA0_8_BIT));
            Omega0_8 = Omega0_8 * OMEGA0_8_LSB;
            DLOG(INFO) << "Omega0_8= " << Omega0_8;
            Omega_dot_8 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_DOT_8_BIT));
            Omega_dot_8 = Omega_dot_8 * OMEGA_DOT_8_LSB;
            DLOG(INFO) << "Omega_dot_8= " << Omega_dot_8;
            flag_almanac_2 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 9:  // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
            IOD_a_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_A_9_BIT));
            DLOG(INFO) << "IOD_a_9= " << IOD_a_9;
            WN_a_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_A_9_BIT));
            DLOG(INFO) << "WN_a_9= " << WN_a_9;
            t0a_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T0A_9_BIT));
            t0a_9 = t0a_9 * T0A_9_LSB;
            DLOG(INFO) << "t0a_9= " << t0a_9;
            M0_9 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_9_BIT));
            M0_9 = M0_9 * M0_9_LSB;
            DLOG(INFO) << "M0_9= " << M0_9;
            af0_9 = static_cast<double>(read_navigation_signed(data_jk_bits, AF0_9_BIT));
            af0_9 = af0_9 * AF0_9_LSB;
            DLOG(INFO) << "af0_9= " << af0_9;
            af1_9 = static_cast<double>(read_navigation_signed(data_jk_bits, AF1_9_BIT));
            af1_9 = af1_9 * AF1_9_LSB;
            DLOG(INFO) << "af1_9= " << af1_9;
            E5b_HS_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E5B_HS_9_BIT));
            DLOG(INFO) << "E5b_HS_9= " << E5b_HS_9;
            E1B_HS_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E1_B_HS_9_BIT));
            DLOG(INFO) << "E1B_HS_9= " << E1B_HS_9;
            SVID3_9 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SVI_D3_9_BIT));
            DLOG(INFO) << "SVID3_9= " << SVID3_9;
            DELTA_A_9 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_9_BIT));
            DELTA_A_9 = DELTA_A_9 * DELTA_A_9_LSB;
            DLOG(INFO) << "DELTA_A_9= " << DELTA_A_9;
            e_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E_9_BIT));
            e_9 = e_9 * E_9_LSB;
            DLOG(INFO) << "e_9= " << e_9;
            omega_9 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_9_BIT));
            omega_9 = omega_9 * OMEGA_9_LSB;
            DLOG(INFO) << "omega_9= " << omega_9;
            delta_i_9 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_I_9_BIT));
            delta_i_9 = delta_i_9 * DELTA_I_9_LSB;
            DLOG(INFO) << "delta_i_9= " << delta_i_9;
            flag_almanac_3 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 10:  // Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters
            IOD_a_10 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_A_10_BIT));
            DLOG(INFO) << "IOD_a_10= " << IOD_a_10;
            Omega0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA0_10_BIT));
            Omega0_10 = Omega0_10 * OMEGA0_10_LSB;
            DLOG(INFO) << "Omega0_10= " << Omega0_10;
            Omega_dot_10 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_DOT_10_BIT));
            Omega_dot_10 = Omega_dot_10 * OMEGA_DOT_10_LSB;
            DLOG(INFO) << "Omega_dot_10= " << Omega_dot_10;
            M0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_10_BIT));
            M0_10 = M0_10 * M0_10_LSB;
            DLOG(INFO) << "M0_10= " << M0_10;
            af0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, AF0_10_BIT));
            af0_10 = af0_10 * AF0_10_LSB;
            DLOG(INFO) << "af0_10= " << af0_10;
            af1_10 = static_cast<double>(read_navigation_signed(data_jk_bits, AF1_10_BIT));
            af1_10 = af1_10 * AF1_10_LSB;
            DLOG(INFO) << "af1_10= " << af1_10;
            E5b_HS_10 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E5B_HS_10_BIT));
            DLOG(INFO) << "E5b_HS_10= " << E5b_HS_10;
            E1B_HS_10 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, E1_B_HS_10_BIT));
            DLOG(INFO) << "E1B_HS_10= " << E1B_HS_10;
            A_0G_10 = static_cast<double>(read_navigation_signed(data_jk_bits, A_0_G_10_BIT));
            A_0G_10 = A_0G_10 * A_0G_10_LSB;
            flag_GGTO_1 = true;
            DLOG(INFO) << "A_0G_10= " << A_0G_10;
            A_1G_10 = static_cast<double>(read_navigation_signed(data_jk_bits, A_1_G_10_BIT));
            A_1G_10 = A_1G_10 * A_1G_10_LSB;
            flag_GGTO_2 = true;
            DLOG(INFO) << "A_1G_10= " << A_1G_10;
            t_0G_10 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, T_0_G_10_BIT));
            t_0G_10 = t_0G_10 * T_0_G_10_LSB;
            flag_GGTO_3 = true;
            DLOG(INFO) << "t_0G_10= " << t_0G_10;
            WN_0G_10 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_0_G_10_BIT));
            flag_GGTO_4 = true;
            DLOG(INFO) << "WN_0G_10= " << WN_0G_10;
            flag_almanac_4 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 0:  // Word type 0: I/NAV Spare Word
            Time_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TIME_0_BIT));
            DLOG(INFO) << "Time_0= " << Time_0;
            WN_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_0_BIT));
            DLOG(INFO) << "WN_0= " << WN_0;
            TOW_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TOW_0_BIT));
            DLOG(INFO) << "TOW_0= " << TOW_0;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        default:
            break;
        }
    return page_number;
}

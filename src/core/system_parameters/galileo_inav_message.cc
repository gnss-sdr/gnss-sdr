/*!
 * \file galileo_inav_message.cc
 * \brief  Implementation of a Galileo I/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 2.0 (Jan. 2021)
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_inav_message.h"
#include "galileo_reduced_ced.h"
#include "reed_solomon.h"
#include <boost/crc.hpp>             // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>  // for boost::dynamic_bitset
#include <glog/logging.h>            // for DLOG
#include <algorithm>                 // for reverse
#include <iostream>                  // for operator<<
#include <limits>                    // for std::numeric_limits
#include <numeric>                   // for std::accumulate


using CRC_Galileo_INAV_type = boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false>;


Galileo_Inav_Message::Galileo_Inav_Message()
{
    rs_buffer = std::vector<uint8_t>(INAV_RS_BUFFER_LENGTH, 0);
    // Instantiate ReedSolomon without encoding capabilities, saves some memory
    rs = std::make_unique<ReedSolomon>(60, 29, 1, 195, 0, 137);
    inav_rs_pages = std::vector<int>(8, 0);
}


// here the compiler knows how to destrcut rs
Galileo_Inav_Message::~Galileo_Inav_Message() = default;


bool Galileo_Inav_Message::CRC_test(const std::bitset<GALILEO_DATA_FRAME_BITS>& bits, uint32_t checksum) const
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


uint64_t Galileo_Inav_Message::read_navigation_unsigned(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
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


uint8_t Galileo_Inav_Message::read_octet_unsigned(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    uint8_t value = 0;
    const int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  // shift left
                    if (static_cast<int>(bits[GALILEO_DATA_JK_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


uint64_t Galileo_Inav_Message::read_page_type_unsigned(const std::bitset<GALILEO_PAGE_TYPE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
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


int64_t Galileo_Inav_Message::read_navigation_signed(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
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


bool Galileo_Inav_Message::read_navigation_bool(const std::bitset<GALILEO_DATA_JK_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
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

    if (enable_rs)
        {
            // Implement FEC2 Erasure Correction defined in Galileo ICD 2.0
            if (std::accumulate(inav_rs_pages.begin(), inav_rs_pages.end(), 0) == 4)
                {
                    // Four different INAV pages received with CRC ok
                    // so we can decode the buffer and retrieve data from missing pages

                    // Generate erasure vector
                    std::vector<int> erasure_positions;
                    erasure_positions.reserve(60);  // max number of erasure positions
                    if (inav_rs_pages[0] == 0)
                        {
                            // we always know rs_buffer[0], so we start at 1
                            for (int i = 1; i < 16; i++)
                                {
                                    erasure_positions.push_back(i);
                                }
                        }
                    if (inav_rs_pages[1] == 0)
                        {
                            for (int i = 16; i < 30; i++)
                                {
                                    erasure_positions.push_back(i);
                                }
                        }
                    if (inav_rs_pages[2] == 0)
                        {
                            for (int i = 30; i < 44; i++)
                                {
                                    erasure_positions.push_back(i);
                                }
                        }
                    if (inav_rs_pages[3] == 0)
                        {
                            for (int i = 44; i < 58; i++)
                                {
                                    erasure_positions.push_back(i);
                                }
                        }
                    if (inav_rs_pages[4] == 0)
                        {
                            for (int i = 58; i < 73; i++)
                                {
                                    erasure_positions.push_back(i + 137);  // erasure position refers to the unshortened code, so we add 137
                                }
                        }
                    if (inav_rs_pages[5] == 0)
                        {
                            for (int i = 73; i < 88; i++)
                                {
                                    erasure_positions.push_back(i + 137);
                                }
                        }
                    if (inav_rs_pages[6] == 0)
                        {
                            for (int i = 88; i < 103; i++)
                                {
                                    erasure_positions.push_back(i + 137);
                                }
                        }
                    if (inav_rs_pages[7] == 0)
                        {
                            for (int i = 103; i < 118; i++)
                                {
                                    erasure_positions.push_back(i + 137);
                                }
                        }

                    // Decode rs_buffer
                    int result = rs->decode(rs_buffer, erasure_positions);

                    // if decoding ok
                    if (result >= 0)
                        {
                            if (inav_rs_pages[0] == 0)
                                {
                                    std::bitset<GALILEO_DATA_JK_BITS> missing_bits = regenerate_page_1(rs_buffer);
                                    read_page_1(missing_bits);
                                }
                            if (inav_rs_pages[1] == 0)
                                {
                                    std::bitset<GALILEO_DATA_JK_BITS> missing_bits = regenerate_page_2(rs_buffer);
                                    read_page_2(missing_bits);
                                }
                            if (inav_rs_pages[2] == 0)
                                {
                                    std::bitset<GALILEO_DATA_JK_BITS> missing_bits = regenerate_page_3(rs_buffer);
                                    read_page_3(missing_bits);
                                }
                            if (inav_rs_pages[3] == 0)
                                {
                                    std::bitset<GALILEO_DATA_JK_BITS> missing_bits = regenerate_page_4(rs_buffer);
                                    read_page_4(missing_bits);
                                }

                            // Reset flags
                            inav_rs_pages = std::vector<int>(8, 0);
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


bool Galileo_Inav_Message::have_new_reduced_ced()
{
    // Check if we have a new CED data set stored in the galileo navigation class
    if ((flag_CED == true) && (WN_5 > 0))  // We need the week number to compute GST
        {
            flag_CED = false;
            return true;
        }
    return false;
}


Galileo_Ephemeris Galileo_Inav_Message::get_ephemeris() const
{
    Galileo_Ephemeris ephemeris;
    ephemeris.flag_all_ephemeris = flag_all_ephemeris;
    ephemeris.IOD_ephemeris = IOD_ephemeris;
    ephemeris.IOD_nav = IOD_nav_1;
    ephemeris.PRN = SV_ID_PRN_4;
    ephemeris.M_0 = M0_1;              // Mean anomaly at reference time [semi-circles]
    ephemeris.delta_n = delta_n_3;     // Mean motion difference from computed value  [semi-circles/sec]
    ephemeris.ecc = e_1;               // Eccentricity
    ephemeris.sqrtA = A_1;             // Square root of the semi-major axis [meters^1/2]
    ephemeris.OMEGA_0 = OMEGA_0_2;     // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    ephemeris.i_0 = i_0_2;             // Inclination angle at reference time  [semi-circles]
    ephemeris.omega = omega_2;         // Argument of perigee [semi-circles]
    ephemeris.OMEGAdot = OMEGA_dot_3;  // Rate of right ascension [semi-circles/sec]
    ephemeris.idot = iDot_2;           // Rate of inclination angle [semi-circles/sec]
    ephemeris.Cuc = C_uc_3;            // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    ephemeris.Cus = C_us_3;            // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    ephemeris.Crc = C_rc_3;            // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    ephemeris.Crs = C_rs_3;            // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    ephemeris.Cic = C_ic_4;            // Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    ephemeris.Cis = C_is_4;            // Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    ephemeris.toe = t0e_1;             // Ephemeris reference time [s]

    // Clock correction parameters
    ephemeris.toc = t0c_4;  // Clock correction data reference Time of Week [sec]
    ephemeris.af0 = af0_4;  // SV clock bias correction coefficient [s]
    ephemeris.af1 = af1_4;  // SV clock drift correction coefficient [s/s]
    ephemeris.af2 = af2_4;  // SV clock drift rate correction coefficient [s/s^2]

    // GST
    ephemeris.WN = WN_5;    // Week number
    ephemeris.tow = TOW_5;  // Time of Week

    ephemeris.SISA = SISA_3;
    ephemeris.E5b_HS = E5b_HS_5;    // E5b Signal Health Status
    ephemeris.E1B_HS = E1B_HS_5;    // E1B Signal Health Status
    ephemeris.E5b_DVS = E5b_DVS_5;  // E5b Data Validity Status
    ephemeris.E1B_DVS = E1B_DVS_5;  // E1B Data Validity Status

    ephemeris.BGD_E1E5a = BGD_E1E5a_5;  // E1-E5a Broadcast Group Delay [s]
    ephemeris.BGD_E1E5b = BGD_E1E5b_5;  // E1-E5b Broadcast Group Delay [s]

    ephemeris.satClkDrift = Galileo_satClkDrift;

    return ephemeris;
}


Galileo_Iono Galileo_Inav_Message::get_iono() const
{
    Galileo_Iono iono;
    // Ionospheric correction
    iono.ai0 = ai0_5;  // Effective Ionisation Level 1st order parameter [sfu]
    iono.ai1 = ai1_5;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    iono.ai2 = ai2_5;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // GST
    // This is the ONLY page containing the Week Number (WN)
    iono.tow = TOW_5;
    iono.WN = WN_5;

    // Ionospheric disturbance flag
    iono.Region1_flag = Region1_flag_5;  // Ionospheric Disturbance Flag for region 1
    iono.Region2_flag = Region2_flag_5;  // Ionospheric Disturbance Flag for region 2
    iono.Region3_flag = Region3_flag_5;  // Ionospheric Disturbance Flag for region 3
    iono.Region4_flag = Region4_flag_5;  // Ionospheric Disturbance Flag for region 4
    iono.Region5_flag = Region5_flag_5;  // Ionospheric Disturbance Flag for region 5

    return iono;
}


Galileo_Utc_Model Galileo_Inav_Message::get_utc_model() const
{
    Galileo_Utc_Model utc_model;
    // Word type 6: GST-UTC conversion parameters
    utc_model.A0 = A0_6;
    utc_model.A1 = A1_6;
    utc_model.Delta_tLS = Delta_tLS_6;
    utc_model.tot = t0t_6;
    utc_model.WNot = WNot_6;
    utc_model.WN_LSF = WN_LSF_6;
    utc_model.DN = DN_6;
    utc_model.Delta_tLSF = Delta_tLSF_6;
    utc_model.flag_utc_model = flag_utc_model;
    // GPS to Galileo GST conversion parameters
    utc_model.A_0G = A_0G_10;
    utc_model.A_1G = A_1G_10;
    utc_model.t_0G = t_0G_10;
    utc_model.WN_0G = WN_0G_10;
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


Galileo_Ephemeris Galileo_Inav_Message::get_reduced_ced() const
{
    Galileo_Reduced_CED ced{};
    ced.PRN = SV_ID_PRN_4;
    // From ICD: TOTRedCED is the start time of transmission of the
    // Reduced CED word 16 in GST
    if (TOW_5 > TOW_6)
        {
            ced.TOTRedCED = WN_5 * 604800 + TOW_5 + 4;  // According to ICD 2.0, Table 38
        }
    else
        {
            ced.TOTRedCED = WN_5 * 604800 + TOW_6 + 10;  // According to ICD 2.0, Table 38
        }
    std::array<int32_t, 4> iod_navs = {IOD_nav_1, IOD_nav_2, IOD_nav_3, IOD_nav_4};
    int32_t max_IOD_nav = IOD_nav_1;
    for (int i = 1; i < 4; i++)
        {
            if (iod_navs[i] > max_IOD_nav)
                {
                    max_IOD_nav = iod_navs[i];
                }
        }
    ced.IODnav = max_IOD_nav;
    ced.DeltaAred = ced_DeltaAred;
    ced.exred = ced_exred;
    ced.eyred = ced_eyred;
    ced.Deltai0red = ced_Deltai0red;
    ced.Omega0red = ced_Omega0red;
    ced.lambda0red = ced_lambda0red;
    ced.af0red = ced_af0red;
    ced.af1red = ced_af1red;

    Galileo_Ephemeris eph = ced.compute_eph();
    eph.BGD_E1E5a = BGD_E1E5a_5;
    eph.BGD_E1E5b = BGD_E1E5b_5;
    return eph;
}


void Galileo_Inav_Message::read_page_1(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits)
{
    IOD_nav_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, IOD_NAV_1_BIT));
    DLOG(INFO) << "IOD_nav_1= " << IOD_nav_1;
    t0e_1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, T0_E_1_BIT));
    t0e_1 = t0e_1 * T0E_1_LSB;
    DLOG(INFO) << "t0e_1= " << t0e_1;
    M0_1 = static_cast<double>(read_navigation_signed(data_bits, M0_1_BIT));
    M0_1 = M0_1 * M0_1_LSB;
    DLOG(INFO) << "M0_1= " << M0_1;
    e_1 = static_cast<double>(read_navigation_unsigned(data_bits, E_1_BIT));
    e_1 = e_1 * E_1_LSB;
    DLOG(INFO) << "e_1= " << e_1;
    A_1 = static_cast<double>(read_navigation_unsigned(data_bits, A_1_BIT));
    A_1 = A_1 * A_1_LSB_GAL;
    DLOG(INFO) << "A_1= " << A_1;
    flag_ephemeris_1 = true;
    DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
}


void Galileo_Inav_Message::read_page_2(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits)
{
    IOD_nav_2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, IOD_NAV_2_BIT));
    DLOG(INFO) << "IOD_nav_2= " << IOD_nav_2;
    OMEGA_0_2 = static_cast<double>(read_navigation_signed(data_bits, OMEGA_0_2_BIT));
    OMEGA_0_2 = OMEGA_0_2 * OMEGA_0_2_LSB;
    DLOG(INFO) << "OMEGA_0_2= " << OMEGA_0_2;
    i_0_2 = static_cast<double>(read_navigation_signed(data_bits, I_0_2_BIT));
    i_0_2 = i_0_2 * I_0_2_LSB;
    DLOG(INFO) << "i_0_2= " << i_0_2;
    omega_2 = static_cast<double>(read_navigation_signed(data_bits, OMEGA_2_BIT));
    omega_2 = omega_2 * OMEGA_2_LSB;
    DLOG(INFO) << "omega_2= " << omega_2;
    iDot_2 = static_cast<double>(read_navigation_signed(data_bits, I_DOT_2_BIT));
    iDot_2 = iDot_2 * I_DOT_2_LSB;
    DLOG(INFO) << "iDot_2= " << iDot_2;
    flag_ephemeris_2 = true;
    DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
}


void Galileo_Inav_Message::read_page_3(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits)
{
    IOD_nav_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, IOD_NAV_3_BIT));
    DLOG(INFO) << "IOD_nav_3= " << IOD_nav_3;
    OMEGA_dot_3 = static_cast<double>(read_navigation_signed(data_bits, OMEGA_DOT_3_BIT));
    OMEGA_dot_3 = OMEGA_dot_3 * OMEGA_DOT_3_LSB;
    DLOG(INFO) << "OMEGA_dot_3= " << OMEGA_dot_3;
    delta_n_3 = static_cast<double>(read_navigation_signed(data_bits, DELTA_N_3_BIT));
    delta_n_3 = delta_n_3 * DELTA_N_3_LSB;
    DLOG(INFO) << "delta_n_3= " << delta_n_3;
    C_uc_3 = static_cast<double>(read_navigation_signed(data_bits, C_UC_3_BIT));
    C_uc_3 = C_uc_3 * C_UC_3_LSB;
    DLOG(INFO) << "C_uc_3= " << C_uc_3;
    C_us_3 = static_cast<double>(read_navigation_signed(data_bits, C_US_3_BIT));
    C_us_3 = C_us_3 * C_US_3_LSB;
    DLOG(INFO) << "C_us_3= " << C_us_3;
    C_rc_3 = static_cast<double>(read_navigation_signed(data_bits, C_RC_3_BIT));
    C_rc_3 = C_rc_3 * C_RC_3_LSB;
    DLOG(INFO) << "C_rc_3= " << C_rc_3;
    C_rs_3 = static_cast<double>(read_navigation_signed(data_bits, C_RS_3_BIT));
    C_rs_3 = C_rs_3 * C_RS_3_LSB;
    DLOG(INFO) << "C_rs_3= " << C_rs_3;
    SISA_3 = static_cast<int32_t>(read_navigation_unsigned(data_bits, SISA_3_BIT));
    DLOG(INFO) << "SISA_3= " << SISA_3;
    flag_ephemeris_3 = true;
    DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
}


void Galileo_Inav_Message::read_page_4(const std::bitset<GALILEO_DATA_JK_BITS>& data_bits)
{
    IOD_nav_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, IOD_NAV_4_BIT));
    DLOG(INFO) << "IOD_nav_4= " << IOD_nav_4;
    SV_ID_PRN_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, SV_ID_PRN_4_BIT));
    DLOG(INFO) << "SV_ID_PRN_4= " << SV_ID_PRN_4;
    C_ic_4 = static_cast<double>(read_navigation_signed(data_bits, C_IC_4_BIT));
    C_ic_4 = C_ic_4 * C_IC_4_LSB;
    DLOG(INFO) << "C_ic_4= " << C_ic_4;
    C_is_4 = static_cast<double>(read_navigation_signed(data_bits, C_IS_4_BIT));
    C_is_4 = C_is_4 * C_IS_4_LSB;
    DLOG(INFO) << "C_is_4= " << C_is_4;
    // Clock correction parameters
    t0c_4 = static_cast<int32_t>(read_navigation_unsigned(data_bits, T0C_4_BIT));
    t0c_4 = t0c_4 * T0C_4_LSB;
    DLOG(INFO) << "t0c_4= " << t0c_4;
    af0_4 = static_cast<double>(read_navigation_signed(data_bits, AF0_4_BIT));
    af0_4 = af0_4 * AF0_4_LSB;
    DLOG(INFO) << "af0_4 = " << af0_4;
    af1_4 = static_cast<double>(read_navigation_signed(data_bits, AF1_4_BIT));
    af1_4 = af1_4 * AF1_4_LSB;
    DLOG(INFO) << "af1_4 = " << af1_4;
    af2_4 = static_cast<double>(read_navigation_signed(data_bits, AF2_4_BIT));
    af2_4 = af2_4 * AF2_4_LSB;
    DLOG(INFO) << "af2_4 = " << af2_4;
    spare_4 = static_cast<double>(read_navigation_unsigned(data_bits, SPARE_4_BIT));
    DLOG(INFO) << "spare_4 = " << spare_4;
    flag_ephemeris_4 = true;
    DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
}


std::bitset<GALILEO_DATA_JK_BITS> Galileo_Inav_Message::regenerate_page_1(const std::vector<uint8_t>& decoded) const
{
    std::bitset<GALILEO_DATA_JK_BITS> data_bits;
    // Set page type to 1
    data_bits.set(5);
    std::bitset<8> c0(decoded[0]);
    std::bitset<8> c1(decoded[1]);
    for (int i = 0; i < 8; i++)
        {
            data_bits[6 + i] = c1[i];
        }
    data_bits[14] = c0[6];
    data_bits[15] = c0[7];
    for (int k = 2; k < 16; k++)
        {
            std::bitset<8> octet(decoded[k]);
            for (int i = 0; i < 8; i++)
                {
                    data_bits[i + k * 8] = octet[i];
                }
        }
    return data_bits;
}


std::bitset<GALILEO_DATA_JK_BITS> Galileo_Inav_Message::regenerate_page_2(const std::vector<uint8_t>& decoded) const
{
    std::bitset<GALILEO_DATA_JK_BITS> data_bits;
    // Set page type to 2
    data_bits.set(4);

    std::bitset<10> iodnav(current_IODnav);

    for (int i = 0; i < 10; i++)
        {
            data_bits[6 + i] = iodnav[i];
        }
    for (int k = 0; k < 14; k++)
        {
            std::bitset<8> octet(decoded[k + 16]);
            for (int i = 0; i < 8; i++)
                {
                    data_bits[16 + i + k * 8] = octet[i];
                }
        }
    return data_bits;
}


std::bitset<GALILEO_DATA_JK_BITS> Galileo_Inav_Message::regenerate_page_3(const std::vector<uint8_t>& decoded) const
{
    std::bitset<GALILEO_DATA_JK_BITS> data_bits;
    // Set page type to 3
    data_bits.set(4);
    data_bits.set(5);

    std::bitset<10> iodnav(current_IODnav);

    for (int i = 0; i < 10; i++)
        {
            data_bits[6 + i] = iodnav[i];
        }
    for (int k = 0; k < 14; k++)
        {
            std::bitset<8> octet(decoded[k + 30]);
            for (int i = 0; i < 8; i++)
                {
                    data_bits[16 + i + k * 8] = octet[i];
                }
        }
    return data_bits;
}


std::bitset<GALILEO_DATA_JK_BITS> Galileo_Inav_Message::regenerate_page_4(const std::vector<uint8_t>& decoded) const
{
    std::bitset<GALILEO_DATA_JK_BITS> data_bits;
    // Set page type to 4
    data_bits.set(3);

    std::bitset<10> iodnav(current_IODnav);

    for (int i = 0; i < 10; i++)
        {
            data_bits[6 + i] = iodnav[i];
        }
    for (int k = 0; k < 14; k++)
        {
            std::bitset<8> octet(decoded[k + 44]);
            for (int i = 0; i < 8; i++)
                {
                    data_bits[16 + i + k * 8] = octet[i];
                }
        }
    return data_bits;
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
            {
                read_page_1(data_jk_bits);
                if (enable_rs)
                    {
                        if (current_IODnav == 0)
                            {
                                current_IODnav = IOD_nav_1;
                            }
                        if (current_IODnav != IOD_nav_1)
                            {
                                // IODnav changed, reset buffer
                                current_IODnav = IOD_nav_1;
                                rs_buffer = std::vector<uint8_t>(INAV_RS_BUFFER_LENGTH, 0);
                                // Reed-Solomon data is invalid
                                inav_rs_pages = std::vector<int>(8, 0);
                            }

                        // Store RS information vector C_{RS,0}
                        std::vector<std::pair<int32_t, int32_t>> info_octet_bits({{1, 6}, {15, 2}});
                        rs_buffer[0] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                        info_octet_bits = std::vector<std::pair<int32_t, int32_t>>({{7, BITS_IN_OCTET}});
                        rs_buffer[1] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 2; i < 16; i++)
                            {
                                info_octet_bits = std::vector<std::pair<int32_t, int32_t>>({{start_bit, BITS_IN_OCTET}});
                                rs_buffer[i] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[0] = 1;
                    }

                break;
            }

        case 2:  // Word type 2: Ephemeris (2/4)
            {
                read_page_2(data_jk_bits);
                if (enable_rs)
                    {
                        if (current_IODnav == 0)
                            {
                                current_IODnav = IOD_nav_2;
                            }
                        if (current_IODnav != IOD_nav_2)
                            {
                                // IODnav changed, reset buffer
                                current_IODnav = IOD_nav_2;
                                rs_buffer = std::vector<uint8_t>(INAV_RS_BUFFER_LENGTH, 0);
                                // Reed-Solomon data is invalid
                                inav_rs_pages = std::vector<int>(8, 0);
                            }

                        // Store RS information vector C_{RS,1}
                        rs_buffer[0] = 4 + current_IODnav % 4;  // we always know c_{0,0}
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 16; i < 30; i++)
                            {
                                std::vector<std::pair<int32_t, int32_t>> info_octet_bits({{start_bit, BITS_IN_OCTET}});
                                rs_buffer[i] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[1] = 1;
                    }
                break;
            }
        case 3:  // Word type 3: Ephemeris (3/4) and SISA
            {
                read_page_3(data_jk_bits);
                if (enable_rs)
                    {
                        if (current_IODnav == 0)
                            {
                                current_IODnav = IOD_nav_3;
                            }
                        if (current_IODnav != IOD_nav_3)
                            {
                                // IODnav changed, reset buffer
                                current_IODnav = IOD_nav_3;
                                rs_buffer = std::vector<uint8_t>(INAV_RS_BUFFER_LENGTH, 0);
                                // Reed-Solomon data is invalid
                                inav_rs_pages = std::vector<int>(8, 0);
                            }

                        // Store RS information vector C_{RS,2}
                        rs_buffer[0] = 4 + current_IODnav % 4;  // we always know c_{0,0}
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 30; i < 44; i++)
                            {
                                std::vector<std::pair<int32_t, int32_t>> info_octet_bits({{start_bit, BITS_IN_OCTET}});
                                rs_buffer[i] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[2] = 1;
                    }
                break;
            }

        case 4:  // Word type 4: Ephemeris (4/4) and Clock correction parameters
            {
                read_page_4(data_jk_bits);
                if (enable_rs)
                    {
                        if (current_IODnav == 0)
                            {
                                current_IODnav = IOD_nav_4;
                            }
                        if (current_IODnav != IOD_nav_4)
                            {
                                // IODnav changed, reset buffer
                                current_IODnav = IOD_nav_4;
                                rs_buffer = std::vector<uint8_t>(INAV_RS_BUFFER_LENGTH, 0);
                                // Reed-Solomon data is invalid
                                inav_rs_pages = std::vector<int>(8, 0);
                            }

                        // Store RS information vector C_{RS,3}
                        rs_buffer[0] = 4 + current_IODnav % 4;  // we always know c_{0,0}
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 44; i < INAV_RS_INFO_VECTOR_LENGTH; i++)
                            {
                                std::vector<std::pair<int32_t, int32_t>> info_octet_bits({{start_bit, BITS_IN_OCTET}});
                                rs_buffer[i] = read_octet_unsigned(data_jk_bits, info_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[3] = 1;
                    }
                break;
            }

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

        case 16:  // Word type 16: Reduced Clock and Ephemeris Data (CED) parameters
            DLOG(INFO) << "Word type 16 arrived";
            ced_DeltaAred = static_cast<double>(read_navigation_signed(data_jk_bits, CED_DeltaAred_BIT));
            ced_DeltaAred = ced_DeltaAred * CED_DeltaAred_LSB;
            DLOG(INFO) << "DeltaAred = " << ced_DeltaAred;
            ced_exred = static_cast<double>(read_navigation_signed(data_jk_bits, CED_exred_BIT));
            ced_exred = ced_exred * CED_exred_LSB;
            DLOG(INFO) << "exred = " << ced_exred;
            ced_eyred = static_cast<double>(read_navigation_signed(data_jk_bits, CED_eyred_BIT));
            ced_eyred = ced_eyred * CED_eyred_LSB;
            DLOG(INFO) << "eyred = " << ced_eyred;
            ced_Deltai0red = static_cast<double>(read_navigation_signed(data_jk_bits, CED_Deltai0red_BIT));
            ced_Deltai0red = ced_Deltai0red * CED_Deltai0red_LSB;
            DLOG(INFO) << "Deltai0red = " << ced_Deltai0red;
            ced_Omega0red = static_cast<double>(read_navigation_signed(data_jk_bits, CED_Omega0red_BIT));
            ced_Omega0red = ced_Omega0red * CED_Omega0red_LSB;
            DLOG(INFO) << "Omega0red = " << ced_Omega0red;
            ced_lambda0red = static_cast<double>(read_navigation_signed(data_jk_bits, CED_lambda0red_BIT));
            ced_lambda0red = ced_lambda0red * CED_lambda0red_LSB;
            DLOG(INFO) << "lambda0red = " << ced_lambda0red;
            ced_af0red = static_cast<double>(read_navigation_signed(data_jk_bits, CED_af0red_BIT));
            ced_af0red = ced_af0red * CED_af0red_LSB;
            DLOG(INFO) << "af0red = " << ced_af0red;
            ced_af1red = static_cast<double>(read_navigation_signed(data_jk_bits, CED_af1red_BIT));
            ced_af1red = ced_af1red * CED_af1red_LSB;
            DLOG(INFO) << "af1red = " << ced_af1red;
            flag_CED = true;
            break;

        case 17:  // Word type 17: FEC2 Reed-Solomon for CED
            {
                if (enable_rs)
                    {
                        IODnav_LSB17 = read_octet_unsigned(data_jk_bits, RS_IODNAV_LSBS);
                        DLOG(INFO) << "IODnav 2 LSBs in Word type 17: " << static_cast<float>(IODnav_LSB17);
                        if (IODnav_LSB17 != static_cast<uint8_t>((current_IODnav % 4)))
                            {
                                // IODnav changed, information vector is invalid
                                inav_rs_pages[0] = 0;
                                inav_rs_pages[1] = 0;
                                inav_rs_pages[2] = 0;
                                inav_rs_pages[3] = 0;
                            }
                        // Store RS parity vector gamma_{RS,0}
                        std::vector<std::pair<int32_t, int32_t>> gamma_octet_bits({{FIRST_RS_BIT, BITS_IN_OCTET}});
                        rs_buffer[INAV_RS_INFO_VECTOR_LENGTH] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 1; i < INAV_RS_SUBVECTOR_LENGTH; i++)
                            {
                                gamma_octet_bits[0] = std::pair<int32_t, int32_t>({start_bit, BITS_IN_OCTET});
                                rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + i] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[4] = 1;
                    }
                break;
            }

        case 18:  // Word type 18: FEC2 Reed-Solomon for CED
            {
                if (enable_rs)
                    {
                        IODnav_LSB18 = read_octet_unsigned(data_jk_bits, RS_IODNAV_LSBS);
                        DLOG(INFO) << "IODnav 2 LSBs in Word type 18: " << static_cast<float>(IODnav_LSB18);
                        if (IODnav_LSB18 != static_cast<uint8_t>((current_IODnav % 4)))
                            {
                                // IODnav changed, information vector is invalid
                                inav_rs_pages[0] = 0;
                                inav_rs_pages[1] = 0;
                                inav_rs_pages[2] = 0;
                                inav_rs_pages[3] = 0;
                            }
                        // Store RS parity vector gamma_{RS,1}
                        std::vector<std::pair<int32_t, int32_t>> gamma_octet_bits({{FIRST_RS_BIT, BITS_IN_OCTET}});
                        rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + INAV_RS_SUBVECTOR_LENGTH] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = INAV_RS_SUBVECTOR_LENGTH + 1; i < 2 * INAV_RS_SUBVECTOR_LENGTH; i++)
                            {
                                gamma_octet_bits[0] = std::pair<int32_t, int32_t>({start_bit, BITS_IN_OCTET});
                                rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + i] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[5] = 1;
                    }
                break;
            }

        case 19:  // Word type 19: FEC2 Reed-Solomon for CED
            {
                if (enable_rs)
                    {
                        IODnav_LSB19 = read_octet_unsigned(data_jk_bits, RS_IODNAV_LSBS);
                        DLOG(INFO) << "IODnav 2 LSBs in Word type 19: " << static_cast<float>(IODnav_LSB19);
                        if (IODnav_LSB19 != static_cast<uint8_t>((current_IODnav % 4)))
                            {
                                // IODnav changed, information vector is invalid
                                inav_rs_pages[0] = 0;
                                inav_rs_pages[1] = 0;
                                inav_rs_pages[2] = 0;
                                inav_rs_pages[3] = 0;
                            }
                        // Store RS parity vector gamma_{RS,2}
                        std::vector<std::pair<int32_t, int32_t>> gamma_octet_bits({{FIRST_RS_BIT, BITS_IN_OCTET}});
                        rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + 2 * INAV_RS_SUBVECTOR_LENGTH] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 2 * INAV_RS_SUBVECTOR_LENGTH + 1; i < 3 * INAV_RS_SUBVECTOR_LENGTH; i++)
                            {
                                gamma_octet_bits[0] = std::pair<int32_t, int32_t>({start_bit, BITS_IN_OCTET});
                                rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + i] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[6] = 1;
                    }
                break;
            }

        case 20:  // Word type 20: FEC2 Reed-Solomon for CED
            {
                if (enable_rs)
                    {
                        IODnav_LSB20 = read_octet_unsigned(data_jk_bits, RS_IODNAV_LSBS);
                        DLOG(INFO) << "IODnav 2 LSBs in Word type 20: " << static_cast<float>(IODnav_LSB20);
                        if (IODnav_LSB20 != static_cast<uint8_t>((current_IODnav % 4)))
                            {
                                // IODnav changed, information vector is invalid
                                inav_rs_pages[0] = 0;
                                inav_rs_pages[1] = 0;
                                inav_rs_pages[2] = 0;
                                inav_rs_pages[3] = 0;
                            }
                        // Store RS parity vector gamma_{RS,4}
                        std::vector<std::pair<int32_t, int32_t>> gamma_octet_bits({{FIRST_RS_BIT, BITS_IN_OCTET}});
                        rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + 3 * INAV_RS_SUBVECTOR_LENGTH] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                        int32_t start_bit = FIRST_RS_BIT_AFTER_IODNAV;
                        for (size_t i = 3 * INAV_RS_SUBVECTOR_LENGTH + 1; i < 4 * INAV_RS_SUBVECTOR_LENGTH; i++)
                            {
                                gamma_octet_bits[0] = std::pair<int32_t, int32_t>({start_bit, BITS_IN_OCTET});
                                rs_buffer[INAV_RS_INFO_VECTOR_LENGTH + i] = read_octet_unsigned(data_jk_bits, gamma_octet_bits);
                                start_bit += BITS_IN_OCTET;
                            }
                        inav_rs_pages[7] = 1;
                    }
                break;
            }

        case 0:  // Word type 0: I/NAV Spare Word
            Time_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TIME_0_BIT));
            DLOG(INFO) << "Time_0= " << Time_0;
            if (Time_0 == 2)  // valid data
                {
                    WN_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, WN_0_BIT));
                    DLOG(INFO) << "WN_0= " << WN_0;
                    TOW_0 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, TOW_0_BIT));
                    flag_TOW_set = true;  // set to false externally
                    flag_TOW_0 = true;    // set to false externally
                    DLOG(INFO) << "TOW_0= " << TOW_0;
                    DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
                }
            break;

        default:
            break;
        }
    return page_number;
}

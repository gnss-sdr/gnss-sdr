/*!
 * \file galileo_navigation_message.cc
 * \brief  Implementation of a Galileo I/NAV Data message
 *         as described in Galileo OS SIS ICD Issue 1.1 (Sept. 2010)
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_navigation_message.h"
#include <boost/crc.hpp>  // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include <iostream>


typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_Galileo_INAV_type;


void Galileo_Navigation_Message::reset()
{
    flag_even_word = 0;
    Page_type_time_stamp = 0;

    flag_CRC_test = false;
    flag_all_ephemeris = false;  // flag indicating that all words containing ephemeris have been received
    flag_ephemeris_1 = false;    // flag indicating that ephemeris 1/4 (word 1) have been received
    flag_ephemeris_2 = false;    // flag indicating that ephemeris 2/4 (word 2) have been received
    flag_ephemeris_3 = false;    // flag indicating that ephemeris 3/4 (word 3) have been received
    flag_ephemeris_4 = false;    // flag indicating that ephemeris 4/4 (word 4) have been received

    flag_iono_and_GST = false;  // flag indicating that ionospheric parameters (word 5) have been received
    flag_utc_model = false;     // flag indicating that utc model parameters (word 6) have been received

    flag_all_almanac = false;  // flag indicating that all almanac have been received
    flag_almanac_1 = false;    // flag indicating that almanac 1/4 (word 7) have been received
    flag_almanac_2 = false;    // flag indicating that almanac 2/4 (word 8) have been received
    flag_almanac_3 = false;    // flag indicating that almanac 3/4 (word 9) have been received
    flag_almanac_4 = false;    // flag indicating that almanac 4/4 (word 10) have been received

    flag_TOW_5 = false;
    flag_TOW_set = false;

    flag_GGTO = false;
    flag_GGTO_1 = false;
    flag_GGTO_2 = false;
    flag_GGTO_3 = false;
    flag_GGTO_4 = false;

    IOD_ephemeris = 0;

    // Word type 1: Ephemeris (1/4)
    IOD_nav_1 = 0;
    t0e_1 = 0.0;
    M0_1 = 0.0;
    e_1 = 0.0;
    A_1 = 0.0;

    // Word type 2: Ephemeris (2/4)
    IOD_nav_2 = 0;    // IOD_nav page 2
    OMEGA_0_2 = 0.0;  // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    i_0_2 = 0.0;      // Inclination angle at reference time  [semi-circles]
    omega_2 = 0.0;    // Argument of perigee [semi-circles]
    iDot_2 = 0.0;     // Rate of inclination angle [semi-circles/sec]

    // Word type 3: Ephemeris (3/4) and SISA
    IOD_nav_3 = 0;
    OMEGA_dot_3 = 0.0;  // Rate of right ascension [semi-circles/sec]
    delta_n_3 = 0.0;    // Mean motion difference from computed value  [semi-circles/sec]
    C_uc_3 = 0.0;       // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    C_us_3 = 0.0;       // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    C_rc_3 = 0.0;       // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    C_rs_3 = 0.0;       // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    SISA_3 = 0.0;       //

    // Word type 4: Ephemeris (4/4) and Clock correction parameter/
    IOD_nav_4 = 0;
    SV_ID_PRN_4 = 0;
    C_ic_4 = 0.0;  // Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    C_is_4 = 0.0;  // Amplitude of the sine harmonic correction term to the angle of inclination [radians]

    // Clock correction parameters
    t0c_4 = 0.0;
    af0_4 = 0.0;
    af1_4 = 0.0;
    af2_4 = 0.0;
    spare_4 = 0.0;

    // Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST
    // Ionospheric correction
    ai0_5 = 0.0;
    ai1_5 = 0.0;
    ai2_5 = 0.0;

    // Ionospheric disturbance flag
    Region1_flag_5 = false;  // Region1_flag_5;
    Region2_flag_5 = false;
    Region3_flag_5 = false;
    Region4_flag_5 = false;
    Region5_flag_5 = false;
    BGD_E1E5a_5 = 0.0;
    BGD_E1E5b_5 = 0.0;
    E5b_HS_5 = 0.0;
    E1B_HS_5 = 0.0;
    E5b_DVS_5 = 0.0;
    E1B_DVS_5 = 0.0;

    // GST
    WN_5 = 0.0;
    TOW_5 = 0.0;
    spare_5 = 0.0;

    // Word type 6: GST-UTC conversion parameters
    A0_6 = 0.0;
    A1_6 = 0.0;
    Delta_tLS_6 = 0.0;
    t0t_6 = 0.0;
    WNot_6 = 0.0;
    WN_LSF_6 = 0.0;
    DN_6 = 0.0;
    Delta_tLSF_6 = 0.0;
    TOW_6 = 0.0;

    // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
    IOD_a_7 = 0;
    WN_a_7 = 0.0;
    t0a_7 = 0.0;
    SVID1_7 = 0;
    DELTA_A_7 = 0.0;
    e_7 = 0.0;
    omega_7 = 0.0;
    delta_i_7 = 0.0;
    Omega0_7 = 0.0;
    Omega_dot_7 = 0.0;
    M0_7 = 0.0;

    // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)
    IOD_a_8 = 0;
    af0_8 = 0.0;
    af1_8 = 0.0;
    E5b_HS_8 = 0.0;
    E1B_HS_8 = 0.0;
    SVID2_8 = 0;
    DELTA_A_8 = 0.0;
    e_8 = 0.0;
    omega_8 = 0.0;
    delta_i_8 = 0.0;
    Omega0_8 = 0.0;
    Omega_dot_8 = 0.0;

    // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
    IOD_a_9 = 0;
    WN_a_9 = 0.0;
    t0a_9 = 0.0;
    M0_9 = 0.0;
    af0_9 = 0.0;
    af1_9 = 0.0;
    E5b_HS_9 = 0.0;
    E1B_HS_9 = 0.0;
    SVID3_9 = 0;
    DELTA_A_9 = 0.0;
    e_9 = 0.0;
    omega_9 = 0.0;
    delta_i_9 = 0.0;

    // Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters
    IOD_a_10 = 0;
    Omega0_10 = 0.0;
    Omega_dot_10 = 0.0;
    M0_10 = 0.0;
    af0_10 = 0.0;
    af1_10 = 0.0;
    E5b_HS_10 = 0.0;
    E1B_HS_10 = 0.0;

    // GST-GPS
    A_0G_10 = 0.0;
    A_1G_10 = 0.0;
    t_0G_10 = 0.0;
    WN_0G_10 = 0.0;

    // Word type 0: I/NAV Spare Word
    Time_0 = 0.0;
    WN_0 = 0.0;
    TOW_0 = 0.0;

    flag_TOW_6 = false;

    Galileo_satClkDrift = 0.0;
    Galileo_dtr = 0.0;

    // satellite positions
    galileo_satpos_X = 0.0;
    galileo_satpos_Y = 0.0;
    galileo_satpos_Z = 0.0;
    // Satellite velocity
    galileo_satvel_X = 0.0;
    galileo_satvel_Y = 0.0;
    galileo_satvel_Z = 0.0;
}


Galileo_Navigation_Message::Galileo_Navigation_Message()
{
    reset();
}


bool Galileo_Navigation_Message::CRC_test(std::bitset<GALILEO_DATA_FRAME_BITS> bits, boost::uint32_t checksum)
{
    CRC_Galileo_INAV_type CRC_Galileo;

    boost::uint32_t crc_computed;
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


uint64_t Galileo_Navigation_Message::read_navigation_unsigned(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> > parameter)
{
    uint64_t value = 0ULL;
    int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  // shift left
                    if (bits[GALILEO_DATA_JK_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


uint64_t Galileo_Navigation_Message::read_page_type_unsigned(std::bitset<GALILEO_PAGE_TYPE_BITS> bits, const std::vector<std::pair<int32_t, int32_t> > parameter)
{
    uint64_t value = 0ULL;
    int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  // shift left
                    if (bits[GALILEO_PAGE_TYPE_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Galileo_Navigation_Message::read_navigation_signed(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> > parameter)
{
    int64_t value = 0LL;
    int32_t num_of_slices = parameter.size();

    // read the MSB and perform the sign extension
    if (bits[GALILEO_DATA_JK_BITS - parameter[0].first] == 1)
        {
            value ^= 0xFFFFFFFFFFFFFFFF;  // 64 bits variable
        }
    else
        {
            value &= 0;
        }

    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;                  // shift left
                    value &= 0xFFFFFFFFFFFFFFFE;  // reset the corresponding bit (for the 64 bits variable)
                    if (bits[GALILEO_DATA_JK_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


bool Galileo_Navigation_Message::read_navigation_bool(std::bitset<GALILEO_DATA_JK_BITS> bits, const std::vector<std::pair<int32_t, int32_t> > parameter)
{
    bool value;
    if (bits[GALILEO_DATA_JK_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


void Galileo_Navigation_Message::split_page(std::string page_string, int32_t flag_even_word)
{
    // ToDo: Clean all the tests and create an independent google test code for the telemetry decoder.
    //char correct_tail[7]="011110"; //the viterbi decoder output change the tail to this value (why?)
    //char correct_tail[7]="000000";

    int32_t Page_type = 0;
    // std::cout << "Start decoding Galileo I/NAV " << std::endl;

    if (page_string.at(0) == '1')  // if page is odd
        {
            // std::cout<< "page_string.at(0) split page="<<page_string.at(0) << std::endl;
            std::string page_Odd = page_string;
            // std::cout<<"Page odd string in split page"<< std::endl << page_Odd << std::endl;

            if (flag_even_word == 1)  // An odd page has been received but the previous even page is kept in memory and it is considered to join pages
                {
                    std::string page_INAV_even = page_Even;
                    std::string page_INAV = page_INAV_even + page_Odd;  // Join pages: Even + Odd = INAV page
                    std::string Even_bit = page_INAV.substr(0, 1);
                    std::string Page_type_even = page_INAV.substr(1, 1);
                    std::string nominal = "0";

                    //if (Page_type_even.compare(nominal) != 0)
                    //        std::cout << "Alert frame "<< std::endl;
                    //else std::cout << "Nominal Page" << std::endl;

                    std::string Data_k = page_INAV.substr(2, 112);
                    std::string Odd_bit = page_INAV.substr(114, 1);
                    std::string Page_type_Odd = page_INAV.substr(115, 1);
                    std::string Data_j = page_INAV.substr(116, 16);

                    std::string Reserved_1 = page_INAV.substr(132, 40);
                    std::string SAR = page_INAV.substr(172, 22);
                    std::string Spare = page_INAV.substr(194, 2);
                    std::string CRC_data = page_INAV.substr(196, 24);
                    std::string Reserved_2 = page_INAV.substr(220, 8);
                    std::string Tail_odd = page_INAV.substr(228, 6);

                    // ************ CRC checksum control *******/
                    std::stringstream TLM_word_for_CRC_stream;
                    TLM_word_for_CRC_stream << page_INAV;
                    std::string TLM_word_for_CRC;
                    TLM_word_for_CRC = TLM_word_for_CRC_stream.str().substr(0, GALILEO_DATA_FRAME_BITS);
                    std::bitset<GALILEO_DATA_FRAME_BITS> TLM_word_for_CRC_bits(TLM_word_for_CRC);
                    std::bitset<24> checksum(CRC_data);

                    //if (Tail_odd.compare(correct_tail) != 0)
                    //        std::cout << "Tail odd is not correct!" << std::endl;
                    //else std::cout<<"Tail odd is correct!"<<std::endl;

                    if (CRC_test(TLM_word_for_CRC_bits, checksum.to_ulong()) == true)
                        {
                            flag_CRC_test = true;
                            // CRC correct: Decode word
                            std::string page_number_bits = Data_k.substr(0, 6);
                            std::bitset<GALILEO_PAGE_TYPE_BITS> page_type_bits(page_number_bits);  // from string to bitset
                            Page_type = static_cast<int32_t>(read_page_type_unsigned(page_type_bits, type));
                            Page_type_time_stamp = Page_type;
                            std::string Data_jk_ephemeris = Data_k + Data_j;
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
            std::string tail_Even = page_string.substr(114, 6);
            //std::cout << "tail_even_string: " << tail_Even <<std::endl;
            //if (tail_Even.compare(correct_tail) != 0)
            //     std::cout << "Tail even is not correct!" << std::endl;
            //else std::cout<<"Tail even is correct!"<< std::endl;
        }
}


bool Galileo_Navigation_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if ((flag_ephemeris_1 == true) and (flag_ephemeris_2 == true) and (flag_ephemeris_3 == true) and (flag_ephemeris_4 == true) and (flag_iono_and_GST == true))
        {
            // if all ephemeris pages have the same IOD, then they belong to the same block
            if ((IOD_nav_1 == IOD_nav_2) and (IOD_nav_3 == IOD_nav_4) and (IOD_nav_1 == IOD_nav_3))
                {
                    std::cout << "Ephemeris (1, 2, 3, 4) have been received and belong to the same batch" << std::endl;
                    flag_ephemeris_1 = false;  // clear the flag
                    flag_ephemeris_2 = false;  // clear the flag
                    flag_ephemeris_3 = false;  // clear the flag
                    flag_ephemeris_4 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    IOD_ephemeris = IOD_nav_1;
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


bool Galileo_Navigation_Message::have_new_iono_and_GST()  // Check if we have a new iono data set stored in the galileo navigation class
{
    if ((flag_iono_and_GST == true) and (flag_utc_model == true))  // the condition on flag_utc_model is added to have a time stamp for iono
        {
            flag_iono_and_GST = false;  // clear the flag
            return true;
        }
    else
        return false;
}


bool Galileo_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model == true)
        {
            flag_utc_model = false;  // clear the flag
            return true;
        }
    else
        return false;
}


bool Galileo_Navigation_Message::have_new_almanac()  // Check if we have a new almanac data set stored in the galileo navigation class
{
    if ((flag_almanac_1 == true) and (flag_almanac_2 == true) and (flag_almanac_3 == true) and (flag_almanac_4 == true))
        {
            // All almanac have been received
            flag_almanac_1 = false;
            flag_almanac_2 = false;
            flag_almanac_3 = false;
            flag_almanac_4 = false;
            flag_all_almanac = true;
            return true;
        }
    else
        return false;
}


Galileo_Ephemeris Galileo_Navigation_Message::get_ephemeris()
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


Galileo_Iono Galileo_Navigation_Message::get_iono()
{
    Galileo_Iono iono;
    // Ionospheric correction
    iono.ai0_5 = ai0_5;  // Effective Ionisation Level 1st order parameter [sfu]
    iono.ai1_5 = ai1_5;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    iono.ai2_5 = ai2_5;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // Ionospheric disturbance flag
    iono.Region1_flag_5 = Region1_flag_5;  // Ionospheric Disturbance Flag for region 1
    iono.Region2_flag_5 = Region2_flag_5;  // Ionospheric Disturbance Flag for region 2
    iono.Region3_flag_5 = Region3_flag_5;  // Ionospheric Disturbance Flag for region 3
    iono.Region4_flag_5 = Region4_flag_5;  // Ionospheric Disturbance Flag for region 4
    iono.Region5_flag_5 = Region5_flag_5;  // Ionospheric Disturbance Flag for region 5

    // GST
    // This is the ONLY page containing the Week Number (WN)
    iono.TOW_5 = TOW_5;
    iono.WN_5 = WN_5;
    return iono;
}


Galileo_Utc_Model Galileo_Navigation_Message::get_utc_model()
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
    return utc_model;
}


Galileo_Almanac Galileo_Navigation_Message::get_almanac()
{
    Galileo_Almanac almanac;
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

    // GPS to Galileo GST conversion parameters
    almanac.A_0G_10 = A_0G_10;
    almanac.A_1G_10 = A_1G_10;
    almanac.t_0G_10 = t_0G_10;
    almanac.WN_0G_10 = WN_0G_10;

    return almanac;
}


int32_t Galileo_Navigation_Message::page_jk_decoder(const char *data_jk)
{
    int32_t page_number = 0;

    std::string data_jk_string = data_jk;
    std::bitset<GALILEO_DATA_JK_BITS> data_jk_bits(data_jk_string);

    page_number = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, PAGE_TYPE_bit));
    LOG(INFO) << "Page number = " << page_number;

    switch (page_number)
        {
        case 1:  // Word type 1: Ephemeris (1/4)
            IOD_nav_1 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_nav_1_bit));
            DLOG(INFO) << "IOD_nav_1= " << IOD_nav_1;
            t0e_1 = static_cast<double>(read_navigation_unsigned(data_jk_bits, T0E_1_bit));
            t0e_1 = t0e_1 * t0e_1_LSB;
            DLOG(INFO) << "t0e_1= " << t0e_1;
            M0_1 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_1_bit));
            M0_1 = M0_1 * M0_1_LSB;
            DLOG(INFO) << "M0_1= " << M0_1;
            e_1 = static_cast<double>(read_navigation_unsigned(data_jk_bits, e_1_bit));
            e_1 = e_1 * e_1_LSB;
            DLOG(INFO) << "e_1= " << e_1;
            A_1 = static_cast<double>(read_navigation_unsigned(data_jk_bits, A_1_bit));
            A_1 = A_1 * A_1_LSB_gal;
            DLOG(INFO) << "A_1= " << A_1;
            flag_ephemeris_1 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 2:  // Word type 2: Ephemeris (2/4)
            IOD_nav_2 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_nav_2_bit));
            DLOG(INFO) << "IOD_nav_2= " << IOD_nav_2;
            OMEGA_0_2 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_0_2_bit));
            OMEGA_0_2 = OMEGA_0_2 * OMEGA_0_2_LSB;
            DLOG(INFO) << "OMEGA_0_2= " << OMEGA_0_2;
            i_0_2 = static_cast<double>(read_navigation_signed(data_jk_bits, i_0_2_bit));
            i_0_2 = i_0_2 * i_0_2_LSB;
            DLOG(INFO) << "i_0_2= " << i_0_2;
            omega_2 = static_cast<double>(read_navigation_signed(data_jk_bits, omega_2_bit));
            omega_2 = omega_2 * omega_2_LSB;
            DLOG(INFO) << "omega_2= " << omega_2;
            iDot_2 = static_cast<double>(read_navigation_signed(data_jk_bits, iDot_2_bit));
            iDot_2 = iDot_2 * iDot_2_LSB;
            DLOG(INFO) << "iDot_2= " << iDot_2;
            flag_ephemeris_2 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 3:  // Word type 3: Ephemeris (3/4) and SISA
            IOD_nav_3 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_nav_3_bit));
            DLOG(INFO) << "IOD_nav_3= " << IOD_nav_3;
            OMEGA_dot_3 = static_cast<double>(read_navigation_signed(data_jk_bits, OMEGA_dot_3_bit));
            OMEGA_dot_3 = OMEGA_dot_3 * OMEGA_dot_3_LSB;
            DLOG(INFO) << "OMEGA_dot_3= " << OMEGA_dot_3;
            delta_n_3 = static_cast<double>(read_navigation_signed(data_jk_bits, delta_n_3_bit));
            delta_n_3 = delta_n_3 * delta_n_3_LSB;
            DLOG(INFO) << "delta_n_3= " << delta_n_3;
            C_uc_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_uc_3_bit));
            C_uc_3 = C_uc_3 * C_uc_3_LSB;
            DLOG(INFO) << "C_uc_3= " << C_uc_3;
            C_us_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_us_3_bit));
            C_us_3 = C_us_3 * C_us_3_LSB;
            DLOG(INFO) << "C_us_3= " << C_us_3;
            C_rc_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_rc_3_bit));
            C_rc_3 = C_rc_3 * C_rc_3_LSB;
            DLOG(INFO) << "C_rc_3= " << C_rc_3;
            C_rs_3 = static_cast<double>(read_navigation_signed(data_jk_bits, C_rs_3_bit));
            C_rs_3 = C_rs_3 * C_rs_3_LSB;
            DLOG(INFO) << "C_rs_3= " << C_rs_3;
            SISA_3 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SISA_3_bit));
            DLOG(INFO) << "SISA_3= " << SISA_3;
            flag_ephemeris_3 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 4:  // Word type 4: Ephemeris (4/4) and Clock correction parameters
            IOD_nav_4 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, IOD_nav_4_bit));
            DLOG(INFO) << "IOD_nav_4= " << IOD_nav_4;
            SV_ID_PRN_4 = static_cast<int32_t>(read_navigation_unsigned(data_jk_bits, SV_ID_PRN_4_bit));
            DLOG(INFO) << "SV_ID_PRN_4= " << SV_ID_PRN_4;
            C_ic_4 = static_cast<double>(read_navigation_signed(data_jk_bits, C_ic_4_bit));
            C_ic_4 = C_ic_4 * C_ic_4_LSB;
            DLOG(INFO) << "C_ic_4= " << C_ic_4;
            C_is_4 = static_cast<double>(read_navigation_signed(data_jk_bits, C_is_4_bit));
            C_is_4 = C_is_4 * C_is_4_LSB;
            DLOG(INFO) << "C_is_4= " << C_is_4;
            // Clock correction parameters
            t0c_4 = static_cast<double>(read_navigation_unsigned(data_jk_bits, t0c_4_bit));
            t0c_4 = t0c_4 * t0c_4_LSB;
            DLOG(INFO) << "t0c_4= " << t0c_4;
            af0_4 = static_cast<double>(read_navigation_signed(data_jk_bits, af0_4_bit));
            af0_4 = af0_4 * af0_4_LSB;
            DLOG(INFO) << "af0_4 = " << af0_4;
            af1_4 = static_cast<double>(read_navigation_signed(data_jk_bits, af1_4_bit));
            af1_4 = af1_4 * af1_4_LSB;
            DLOG(INFO) << "af1_4 = " << af1_4;
            af2_4 = static_cast<double>(read_navigation_signed(data_jk_bits, af2_4_bit));
            af2_4 = af2_4 * af2_4_LSB;
            DLOG(INFO) << "af2_4 = " << af2_4;
            spare_4 = static_cast<double>(read_navigation_unsigned(data_jk_bits, spare_4_bit));
            DLOG(INFO) << "spare_4 = " << spare_4;
            flag_ephemeris_4 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 5:  // Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST
            // Ionospheric correction
            ai0_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, ai0_5_bit));
            ai0_5 = ai0_5 * ai0_5_LSB;
            DLOG(INFO) << "ai0_5= " << ai0_5;
            ai1_5 = static_cast<double>(read_navigation_signed(data_jk_bits, ai1_5_bit));
            ai1_5 = ai1_5 * ai1_5_LSB;
            DLOG(INFO) << "ai1_5= " << ai1_5;
            ai2_5 = static_cast<double>(read_navigation_signed(data_jk_bits, ai2_5_bit));
            ai2_5 = ai2_5 * ai2_5_LSB;
            DLOG(INFO) << "ai2_5= " << ai2_5;
            // Ionospheric disturbance flag
            Region1_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, Region1_5_bit));
            DLOG(INFO) << "Region1_flag_5= " << Region1_flag_5;
            Region2_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, Region2_5_bit));
            DLOG(INFO) << "Region2_flag_5= " << Region2_flag_5;
            Region3_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, Region3_5_bit));
            DLOG(INFO) << "Region3_flag_5= " << Region3_flag_5;
            Region4_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, Region4_5_bit));
            DLOG(INFO) << "Region4_flag_5= " << Region4_flag_5;
            Region5_flag_5 = static_cast<bool>(read_navigation_bool(data_jk_bits, Region5_5_bit));
            DLOG(INFO) << "Region5_flag_5= " << Region5_flag_5;
            BGD_E1E5a_5 = static_cast<double>(read_navigation_signed(data_jk_bits, BGD_E1E5a_5_bit));
            BGD_E1E5a_5 = BGD_E1E5a_5 * BGD_E1E5a_5_LSB;
            DLOG(INFO) << "BGD_E1E5a_5= " << BGD_E1E5a_5;
            BGD_E1E5b_5 = static_cast<double>(read_navigation_signed(data_jk_bits, BGD_E1E5b_5_bit));
            BGD_E1E5b_5 = BGD_E1E5b_5 * BGD_E1E5b_5_LSB;
            DLOG(INFO) << "BGD_E1E5b_5= " << BGD_E1E5b_5;
            E5b_HS_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E5b_HS_5_bit));
            DLOG(INFO) << "E5b_HS_5= " << E5b_HS_5;
            E1B_HS_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E1B_HS_5_bit));
            DLOG(INFO) << "E1B_HS_5= " << E1B_HS_5;
            E5b_DVS_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E5b_DVS_5_bit));
            DLOG(INFO) << "E5b_DVS_5= " << E5b_DVS_5;
            E1B_DVS_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E1B_DVS_5_bit));
            DLOG(INFO) << "E1B_DVS_5= " << E1B_DVS_5;
            // GST
            WN_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_5_bit));
            DLOG(INFO) << "WN_5= " << WN_5;
            TOW_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, TOW_5_bit));
            DLOG(INFO) << "TOW_5= " << TOW_5;
            flag_TOW_5 = true;  // set to false externally
            spare_5 = static_cast<double>(read_navigation_unsigned(data_jk_bits, spare_5_bit));
            DLOG(INFO) << "spare_5= " << spare_5;
            flag_iono_and_GST = true;  // set to false externally
            flag_TOW_set = true;       // set to false externally
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 6:  // Word type 6: GST-UTC conversion parameters
            A0_6 = static_cast<double>(read_navigation_signed(data_jk_bits, A0_6_bit));
            A0_6 = A0_6 * A0_6_LSB;
            DLOG(INFO) << "A0_6= " << A0_6;
            A1_6 = static_cast<double>(read_navigation_signed(data_jk_bits, A1_6_bit));
            A1_6 = A1_6 * A1_6_LSB;
            DLOG(INFO) << "A1_6= " << A1_6;
            Delta_tLS_6 = static_cast<double>(read_navigation_signed(data_jk_bits, Delta_tLS_6_bit));
            DLOG(INFO) << "Delta_tLS_6= " << Delta_tLS_6;
            t0t_6 = static_cast<double>(read_navigation_unsigned(data_jk_bits, t0t_6_bit));
            t0t_6 = t0t_6 * t0t_6_LSB;
            DLOG(INFO) << "t0t_6= " << t0t_6;
            WNot_6 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WNot_6_bit));
            DLOG(INFO) << "WNot_6= " << WNot_6;
            WN_LSF_6 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_LSF_6_bit));
            DLOG(INFO) << "WN_LSF_6= " << WN_LSF_6;
            DN_6 = static_cast<double>(read_navigation_unsigned(data_jk_bits, DN_6_bit));
            DLOG(INFO) << "DN_6= " << DN_6;
            Delta_tLSF_6 = static_cast<double>(read_navigation_signed(data_jk_bits, Delta_tLSF_6_bit));
            DLOG(INFO) << "Delta_tLSF_6= " << Delta_tLSF_6;
            TOW_6 = static_cast<double>(read_navigation_unsigned(data_jk_bits, TOW_6_bit));
            DLOG(INFO) << "TOW_6= " << TOW_6;
            flag_TOW_6 = true;      // set to false externally
            flag_utc_model = true;  // set to false externally
            flag_TOW_set = true;    // set to false externally
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 7:  // Word type 7: Almanac for SVID1 (1/2), almanac reference time and almanac reference week number
            IOD_a_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, IOD_a_7_bit));
            DLOG(INFO) << "IOD_a_7= " << IOD_a_7;
            WN_a_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_a_7_bit));
            DLOG(INFO) << "WN_a_7= " << WN_a_7;
            t0a_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, t0a_7_bit));
            t0a_7 = t0a_7 * t0a_7_LSB;
            DLOG(INFO) << "t0a_7= " << t0a_7;
            SVID1_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SVID1_7_bit));
            DLOG(INFO) << "SVID1_7= " << SVID1_7;
            DELTA_A_7 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_7_bit));
            DELTA_A_7 = DELTA_A_7 * DELTA_A_7_LSB;
            DLOG(INFO) << "DELTA_A_7= " << DELTA_A_7;
            e_7 = static_cast<double>(read_navigation_unsigned(data_jk_bits, e_7_bit));
            e_7 = e_7 * e_7_LSB;
            DLOG(INFO) << "e_7= " << e_7;
            omega_7 = static_cast<double>(read_navigation_signed(data_jk_bits, omega_7_bit));
            omega_7 = omega_7 * omega_7_LSB;
            DLOG(INFO) << "omega_7= " << omega_7;
            delta_i_7 = static_cast<double>(read_navigation_signed(data_jk_bits, delta_i_7_bit));
            delta_i_7 = delta_i_7 * delta_i_7_LSB;
            DLOG(INFO) << "delta_i_7= " << delta_i_7;
            Omega0_7 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega0_7_bit));
            Omega0_7 = Omega0_7 * Omega0_7_LSB;
            DLOG(INFO) << "Omega0_7= " << Omega0_7;
            Omega_dot_7 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega_dot_7_bit));
            Omega_dot_7 = Omega_dot_7 * Omega_dot_7_LSB;
            DLOG(INFO) << "Omega_dot_7= " << Omega_dot_7;
            M0_7 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_7_bit));
            M0_7 = M0_7 * M0_7_LSB;
            DLOG(INFO) << "M0_7= " << M0_7;
            flag_almanac_1 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 8:  // Word type 8: Almanac for SVID1 (2/2) and SVID2 (1/2)*/
            IOD_a_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, IOD_a_8_bit));
            DLOG(INFO) << "IOD_a_8= " << IOD_a_8;
            af0_8 = static_cast<double>(read_navigation_signed(data_jk_bits, af0_8_bit));
            af0_8 = af0_8 * af0_8_LSB;
            DLOG(INFO) << "af0_8= " << af0_8;
            af1_8 = static_cast<double>(read_navigation_signed(data_jk_bits, af1_8_bit));
            af1_8 = af1_8 * af1_8_LSB;
            DLOG(INFO) << "af1_8= " << af1_8;
            E5b_HS_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E5b_HS_8_bit));
            DLOG(INFO) << "E5b_HS_8= " << E5b_HS_8;
            E1B_HS_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E1B_HS_8_bit));
            DLOG(INFO) << "E1B_HS_8= " << E1B_HS_8;
            SVID2_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SVID2_8_bit));
            DLOG(INFO) << "SVID2_8= " << SVID2_8;
            DELTA_A_8 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_8_bit));
            DELTA_A_8 = DELTA_A_8 * DELTA_A_8_LSB;
            DLOG(INFO) << "DELTA_A_8= " << DELTA_A_8;
            e_8 = static_cast<double>(read_navigation_unsigned(data_jk_bits, e_8_bit));
            e_8 = e_8 * e_8_LSB;
            DLOG(INFO) << "e_8= " << e_8;
            omega_8 = static_cast<double>(read_navigation_signed(data_jk_bits, omega_8_bit));
            omega_8 = omega_8 * omega_8_LSB;
            DLOG(INFO) << "omega_8= " << omega_8;
            delta_i_8 = static_cast<double>(read_navigation_signed(data_jk_bits, delta_i_8_bit));
            delta_i_8 = delta_i_8 * delta_i_8_LSB;
            DLOG(INFO) << "delta_i_8= " << delta_i_8;
            Omega0_8 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega0_8_bit));
            Omega0_8 = Omega0_8 * Omega0_8_LSB;
            DLOG(INFO) << "Omega0_8= " << Omega0_8;
            Omega_dot_8 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega_dot_8_bit));
            Omega_dot_8 = Omega_dot_8 * Omega_dot_8_LSB;
            DLOG(INFO) << "Omega_dot_8= " << Omega_dot_8;
            flag_almanac_2 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 9:  // Word type 9: Almanac for SVID2 (2/2) and SVID3 (1/2)
            IOD_a_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, IOD_a_9_bit));
            DLOG(INFO) << "IOD_a_9= " << IOD_a_9;
            WN_a_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_a_9_bit));
            DLOG(INFO) << "WN_a_9= " << WN_a_9;
            t0a_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, t0a_9_bit));
            t0a_9 = t0a_9 * t0a_9_LSB;
            DLOG(INFO) << "t0a_9= " << t0a_9;
            M0_9 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_9_bit));
            M0_9 = M0_9 * M0_9_LSB;
            DLOG(INFO) << "M0_9= " << M0_9;
            af0_9 = static_cast<double>(read_navigation_signed(data_jk_bits, af0_9_bit));
            af0_9 = af0_9 * af0_9_LSB;
            DLOG(INFO) << "af0_9= " << af0_9;
            af1_9 = static_cast<double>(read_navigation_signed(data_jk_bits, af1_9_bit));
            af1_9 = af1_9 * af1_9_LSB;
            DLOG(INFO) << "af1_9= " << af1_9;
            E5b_HS_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E5b_HS_9_bit));
            DLOG(INFO) << "E5b_HS_9= " << E5b_HS_9;
            E1B_HS_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E1B_HS_9_bit));
            DLOG(INFO) << "E1B_HS_9= " << E1B_HS_9;
            SVID3_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, SVID3_9_bit));
            DLOG(INFO) << "SVID3_9= " << SVID3_9;
            DELTA_A_9 = static_cast<double>(read_navigation_signed(data_jk_bits, DELTA_A_9_bit));
            DELTA_A_9 = DELTA_A_9 * DELTA_A_9_LSB;
            DLOG(INFO) << "DELTA_A_9= " << DELTA_A_9;
            e_9 = static_cast<double>(read_navigation_unsigned(data_jk_bits, e_9_bit));
            e_9 = e_9 * e_9_LSB;
            DLOG(INFO) << "e_9= " << e_9;
            omega_9 = static_cast<double>(read_navigation_signed(data_jk_bits, omega_9_bit));
            omega_9 = omega_9 * omega_9_LSB;
            DLOG(INFO) << "omega_9= " << omega_9;
            delta_i_9 = static_cast<double>(read_navigation_signed(data_jk_bits, delta_i_9_bit));
            delta_i_9 = delta_i_9 * delta_i_9_LSB;
            DLOG(INFO) << "delta_i_9= " << delta_i_9;
            flag_almanac_3 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 10:  // Word type 10: Almanac for SVID3 (2/2) and GST-GPS conversion parameters
            IOD_a_10 = static_cast<double>(read_navigation_unsigned(data_jk_bits, IOD_a_10_bit));
            DLOG(INFO) << "IOD_a_10= " << IOD_a_10;
            Omega0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega0_10_bit));
            Omega0_10 = Omega0_10 * Omega0_10_LSB;
            DLOG(INFO) << "Omega0_10= " << Omega0_10;
            Omega_dot_10 = static_cast<double>(read_navigation_signed(data_jk_bits, Omega_dot_10_bit));
            Omega_dot_10 = Omega_dot_10 * Omega_dot_10_LSB;
            DLOG(INFO) << "Omega_dot_10= " << Omega_dot_10;
            M0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, M0_10_bit));
            M0_10 = M0_10 * M0_10_LSB;
            DLOG(INFO) << "M0_10= " << M0_10;
            af0_10 = static_cast<double>(read_navigation_signed(data_jk_bits, af0_10_bit));
            af0_10 = af0_10 * af0_10_LSB;
            DLOG(INFO) << "af0_10= " << af0_10;
            af1_10 = static_cast<double>(read_navigation_signed(data_jk_bits, af1_10_bit));
            af1_10 = af1_10 * af1_10_LSB;
            DLOG(INFO) << "af1_10= " << af1_10;
            E5b_HS_10 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E5b_HS_10_bit));
            DLOG(INFO) << "E5b_HS_10= " << E5b_HS_10;
            E1B_HS_10 = static_cast<double>(read_navigation_unsigned(data_jk_bits, E1B_HS_10_bit));
            DLOG(INFO) << "E1B_HS_10= " << E1B_HS_10;
            A_0G_10 = static_cast<double>(read_navigation_signed(data_jk_bits, A_0G_10_bit));
            A_0G_10 = A_0G_10 * A_0G_10_LSB;
            flag_GGTO_1 = true;
            DLOG(INFO) << "A_0G_10= " << A_0G_10;
            A_1G_10 = static_cast<double>(read_navigation_signed(data_jk_bits, A_1G_10_bit));
            A_1G_10 = A_1G_10 * A_1G_10_LSB;
            flag_GGTO_2 = true;
            DLOG(INFO) << "A_1G_10= " << A_1G_10;
            t_0G_10 = static_cast<double>(read_navigation_unsigned(data_jk_bits, t_0G_10_bit));
            t_0G_10 = t_0G_10 * t_0G_10_LSB;
            flag_GGTO_3 = true;
            DLOG(INFO) << "t_0G_10= " << t_0G_10;
            WN_0G_10 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_0G_10_bit));
            flag_GGTO_4 = true;
            DLOG(INFO) << "WN_0G_10= " << WN_0G_10;
            flag_almanac_4 = true;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;

        case 0:  // Word type 0: I/NAV Spare Word
            Time_0 = static_cast<double>(read_navigation_unsigned(data_jk_bits, Time_0_bit));
            DLOG(INFO) << "Time_0= " << Time_0;
            WN_0 = static_cast<double>(read_navigation_unsigned(data_jk_bits, WN_0_bit));
            DLOG(INFO) << "WN_0= " << WN_0;
            TOW_0 = static_cast<double>(read_navigation_unsigned(data_jk_bits, TOW_0_bit));
            DLOG(INFO) << "TOW_0= " << TOW_0;
            DLOG(INFO) << "flag_tow_set" << flag_TOW_set;
            break;
        }
    return page_number;
}

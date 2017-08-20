/*!
 * \file glonass_gnav_navigation_message.cc
 * \brief  Implementation of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
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

#include "glonass_gnav_navigation_message.h"
#include <boost/crc.hpp>
#include <boost/dynamic_bitset.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <gnss_satellite.h>


void Glonass_Gnav_Navigation_Message::reset()
{
    //!< Satellite Identification
    i_channel_ID = 0;               //!< Channel ID assigned by the receiver
    i_satellite_freq_channel = 0;   //!< SV Frequency Slot Number
    i_satellite_slot_number = 0;    //!< SV Orbit Slot Number

    //!< Ephmeris Flags
    flag_all_ephemeris = false;
    flag_ephemeris_str_1 = false;
    flag_ephemeris_str_2 = false;
    flag_ephemeris_str_3 = false;
    flag_ephemeris_str_4 = false;

    //!< Almanac Flags
    flag_all_almanac = false;
    flag_almanac_str_6  = false;
    flag_almanac_str_7  = false;
    flag_almanac_str_8  = false;
    flag_almanac_str_9  = false;
    flag_almanac_str_10 = false;
    flag_almanac_str_11 = false;
    flag_almanac_str_12 = false;
    flag_almanac_str_13 = false;
    flag_almanac_str_14 = false;
    flag_almanac_str_15 = false;

    //!< UTC and System Clocks Flags
    flag_utc_model_valid  = false;      //!< If set, it indicates that the UTC model parameters are filled
    flag_utc_model_str_5 = false;      //!< Clock info send in string 5 of navigation data
    flag_utc_model_str_15 = false;     //!< Clock info send in string 15 of frame 5 of navigation data
    flag_TOW_5 = false;
    flag_TOW_6 = false;
    flag_TOW_set = false;              //!< it is true when page 5 or page 6 arrives

    //broadcast orbit 1
    //TODO Need to send the information regarding the frame number
    d_TOW = false;           //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    d_TOW_F1 = false;        //!< Time of GPS Week from HOW word of Subframe 1 [s]
    d_TOW_F2 = false;        //!< Time of GPS Week from HOW word of Subframe 2 [s]
    d_TOW_F3 = false;        //!< Time of GPS Week from HOW word of Subframe 3 [s]
    d_TOW_F4 = false;        //!< Time of GPS Week from HOW word of Subframe 4 [s]
    d_TOW_F5 = false;        //!< Time of GPS Week from HOW word of Subframe 5 [s]

    // Clock terms
    d_satClkCorr = 0.0;
    d_dtr = 0.0;
    d_satClkDrift = 0.0;


    std::map<int,std::string> satelliteBlock; //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    auto gnss_sat = Gnss_Satellite();
    std::string _system ("GLONASS");
    //TODO SHould number of channels be hardcoded?
    for(unsigned int i = 1; i < 14; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Glonass_Gnav_Navigation_Message::Glonass_Gnav_Navigation_Message()
{
    reset();
}


bool Glonass_Gnav_Navigation_Message::CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS> bits)
{
    int sum_bits = 0;
    int sum_hamming = 0;
    int C1 = 0;
    int C2 = 0;
    int C3 = 0;
    int C4 = 0;
    int C5 = 0;
    int C6 = 0;
    int C7 = 0;
    int C_Sigma = 0;
    std::vector<int> string_bits(GLONASS_GNAV_STRING_BITS);

	//!< Populate data and hamming code vectors
	for(int i = 0; i < static_cast<int>(GLONASS_GNAV_STRING_BITS); i++)
		{
			string_bits[i] = static_cast<int>(bits[i]);
		}


    //!< Compute C1 term
    sum_bits = 0;
    for(int i = 0; i < static_cast<int>(GLONASS_GNAV_CRC_I_INDEX.size()); i++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_I_INDEX[i]];
        }
    C1 = string_bits[0]^(sum_bits%2);

    //!< Compute C2 term
    sum_bits = 0;
    for(int j = 0; j < static_cast<int>(GLONASS_GNAV_CRC_J_INDEX.size()); j++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_J_INDEX[j]];
        }
    C2 = (string_bits[1])^(sum_bits%2);

    //!< Compute C3 term
    sum_bits = 0;
    for(int k = 0; k < static_cast<int>(GLONASS_GNAV_CRC_K_INDEX.size()); k++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_K_INDEX[k]];
        }
    C3 = string_bits[2]^(sum_bits%2);

    //!< Compute C4 term
    sum_bits = 0;
    for(int l = 0; l < static_cast<int>(GLONASS_GNAV_CRC_L_INDEX.size()); l++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_L_INDEX[l]];
        }
    C4 = string_bits[3]^(sum_bits%2);

    //!< Compute C5 term
    sum_bits = 0;
    for(int m = 0; m < static_cast<int>(GLONASS_GNAV_CRC_M_INDEX.size()); m++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_M_INDEX[m]];
        }
    C5 = string_bits[4]^(sum_bits%2);

    //!< Compute C6 term
    sum_bits = 0;
    for(int n = 0; n < static_cast<int>(GLONASS_GNAV_CRC_N_INDEX.size()); n++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_N_INDEX[n]];
        }
    C6 = string_bits[5]^(sum_bits%2);

    //!< Compute C7 term
    sum_bits = 0;
    for(int p = 0; p < static_cast<int>(GLONASS_GNAV_CRC_P_INDEX.size()); p++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_P_INDEX[p]];
        }
    C7 = string_bits[6]^(sum_bits%2);

    //!< Compute C_Sigma term
    sum_bits = 0;
    sum_hamming = 0;
    for(int q = 0; q < static_cast<int>(GLONASS_GNAV_CRC_Q_INDEX.size()); q++)
        {
            sum_bits += string_bits[GLONASS_GNAV_CRC_Q_INDEX[q]];
        }
    for(int q = 0; q < 8; q++)
        {
            sum_hamming += string_bits[q];
        }
    C_Sigma = (sum_hamming%2)^(sum_bits%2);

    //!< Verification of the data
    // All of the checksums are equal to zero
    if((C1 & C2 & C3 & C4 & C5 & C6 & C7 & C_Sigma) == 0 )
        {
            return true;
        }
    // only one of the checksums (C1,...,C7) is equal to zero but C_Sigma = 1
    else if(C_Sigma == 1 && C1+C2+C3+C4+C5+C6+C7 == 6)
        {
            return true;
        }
    else
        {
            return false;
        }
}


bool Glonass_Gnav_Navigation_Message::read_navigation_bool(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    bool value;

    if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Glonass_Gnav_Navigation_Message::read_navigation_unsigned(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}


signed long int Glonass_Gnav_Navigation_Message::read_navigation_signed(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    signed long int value = 0;
    signed long int sign = 0;
    int num_of_slices = parameter.size();
    // read the MSB and perform the sign extension
	if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
		{
			sign = -1;
		}
	else
		{
			sign = 1;
		}
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return (sign*value);
}


unsigned int Glonass_Gnav_Navigation_Message::get_frame_number(unsigned int satellite_slot_number)
{
    unsigned int frame_ID = 0;

    if(satellite_slot_number >= 1 and satellite_slot_number <= 5 )
        {
            frame_ID = 1;
        }
    else if(satellite_slot_number >= 6 and satellite_slot_number <= 10 )
        {
            frame_ID = 2;
        }
    else if(satellite_slot_number >= 11 and satellite_slot_number <= 15 )
        {
            frame_ID = 3;
        }
    else if(satellite_slot_number >= 16 and satellite_slot_number <= 20 )
        {
            frame_ID = 4;
        }
    else if(satellite_slot_number >= 21 and satellite_slot_number <= 24 )
        {
            frame_ID = 5;
        }
    else
        {
            //TODO Find print statement and make it an error
            frame_ID = 0;
        }

    return frame_ID;
}


int Glonass_Gnav_Navigation_Message::string_decoder(char * frame_string)
{
    int string_ID = 0;
    int J = 0;
    frame_ID = 0;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<GLONASS_GNAV_STRING_BITS> string_bits = std::bitset<GLONASS_GNAV_STRING_BITS>(std::string(frame_string));
    string_ID = static_cast<int>(read_navigation_unsigned(string_bits, STRING_ID));

    CRC_test(string_bits);

    // Decode all 15 string messages
    switch (string_ID)
        {
        case 1:
            //--- It is string 1 -----------------------------------------------
            gnav_ephemeris.d_P_1 = (static_cast<double>(read_navigation_unsigned(string_bits, P1)) + 1)*15;
            gnav_ephemeris.d_t_k =  static_cast<double>(read_navigation_unsigned(string_bits, T_K_HR)) * 3600 +
                                    static_cast<double>(read_navigation_unsigned(string_bits, T_K_MIN)) * 60 +
                                    static_cast<double>(read_navigation_unsigned(string_bits, T_K_SEC)) * 30;
            gnav_ephemeris.d_VXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT)) * TWO_N20;
            gnav_ephemeris.d_AXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT_DOT)) * TWO_N30;
            gnav_ephemeris.d_Xn = static_cast<double>(read_navigation_signed(string_bits, X_N)) * TWO_N11;

            flag_ephemeris_str_1 = true;

            break;

        case 2:
            //--- It is string 2 -----------------------------------------------
            if (flag_ephemeris_str_1 == true)
                {
                    gnav_ephemeris.d_B_n = static_cast<double>(read_navigation_unsigned(string_bits, B_N));
                    gnav_ephemeris.d_P_2 = static_cast<double>(read_navigation_unsigned(string_bits, P2));
                    gnav_ephemeris.d_t_b = static_cast<double>(read_navigation_unsigned(string_bits, T_B))*gnav_ephemeris.d_P_1*60;
                    gnav_ephemeris.d_VYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT))* TWO_N20;
                    gnav_ephemeris.d_AYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT_DOT)) * TWO_N30;
                    gnav_ephemeris.d_Yn = static_cast<double>(read_navigation_signed(string_bits, Y_N)) * TWO_N11;

                    gnav_ephemeris.d_iode   = read_navigation_unsigned(string_bits, T_B);
                    flag_ephemeris_str_2 = true;
                }

            break;

        case 3:
            // --- It is string 3 ----------------------------------------------
            gnav_ephemeris.d_P_3 = static_cast<double>(read_navigation_unsigned(string_bits, P3));
            gnav_ephemeris.d_gamma_n = static_cast<double>(read_navigation_signed(string_bits, GAMMA_N)) * TWO_N40;
            gnav_ephemeris.d_P = static_cast<double>(read_navigation_unsigned(string_bits, P));
            gnav_ephemeris.d_l3rd_n = static_cast<double>(read_navigation_unsigned(string_bits, EPH_L_N));
            gnav_ephemeris.d_VZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT)) * TWO_N20;
            gnav_ephemeris.d_AZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT_DOT)) * TWO_N30;
            gnav_ephemeris.d_Zn = static_cast<double>(read_navigation_signed(string_bits, Z_N)) * TWO_N11;

            flag_ephemeris_str_3 = true;

            break;

        case 4:
            // --- It is string 4 ----------------------------------------------
            gnav_ephemeris.d_tau_n = static_cast<double>(read_navigation_signed(string_bits, TAU_N)) * TWO_N30;
            gnav_ephemeris.d_Delta_tau_n = static_cast<double>(read_navigation_signed(string_bits, DELTA_TAU_N)) * TWO_N30;
            gnav_ephemeris.d_E_n = static_cast<double>(read_navigation_unsigned(string_bits, E_N));
            gnav_ephemeris.d_P_4 = static_cast<double>(read_navigation_unsigned(string_bits, P4));
            gnav_ephemeris.d_F_T = static_cast<double>(read_navigation_unsigned(string_bits, F_T));
            gnav_ephemeris.d_N_T = static_cast<double>(read_navigation_unsigned(string_bits, N_T));
            gnav_ephemeris.d_n = static_cast<double>(read_navigation_unsigned(string_bits, N));
            gnav_ephemeris.d_M = static_cast<double>(read_navigation_unsigned(string_bits, M));

            // Fill in ephemeris deliverables in the code
            gnav_ephemeris.i_satellite_slot_number = gnav_ephemeris.d_n;
            gnav_ephemeris.i_satellite_PRN = gnav_ephemeris.d_n;
            gnav_ephemeris.d_D4Y = gnav_ephemeris.d_N_T;

            flag_ephemeris_str_4 = true;

            break;

        case 5:
            // --- It is string 5 ----------------------------------------------
            gnav_utc_model.d_N_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_utc_model.d_tau_c = static_cast<double>(read_navigation_signed(string_bits, TAU_C)) * TWO_N31;
            gnav_utc_model.d_N_4 = static_cast<double>(read_navigation_unsigned(string_bits, N_4));
            gnav_utc_model.d_tau_gps = static_cast<double>(read_navigation_signed(string_bits, TAU_GPS)) * TWO_N30;
            gnav_ephemeris.d_l5th_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

            // Compute Year and DoY based on Algorithm A3.11 of GLONASS ICD
            if(flag_ephemeris_str_4 == true)
                {
                    //Current year number J in the four-year interval is calculated:
                    if(gnav_ephemeris.d_N_T >= 1 && gnav_ephemeris.d_N_T <= 366)
                        {
                            J = 1;
                        }
                    else if ( gnav_ephemeris.d_N_T >= 367 && gnav_ephemeris.d_N_T <= 731)
                        {
                            J = 2;
                        }
                    else if (gnav_ephemeris.d_N_T >= 732  && gnav_ephemeris.d_N_T <= 1096)
                        {
                            J = 3;
                        }
                    else if (gnav_ephemeris.d_N_T >= 1097 && gnav_ephemeris.d_N_T <= 1461)
                        {
                            J = 4;
                        }
                    // 2). Current year in common form is calculated by the following formula:
                    gnav_ephemeris.d_yr = 1996 + 4.0*(gnav_utc_model.d_N_4  - 1.0) + (J - 1.0);
                }
            break;

        case 6:
            // --- It is string 6 ----------------------------------------------
            i_satellite_slot_number = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            frame_ID = get_frame_number(i_satellite_slot_number);

            gnav_almanac[i_satellite_slot_number - 1].d_C_n = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_6 = true;

            break;

        case 7:
            // --- It is string 7 ----------------------------------------------
            if (flag_almanac_str_6 == true)
              {
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15;
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                // Set satellite information for redundancy purposes
                if(gnav_almanac[i_satellite_slot_number - 1].d_H_n_A > 24)
                    {
                        gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].d_H_n_A - 32.0;
                    }
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_satellite_slot_number - 1].d_n_A;
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_PRN = gnav_almanac[i_satellite_slot_number - 1].d_n_A;

                if(i_satellite_slot_number == gnav_ephemeris.i_satellite_slot_number)
                    {
                        gnav_ephemeris.i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel;
                    }
                flag_almanac_str_7 = true;
              }


            break;
        case 8:
            // --- It is string 8 ----------------------------------------------
            i_satellite_slot_number = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            frame_ID = get_frame_number(i_satellite_slot_number);

            gnav_almanac[i_satellite_slot_number - 1].d_C_n = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_8 = true;

            break;

        case 9:
            // --- It is string 9 ----------------------------------------------
            if (flag_almanac_str_8 == true)
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15;
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A)) -32.0;
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                // Set satellite information for redundancy purposes
                if(gnav_almanac[i_satellite_slot_number - 1].d_H_n_A > 24)
                    {
                        gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].d_H_n_A - 32.0;
                    }
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_satellite_slot_number - 1].d_n_A;
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_PRN = gnav_almanac[i_satellite_slot_number - 1].d_n_A;

                flag_almanac_str_9 = true;
              }
            break;
        case 10:
            // --- It is string 10 ---------------------------------------------
            i_satellite_slot_number = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            frame_ID = get_frame_number(i_satellite_slot_number);

            gnav_almanac[i_satellite_slot_number - 1].d_C_n = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_10 = true;

            break;

        case 11:
            // --- It is string 11 ---------------------------------------------
            if (flag_almanac_str_10 == true)
              {
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15;
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A)) - 32.0;
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                // Set satellite information for redundancy purposes
                if(gnav_almanac[i_satellite_slot_number - 1].d_H_n_A > 24)
                    {
                        gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].d_H_n_A - 32.0;
                    }
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_satellite_slot_number - 1].d_n_A;
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_PRN = gnav_almanac[i_satellite_slot_number - 1].d_n_A;

                flag_almanac_str_11 = true;
              }
            break;
        case 12:
            // --- It is string 12 ---------------------------------------------
            i_satellite_slot_number = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            frame_ID = get_frame_number(i_satellite_slot_number);

            gnav_almanac[i_satellite_slot_number - 1].d_C_n = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20;
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_12 = true;

            break;

        case 13:
            // --- It is string 13 ---------------------------------------------
            if (flag_almanac_str_12 == true)
              {
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15;
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A)) - 32.0;
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                // Set satellite information for redundancy purposes
                if(gnav_almanac[i_satellite_slot_number - 1].d_H_n_A > 24)
                    {
                        gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].d_H_n_A - 32.0;
                    }
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_satellite_slot_number - 1].d_n_A;
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_PRN = gnav_almanac[i_satellite_slot_number - 1].d_n_A;

                flag_almanac_str_13 = true;
              }
            break;
        case 14:
            // --- It is string 14 ---------------------------------------------
            if( frame_ID == 5)
              {
                gnav_utc_model.d_B1 = static_cast<double>(read_navigation_unsigned(string_bits, B1));
                gnav_utc_model.d_B2 = static_cast<double>(read_navigation_unsigned(string_bits, B2));
              }
            else
              {
                i_satellite_slot_number = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
                frame_ID = get_frame_number(i_satellite_slot_number);

                gnav_almanac[i_satellite_slot_number - 1].d_C_n = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
                gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, n_A));
                gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
                gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20;
                gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

                flag_almanac_str_14 = true;
              }


            break;

        case 15:
            // --- It is string 9 ----------------------------------------------
            if (frame_ID != 5 and flag_almanac_str_14 == true )
              {
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15;
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A)) - 32.0;
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                // Set satellite information for redundancy purposes
                if(gnav_almanac[i_satellite_slot_number - 1].d_H_n_A > 24)
                    {
                        gnav_almanac[i_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_satellite_slot_number - 1].d_H_n_A - 32.0;
                    }
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_satellite_slot_number - 1].d_n_A;
                gnav_almanac[i_satellite_slot_number - 1].i_satellite_PRN = gnav_almanac[i_satellite_slot_number - 1].d_n_A;

                flag_almanac_str_15 = true;
              }
            break;
        default:
            break;
        } // switch subframeID ...

    return frame_ID;
}


double Glonass_Gnav_Navigation_Message::utc_time(const double glonass_time_corrected) const
{
    double t_utc;

    t_utc = glonass_time_corrected + 3*3600 + gnav_utc_model.d_tau_c;
    return t_utc;
}


Glonass_Gnav_Ephemeris Glonass_Gnav_Navigation_Message::get_ephemeris()
{
    return gnav_ephemeris;
}


Glonass_Gnav_Utc_Model Glonass_Gnav_Navigation_Message::get_utc_model()
{
    return gnav_utc_model;
}


Glonass_Gnav_Almanac Glonass_Gnav_Navigation_Message::get_almanac( int satellite_slot_number)
{
    return gnav_almanac[satellite_slot_number - 1];
}


bool Glonass_Gnav_Navigation_Message::have_new_ephemeris() //Check if we have a new ephemeris stored in the galileo navigation class
{
    bool flag_data_valid = false;
    bool b_valid_ephemeris_set_flag = false;

    if ((flag_ephemeris_str_1 == true) and (flag_ephemeris_str_2 == true) and (flag_ephemeris_str_3 == true) and (flag_ephemeris_str_4 == true))
        {
            //if all ephemeris pages have the same IOD, then they belong to the same block
            if ((gnav_ephemeris.d_t_b == 0) )
                {
                    flag_ephemeris_str_1 = false;// clear the flag
                    flag_ephemeris_str_2 = false;// clear the flag
                    flag_ephemeris_str_3 = false;// clear the flag
                    flag_ephemeris_str_4 = false;// clear the flag
                    flag_all_ephemeris = true;
                    // std::cout << "Ephemeris (1, 2, 3, 4) have been received and belong to the same batch" << std::endl;
                    // std::cout << "Batch number: "<< IOD_ephemeris << std::endl;
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


bool Glonass_Gnav_Navigation_Message::have_new_utc_model() // Check if we have a new utc data set stored in the galileo navigation class
{
    bool flag_utc_model = true;
    if (flag_utc_model == true)
        {
            flag_utc_model = false; // clear the flag
            return true;
        }
    else
        return false;
}


bool Glonass_Gnav_Navigation_Message::have_new_almanac() //Check if we have a new almanac data set stored in the galileo navigation class
{
    if ((flag_almanac_str_6 == true) and (flag_almanac_str_7 == true) and
        (flag_almanac_str_8 == true) and (flag_almanac_str_9 == true) and
        (flag_almanac_str_10 == true) and (flag_almanac_str_11 == true) and
        (flag_almanac_str_12 == true) and (flag_almanac_str_13 == true) and
        (flag_almanac_str_14 == true) and (flag_almanac_str_15 == true))
        {
            //All almanac have been received
            flag_almanac_str_6 = false;
            flag_almanac_str_7 = false;
            flag_almanac_str_8 = false;
            flag_almanac_str_9 = false;
            flag_almanac_str_10 = false;
            flag_almanac_str_11 = false;
            flag_almanac_str_12 = false;
            flag_almanac_str_13 = false;
            flag_almanac_str_14 = false;
            flag_almanac_str_15 = false;
            flag_all_almanac = true;
            return true;
        }
    else
        return false;
}

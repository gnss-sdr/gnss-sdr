/*!
 * \file glonass_gnav_navigation_message.cc
 * \brief  Implementation of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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

#include "glonass_gnav_navigation_message.h"
#include "MATH_CONSTANTS.h"  // for TWO_N20, TWO_N30, TWO_N14, TWO_N15, TWO_N18
#include "gnss_satellite.h"
#include <glog/logging.h>
#include <cstddef>  // for size_t
#include <ostream>  // for operator<<


Glonass_Gnav_Navigation_Message::Glonass_Gnav_Navigation_Message()
{
    auto gnss_sat = Gnss_Satellite();
    std::string _system("GLONASS");
    // TODO SHould number of channels be hardcoded?
    for (uint32_t i = 1; i < 14; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


bool Glonass_Gnav_Navigation_Message::CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS>& bits) const
{
    uint32_t sum_bits = 0;
    int32_t sum_hamming = 0;
    std::vector<uint32_t> string_bits(GLONASS_GNAV_STRING_BITS);

    // Populate data and hamming code vectors
    for (size_t i = 0; i < string_bits.size(); i++)
        {
            string_bits[i] = static_cast<uint32_t>(bits[i]);
        }

    // Compute C1 term
    sum_bits = 0;
    for (int i : GLONASS_GNAV_CRC_I_INDEX)
        {
            sum_bits += string_bits[i - 1];
        }
    const int32_t C1 = string_bits[0] ^ (sum_bits % 2);

    // Compute C2 term
    sum_bits = 0;
    for (int j : GLONASS_GNAV_CRC_J_INDEX)
        {
            sum_bits += string_bits[j - 1];
        }
    const int32_t C2 = (string_bits[1]) ^ (sum_bits % 2);

    // Compute C3 term
    sum_bits = 0;
    for (int k : GLONASS_GNAV_CRC_K_INDEX)
        {
            sum_bits += string_bits[k - 1];
        }
    const int32_t C3 = string_bits[2] ^ (sum_bits % 2);

    // Compute C4 term
    sum_bits = 0;
    for (int l : GLONASS_GNAV_CRC_L_INDEX)
        {
            sum_bits += string_bits[l - 1];
        }
    const int32_t C4 = string_bits[3] ^ (sum_bits % 2);

    // Compute C5 term
    sum_bits = 0;
    for (int m : GLONASS_GNAV_CRC_M_INDEX)
        {
            sum_bits += string_bits[m - 1];
        }
    const int32_t C5 = string_bits[4] ^ (sum_bits % 2);

    // Compute C6 term
    sum_bits = 0;
    for (int n : GLONASS_GNAV_CRC_N_INDEX)
        {
            sum_bits += string_bits[n - 1];
        }
    const int32_t C6 = string_bits[5] ^ (sum_bits % 2);

    // Compute C7 term
    sum_bits = 0;
    for (int p : GLONASS_GNAV_CRC_P_INDEX)
        {
            sum_bits += string_bits[p - 1];
        }
    const int32_t C7 = string_bits[6] ^ (sum_bits % 2);

    // Compute C_Sigma term
    sum_bits = 0;
    for (int q : GLONASS_GNAV_CRC_Q_INDEX)
        {
            sum_bits += string_bits[q - 1];
        }
    for (int32_t q = 0; q < 8; q++)
        {
            sum_hamming += string_bits[q];
        }
    const int32_t C_Sigma = (sum_hamming % 2) ^ (sum_bits % 2);

    // Verification of the data
    // (a-i) All checksums (C1,...,C7 and C_Sigma) are equal to zero
    if ((C1 + C2 + C3 + C4 + C5 + C6 + C7 + C_Sigma) == 0)
        {
            return true;
        }
    // (a-ii) Only one of the checksums (C1,...,C7) is equal to 1 and C_Sigma = 1
    if (C_Sigma == 1 && C1 + C2 + C3 + C4 + C5 + C6 + C7 == 1)
        {
            return true;
        }

    if (C_Sigma && (sum_bits & 1))
        {
            int32_t syndrome = C1;
            syndrome |= (C2 ? 2 : 0);
            syndrome |= (C3 ? 4 : 0);
            syndrome |= (C4 ? 8 : 0);
            syndrome |= (C5 ? 16 : 0);
            syndrome |= (C6 ? 32 : 0);
            syndrome |= (C7 ? 64 : 0);
            if (syndrome < 85)
                {
                    const int32_t locator = GLONASS_GNAV_ECC_LOCATOR[syndrome];
                    bits[locator] = !bits[locator];
                    return true;
                }
            else
                {
                    return false;
                }
        }

    // All other conditions are assumed errors.
    return false;
}


bool Glonass_Gnav_Navigation_Message::read_navigation_bool(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
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


uint64_t Glonass_Gnav_Navigation_Message::read_navigation_unsigned(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    uint64_t value = 0ULL;
    const int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1ULL;  // shift left
                    if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1ULL;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Glonass_Gnav_Navigation_Message::read_navigation_signed(const std::bitset<GLONASS_GNAV_STRING_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    int64_t value = 0LL;
    int64_t sign = 0LL;
    const int32_t num_of_slices = parameter.size();
    // read the MSB and perform the sign extension
    if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
        {
            sign = -1LL;
        }
    else
        {
            sign = 1LL;
        }
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1;  // shift left
                    if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1LL;  // insert the bit
                        }
                }
        }
    return (sign * value);
}


uint32_t Glonass_Gnav_Navigation_Message::get_frame_number(uint32_t satellite_slot_number)
{
    uint32_t frame_ID = 0U;

    if (satellite_slot_number >= 1 and satellite_slot_number <= 5)
        {
            frame_ID = 1U;
        }
    else if (satellite_slot_number >= 6 and satellite_slot_number <= 10)
        {
            frame_ID = 2U;
        }
    else if (satellite_slot_number >= 11 and satellite_slot_number <= 15)
        {
            frame_ID = 3U;
        }
    else if (satellite_slot_number >= 16 and satellite_slot_number <= 20)
        {
            frame_ID = 4U;
        }
    else if (satellite_slot_number >= 21 and satellite_slot_number <= 24)
        {
            frame_ID = 5U;
        }
    else
        {
            LOG(WARNING) << "GLONASS GNAV: Invalid Satellite Slot Number";
            frame_ID = 0U;
        }

    return frame_ID;
}


int32_t Glonass_Gnav_Navigation_Message::string_decoder(const std::string& frame_string)
{
    int32_t J = 0;
    d_frame_ID = 0U;
    uint64_t P_1_tmp = 0;

    // Unpack bytes to bits
    std::bitset<GLONASS_GNAV_STRING_BITS> string_bits(frame_string);

    // Perform data verification and exit code if error in bit sequence
    flag_CRC_test = CRC_test(string_bits);
    if (flag_CRC_test == false)
        {
            return 0;
        }

    // Decode all 15 string messages
    d_string_ID = static_cast<uint32_t>(read_navigation_unsigned(string_bits, STRING_ID));
    switch (d_string_ID)
        {
        case 1:
            // --- It is string 1 -----------------------------------------------
            P_1_tmp = read_navigation_unsigned(string_bits, P1);
            gnav_ephemeris.d_P_1 = (P_1_tmp == 0) ? 0. : (P_1_tmp + 1) * 15;
            gnav_ephemeris.d_t_k = static_cast<double>(read_navigation_unsigned(string_bits, T_K_HR)) * 3600 +
                                   static_cast<double>(read_navigation_unsigned(string_bits, T_K_MIN)) * 60 +
                                   static_cast<double>(read_navigation_unsigned(string_bits, T_K_SEC)) * 30;
            gnav_ephemeris.d_VXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT)) * TWO_N20;
            gnav_ephemeris.d_AXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT_DOT)) * TWO_N30;
            gnav_ephemeris.d_Xn = static_cast<double>(read_navigation_signed(string_bits, X_N)) * TWO_N11;

            flag_ephemeris_str_1 = true;

            break;

        case 2:
            // --- It is string 2 -----------------------------------------------
            if (flag_ephemeris_str_1 == true)
                {
                    gnav_ephemeris.d_B_n = static_cast<double>(read_navigation_unsigned(string_bits, B_N));
                    gnav_ephemeris.d_P_2 = static_cast<bool>(read_navigation_bool(string_bits, P2));
                    gnav_ephemeris.d_t_b = static_cast<double>(read_navigation_unsigned(string_bits, T_B)) * 15 * 60;
                    gnav_ephemeris.d_VYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT)) * TWO_N20;
                    gnav_ephemeris.d_AYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT_DOT)) * TWO_N30;
                    gnav_ephemeris.d_Yn = static_cast<double>(read_navigation_signed(string_bits, Y_N)) * TWO_N11;

                    gnav_ephemeris.d_iode = read_navigation_unsigned(string_bits, T_B);
                    flag_ephemeris_str_2 = true;
                }

            break;

        case 3:
            // --- It is string 3 ----------------------------------------------
            if (flag_ephemeris_str_2 == true)
                {
                    gnav_ephemeris.d_P_3 = static_cast<bool>(read_navigation_bool(string_bits, P3));
                    gnav_ephemeris.d_gamma_n = static_cast<double>(read_navigation_signed(string_bits, GAMMA_N)) * TWO_N40;
                    gnav_ephemeris.d_P = static_cast<double>(read_navigation_unsigned(string_bits, P));
                    gnav_ephemeris.d_l3rd_n = static_cast<bool>(read_navigation_bool(string_bits, EPH_L_N));
                    gnav_ephemeris.d_VZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT)) * TWO_N20;
                    gnav_ephemeris.d_AZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT_DOT)) * TWO_N30;
                    gnav_ephemeris.d_Zn = static_cast<double>(read_navigation_signed(string_bits, Z_N)) * TWO_N11;

                    flag_ephemeris_str_3 = true;
                }

            break;

        case 4:
            // --- It is string 4 ----------------------------------------------
            if (flag_ephemeris_str_3 == true)
                {
                    gnav_ephemeris.d_tau_n = static_cast<double>(read_navigation_signed(string_bits, TAU_N)) * TWO_N30;
                    gnav_ephemeris.d_Delta_tau_n = static_cast<double>(read_navigation_signed(string_bits, DELTA_TAU_N)) * TWO_N30;
                    gnav_ephemeris.d_E_n = static_cast<double>(read_navigation_unsigned(string_bits, E_N));
                    gnav_ephemeris.d_P_4 = static_cast<bool>(read_navigation_bool(string_bits, P4));
                    gnav_ephemeris.d_F_T = static_cast<double>(read_navigation_unsigned(string_bits, F_T));
                    gnav_ephemeris.d_N_T = static_cast<double>(read_navigation_unsigned(string_bits, N_T));
                    gnav_ephemeris.d_n = static_cast<double>(read_navigation_unsigned(string_bits, N));
                    gnav_ephemeris.d_M = static_cast<double>(read_navigation_unsigned(string_bits, M));

                    // Fill in ephemeris deliverables in the code
                    flag_update_slot_number = true;
                    gnav_ephemeris.i_satellite_slot_number = static_cast<uint32_t>(gnav_ephemeris.d_n);
                    gnav_ephemeris.PRN = static_cast<uint32_t>(gnav_ephemeris.d_n);

                    flag_ephemeris_str_4 = true;
                }

            break;

        case 5:
            // --- It is string 5 ----------------------------------------------
            if (flag_ephemeris_str_4 == true)
                {
                    gnav_utc_model.d_N_A = static_cast<double>(read_navigation_unsigned(string_bits, DAY_NUMBER_A));
                    gnav_utc_model.d_tau_c = static_cast<double>(read_navigation_signed(string_bits, TAU_C)) * TWO_N31;
                    gnav_utc_model.d_N_4 = static_cast<double>(read_navigation_unsigned(string_bits, N_4));
                    gnav_utc_model.d_tau_gps = static_cast<double>(read_navigation_signed(string_bits, TAU_GPS)) * TWO_N30;
                    gnav_ephemeris.d_l5th_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    flag_utc_model_str_5 = true;

                    // Compute Year and DoY based on Algorithm A3.11 of GLONASS ICD
                    // 1). Current year number J in the four-year interval is calculated
                    if (gnav_ephemeris.d_N_T >= 1 && gnav_ephemeris.d_N_T <= 366)
                        {
                            J = 1;
                        }
                    else if (gnav_ephemeris.d_N_T >= 367 && gnav_ephemeris.d_N_T <= 731)
                        {
                            J = 2;
                        }
                    else if (gnav_ephemeris.d_N_T >= 732 && gnav_ephemeris.d_N_T <= 1096)
                        {
                            J = 3;
                        }
                    else if (gnav_ephemeris.d_N_T >= 1097 && gnav_ephemeris.d_N_T <= 1461)
                        {
                            J = 4;
                        }
                    // 2). Current year in common form is calculated by the following formula:
                    gnav_ephemeris.d_yr = 1996 + 4.0 * (gnav_utc_model.d_N_4 - 1.0) + (J - 1.0);
                    gnav_ephemeris.d_tau_c = gnav_utc_model.d_tau_c;

                    // 3). Set TOW once the year has been defined, it helps with leap second determination
                    if (flag_ephemeris_str_1 == true)
                        {
                            gnav_ephemeris.glot_to_gpst(gnav_ephemeris.d_t_k + 10, gnav_utc_model.d_tau_c, gnav_utc_model.d_tau_gps, &gnav_ephemeris.d_WN, &gnav_ephemeris.d_TOW);
                            flag_TOW_set = true;
                            flag_TOW_new = true;
                        }

                    // 4) Set time of day (tod) when ephemeris data is complety decoded
                    gnav_ephemeris.d_tod = gnav_ephemeris.d_t_k + 2 * d_string_ID;
                }

            break;

        case 6:
            // --- It is string 6 ----------------------------------------------
            i_alm_satellite_slot_number = static_cast<uint32_t>(read_navigation_unsigned(string_bits, N_A));
            d_frame_ID = get_frame_number(i_alm_satellite_slot_number);
            // Make sure a valid frame_ID or satellite slot number is returned
            if (d_frame_ID == 0)
                {
                    return 0;
                }

            gnav_almanac[i_alm_satellite_slot_number - 1].d_C_n = static_cast<bool>(read_navigation_bool(string_bits, C_N));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_6 = true;

            break;

        case 7:
            // --- It is string 7 ----------------------------------------------
            if (flag_almanac_str_6 == true)
                {
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_l_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    // Set satellite information for redundancy purposes
                    if (gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A > 24)
                        {
                            gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A - 32.0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;
                    gnav_almanac[i_alm_satellite_slot_number - 1].PRN = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;

                    if (i_alm_satellite_slot_number == gnav_ephemeris.i_satellite_slot_number)
                        {
                            gnav_ephemeris.i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel;
                        }
                    flag_almanac_str_7 = true;
                }

            break;

        case 8:
            // --- It is string 8 ----------------------------------------------
            i_alm_satellite_slot_number = static_cast<uint32_t>(read_navigation_unsigned(string_bits, N_A));
            d_frame_ID = get_frame_number(i_alm_satellite_slot_number);
            // Make sure a valid frame_ID or satellite slot number is returned
            if (d_frame_ID == 0)
                {
                    return 0;
                }

            gnav_almanac[i_alm_satellite_slot_number - 1].d_C_n = static_cast<bool>(read_navigation_bool(string_bits, C_N));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_8 = true;

            break;

        case 9:
            // --- It is string 9 ----------------------------------------------
            if (flag_almanac_str_8 == true)
                {
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_l_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    // Set satellite information for redundancy purposes
                    if (gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A > 24)
                        {
                            gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A - 32.0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;
                    gnav_almanac[i_alm_satellite_slot_number - 1].PRN = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;

                    flag_almanac_str_9 = true;
                }
            break;

        case 10:
            // --- It is string 10 ---------------------------------------------
            i_alm_satellite_slot_number = static_cast<uint32_t>(read_navigation_unsigned(string_bits, N_A));
            d_frame_ID = get_frame_number(i_alm_satellite_slot_number);
            // Make sure a valid frame_ID or satellite slot number is returned
            if (d_frame_ID == 0)
                {
                    return 0;
                }

            gnav_almanac[i_alm_satellite_slot_number - 1].d_C_n = static_cast<bool>(read_navigation_bool(string_bits, C_N));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_10 = true;

            break;

        case 11:
            // --- It is string 11 ---------------------------------------------
            if (flag_almanac_str_10 == true)
                {
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_l_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    // Set satellite information for redundancy purposes
                    if (gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A > 24)
                        {
                            gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A - 32.0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;
                    gnav_almanac[i_alm_satellite_slot_number - 1].PRN = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;

                    flag_almanac_str_11 = true;
                }
            break;

        case 12:
            // --- It is string 12 ---------------------------------------------
            i_alm_satellite_slot_number = static_cast<uint32_t>(read_navigation_unsigned(string_bits, N_A));
            d_frame_ID = get_frame_number(i_alm_satellite_slot_number);
            // Make sure a valid frame_ID or satellite slot number is returned
            if (d_frame_ID == 0)
                {
                    return 0;
                }
            gnav_almanac[i_alm_satellite_slot_number - 1].d_C_n = static_cast<bool>(read_navigation_bool(string_bits, C_N));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_alm_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20 * GNSS_PI;
            gnav_almanac[i_alm_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

            flag_almanac_str_12 = true;

            break;

        case 13:
            // --- It is string 13 ---------------------------------------------
            if (flag_almanac_str_12 == true)
                {
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_l_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    // Set satellite information for redundancy purposes
                    if (gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A > 24)
                        {
                            gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A - 32.0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;
                    gnav_almanac[i_alm_satellite_slot_number - 1].PRN = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;

                    flag_almanac_str_13 = true;
                }
            break;

        case 14:
            // --- It is string 14 ---------------------------------------------
            if (d_frame_ID == 5)
                {
                    gnav_utc_model.d_B1 = static_cast<double>(read_navigation_unsigned(string_bits, B1));
                    gnav_utc_model.d_B2 = static_cast<double>(read_navigation_unsigned(string_bits, B2));
                }
            else
                {
                    i_alm_satellite_slot_number = static_cast<uint32_t>(read_navigation_unsigned(string_bits, N_A));
                    d_frame_ID = get_frame_number(i_alm_satellite_slot_number);
                    // Make sure a valid frame_ID or satellite slot number is returned
                    if (d_frame_ID == 0)
                        {
                            return 0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_C_n = static_cast<bool>(read_navigation_bool(string_bits, C_N));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A)) * TWO_N18;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_signed(string_bits, LAMBDA_N_A)) * TWO_N20 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_I_N_A)) * TWO_N20 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A)) * TWO_N20;

                    flag_almanac_str_14 = true;
                }
            break;

        case 15:
            // --- It is string 15 ----------------------------------------------
            if (d_frame_ID != 5 and flag_almanac_str_14 == true)
                {
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_signed(string_bits, OMEGA_N_A)) * TWO_N15 * GNSS_PI;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A)) * TWO_N5;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_N_A)) * TWO_N9;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_signed(string_bits, DELTA_T_DOT_N_A)) * TWO_N14;
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                    gnav_almanac[i_alm_satellite_slot_number - 1].d_l_n = static_cast<bool>(read_navigation_bool(string_bits, ALM_L_N));

                    // Set satellite information for redundancy purposes
                    if (gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A > 24)
                        {
                            gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_freq_channel = gnav_almanac[i_alm_satellite_slot_number - 1].d_H_n_A - 32.0;
                        }
                    gnav_almanac[i_alm_satellite_slot_number - 1].i_satellite_slot_number = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;
                    gnav_almanac[i_alm_satellite_slot_number - 1].PRN = gnav_almanac[i_alm_satellite_slot_number - 1].d_n_A;

                    flag_almanac_str_15 = true;
                }
            break;
        default:
            LOG(INFO) << "GLONASS GNAV: Invalid String ID of received. Received " << d_string_ID
                      << ", but acceptable range is from 1-15";

            break;
        }  // switch string ID

    return d_string_ID;
}


Glonass_Gnav_Almanac Glonass_Gnav_Navigation_Message::get_almanac(uint32_t satellite_slot_number) const
{
    return gnav_almanac[satellite_slot_number - 1];
}


bool Glonass_Gnav_Navigation_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    bool new_eph = false;
    // We need to make sure we have received the ephemeris info plus the time info
    if ((flag_ephemeris_str_1 == true) and (flag_ephemeris_str_2 == true) and
        (flag_ephemeris_str_3 == true) and (flag_ephemeris_str_4 == true) and
        (flag_utc_model_str_5 == true))
        {
            if (d_previous_tb != gnav_ephemeris.d_t_b)
                {
                    flag_ephemeris_str_1 = false;  // clear the flag
                    flag_ephemeris_str_2 = false;  // clear the flag
                    flag_ephemeris_str_3 = false;  // clear the flag
                    flag_ephemeris_str_4 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    // Update the time of ephemeris information
                    d_previous_tb = gnav_ephemeris.d_t_b;
                    DLOG(INFO) << "GLONASS GNAV Ephemeris (1, 2, 3, 4) have been received and belong to the same batch";
                    new_eph = true;
                }
        }

    return new_eph;
}


bool Glonass_Gnav_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model_str_5 == true)
        {
            flag_utc_model_str_5 = false;  // clear the flag
            return true;
        }

    return false;
}


bool Glonass_Gnav_Navigation_Message::have_new_almanac()  // Check if we have a new almanac data set stored in the galileo navigation class
{
    bool new_alm = false;
    if ((flag_almanac_str_6 == true) and (flag_almanac_str_7 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != gnav_utc_model.d_N_A)
                {
                    // All Almanac data have been received for this satellite
                    flag_almanac_str_6 = false;
                    flag_almanac_str_7 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_8 == true) and (flag_almanac_str_9 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != gnav_utc_model.d_N_A)
                {
                    flag_almanac_str_8 = false;
                    flag_almanac_str_9 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_10 == true) and (flag_almanac_str_11 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != gnav_utc_model.d_N_A)
                {
                    flag_almanac_str_10 = false;
                    flag_almanac_str_11 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_12 == true) and (flag_almanac_str_13 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != gnav_utc_model.d_N_A)
                {
                    flag_almanac_str_12 = false;
                    flag_almanac_str_13 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_14 == true) and (flag_almanac_str_15 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != gnav_utc_model.d_N_A)
                {
                    flag_almanac_str_14 = false;
                    flag_almanac_str_15 = false;
                    new_alm = true;
                }
        }

    return new_alm;
}

/*!
m * \file gps_navigation_message.cc
 * \brief  Implementation of a GPS NAV Data message decoder as described in IS-GPS-200E
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_navigation_message.h"
#include "gnss_satellite.h"
#include <cmath>
#include <iostream>


void Gps_Navigation_Message::reset()
{
    b_valid_ephemeris_set_flag = false;
    d_TOW = 0.0;
    d_TOW_SF1 = 0;
    d_TOW_SF2 = 0;
    d_TOW_SF3 = 0;
    d_TOW_SF4 = 0;
    d_TOW_SF5 = 0;
    d_IODE_SF2 = 0;
    d_IODE_SF3 = 0;
    d_Crs = 0;
    d_Delta_n = 0;
    d_M_0 = 0;
    d_Cuc = 0;
    d_e_eccentricity = 0;
    d_Cus = 0;
    d_sqrt_A = 0;
    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0;
    d_OMEGA0 = 0;
    d_Cis = 0;
    d_i_0 = 0;
    d_Crc = 0;
    d_OMEGA = 0;
    d_OMEGA_DOT = 0;
    d_IDOT = 0;
    i_code_on_L2 = 0;
    i_GPS_week = 0;
    b_L2_P_data_flag = false;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD = 0;
    d_IODC = -1.0;
    i_AODO = 0;

    b_fit_interval_flag = false;
    d_spare1 = 0;
    d_spare2 = 0;

    d_A_f0 = 0;
    d_A_f1 = 0;
    d_A_f2 = 0;

    //clock terms
    //d_master_clock=0;
    d_dtr = 0;
    d_satClkCorr = 0;
    d_satClkDrift = 0;

    // satellite positions
    d_satpos_X = 0;
    d_satpos_Y = 0;
    d_satpos_Z = 0;

    // info
    i_channel_ID = 0;
    i_satellite_PRN = 0;

    // time synchro
    d_subframe_timestamp_ms = 0;

    // flags
    b_alert_flag = false;
    b_integrity_status_flag = false;
    b_antispoofing_flag = false;

    // Ionosphere and UTC
    flag_iono_valid = false;
    flag_utc_model_valid = false;
    d_alpha0 = 0;
    d_alpha1 = 0;
    d_alpha2 = 0;
    d_alpha3 = 0;
    d_beta0 = 0;
    d_beta1 = 0;
    d_beta2 = 0;
    d_beta3 = 0;
    d_A1 = 0;
    d_A0 = 0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0;

    //Almanac
    d_Toa = 0;
    i_WN_A = 0;
    for (int i = 1; i < 32; i++)
        {
            almanacHealth[i] = 0;
        }

    // Satellite velocity
    d_satvel_X = 0;
    d_satvel_Y = 0;
    d_satvel_Z = 0;

    auto gnss_sat = Gnss_Satellite();
    std::string _system("GPS");
    for (unsigned int i = 1; i < 33; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Gps_Navigation_Message::Gps_Navigation_Message()
{
    reset();
}


void Gps_Navigation_Message::print_gps_word_bytes(unsigned int GPS_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(GPS_word);
    std::cout << std::endl;
}


bool Gps_Navigation_Message::read_navigation_bool(std::bitset<GPS_SUBFRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    bool value;

    if (bits[GPS_SUBFRAME_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Gps_Navigation_Message::read_navigation_unsigned(std::bitset<GPS_SUBFRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[GPS_SUBFRAME_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


signed long int Gps_Navigation_Message::read_navigation_signed(std::bitset<GPS_SUBFRAME_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8)  // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GPS_SUBFRAME_BITS - parameter[0].first] == 1)
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
                            if (bits[GPS_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GPS_SUBFRAME_BITS - parameter[0].first] == 1)
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
                            if (bits[GPS_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    return value;
}

int Gps_Navigation_Message::subframe_decoder(char *subframe)
{
    int subframe_ID = 0;

    //double tmp_TOW;

    unsigned int gps_word;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<GPS_SUBFRAME_BITS> subframe_bits;
    std::bitset<GPS_WORD_BITS + 2> word_bits;
    for (int i = 0; i < 10; i++)
        {
            memcpy(&gps_word, &subframe[i * 4], sizeof(char) * 4);
            word_bits = std::bitset<(GPS_WORD_BITS + 2)>(gps_word);
            for (int j = 0; j < GPS_WORD_BITS; j++)
                {
                    subframe_bits[GPS_WORD_BITS * (9 - i) + j] = word_bits[j];
                }
        }

    subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, SUBFRAME_ID));

    // Decode all 5 sub-frames
    switch (subframe_ID)
    {
    //--- Decode the sub-frame id ------------------------------------------
    // ICD (IS-GPS-200E Appendix II). http://www.losangeles.af.mil/shared/media/document/AFD-100813-045.pdf
    case 1:
        //--- It is subframe 1 -------------------------------------
        // Compute the time of week (TOW) of the first sub-frames in the array ====
        // The transmitted TOW is actual TOW of the next subframe
        // (the variable subframe at this point contains bits of the last subframe).
        //TOW = bin2dec(subframe(31:47)) * 6;
        d_TOW_SF1 = static_cast<double>(read_navigation_unsigned(subframe_bits, TOW));
        //we are in the first subframe (the transmitted TOW is the start time of the next subframe) !
        d_TOW_SF1 = d_TOW_SF1 * 6.0;
        d_TOW = d_TOW_SF1; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        i_GPS_week = static_cast<int>(read_navigation_unsigned(subframe_bits, GPS_WEEK));
        i_SV_accuracy = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_ACCURACY));  // (20.3.3.3.1.3)
        i_SV_health = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_HEALTH));
        b_L2_P_data_flag = read_navigation_bool(subframe_bits, L2_P_DATA_FLAG); //
        i_code_on_L2 = static_cast<int>(read_navigation_unsigned(subframe_bits, CA_OR_P_ON_L2));
        d_TGD = static_cast<double>(read_navigation_signed(subframe_bits, T_GD));
        d_TGD = d_TGD * T_GD_LSB;
        d_IODC = static_cast<double>(read_navigation_unsigned(subframe_bits, IODC));
        d_Toc = static_cast<double>(read_navigation_unsigned(subframe_bits, T_OC));
        d_Toc = d_Toc * T_OC_LSB;
        d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, A_F0));
        d_A_f0 = d_A_f0 * A_F0_LSB;
        d_A_f1 = static_cast<double>(read_navigation_signed(subframe_bits, A_F1));
        d_A_f1 = d_A_f1 * A_F1_LSB;
        d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, A_F2));
        d_A_f2 = d_A_f2 * A_F2_LSB;

            break;

    case 2:  //--- It is subframe 2 -------------------
        d_TOW_SF2 = static_cast<double>(read_navigation_unsigned(subframe_bits, TOW));
        d_TOW_SF2 = d_TOW_SF2 * 6.0;
        d_TOW = d_TOW_SF2; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        d_IODE_SF2 = static_cast<double>(read_navigation_unsigned(subframe_bits, IODE_SF2));
        d_Crs = static_cast<double>(read_navigation_signed(subframe_bits, C_RS));
        d_Crs = d_Crs * C_RS_LSB;
        d_Delta_n = static_cast<double>(read_navigation_signed(subframe_bits, DELTA_N));
        d_Delta_n = d_Delta_n * DELTA_N_LSB;
        d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, M_0));
        d_M_0 = d_M_0 * M_0_LSB;
        d_Cuc = static_cast<double>(read_navigation_signed(subframe_bits, C_UC));
        d_Cuc = d_Cuc * C_UC_LSB;
        d_e_eccentricity = static_cast<double>(read_navigation_unsigned(subframe_bits, E));
        d_e_eccentricity = d_e_eccentricity * E_LSB;
        d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, C_US));
        d_Cus = d_Cus * C_US_LSB;
        d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, SQRT_A));
        d_sqrt_A = d_sqrt_A * SQRT_A_LSB;
        d_Toe = static_cast<double>(read_navigation_unsigned(subframe_bits, T_OE));
        d_Toe = d_Toe * T_OE_LSB;
        b_fit_interval_flag = read_navigation_bool(subframe_bits, FIT_INTERVAL_FLAG);
        i_AODO = static_cast<int>(read_navigation_unsigned(subframe_bits, AODO));
        i_AODO = i_AODO * AODO_LSB;

            break;

    case 3: // --- It is subframe 3 -------------------------------------
        d_TOW_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, TOW));
        d_TOW_SF3 = d_TOW_SF3 * 6.0;
        d_TOW = d_TOW_SF3; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        d_Cic = static_cast<double>(read_navigation_signed(subframe_bits, C_IC));
        d_Cic = d_Cic * C_IC_LSB;
        d_OMEGA0 = static_cast<double>(read_navigation_signed(subframe_bits, OMEGA_0));
        d_OMEGA0 = d_OMEGA0 * OMEGA_0_LSB;
        d_Cis = static_cast<double>(read_navigation_signed(subframe_bits, C_IS));
        d_Cis = d_Cis * C_IS_LSB;
        d_i_0 = static_cast<double>(read_navigation_signed(subframe_bits, I_0));
        d_i_0 = d_i_0 * I_0_LSB;
        d_Crc = static_cast<double>(read_navigation_signed(subframe_bits, C_RC));
        d_Crc = d_Crc * C_RC_LSB;
        d_OMEGA = static_cast<double>(read_navigation_signed(subframe_bits, OMEGA));
        d_OMEGA = d_OMEGA * OMEGA_LSB;
        d_OMEGA_DOT = static_cast<double>(read_navigation_signed(subframe_bits, OMEGA_DOT));
        d_OMEGA_DOT = d_OMEGA_DOT * OMEGA_DOT_LSB;
        d_IODE_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, IODE_SF3));
        d_IDOT = static_cast<double>(read_navigation_signed(subframe_bits, I_DOT));
        d_IDOT = d_IDOT * I_DOT_LSB;

            break;

    case 4: // --- It is subframe 4 ---------- Almanac, ionospheric model, UTC parameters, SV health (PRN: 25-32)
        int SV_data_ID;
        int SV_page;
        d_TOW_SF4 = static_cast<double>(read_navigation_unsigned(subframe_bits, TOW));
        d_TOW_SF4 = d_TOW_SF4 * 6.0;
        d_TOW = d_TOW_SF4; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        SV_data_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_DATA_ID));
        SV_page = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_PAGE));
        if (SV_page > 24 && SV_page < 33) // Page 4 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                //! \TODO read almanac
                if(SV_data_ID){}
            }

            if (SV_page == 52)  // Page 13 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
                {
                    //! \TODO read Estimated Range Deviation (ERD) values
                }

            if (SV_page == 56)  // Page 18 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
                {
                    // Page 18 - Ionospheric and UTC data
                    d_alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, ALPHA_0));
                    d_alpha0 = d_alpha0 * ALPHA_0_LSB;
                    d_alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, ALPHA_1));
                    d_alpha1 = d_alpha1 * ALPHA_1_LSB;
                    d_alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, ALPHA_2));
                    d_alpha2 = d_alpha2 * ALPHA_2_LSB;
                    d_alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, ALPHA_3));
                    d_alpha3 = d_alpha3 * ALPHA_3_LSB;
                    d_beta0 = static_cast<double>(read_navigation_signed(subframe_bits, BETA_0));
                    d_beta0 = d_beta0 * BETA_0_LSB;
                    d_beta1 = static_cast<double>(read_navigation_signed(subframe_bits, BETA_1));
                    d_beta1 = d_beta1 * BETA_1_LSB;
                    d_beta2 = static_cast<double>(read_navigation_signed(subframe_bits, BETA_2));
                    d_beta2 = d_beta2 * BETA_2_LSB;
                    d_beta3 = static_cast<double>(read_navigation_signed(subframe_bits, BETA_3));
                    d_beta3 = d_beta3 * BETA_3_LSB;
                    d_A1 = static_cast<double>(read_navigation_signed(subframe_bits, A_1));
                    d_A1 = d_A1 * A_1_LSB;
                    d_A0 = static_cast<double>(read_navigation_signed(subframe_bits, A_0));
                    d_A0 = d_A0 * A_0_LSB;
                    d_t_OT = static_cast<double>(read_navigation_unsigned(subframe_bits, T_OT));
                    d_t_OT = d_t_OT * T_OT_LSB;
                    i_WN_T = static_cast<int>(read_navigation_unsigned(subframe_bits, WN_T));
                    d_DeltaT_LS = static_cast<double>(read_navigation_signed(subframe_bits, DELTAT_LS));
                    i_WN_LSF = static_cast<int>(read_navigation_unsigned(subframe_bits, WN_LSF));
                    i_DN = static_cast<int>(read_navigation_unsigned(subframe_bits, DN));  // Right-justified ?
                    d_DeltaT_LSF = static_cast<double>(read_navigation_signed(subframe_bits, DELTAT_LSF));
                    flag_iono_valid = true;
                    flag_utc_model_valid = true;
                }
            if (SV_page == 57)
                {
                    // Reserved
                }

            if (SV_page == 63)  // Page 25 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
                {
                    // Page 25 Anti-Spoofing, SV config and almanac health (PRN: 25-32)
                    //! \TODO Read Anti-Spoofing, SV config
                    almanacHealth[25] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV25));
                    almanacHealth[26] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV26));
                    almanacHealth[27] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV27));
                    almanacHealth[28] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV28));
                    almanacHealth[29] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV29));
                    almanacHealth[30] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV30));
                    almanacHealth[31] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV31));
                    almanacHealth[32] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV32));
                }

            break;

    case 5://--- It is subframe 5 -----------------almanac health (PRN: 1-24) and Almanac reference week number and time.
        int SV_data_ID_5;
        int SV_page_5;
        d_TOW_SF5 = static_cast<double>(read_navigation_unsigned(subframe_bits, TOW));
        d_TOW_SF5 = d_TOW_SF5 * 6.0;
        d_TOW = d_TOW_SF5; // Set transmission time
        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        SV_data_ID_5 = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_DATA_ID));
        SV_page_5 = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_PAGE));
        if (SV_page_5 < 25)
            {
                //! \TODO read almanac
                if(SV_data_ID_5){}
            }
        if (SV_page_5 == 51) // Page 25 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                d_Toa = static_cast<double>(read_navigation_unsigned(subframe_bits, T_OA));
                d_Toa = d_Toa * T_OA_LSB;
                i_WN_A = static_cast<int>(read_navigation_unsigned(subframe_bits, WN_A));
                almanacHealth[1] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV1));
                almanacHealth[2] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV2));
                almanacHealth[3] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV3));
                almanacHealth[4] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV4));
                almanacHealth[5] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV5));
                almanacHealth[6] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV6));
                almanacHealth[7] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV7));
                almanacHealth[8] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV8));
                almanacHealth[9] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV9));
                almanacHealth[10] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV10));
                almanacHealth[11] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV11));
                almanacHealth[12] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV12));
                almanacHealth[13] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV13));
                almanacHealth[14] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV14));
                almanacHealth[15] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV15));
                almanacHealth[16] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV16));
                almanacHealth[17] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV17));
                almanacHealth[18] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV18));
                almanacHealth[19] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV19));
                almanacHealth[20] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV20));
                almanacHealth[21] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV21));
                almanacHealth[22] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV22));
                almanacHealth[23] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV23));
                almanacHealth[24] = static_cast<int>(read_navigation_unsigned(subframe_bits, HEALTH_SV24));
            }
        break;

        default:
            break;
        }  // switch subframeID ...

    return subframe_ID;
}


double Gps_Navigation_Message::utc_time(const double gpstime_corrected) const
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC = d_DeltaT_LS + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>((i_GPS_week - i_WN_T)));

    // Determine if the effectivity time of the leap second event is in the past
    int weeksToLeapSecondEvent = i_WN_LSF - i_GPS_week;

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
        {
            //Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                }
            else  //we are in the same week than the leap second event
                {
                    if (std::abs(gpstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            /* 20.3.3.5.2.4a
                             * Whenever the effectivity time indicated by the WN_LSF and the DN values
                             * is not in the past (relative to the user's present time), and the user's
                             * present time does not fall in the time span which starts at six hours prior
                             * to the effectivity time and ends at six hours after the effectivity time,
                             * the UTC/GPS-time relationship is given by
                             */
                            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            /* 20.3.3.5.2.4b
                             * Whenever the user's current time falls within the time span of six hours
                             * prior to the effectivity time to six hours after the effectivity time,
                             * proper accommodation of the leap second event with a possible week number
                             * transition is provided by the following expression for UTC:
                             */
                            int W = fmod(gpstime_corrected - Delta_t_UTC - 43200, 86400) + 43200;
                            t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - d_DeltaT_LS);
                            //implement something to handle a leap second event!
                        }
                    if ((gpstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>((i_GPS_week - i_WN_T)));
                            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                        }
                }
        }
    else  // the effectivity time is in the past
        {
            /* 20.3.3.5.2.4c
             * Whenever the effectivity time of the leap second event, as indicated by the
             * WNLSF and DN values, is in the "past" (relative to the user's current time),
             * and the user�s current time does not fall in the time span as given above
             * in 20.3.3.5.2.4b,*/
            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>((i_GPS_week - i_WN_T)));
            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 43200 * floor(gpstime_corrected / 43200);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}


Gps_Ephemeris Gps_Navigation_Message::get_ephemeris()
{
    Gps_Ephemeris ephemeris;
    ephemeris.i_satellite_PRN = i_satellite_PRN;
    ephemeris.d_TOW = d_TOW;
    ephemeris.d_Crs = d_Crs;
    ephemeris.d_Delta_n = d_Delta_n;
    ephemeris.d_M_0 = d_M_0;
    ephemeris.d_Cuc = d_Cuc;
    ephemeris.d_e_eccentricity = d_e_eccentricity;
    ephemeris.d_Cus = d_Cus;
    ephemeris.d_sqrt_A = d_sqrt_A;
    ephemeris.d_Toe = d_Toe;
    ephemeris.d_Toc = d_Toc;
    ephemeris.d_Cic = d_Cic;
    ephemeris.d_OMEGA0 = d_OMEGA0;
    ephemeris.d_Cis = d_Cis;
    ephemeris.d_i_0 = d_i_0;
    ephemeris.d_Crc = d_Crc;
    ephemeris.d_OMEGA = d_OMEGA;
    ephemeris.d_OMEGA_DOT = d_OMEGA_DOT;
    ephemeris.d_IDOT = d_IDOT;
    ephemeris.i_code_on_L2 = i_code_on_L2;
    ephemeris.i_GPS_week = i_GPS_week;
    ephemeris.b_L2_P_data_flag = b_L2_P_data_flag;
    ephemeris.i_SV_accuracy = i_SV_accuracy;
    ephemeris.i_SV_health = i_SV_health;
    ephemeris.d_TGD = d_TGD;
    ephemeris.d_IODC = d_IODC;
    ephemeris.d_IODE_SF2 = d_IODE_SF2;
    ephemeris.d_IODE_SF3 = d_IODE_SF3;
    ephemeris.i_AODO = i_AODO;
    ephemeris.b_fit_interval_flag = b_fit_interval_flag;
    ephemeris.d_spare1 = d_spare1;
    ephemeris.d_spare2 = d_spare2;
    ephemeris.d_A_f0 = d_A_f0;
    ephemeris.d_A_f1 = d_A_f1;
    ephemeris.d_A_f2 = d_A_f2;
    ephemeris.b_integrity_status_flag = b_integrity_status_flag;
    ephemeris.b_alert_flag = b_alert_flag;
    ephemeris.b_antispoofing_flag = b_antispoofing_flag;
    ephemeris.d_satClkDrift = d_satClkDrift;
    ephemeris.d_dtr = d_dtr;
    ephemeris.d_satpos_X = d_satpos_X;
    ephemeris.d_satpos_Y = d_satpos_Y;
    ephemeris.d_satpos_Z = d_satpos_Z;
    ephemeris.d_satvel_X = d_satvel_X;
    ephemeris.d_satvel_Y = d_satvel_Y;
    ephemeris.d_satvel_Z = d_satvel_Z;

    return ephemeris;
}


Gps_Iono Gps_Navigation_Message::get_iono()
{
    Gps_Iono iono;
    iono.d_alpha0 = d_alpha0;
    iono.d_alpha1 = d_alpha1;
    iono.d_alpha2 = d_alpha2;
    iono.d_alpha3 = d_alpha3;
    iono.d_beta0 = d_beta0;
    iono.d_beta1 = d_beta1;
    iono.d_beta2 = d_beta2;
    iono.d_beta3 = d_beta3;
    iono.valid = flag_iono_valid;
    //WARNING: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_iono_valid = false;
    return iono;
}


Gps_Utc_Model Gps_Navigation_Message::get_utc_model()
{
    Gps_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.d_A1 = d_A1;
    utc_model.d_A0 = d_A0;
    utc_model.d_t_OT = d_t_OT;
    utc_model.i_WN_T = i_WN_T;
    utc_model.d_DeltaT_LS = d_DeltaT_LS;
    utc_model.i_WN_LSF = i_WN_LSF;
    utc_model.i_DN = i_DN;
    utc_model.d_DeltaT_LSF = d_DeltaT_LSF;
    // warning: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_utc_model_valid = false;
    return utc_model;
}


bool Gps_Navigation_Message::satellite_validation()
{
    bool flag_data_valid = false;
    b_valid_ephemeris_set_flag = false;

    // First Step:
    // check Issue Of Ephemeris Data (IODE IODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_TOW_SF1 != 0.0 and d_TOW_SF2 != 0.0 and d_TOW_SF3 != 0.0)
        {
            if (d_IODE_SF2 == d_IODE_SF3 and d_IODC == d_IODE_SF2 and d_IODC != -1.0)
                {
                    flag_data_valid = true;
                    b_valid_ephemeris_set_flag = true;
                }
        }
    return flag_data_valid;
}

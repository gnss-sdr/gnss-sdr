/*!
m * \file beidou_navigation_message.cc
 * \brief  Implementation of a BeiDou D1 NAV Data message decoder as described in BeiDou ICD Version 2.1
 *
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_navigation_message.h"
#include <cmath>
#include <iostream>
#include <gnss_satellite.h>


void Beidou_Navigation_Message_D1::reset()
{
    b_valid_ephemeris_set_flag = false;
    d_SOW = 0;
    d_SOW_SF1 = 0;
    d_SOW_SF2 = 0;
    d_SOW_SF3 = 0;
    d_SOW_SF4 = 0;
    d_SOW_SF5 = 0;
    d_AODE = 0;
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
    i_BEIDOU_week = 0;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD1 = 0;
    d_TGD2 = 0;
    d_AODC = -1;
//    i_AODO = 0;

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
    d_A1UTC = 0;
    d_A0UTC = 0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF= 0;

    //Almanac
    d_Toa = 0;
    i_WN_A = 0;
    for (int i=1; i < 36; i++ )
        {
            almanacHealth[i] = 0;
        }

    // Satellite velocity
    d_satvel_X = 0;
    d_satvel_Y = 0;
    d_satvel_Z = 0;
    d_A1GPS = 0;
    d_A0GPS = 0;
    d_A1GAL = 0;
    d_A0GAL = 0;
    d_A1GLO = 0;
    d_A0GLO = 0;
    d_AODE_SF1 = 0;
    d_SQRT_A_ALMANAC = 0;
    d_A1_ALMANAC = 0;
    d_A0_ALMANAC = 0;
    d_OMEGA0_ALMANAC = 0;
    d_E_ALMANAC = 0;
    d_DELTA_I = 0;
    d_TOA = 0;
    d_OMEGA_DOT_ALMANAC = 0;
    d_OMEGA_ALMANAC = 0;
    d_M0_ALMANAC = 0;
    almanac_WN = 0;
    d_toa2 = 0;
    d_A0 = 0;
    d_A1 = 0;
    d_A2 = 0;

    auto gnss_sat = Gnss_Satellite();
    std::string _system ("Beidou");
    for(unsigned int i = 1; i < 36; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}



Beidou_Navigation_Message_D1::Beidou_Navigation_Message_D1()
{
    reset();
}



void Beidou_Navigation_Message_D1::print_beidou_word_bytes(unsigned int BEIDOU_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(BEIDOU_word);
    std::cout << std::endl;
}



bool Beidou_Navigation_Message_D1::read_navigation_bool(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    bool value;

    if (bits[BEIDOU_SUBFRAME_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Beidou_Navigation_Message_D1::read_navigation_unsigned(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[BEIDOU_SUBFRAME_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}

signed long int Beidou_Navigation_Message_D1::read_navigation_signed(std::bitset<BEIDOU_SUBFRAME_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8) // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[BEIDOU_SUBFRAME_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFFFFFFFFFF; //64 bits variable
                }
            else
                {
                    value &= 0;
                }

            for (int i = 0; i < num_of_slices; i++)
                {
                    for (int j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFFFFFFFFFE; //reset the corresponding bit (for the 64 bits variable)
                            if (bits[BEIDOU_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[BEIDOU_SUBFRAME_BITS - parameter[0].first] == 1)
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
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFE; //reset the corresponding bit
                            if (bits[BEIDOU_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    return value;
}

double Beidou_Navigation_Message_D1::check_t(double time)
{
    double corrTime;
    double half_week = 302400;     // seconds
    corrTime = time;
    if (time > half_week)
        {
            corrTime = time - 2 * half_week;
        }
    else if (time < -half_week)
        {
            corrTime = time + 2 * half_week;
        }
    return corrTime;
}

// User Algorithm for SV Clock Correction.
double Beidou_Navigation_Message_D1::sv_clock_correction(double transmitTime)
{
    double dt;
    dt = check_t(transmitTime - d_Toc);
    d_satClkCorr = (d_A_f2 * dt + d_A_f1) * dt + d_A_f0 + d_dtr;
    double correctedTime = transmitTime - d_satClkCorr;
    return correctedTime;
}

void Beidou_Navigation_Message_D1::satellitePosition(double transmitTime)
{
    double tk;
    double a;
    double n;
    double n0;
    double M;
    double E;
    double E_old;
    double dE;
    double nu;
    double phi;
    double u;
    double r;
    double i;
    double Omega;

    // Find satellite's position ----------------------------------------------

    // Restore semi-major axis
    a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    n0 = sqrt(BEIDOU_GM / (a * a * a));

    // Corrected mean motion
    n = n0 + d_Delta_n;

    // Mean anomaly
    M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2 * BEIDOU_PI), (2 * BEIDOU_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old   = E;
            E       = M + d_e_eccentricity * sin(E);
            dE      = fmod(E - E_old, 2 * BEIDOU_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = BEIDOU_F * d_e_eccentricity * d_sqrt_A * sin(E);

    // Compute the true anomaly
    double tmp_Y = sqrt(1.0 - d_e_eccentricity * d_e_eccentricity) * sin(E);
    double tmp_X = cos(E) - d_e_eccentricity;
    nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    phi = nu + d_OMEGA;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2 * BEIDOU_PI));

    // Correct argument of latitude
    u = phi + d_Cuc * cos(2 * phi) +  d_Cus * sin(2 * phi);

    // Correct radius
    r = a * (1 - d_e_eccentricity * cos(E)) +  d_Crc * cos(2 * phi) +  d_Crs * sin(2 * phi);

    // Correct inclination
    i = d_i_0 + d_IDOT * tk + d_Cic * cos(2 * phi) + d_Cis * sin(2 * phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = d_OMEGA0 + (d_OMEGA_DOT - BEIDOU_OMEGA_EARTH_DOT) * tk - BEIDOU_OMEGA_EARTH_DOT * d_Toe;

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2 * BEIDOU_PI), (2 * BEIDOU_PI));

    // --- Compute satellite coordinates in Earth-fixed coordinates
    d_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    d_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
    d_satpos_Z = sin(u) * r * sin(i);

    // Satellite's velocity. Can be useful for Vector Tracking loops
    double Omega_dot = d_OMEGA_DOT - BEIDOU_OMEGA_EARTH_DOT;
    d_satvel_X = - Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + d_satpos_X * cos(Omega) - d_satpos_Y * cos(i) * sin(Omega);
    d_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + d_satpos_X * sin(Omega) + d_satpos_Y * cos(i) * cos(Omega);
    d_satvel_Z = d_satpos_Y * sin(i);
}





int Beidou_Navigation_Message_D1::subframe_decoder(char *subframe)
{
    int subframe_ID = 0;
    std::cout << "Beidou_Navigation_Message_D1::subframe_decoder" << std::endl;
    std::bitset<BEIDOU_SUBFRAME_BITS> mysubframe_bits;

//double tmp_SOW;

    unsigned int beidou_word;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<BEIDOU_SUBFRAME_BITS> subframe_bits;
    std::bitset<BEIDOU_WORD_BITS + 2> word_bits;
    for (int i = 0; i < 10; i++)
        {
            memcpy(&beidou_word, &subframe[i * 4], sizeof(char) * 4);
            word_bits = std::bitset<(BEIDOU_WORD_BITS + 2) > (beidou_word);
            for (int j = 0; j < BEIDOU_WORD_BITS; j++)
                {
                    subframe_bits[BEIDOU_WORD_BITS * (9 - i) + j] = word_bits[j];
            std::cout << word_bits[j];
                }
std::cout << std::endl;
        }
    for (int i = 0; i < BEIDOU_SUBFRAME_BITS; i++)
        {
            std::cout << subframe_bits[i] ;
        }

            std::cout << std::endl;
    subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_FRAID));

    // Decode all 5 sub-frames
    switch (subframe_ID)
    {
    //--- Decode the sub-frame id ------------------------------------------
    case 1:
        //--- It is subframe 1 -------------------------------------
        // Compute the time of week (SOW) of the first sub-frames in the array ====
        // The transmitted SOW is actual SOW of the next subframe
        // (the variable subframe at this point contains bits of the last subframe).
        //SOW = bin2dec(subframe(31:47)) * 6;
        //we are in the first subframe (the transmitted SOW is the start time of the next subframe) !
        //d_SOW_SF1 = d_SOW_SF1 * 6;
        //b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        //b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        //b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        //i_SV_accuracy = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_ACCURACY));  // (20.3.3.3.1.3)
       // b_L2_P_data_flag = read_navigation_bool(subframe_bits, L2_P_DATA_FLAG); //
        //i_code_on_L2 = static_cast<int>(read_navigation_unsigned(subframe_bits, CA_OR_P_ON_L2));


        d_SOW_SF1 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
        d_SOW = d_SOW_SF1; // Set transmission time

std::cout << "I decoded subframe 1" << std::endl;
std::cout << "TOW: " << d_SOW_SF1  << std::endl;

        i_SV_health = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_SAT_H1));

        d_AODC = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_AODC));
        i_SV_accuracy = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_URAI));  // (20.3.3.3.1.3)

        i_BEIDOU_week = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_WN));

        d_Toc = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOC));
        d_Toc = d_Toc * D1_TOC_LSB;

        d_TGD1 = static_cast<double>(read_navigation_signed(subframe_bits, D1_TGD1));
        d_TGD1 = d_TGD1 * D1_TGD1_LSB;

        d_TGD2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_TGD2));
        d_TGD2 = d_TGD2 * D1_TGD2_LSB;

        d_alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_ALPHA0));
        d_alpha0 = d_alpha0 * D1_ALPHA0_LSB;

	d_alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, D1_ALPHA1));
        d_alpha1 = d_alpha1 * D1_ALPHA1_LSB;
        d_alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_ALPHA2));
        d_alpha2 = d_alpha2 * D1_ALPHA2_LSB;
        d_alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, D1_ALPHA3));
        d_alpha3 = d_alpha3 * D1_ALPHA3_LSB;
        d_beta0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_BETA0));
        d_beta0 = d_beta0 * D1_BETA0_LSB;
        d_beta1 = static_cast<double>(read_navigation_signed(subframe_bits, D1_BETA1));
        d_beta1 = d_beta1 * D1_BETA1_LSB;
        d_beta2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_BETA2));
        d_beta2 = d_beta2 * D1_BETA2_LSB;
        d_beta3 = static_cast<double>(read_navigation_signed(subframe_bits, D1_BETA3));
        d_beta3 = d_beta3 * D1_BETA3_LSB;

        d_A2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A2));
        d_A2 = d_A2 * D1_A2_LSB;

        d_A0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0));
        d_A0 = d_A0 * D1_A0_LSB;

        d_A1 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1));
        d_A1 = d_A1 * D1_A1_LSB;

        d_AODE_SF1 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_AODE));

        //d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, A_F0));
        //d_A_f0 = d_A_f0 * A_F0_LSB;
        //d_A_f1 = static_cast<double>(read_navigation_signed(subframe_bits, A_F1));
        //d_A_f1 = d_A_f1 * A_F1_LSB;
        //d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, A_F2));
        //d_A_f2 = d_A_f2 * A_F2_LSB;

        break;

    case 2:  //--- It is subframe 2 -------------------


        d_SOW_SF2 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
        d_SOW = d_SOW_SF2; // Set transmission time

std::cout << "I decoded subframe 2" << std::endl;
std::cout << "TOW: " << d_SOW_SF2  << std::endl;

        d_Cuc = static_cast<double>(read_navigation_signed(subframe_bits, D1_CUC));
        d_Cuc = d_Cuc * D1_CUC_LSB;

        d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_M0));
        d_M_0 = d_M_0 * D1_M0_LSB;

        d_e_eccentricity = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_E));
        d_e_eccentricity = d_e_eccentricity * D1_E_LSB;

        d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, D1_CUS));
        d_Cus = d_Cus * D1_CUS_LSB;

        d_Crc = static_cast<double>(read_navigation_signed(subframe_bits, D1_CRC));
        d_Crc = d_Crc * D1_CRC_LSB;

        d_Crs = static_cast<double>(read_navigation_signed(subframe_bits, D1_CRS));
        d_Crs = d_Crs * D1_CRS_LSB;

        d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SQRT_A));
        d_sqrt_A = d_sqrt_A * D1_SQRT_A_LSB;

        d_Toe = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOE));
        d_Toe = d_Toe * D1_TOE_LSB;

//        d_SOW = d_SOW_SF2; // Set transmission time
//        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
//        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
//        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
//        b_fit_interval_flag = read_navigation_bool(subframe_bits, FIT_INTERVAL_FLAG);
//        i_AODO = static_cast<int>(read_navigation_unsigned(subframe_bits, AODO));
//        i_AODO = i_AODO * AODO_LSB;

        break;

    case 3: // --- It is subframe 3 -------------------------------------
        
        d_SOW_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
        d_SOW = d_SOW_SF3; // Set transmission time

std::cout << "I decoded subframe 3" << std::endl;
std::cout << "TOW: " << d_SOW_SF3  << std::endl;

        d_Toe = d_Toe * D1_TOE_LSB;

        d_i_0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_I0));
        d_i_0 = d_i_0 * D1_I0_LSB;

        d_Cic = static_cast<double>(read_navigation_signed(subframe_bits, D1_CIC));
        d_Cic = d_Cic * D1_CIC_LSB;

        d_OMEGA_DOT = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA_DOT));
        d_OMEGA_DOT = d_OMEGA_DOT * D1_OMEGA_DOT_LSB;

        d_Cis = static_cast<double>(read_navigation_signed(subframe_bits, D1_CIS));
        d_Cis = d_Cis * D1_CIS_LSB;

        d_IDOT = static_cast<double>(read_navigation_signed(subframe_bits, D1_IDOT));
        d_IDOT = d_IDOT * D1_IDOT_LSB;

        d_OMEGA0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA0));
        d_OMEGA0 = d_OMEGA0 * D1_OMEGA0_LSB;

        d_OMEGA = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA));
        d_OMEGA = d_OMEGA * D1_OMEGA_LSB;

        //d_SOW_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, SOW));

        //d_SOW_SF3 = d_SOW_SF3 * 6;
        //d_SOW = d_SOW_SF3; // Set transmission time
        //b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        //b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        //b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        //d_AODE_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, AODE_SF3));

        break;

    case 4: // --- It is subframe 4 ---------- Almanac, ionospheric model, UTC parameters, SV health (PRN: 25-32)
        d_SOW_SF4 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
        d_SOW = d_SOW_SF4; // Set transmission time

std::cout << "I decoded subframe 4" << std::endl;
std::cout << "TOW: " << d_SOW_SF4  << std::endl;

        d_SQRT_A_ALMANAC = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SQRT_A_ALMANAC));
        d_SQRT_A_ALMANAC = d_SQRT_A_ALMANAC * D1_SQRT_A_ALMANAC_LSB;

        d_A1_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1_ALMANAC));
        d_A1_ALMANAC = d_A1_ALMANAC * D1_A1_ALMANAC_LSB;

        d_A0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0_ALMANAC));
        d_A0_ALMANAC = d_A0_ALMANAC * D1_A0_ALMANAC_LSB;

        d_OMEGA0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA0_ALMANAC));
        d_OMEGA0_ALMANAC = d_OMEGA0_ALMANAC * D1_OMEGA0_ALMANAC_LSB;

        d_E_ALMANAC = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_E));
        d_E_ALMANAC = d_E_ALMANAC * D1_E_ALMANAC_LSB;

        d_DELTA_I = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_I));
        d_DELTA_I = D1_DELTA_I_LSB;

        d_TOA = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOA));
        d_TOA = d_TOA * D1_TOA_LSB;

        d_OMEGA_DOT_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA_DOT_ALMANAC));
        d_OMEGA_DOT_ALMANAC = D1_OMEGA_DOT_ALMANAC_LSB;

        d_OMEGA_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA_ALMANAC));
        d_OMEGA_ALMANAC = d_OMEGA_ALMANAC * D1_OMEGA_ALMANAC_LSB;

        d_M0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_M0));
        d_M0_ALMANAC = d_M0_ALMANAC * D1_M0_ALMANAC_LSB;

/*        b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
        b_alert_flag = read_navigation_bool(subframe_bits, ALERT_FLAG);
        b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
        SV_data_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_DATA_ID));
        SV_page = static_cast<int>(read_navigation_unsigned(subframe_bits, SV_PAGE));
        if (SV_page > 24 && SV_page < 33) // Page 4 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                //! \TODO read almanac
                if(SV_data_ID){}
            }

        if (SV_page == 52) // Page 13 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                //! \TODO read Estimated Range Deviation (ERD) values
            }

        if (SV_page == 56)  // Page 18 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                // Page 18 - Ionospheric and UTC data
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

        if (SV_page == 63) // Page 25 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                // Page 25 Anti-Spoofing, SV config and almanac health (PRN: 25-32)
                //! \TODO Read Anti-Spoofing, SV config
                almanacHealth[25] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA25));
                almanacHealth[26] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA26));
                almanacHealth[27] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA27));
                almanacHealth[28] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA28));
                almanacHealth[29] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA29));
                almanacHealth[30] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA30));
                almanacHealth[31] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA31));
                almanacHealth[32] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA32));
            }

        break;
*/
    case 5://--- It is subframe 5 -----------------almanac health (PRN: 1-24) and Almanac reference week number and time.
        int SV_page_5;
        d_SOW_SF5 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
        d_SOW = d_SOW_SF5; // Set transmission time

std::cout << "I decoded subframe 5" << std::endl;
std::cout << "TOW: " << d_SOW_SF5  << std::endl;


        SV_page_5 = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_PNUM));

        if (SV_page_5 < 7) 
            {
		d_SOW_SF4 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
		d_SQRT_A_ALMANAC = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SQRT_A_ALMANAC));
		d_SQRT_A_ALMANAC = d_SQRT_A_ALMANAC * D1_SQRT_A_ALMANAC_LSB;

		d_A1UTC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1_ALMANAC));
		d_A1UTC = d_A1UTC * D1_A1_ALMANAC_LSB;

		d_A0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0_ALMANAC));
		d_A0_ALMANAC = d_A0_ALMANAC * D1_A0_ALMANAC_LSB;

		d_OMEGA0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA0_ALMANAC));
		d_OMEGA0_ALMANAC = d_OMEGA0_ALMANAC * D1_OMEGA0_ALMANAC_LSB;

		d_E_ALMANAC = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_E));
		d_E_ALMANAC = d_E_ALMANAC * D1_E_ALMANAC_LSB;

		d_DELTA_I = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_I));
		d_DELTA_I = D1_DELTA_I_LSB;

		d_TOA = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOA));
		d_TOA = d_TOA * D1_TOA_LSB;

		d_OMEGA_DOT_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA_DOT_ALMANAC));
		d_OMEGA_DOT_ALMANAC = D1_OMEGA_DOT_ALMANAC_LSB;

		d_OMEGA_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_OMEGA_ALMANAC));
		d_OMEGA_ALMANAC = d_OMEGA_ALMANAC * D1_OMEGA_ALMANAC_LSB;

		d_M0_ALMANAC = static_cast<double>(read_navigation_signed(subframe_bits, D1_M0));
		d_M0_ALMANAC = d_M0_ALMANAC * D1_M0_ALMANAC_LSB;

            }

        if (SV_page_5 == 7)
            {
                //! \TODO read almanac
                almanacHealth[1] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA1));
                almanacHealth[2] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA2));
                almanacHealth[3] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA3));
                almanacHealth[4] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA4));
                almanacHealth[5] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA5));
                almanacHealth[6] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA6));
                almanacHealth[7] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA7));
                almanacHealth[8] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA8));
                almanacHealth[9] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA9));
                almanacHealth[10] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA10));
                almanacHealth[11] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA11));
                almanacHealth[12] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA12));
                almanacHealth[13] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA13));
                almanacHealth[14] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA14));
                almanacHealth[15] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA15));
                almanacHealth[16] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA16));
                almanacHealth[17] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA17));
                almanacHealth[18] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA18));
                almanacHealth[19] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA19));
            }
        if (SV_page_5 == 8) // Page 25 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                almanacHealth[20] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA20));
                almanacHealth[21] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA21));
                almanacHealth[22] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA22));
                almanacHealth[23] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA23));
                almanacHealth[24] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA24));
                almanacHealth[25] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA25));
                almanacHealth[26] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA26));
                almanacHealth[27] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA27));
                almanacHealth[28] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA28));
                almanacHealth[29] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA29));
                almanacHealth[30] = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_HEA30));
                almanac_WN = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_WNA));
                d_toa2  = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_TOA2));

            }

        if (SV_page_5 == 9) // Page 25 (from Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, IS-GPS-200H, page 110)
            {
                d_A0GPS = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A0GPS));
                d_A0GPS = d_A0GPS * D1_A0GPS_LSB;

                d_A1GPS = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A1GPS));
                d_A1GPS = d_A1GPS * D1_A1GPS_LSB;

                d_A0GAL = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A0GAL));
                d_A0GAL = d_A0GAL * D1_A0GAL_LSB;

                d_A1GAL = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A1GAL));
                d_A1GAL = d_A1GAL* D1_A1GAL_LSB;

                d_A0GLO = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A0GLO));
                d_A0GLO = d_A0GLO * D1_A0GLO_LSB;

                d_A1GLO = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_A1GLO));
                d_A1GLO = d_A1GLO* D1_A1GLO_LSB;
            }
        if (SV_page_5 == 10)
            {
                d_DeltaT_LS = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_T_LS));

            }



        break;

    default:
        break;
    } // switch subframeID ...

    return subframe_ID;
}




double Beidou_Navigation_Message_D1::utc_time(const double beidoutime_corrected) const
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC =  d_DeltaT_LS + d_A0 + d_A1UTC * (beidoutime_corrected);

    // Determine if the effectivity time of the leap second event is in the past
    int  weeksToLeapSecondEvent = i_WN_LSF - i_BEIDOU_week;

    if ((weeksToLeapSecondEvent) >= 0) // is not in the past
        {
            //Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                }
            else //we are in the same week than the leap second event
                {
                    if  ((beidoutime_corrected - secondOfLeapSecondEvent) < (2/3) * 24 * 60 * 60)
                        {
                            t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            if  ((beidoutime_corrected - secondOfLeapSecondEvent) < (5/4) * 24 * 60 * 60)
                                {
                                    int W = fmod(beidoutime_corrected - Delta_t_UTC - 43200, 86400) + 43200;
                                    t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - d_DeltaT_LS);
	                        }
                            else
                                {
                                    t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                                }
                        }
                }
        }
    else // the effectivity time is in the past
        {
            t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 43200 * floor(beidoutime_corrected / 43200);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}



Beidou_Ephemeris Beidou_Navigation_Message_D1::get_ephemeris()
{
    Beidou_Ephemeris ephemeris;
    ephemeris.i_satellite_PRN = i_satellite_PRN;
    ephemeris.d_TOW = d_SOW;
    ephemeris.d_Crs = d_Crs;
    ephemeris.d_Delta_n = d_Delta_n;
    ephemeris.d_M_0 = d_M_0;
    ephemeris.d_Cuc = d_Cuc;
    ephemeris.d_e_eccentricity = d_e_eccentricity;
    ephemeris.d_Cus = d_Cus;
    ephemeris.d_sqrt_A = d_sqrt_A;
    ephemeris.d_Toe =static_cast<float>( (static_cast<int>(d_Toe) <<  15) | static_cast<int>(d_Toe2)) ;
    ephemeris.d_Toc = d_Toc;
    ephemeris.d_Cic = d_Cic;
    ephemeris.d_OMEGA0 = d_OMEGA0;
    ephemeris.d_Cis = d_Cis;
    ephemeris.d_i_0 = d_i_0;
    ephemeris.d_Crc = d_Crc;
    ephemeris.d_OMEGA = d_OMEGA;
    ephemeris.d_OMEGA_DOT = d_OMEGA_DOT;
    ephemeris.d_IDOT = d_IDOT;
    ephemeris.i_BEIDOU_week = i_BEIDOU_week;
    ephemeris.i_SV_accuracy = i_SV_accuracy;
    ephemeris.i_SV_health = i_SV_health;
    ephemeris.d_TGD1 = d_TGD1;
    ephemeris.d_AODC = d_AODC;
    //ephemeris.d_AODE_SF2 = d_AODE_SF2;
    //ephemeris.d_AODE_SF3 = d_AODE_SF3;
    //ephemeris.i_AODO = i_AODO;
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


Beidou_Iono Beidou_Navigation_Message_D1::get_iono()
{
    Beidou_Iono iono;
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


Beidou_Utc_Model Beidou_Navigation_Message_D1::get_utc_model()
{
    Beidou_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.d_A1 = d_A1UTC;
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


bool Beidou_Navigation_Message_D1::satellite_validation()
{
    bool flag_data_valid = false;
    b_valid_ephemeris_set_flag = false;

    // First Step:
    // check Issue Of Ephemeris Data (AODE AODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_SOW_SF1 != 0 and d_SOW_SF2 != 0 and d_SOW_SF3 != 0)
        {
            if (d_AODC!= -1)
                {
                    flag_data_valid = true;
                    b_valid_ephemeris_set_flag = true;
                }
        }
    return flag_data_valid;
}

/*!
 * \file beidou_dnav_navigation_message.cc
 * \brief  Implementation of a BeiDou D1 NAV Data message decoder as described
 * in BeiDou ICD Version 2.1
 *
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
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

#include "beidou_dnav_navigation_message.h"
#include "gnss_satellite.h"
#include <cmath>     // for cos, sin, fmod, sqrt, atan2, fabs, floor
#include <iostream>  // for string, operator<<, cout, ostream, endl


void Beidou_Dnav_Navigation_Message::reset()
{
    // Control variable for message decoding
    flag_eph_valid = false;
    flag_iono_valid = false;
    flag_utc_model_valid = false;
    flag_crc_test = false;

    flag_d1_sf1 = false;
    flag_d1_sf2 = false;
    flag_d1_sf3 = false;
    flag_d1_sf4 = false;
    flag_d1_sf5 = false;
    flag_d1_sf5_p7 = false;
    flag_d1_sf5_p8 = false;
    flag_d1_sf5_p9 = false;
    flag_d1_sf5_p10 = false;
    flag_new_SOW_available = false;
    d_previous_aode = 0.0;

    flag_sf1_p1 = false;
    flag_sf1_p2 = false;
    flag_sf1_p3 = false;
    flag_sf1_p4 = false;
    flag_sf1_p5 = false;
    flag_sf1_p6 = false;
    flag_sf1_p7 = false;
    flag_sf1_p8 = false;
    flag_sf1_p9 = false;
    flag_sf1_p10 = false;

    // D2 NAV Decoding UNique Attributes
    d_A_f1_msb_bits = 0;
    d_A_f1_lsb_bits = 0;
    d_Cuc_msb_bits = 0;
    d_Cuc_lsb_bits = 0;
    d_eccentricity_msb_bits = 0;
    d_eccentricity_lsb_bits = 0;
    d_Cic_msb_bits = 0;
    d_Cic_lsb_bits = 0;
    d_i_0_msb_bits = 0;
    d_i_0_lsb_bits = 0;
    d_OMEGA_msb_bits = 0;
    d_OMEGA_lsb_bits = 0;
    d_OMEGA_DOT_msb_bits = 0;
    d_OMEGA_DOT_lsb_bits = 0;

    // D2 NAV Decoding UNique Attributes
    d_eccentricity_msb = 0;
    d_eccentricity_lsb = 0;

    d_SOW = 0.0;
    d_SOW_SF1 = 0.0;
    d_SOW_SF2 = 0.0;
    d_SOW_SF3 = 0.0;
    d_SOW_SF4 = 0.0;
    d_SOW_SF5 = 0.0;
    d_AODE = 0.0;
    d_Crs = 0.0;
    d_Delta_n = 0.0;
    d_M_0 = 0.0;
    d_Cuc = 0.0;
    d_eccentricity = 0.0;
    d_Cus = 0.0;
    d_sqrt_A = 0.0;
    d_Toe_sf2 = 0.0;
    d_Toe_sf3 = 0.0;
    d_Toe = 0.0;
    d_Toc = 0.0;
    d_Cic = 0.0;
    d_OMEGA0 = 0.0;
    d_Cis = 0.0;
    d_i_0 = 0.0;
    d_Crc = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    d_IDOT = 0.0;
    i_BEIDOU_week = 0;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD1 = 0.0;
    d_TGD2 = 0.0;
    d_AODC = -1.0;
    //    i_AODO = 0;

    b_fit_interval_flag = false;
    d_spare1 = 0.0;
    d_spare2 = 0.0;

    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
    d_A_f2 = 0.0;

    //clock terms
    //d_master_clock=0;
    d_dtr = 0.0;
    d_satClkCorr = 0.0;
    d_satClkDrift = 0.0;

    // satellite positions
    d_satpos_X = 0.0;
    d_satpos_Y = 0.0;
    d_satpos_Z = 0.0;

    // info
    i_channel_ID = 0;
    i_satellite_PRN = 0;
    i_signal_type = 0;

    // time synchro
    d_subframe_timestamp_ms = 0.0;

    // flags
    b_alert_flag = false;
    b_integrity_status_flag = false;
    b_antispoofing_flag = false;

    // Ionosphere and UTC
    flag_iono_valid = false;
    flag_utc_model_valid = false;
    d_alpha0 = 0.0;
    d_alpha1 = 0.0;
    d_alpha2 = 0.0;
    d_alpha3 = 0.0;
    d_beta0 = 0.0;
    d_beta1 = 0.0;
    d_beta2 = 0.0;
    d_beta3 = 0.0;
    d_A1UTC = 0.0;
    d_A0UTC = 0.0;
    d_DeltaT_LS = 0.0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0.0;

    //Almanac
    d_Toa = 0.0;
    i_WN_A = 0;
    for (int32_t i = 1; i < 36; i++)
        {
            almanacHealth[i] = 0;
        }

    // Satellite velocity
    d_satvel_X = 0.0;
    d_satvel_Y = 0.0;
    d_satvel_Z = 0.0;
    d_A1GPS = 0.0;
    d_A0GPS = 0.0;
    d_A1GAL = 0.0;
    d_A0GAL = 0.0;
    d_A1GLO = 0.0;
    d_A0GLO = 0.0;
    d_SQRT_A_ALMANAC = 0.0;
    d_A1_ALMANAC = 0.0;
    d_A0_ALMANAC = 0.0;
    d_OMEGA0_ALMANAC = 0.0;
    d_E_ALMANAC = 0.0;
    d_DELTA_I = 0.0;
    d_TOA = 0.0;
    d_OMEGA_DOT_ALMANAC = 0.0;
    d_OMEGA_ALMANAC = 0.0;
    d_M0_ALMANAC = 0.0;
    almanac_WN = 0;
    d_toa2 = 0.0;
    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
    d_A_f2 = 0.0;

    auto gnss_sat = Gnss_Satellite();
    std::string _system("Beidou");
    for (uint32_t i = 1; i < 36; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Beidou_Dnav_Navigation_Message::Beidou_Dnav_Navigation_Message()
{
    reset();
}


void Beidou_Dnav_Navigation_Message::print_beidou_word_bytes(uint32_t BEIDOU_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(BEIDOU_word);
    std::cout << std::endl;
}


bool Beidou_Dnav_Navigation_Message::read_navigation_bool(
    std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    bool value;

    if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


uint64_t Beidou_Dnav_Navigation_Message::read_navigation_unsigned(
    std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    uint64_t value = 0ULL;
    int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1U;  // shift left
                    if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1U;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Beidou_Dnav_Navigation_Message::read_navigation_signed(
    std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> bits,
    const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    int64_t value = 0;
    int32_t num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int32_t long_int_size_bytes = sizeof(int64_t);
    if (long_int_size_bytes == 8)  // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFFFFFFFFFF;  //64 bits variable
                }
            else
                {
                    value &= 0;
                }

            for (int32_t i = 0; i < num_of_slices; i++)
                {
                    for (int32_t j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1;                  //shift left
                            value &= 0xFFFFFFFFFFFFFFFE;  //reset the corresponding bit (for the 64 bits variable)
                            if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFF;
                }
            else
                {
                    value &= 0;
                }

            for (int32_t i = 0; i < num_of_slices; i++)
                {
                    for (int32_t j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1;          //shift left
                            value &= 0xFFFFFFFE;  //reset the corresponding bit
                            if (bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1;  // insert the bit
                                }
                        }
                }
        }
    return value;
}


double Beidou_Dnav_Navigation_Message::check_t(double time)
{
    double corrTime;
    double half_week = 302400;  // seconds
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


double Beidou_Dnav_Navigation_Message::sv_clock_correction(double transmitTime)
{
    double dt;
    dt = check_t(transmitTime - d_Toc);
    d_satClkCorr = (d_A_f2 * dt + d_A_f1) * dt + d_A_f0 + d_dtr;
    double correctedTime = transmitTime - d_satClkCorr;
    return correctedTime;
}


void Beidou_Dnav_Navigation_Message::satellitePosition(double transmitTime)
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
    tk = check_t(transmitTime - d_Toe_sf2);

    // Computed mean motion
    n0 = sqrt(BEIDOU_DNAV_GM / (a * a * a));

    // Corrected mean motion
    n = n0 + d_Delta_n;

    // Mean anomaly
    M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2 * BEIDOU_DNAV_PI), (2 * BEIDOU_DNAV_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + d_eccentricity * sin(E);
            dE = fmod(E - E_old, 2 * BEIDOU_DNAV_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = BEIDOU_DNAV_F * d_eccentricity * d_sqrt_A * sin(E);

    // Compute the true anomaly
    double tmp_Y = sqrt(1.0 - d_eccentricity * d_eccentricity) * sin(E);
    double tmp_X = cos(E) - d_eccentricity;
    nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    phi = nu + d_OMEGA;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2 * BEIDOU_DNAV_PI));

    // Correct argument of latitude
    u = phi + d_Cuc * cos(2 * phi) + d_Cus * sin(2 * phi);

    // Correct radius
    r = a * (1 - d_eccentricity * cos(E)) + d_Crc * cos(2 * phi) + d_Crs * sin(2 * phi);

    // Correct inclination
    i = d_i_0 + d_IDOT * tk + d_Cic * cos(2 * phi) + d_Cis * sin(2 * phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = d_OMEGA0 + (d_OMEGA_DOT - BEIDOU_DNAV_OMEGA_EARTH_DOT) * tk - BEIDOU_DNAV_OMEGA_EARTH_DOT * d_Toe_sf2;

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2 * BEIDOU_DNAV_PI), (2 * BEIDOU_DNAV_PI));

    // --- Compute satellite coordinates in Earth-fixed coordinates
    d_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    d_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
    d_satpos_Z = sin(u) * r * sin(i);

    // Satellite's velocity. Can be useful for Vector Tracking loops
    double Omega_dot = d_OMEGA_DOT - BEIDOU_DNAV_OMEGA_EARTH_DOT;
    d_satvel_X = -Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + d_satpos_X * cos(Omega) - d_satpos_Y * cos(i) * sin(Omega);
    d_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + d_satpos_X * sin(Omega) + d_satpos_Y * cos(i) * cos(Omega);
    d_satvel_Z = d_satpos_Y * sin(i);
}


int32_t Beidou_Dnav_Navigation_Message::d1_subframe_decoder(std::string const& subframe)
{
    int32_t subframe_ID = 0;
    std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> subframe_bits(subframe);

    subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_FRAID));

    // Perform crc computation (tbd)
    flag_crc_test = true;

    // Decode all 5 sub-frames
    switch (subframe_ID)
        {
        case 1:  // --- It is subframe 1 ---
            d_SOW_SF1 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF1;  // Set transmission time

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

            d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A2));
            d_A_f2 = d_A_f2 * D1_A2_LSB;
            d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0));
            d_A_f0 = d_A_f0 * D1_A0_LSB;
            d_A_f1 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1));
            d_A_f1 = d_A_f1 * D1_A1_LSB;

            d_AODE = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_AODE));

            // Set system flags for message reception
            flag_d1_sf1 = true;
            flag_iono_valid = true;
            flag_new_SOW_available = true;

            break;

        case 2:  // --- It is subframe 2 ---
            d_SOW_SF2 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF2;  // Set transmission time

            d_Cuc = static_cast<double>(read_navigation_signed(subframe_bits, D1_CUC));
            d_Cuc = d_Cuc * D1_CUC_LSB;

            d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, D1_M0));
            d_M_0 = d_M_0 * D1_M0_LSB;

            d_eccentricity = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_E));
            d_eccentricity = d_eccentricity * D1_E_LSB;

            d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, D1_CUS));
            d_Cus = d_Cus * D1_CUS_LSB;

            d_Crc = static_cast<double>(read_navigation_signed(subframe_bits, D1_CRC));
            d_Crc = d_Crc * D1_CRC_LSB;

            d_Crs = static_cast<double>(read_navigation_signed(subframe_bits, D1_CRS));
            d_Crs = d_Crs * D1_CRS_LSB;

            d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SQRT_A));
            d_sqrt_A = d_sqrt_A * D1_SQRT_A_LSB;

            d_Toe_sf2 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOE_SF2));
            d_Toe_sf2 = static_cast<double>((static_cast<uint32_t>(d_Toe_sf2) << 15U));

            // Set system flags for message reception
            flag_d1_sf2 = true;
            flag_new_SOW_available = true;

            break;

        case 3:  // --- It is subframe 3 ---
            d_SOW_SF3 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF3;  // Set transmission time

            d_Toe_sf3 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOE_SF3));

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

            // Set system flags for message reception
            flag_d1_sf3 = true;
            flag_new_SOW_available = true;

            break;

        case 4:  // --- It is subframe 4 ---
            d_SOW_SF4 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF4;  // Set transmission time

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

            // Set system flags for message reception
            flag_d1_sf4 = true;
            flag_new_SOW_available = true;

            break;

        case 5:  // --- It is subframe 5 ---
            int32_t SV_page_5;
            d_SOW_SF5 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF5;  // Set transmission time

            SV_page_5 = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_PNUM));

            if (SV_page_5 < 7)
                {
                    d_SOW_SF4 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
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
                }

            if (SV_page_5 == 7)
                {
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
            if (SV_page_5 == 8)
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
                    d_toa2 = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_TOA2));
                }

            if (SV_page_5 == 9)
                {
                    d_A0GPS = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0GPS)) * D1_A0GPS_LSB;
                    d_A1GPS = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1GPS)) * D1_A1GPS_LSB;
                    d_A0GAL = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0GAL)) * D1_A0GAL_LSB;
                    d_A1GAL = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1GAL)) * D1_A1GAL_LSB;
                    d_A0GLO = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0GLO)) * D1_A0GLO_LSB;
                    d_A1GLO = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1GLO)) * D1_A1GLO_LSB;

                    flag_d1_sf5_p9 = true;
                }
            if (SV_page_5 == 10)
                {
                    d_DeltaT_LS = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_T_LS));
                    d_DeltaT_LSF = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_T_LSF));
                    i_WN_LSF = static_cast<double>(read_navigation_signed(subframe_bits, D1_WN_LSF));
                    d_A0UTC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A0UTC));
                    d_A0UTC = d_A0GPS * D1_A0GPS_LSB;
                    d_A1UTC = static_cast<double>(read_navigation_signed(subframe_bits, D1_A1UTC));
                    d_A1UTC = d_A1UTC * D1_A1UTC_LSB;

                    flag_d1_sf5_p10 = true;
                }

            // Set system flags for message reception
            flag_d1_sf5 = true;
            flag_new_SOW_available = true;

            break;

        default:
            break;
        }  // switch subframeID ...

    return subframe_ID;
}


int32_t Beidou_Dnav_Navigation_Message::d2_subframe_decoder(std::string const& subframe)
{
    int32_t subframe_ID = 0;
    int32_t page_ID = 0;

    std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> subframe_bits(subframe);

    subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_FRAID));
    page_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_PNUM));

    // Perform crc computation (tbd)
    flag_crc_test = true;

    // Decode all 5 sub-frames
    switch (subframe_ID)
        {
        //--- Decode the sub-frame id ------------------------------------------
        case 1:

            switch (page_ID)
                {
                case 1:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    i_SV_health = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_SAT_H1));
                    d_AODC = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_AODC));
                    i_SV_accuracy = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_URAI));  // (20.3.3.3.1.3)
                    i_BEIDOU_week = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_WN));
                    d_Toc = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_TOC)) * D1_TOC_LSB;
                    d_TGD1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_TGD1)) * D1_TGD1_LSB;

                    // Set system flags for message reception
                    flag_sf1_p1 = true;
                    flag_new_SOW_available = true;

                    break;
                case 2:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA0)) * D1_ALPHA0_LSB;
                    d_alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA1)) * D1_ALPHA1_LSB;
                    d_alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA2)) * D1_ALPHA2_LSB;
                    d_alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, D1_ALPHA3)) * D1_ALPHA3_LSB;
                    d_beta0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_BETA0)) * D1_BETA0_LSB;
                    d_beta1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_BETA1)) * D1_BETA1_LSB;
                    d_beta2 = static_cast<double>(read_navigation_signed(subframe_bits, D2_BETA2)) * D1_BETA2_LSB;
                    d_beta3 = static_cast<double>(read_navigation_signed(subframe_bits, D2_BETA3)) * D1_BETA3_LSB;

                    // Set system flags for message reception
                    flag_sf1_p2 = true;
                    flag_iono_valid = true;
                    flag_new_SOW_available = true;

                    break;
                case 3:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_A0)) * D1_A0_LSB;
                    d_A_f1_msb_bits = (read_navigation_unsigned(subframe_bits, D2_A1_MSB));
                    // Adjust for lsb in next page
                    d_A_f1_msb_bits = d_A_f1_msb_bits << 18ULL;

                    // Set system flags for message reception
                    flag_sf1_p3 = true;
                    flag_new_SOW_available = true;

                    break;
                case 4:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_A_f1_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_A1_LSB));
                    d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, D1_A2)) * D1_A2_LSB;
                    d_AODE = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_AODE));
                    d_Delta_n = static_cast<double>(read_navigation_signed(subframe_bits, D2_DELTA_N)) * D1_DELTA_N_LSB;
                    d_Cuc_msb_bits = (read_navigation_unsigned(subframe_bits, D2_CUC_MSB));
                    // Adjust for lsb in next page
                    d_Cuc_msb_bits = d_Cuc_msb_bits << 4U;

                    // Set system flags for message reception
                    flag_sf1_p4 = true;
                    flag_new_SOW_available = true;

                    break;
                case 5:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_Cuc_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_CUC_LSB));
                    d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_M0)) * D1_M0_LSB;
                    d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, D2_CUS)) * D1_CUS_LSB;
                    d_eccentricity_msb = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_E_MSB));
                    d_eccentricity_msb_bits = (read_navigation_unsigned(subframe_bits, D2_E_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_eccentricity_msb = static_cast<uint64_t>((static_cast<uint64_t>(d_eccentricity_msb) << 22U));
                    d_eccentricity_msb_bits = d_eccentricity_msb_bits << 22U;

                    // Set system flags for message reception
                    flag_sf1_p5 = true;
                    flag_new_SOW_available = true;

                    break;
                case 6:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_eccentricity_lsb = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_E_LSB));
                    d_eccentricity_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_E_LSB));
                    d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SQRT_A)) * D1_SQRT_A_LSB;
                    d_Cic_msb_bits = (read_navigation_unsigned(subframe_bits, D2_CIC_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_Cic_msb_bits = d_Cic_msb_bits << 8U;

                    // Set system flags for message reception
                    flag_sf1_p6 = true;
                    flag_new_SOW_available = true;

                    break;
                case 7:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_Cic_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_CIC_LSB));
                    d_Cis = static_cast<double>(read_navigation_signed(subframe_bits, D2_CIS)) * D1_CIS_LSB;
                    d_Toe = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_TOE)) * D1_TOE_LSB;
                    d_i_0_msb_bits = (read_navigation_unsigned(subframe_bits, D2_I0_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_i_0_msb_bits = d_i_0_msb_bits << 11U;

                    // Set system flags for message reception
                    flag_sf1_p7 = true;
                    flag_new_SOW_available = true;

                    break;
                case 8:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_i_0_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_I0_LSB));
                    d_Crc = static_cast<double>(read_navigation_signed(subframe_bits, D2_CRC)) * D1_CRC_LSB;
                    d_Crs = static_cast<double>(read_navigation_signed(subframe_bits, D2_CRS)) * D1_CRS_LSB;
                    d_OMEGA_DOT_msb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_DOT_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_OMEGA_DOT_msb_bits = d_OMEGA_DOT_msb_bits << 5ULL;

                    // Set system flags for message reception
                    flag_sf1_p8 = true;
                    flag_new_SOW_available = true;

                    break;
                case 9:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_OMEGA_DOT_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_DOT_LSB));
                    d_OMEGA0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_OMEGA0)) * D1_OMEGA0_LSB;
                    d_OMEGA_msb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_OMEGA_msb_bits = d_OMEGA_msb_bits << 5U;

                    // Set system flags for message reception
                    flag_sf1_p9 = true;
                    flag_new_SOW_available = true;

                    break;
                case 10:
                    d_SOW = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));
                    d_OMEGA_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_LSB));
                    d_IDOT = static_cast<double>(read_navigation_signed(subframe_bits, D2_IDOT)) * D1_IDOT_LSB;

                    // Set system flags for message reception
                    flag_sf1_p10 = true;
                    flag_new_SOW_available = true;

                    break;
                default:
                    break;
                }

            break;

        case 2:  //--- It is subframe 2 -------------------

            break;

        case 3:  // --- It is subframe 3 -------------------------------------

            break;

        case 4:  // --- It is subframe 4 ---------- Almanac, ionospheric model, UTC parameters, SV health (PRN: 25-32)

            break;

        case 5:  //--- It is subframe 5 -----------------almanac health (PRN: 1-24) and Almanac reference week number and time.

            break;

        default:
            break;
        }  // switch subframeID ...

    return subframe_ID;
}


double Beidou_Dnav_Navigation_Message::utc_time(const double beidoutime_corrected) const
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC = d_DeltaT_LS + d_A0UTC + d_A1UTC * (beidoutime_corrected);

    // Determine if the effectivity time of the leap second event is in the past
    int32_t weeksToLeapSecondEvent = i_WN_LSF - i_BEIDOU_week;

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
        {
            //Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int32_t secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                }
            else  //we are in the same week than the leap second event
                {
                    if ((beidoutime_corrected - secondOfLeapSecondEvent) < (2 / 3) * 24 * 60 * 60)
                        {
                            t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            if ((beidoutime_corrected - secondOfLeapSecondEvent) < (5 / 4) * 24 * 60 * 60)
                                {
                                    int32_t W = fmod(beidoutime_corrected - Delta_t_UTC - 43200, 86400) + 43200;
                                    t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - d_DeltaT_LS);
                                }
                            else
                                {
                                    t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                                }
                        }
                }
        }
    else  // the effectivity time is in the past
        {
            t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 43200 * floor(beidoutime_corrected / 43200);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}


Beidou_Dnav_Ephemeris Beidou_Dnav_Navigation_Message::get_ephemeris()
{
    Beidou_Dnav_Ephemeris eph;

    if (i_satellite_PRN > 0 and i_satellite_PRN < 6)
        {
            std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> subframe_bits;

            // Order as given by eph_t in rtklib
            eph.i_satellite_PRN = i_satellite_PRN;
            eph.d_AODC = d_AODC;
            eph.d_AODE = d_AODE;
            eph.i_SV_accuracy = i_SV_accuracy;
            eph.i_SV_health = i_SV_health;
            eph.i_BEIDOU_week = i_BEIDOU_week;
            eph.i_sig_type = i_signal_type;
            eph.i_nav_type = 2;

            eph.d_TOW = d_SOW;
            eph.d_Toe = d_Toe;
            eph.d_Toc = d_Toc;

            eph.d_sqrt_A = d_sqrt_A;
            eph.d_eccentricity = (d_eccentricity_msb + d_eccentricity_lsb) * D1_E_LSB;
            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_i_0_msb_bits + d_i_0_lsb_bits);
            eph.d_i_0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_I0)) * D1_I0_LSB;
            eph.d_OMEGA0 = d_OMEGA0;
            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_OMEGA_msb_bits + d_OMEGA_lsb_bits);
            eph.d_OMEGA = static_cast<double>(read_navigation_signed(subframe_bits, D2_OMEGA)) * D1_OMEGA_LSB;
            eph.d_M_0 = d_M_0;
            eph.d_Delta_n = d_Delta_n;

            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_OMEGA_DOT_msb_bits + d_OMEGA_DOT_lsb_bits);
            eph.d_OMEGA_DOT = static_cast<double>(read_navigation_signed(subframe_bits, D2_OMEGA_DOT)) * D1_OMEGA_DOT_LSB;
            eph.d_IDOT = d_IDOT;

            eph.d_Crc = d_Crc;
            eph.d_Crs = d_Crs;
            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_Cuc_msb_bits + d_Cuc_lsb_bits);
            eph.d_Cuc = static_cast<double>(read_navigation_signed(subframe_bits, D2_CUC)) * D1_CUC_LSB;
            eph.d_Cus = d_Cus;
            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_Cic_msb_bits + d_Cic_lsb_bits);
            eph.d_Cic = static_cast<double>(read_navigation_signed(subframe_bits, D2_CIC)) * D1_CIC_LSB;
            eph.d_Cis = d_Cis;

            eph.d_A_f0 = d_A_f0;
            subframe_bits = std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>(d_A_f1_msb_bits + d_A_f1_lsb_bits);
            eph.d_A_f1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_A1)) * D1_A1_LSB;
            eph.d_A_f2 = d_A_f2;


            eph.d_TGD1 = d_TGD1;
            eph.d_TGD2 = d_TGD2;
        }
    else
        {
            eph.i_satellite_PRN = i_satellite_PRN;
            eph.d_AODC = d_AODC;
            eph.d_AODE = d_AODE;
            eph.i_SV_accuracy = i_SV_accuracy;
            eph.i_SV_health = i_SV_health;
            eph.i_BEIDOU_week = i_BEIDOU_week;
            eph.i_sig_type = i_signal_type;
            eph.i_nav_type = 1;  // MEO/IGSO

            eph.d_TOW = d_SOW;
            eph.d_Toe = ((d_Toe_sf2 + d_Toe_sf3) * D1_TOE_LSB);
            eph.d_Toc = d_Toc;

            eph.d_sqrt_A = d_sqrt_A;
            eph.d_eccentricity = d_eccentricity;
            eph.d_i_0 = d_i_0;
            eph.d_OMEGA0 = d_OMEGA0;
            eph.d_OMEGA = d_OMEGA;
            eph.d_M_0 = d_M_0;
            eph.d_Delta_n = d_Delta_n;
            eph.d_OMEGA_DOT = d_OMEGA_DOT;
            eph.d_IDOT = d_IDOT;

            eph.d_Crc = d_Crc;
            eph.d_Crs = d_Crs;
            eph.d_Cuc = d_Cuc;
            eph.d_Cus = d_Cus;
            eph.d_Cic = d_Cic;
            eph.d_Cis = d_Cis;

            eph.d_A_f0 = d_A_f0;
            eph.d_A_f1 = d_A_f1;
            eph.d_A_f2 = d_A_f2;

            eph.d_TGD1 = d_TGD1;
            eph.d_TGD2 = d_TGD2;
        }

    return eph;
}


Beidou_Dnav_Iono Beidou_Dnav_Navigation_Message::get_iono()
{
    Beidou_Dnav_Iono iono;
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

Beidou_Dnav_Utc_Model Beidou_Dnav_Navigation_Message::get_utc_model()
{
    Beidou_Dnav_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.d_A1_UTC = d_A1UTC;
    utc_model.d_A0_UTC = d_A0UTC;
    utc_model.d_DeltaT_LS = d_DeltaT_LS;
    utc_model.i_WN_LSF = i_WN_LSF;
    utc_model.i_DN = i_DN;
    utc_model.d_DeltaT_LSF = d_DeltaT_LSF;

    utc_model.d_A0_GPS = d_A0GPS;
    utc_model.d_A1_GPS = d_A1GPS;
    utc_model.d_A0_GAL = d_A0GAL;
    utc_model.d_A1_GAL = d_A1GAL;
    utc_model.d_A0_GLO = d_A0GLO;
    utc_model.d_A1_GLO = d_A1GLO;

    // warning: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_utc_model_valid = false;
    return utc_model;
}

bool Beidou_Dnav_Navigation_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if (i_satellite_PRN > 0 and i_satellite_PRN < 6)
        {
            if ((flag_sf1_p1 == true) and (flag_sf1_p2 == true) and (flag_sf1_p3 == true) and
                (flag_sf1_p4 == true) and (flag_sf1_p5 == true) and (flag_sf1_p6 == true) and
                (flag_sf1_p7 == true) and (flag_sf1_p8 == true) and (flag_sf1_p9 == true) and
                (flag_sf1_p10 == true))
                {
                    // if all ephemeris pages have the same IOD, then they belong to the same block
                    if (d_previous_aode != d_AODE)
                        {
                            // Clear flags for all received pages
                            flag_sf1_p1 = false;
                            flag_sf1_p2 = false;
                            flag_sf1_p3 = false;
                            flag_sf1_p4 = false;
                            flag_sf1_p5 = false;
                            flag_sf1_p6 = false;
                            flag_sf1_p7 = false;
                            flag_sf1_p8 = false;
                            flag_sf1_p9 = false;
                            flag_sf1_p10 = false;

                            flag_eph_valid = true;
                            // Update the time of ephemeris information
                            d_previous_aode = d_AODE;

                            return true;
                        }
                }
        }
    else
        {
            if ((flag_d1_sf1 == true) and (flag_d1_sf2 == true) and (flag_d1_sf3 == true))
                {
                    // if all ephemeris pages have the same IOD, then they belong to the same block
                    if (d_previous_aode != d_AODE)
                        {
                            // Clear flags for all received subframes
                            flag_d1_sf1 = false;
                            flag_d1_sf2 = false;
                            flag_d1_sf3 = false;

                            flag_eph_valid = true;
                            // Update the time of ephemeris information
                            d_previous_aode = d_AODE;

                            return true;
                        }
                }
        }
    return false;
}

bool Beidou_Dnav_Navigation_Message::have_new_iono()
{
    // the condition on flag_utc_model is added to have a time stamp for iono
    if (flag_iono_valid == true)
        {
            return true;
        }

    return false;
}

bool Beidou_Dnav_Navigation_Message::have_new_utc_model()
{
    if (flag_d1_sf5_p9 == true and flag_d1_sf5_p10 == true)
        {
            flag_d1_sf5_p9 = false;
            flag_d1_sf5_p10 = false;
            flag_utc_model_valid = true;

            return true;
        }

    return false;
}

bool Beidou_Dnav_Navigation_Message::have_new_almanac()
{
    if ((flag_d1_sf4 == true) and (flag_d1_sf5 == true))
        {
            // All almanac have been received
            flag_d1_sf4 = false;
            flag_d1_sf5 = false;

            return true;
        }

    return false;
}

bool Beidou_Dnav_Navigation_Message::satellite_validation()
{
    bool flag_data_valid = false;
    flag_eph_valid = false;

    // First Step:
    // check Issue Of Ephemeris Data (AODE AODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_SOW_SF1 != 0 and d_SOW_SF2 != 0 and d_SOW_SF3 != 0)
        {
            if (d_AODC != -1)
                {
                    flag_data_valid = true;
                    flag_eph_valid = true;
                }
        }
    return flag_data_valid;
}

/*!
 * \file beidou_dnav_navigation_message.cc
 * \brief  Implementation of a BeiDou D1/D2 NAV data message decoder as
 * described in BDS-SIS-ICD-B1I Version 3.0
 *
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * \author Carles Fernandez-Prades, 2018-2026. cfernandez(at)cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "beidou_dnav_navigation_message.h"
#include "gnss_satellite.h"
#include "read_navigation.h"
#include <cmath>     // for cos, sin, fmod, sqrt, atan2, fabs, floor
#include <iostream>  // for string, operator<<, cout, ostream
#include <limits>    // for std::numeric_limits
#include <numeric>   // for accumulate

Beidou_Dnav_Navigation_Message::Beidou_Dnav_Navigation_Message()
{
    auto gnss_sat = Gnss_Satellite();
    const std::string _system("Beidou");
    for (uint32_t i = 1; i < 64; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
            almanacHealth[i] = 0;
        }
}


void Beidou_Dnav_Navigation_Message::print_beidou_word_bytes(uint32_t BEIDOU_word) const
{
    std::cout << " Word =" << std::bitset<32>(BEIDOU_word) << '\n';
}


bool Beidou_Dnav_Navigation_Message::format_check(std::string const& subframe) const
{
    if (subframe.size() != static_cast<std::size_t>(BEIDOU_DNAV_SUBFRAME_DATA_BITS))
        {
            return false;
        }

    for (const auto bit : subframe)
        {
            if (bit != '0' && bit != '1')
                {
                    return false;
                }
        }

    return true;
}


void Beidou_Dnav_Navigation_Message::clear_d2_ephemeris_page_flags()
{
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
    flag_d2_ephemeris_collection_started = false;
    d_d2_expected_page = 1;
    d_d2_expected_sow = 0.0;
}


double Beidou_Dnav_Navigation_Message::wrap_dnav_sow(double sow) const
{
    double wrapped_sow = std::fmod(sow, BEIDOU_DNAV_SECONDS_PER_WEEK);
    if (wrapped_sow < 0.0)
        {
            wrapped_sow += BEIDOU_DNAV_SECONDS_PER_WEEK;
        }

    return wrapped_sow;
}


bool Beidou_Dnav_Navigation_Message::d1_ephemeris_sow_is_consistent() const
{
    return wrap_dnav_sow(d_SOW_SF2) == wrap_dnav_sow(d_SOW_SF1 + BEIDOU_DNAV_D1_SUBFRAME_PERIOD_SECONDS) &&
           wrap_dnav_sow(d_SOW_SF3) == wrap_dnav_sow(d_SOW_SF1 + 2.0 * BEIDOU_DNAV_D1_SUBFRAME_PERIOD_SECONDS);
}


bool Beidou_Dnav_Navigation_Message::d2_ephemeris_page_is_expected(int32_t page_ID, double sow)
{
    if (page_ID < 1 || page_ID > 10)
        {
            clear_d2_ephemeris_page_flags();
            return false;
        }

    if (sow < 0.0 || sow >= BEIDOU_DNAV_SECONDS_PER_WEEK)
        {
            clear_d2_ephemeris_page_flags();
            return false;
        }

    if (page_ID == 1)
        {
            clear_d2_ephemeris_page_flags();
            flag_d2_ephemeris_collection_started = true;
            return true;
        }

    if (flag_d2_ephemeris_collection_started == false || page_ID != d_d2_expected_page)
        {
            clear_d2_ephemeris_page_flags();
            return false;
        }

    if (wrap_dnav_sow(sow) != d_d2_expected_sow)
        {
            clear_d2_ephemeris_page_flags();
            return false;
        }

    return true;
}


void Beidou_Dnav_Navigation_Message::advance_d2_ephemeris_page(int32_t page_ID, double sow)
{
    if (page_ID < 1 || page_ID > 10)
        {
            clear_d2_ephemeris_page_flags();
            return;
        }

    d_d2_expected_page = page_ID + 1;
    d_d2_expected_sow = wrap_dnav_sow(sow + BEIDOU_DNAV_D2_SUBFRAME1_PAGE_PERIOD_SECONDS);
    if (page_ID == 10)
        {
            flag_d2_ephemeris_collection_started = false;
            d_d2_expected_page = 1;
            d_d2_expected_sow = 0.0;
        }
}


uint64_t Beidou_Dnav_Navigation_Message::read_navigation_data_unsigned(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits,
    int32_t first_bit,
    int32_t logical_offset,
    int32_t length) const
{
    uint64_t value = 0ULL;
    int32_t logical_bit = 0;
    int32_t transmitted_bit = first_bit;

    while (logical_bit < logical_offset + length && transmitted_bit <= BEIDOU_DNAV_SUBFRAME_DATA_BITS)
        {
            const int32_t word_bit = ((transmitted_bit - 1) % BEIDOU_DNAV_WORD_LENGTH_BITS) + 1;
            const bool parity_bit = (transmitted_bit <= static_cast<int32_t>(BEIDOU_DNAV_WORD_LENGTH_BITS)) ? (word_bit > 26) : (word_bit > 22);
            if (parity_bit == false)
                {
                    if (logical_bit >= logical_offset)
                        {
                            value = (value << 1U) |
                                    static_cast<uint64_t>(bits[BEIDOU_DNAV_SUBFRAME_DATA_BITS - transmitted_bit]);
                        }
                    ++logical_bit;
                }
            ++transmitted_bit;
        }

    return value;
}


int64_t Beidou_Dnav_Navigation_Message::sign_extend(uint64_t value, int32_t length) const
{
    if (length <= 0 || length > 64)
        {
            return 0;
        }
    if (length == 64)
        {
            return static_cast<int64_t>(value);
        }

    const uint64_t sign_bit = 1ULL << (length - 1);
    if ((value & sign_bit) != 0ULL)
        {
            value |= (~0ULL << length);
        }
    return static_cast<int64_t>(value);
}


void Beidou_Dnav_Navigation_Message::decode_almanac(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits,
    int32_t prn,
    int32_t amid,
    bool expanded)
{
    d_SQRT_A_ALMANAC = static_cast<double>(read_navigation_unsigned(bits, D1_SQRT_A_ALMANAC)) * D1_SQRT_A_ALMANAC_LSB;
    d_A1_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_A1_ALMANAC)) * D1_A1_ALMANAC_LSB;
    d_A0_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_A0_ALMANAC)) * D1_A0_ALMANAC_LSB;
    d_OMEGA0_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_OMEGA0_ALMANAC)) * D1_OMEGA0_ALMANAC_LSB;
    d_E_ALMANAC = static_cast<double>(read_navigation_unsigned(bits, D1_E_ALMANAC)) * D1_E_ALMANAC_LSB;
    d_DELTA_I = static_cast<double>(read_navigation_signed(bits, D1_DELTA_I)) * D1_DELTA_I_LSB;
    d_TOA = static_cast<double>(read_navigation_unsigned(bits, D1_TOA)) * D1_TOA_LSB;
    d_OMEGA_DOT_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_OMEGA_DOT_ALMANAC)) * D1_OMEGA_DOT_ALMANAC_LSB;
    d_OMEGA_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_OMEGA_ALMANAC)) * D1_OMEGA_ALMANAC_LSB;
    d_M0_ALMANAC = static_cast<double>(read_navigation_signed(bits, D1_M0_ALMANAC)) * D1_M0_ALMANAC_LSB;

    d_almanac.PRN = static_cast<uint32_t>(prn);
    d_almanac.sqrtA = d_SQRT_A_ALMANAC;
    d_almanac.af1 = d_A1_ALMANAC;
    d_almanac.af0 = d_A0_ALMANAC;
    d_almanac.OMEGA_0 = d_OMEGA0_ALMANAC;
    d_almanac.ecc = d_E_ALMANAC;
    d_almanac.delta_i = d_DELTA_I;
    d_almanac.toa = static_cast<int32_t>(d_TOA);
    d_almanac.OMEGAdot = d_OMEGA_DOT_ALMANAC;
    d_almanac.omega = d_OMEGA_ALMANAC;
    d_almanac.M_0 = d_M0_ALMANAC;
    d_almanac.WNa = almanac_WN;
    d_almanac.SV_health = almanacHealth[prn];
    d_almanac.AmEpID = i_AmEpID;
    d_almanac.AmID = amid;
    d_almanac.expanded = expanded;
    flag_new_almanac = true;
}


void Beidou_Dnav_Navigation_Message::decode_almanac_health(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits,
    int32_t count,
    int32_t first_prn)
{
    for (int32_t i = 0; i < count; ++i)
        {
            const int32_t prn = first_prn + i;
            almanacHealth[prn] = static_cast<int32_t>(read_navigation_data_unsigned(bits, 51, i * 9, 9));
            if (d_almanac.PRN == static_cast<uint32_t>(prn))
                {
                    d_almanac.SV_health = almanacHealth[prn];
                }
        }
}


void Beidou_Dnav_Navigation_Message::decode_time_offsets(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits)
{
    d_A0GPS = static_cast<double>(read_navigation_signed(bits, D1_A0GPS)) * D1_A0GPS_LSB;
    d_A1GPS = static_cast<double>(read_navigation_signed(bits, D1_A1GPS)) * D1_A1GPS_LSB;
    d_A0GAL = static_cast<double>(read_navigation_signed(bits, D1_A0GAL)) * D1_A0GAL_LSB;
    d_A1GAL = static_cast<double>(read_navigation_signed(bits, D1_A1GAL)) * D1_A1GAL_LSB;
    d_A0GLO = static_cast<double>(read_navigation_signed(bits, D1_A0GLO)) * D1_A0GLO_LSB;
    d_A1GLO = static_cast<double>(read_navigation_signed(bits, D1_A1GLO)) * D1_A1GLO_LSB;
    flag_d1_sf5_p9 = true;
}


void Beidou_Dnav_Navigation_Message::decode_utc_parameters(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits)
{
    i_DeltaT_LS = static_cast<int32_t>(read_navigation_signed(bits, D1_DELTA_T_LS));
    d_DeltaT_LSF = static_cast<double>(read_navigation_signed(bits, D1_DELTA_T_LSF));
    i_WN_LSF = static_cast<int32_t>(read_navigation_unsigned(bits, D1_WN_LSF));
    d_A0UTC = static_cast<double>(read_navigation_signed(bits, D1_A0UTC)) * D1_A0UTC_LSB;
    d_A1UTC = static_cast<double>(read_navigation_signed(bits, D1_A1UTC)) * D1_A1UTC_LSB;
    i_DN = static_cast<int32_t>(read_navigation_unsigned(bits, D1_DN));
    flag_d1_sf5_p10 = true;
}


void Beidou_Dnav_Navigation_Message::decode_d2_iono_grid(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits,
    int32_t page_ID)
{
    int32_t first_igp = 0;
    int32_t count = 0;
    if (page_ID >= 1 && page_ID <= 12)
        {
            first_igp = 1 + (page_ID - 1) * 13;
            count = 13;
        }
    else if (page_ID == 13)
        {
            first_igp = 157;
            count = 4;
        }
    else if (page_ID >= 61 && page_ID <= 72)
        {
            first_igp = 161 + (page_ID - 61) * 13;
            count = 13;
        }
    else if (page_ID == 73)
        {
            first_igp = 317;
            count = 4;
        }
    else
        {
            return;
        }

    for (int32_t i = 0; i < count; ++i)
        {
            const uint64_t ion = read_navigation_data_unsigned(bits, D2_ION_LOGICAL_FIRST_BIT, i * 13, 13);
            const auto delay_code = static_cast<int32_t>(ion >> 4U);
            Beidou_Dnav_Iono_Grid_Point grid_point;
            grid_point.vertical_delay = static_cast<double>(delay_code) * D2_ION_DELAY_LSB;
            grid_point.GIVEI = static_cast<int32_t>(ion & 0x0FULL);
            grid_point.monitored = delay_code < 510;
            grid_point.available = delay_code != 511;
            d_iono_grid[first_igp + i] = grid_point;
        }

    flag_iono_grid_valid = true;
    flag_iono_valid = true;
}


void Beidou_Dnav_Navigation_Message::update_d2_bdid(uint64_t value, int32_t first_prn, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
        {
            d_d2_bdid[static_cast<std::size_t>(first_prn + i - 1)] =
                ((value >> (count - i - 1)) & 1ULL) != 0ULL;
        }
}


void Beidou_Dnav_Navigation_Message::update_d2_udrei(
    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits,
    int32_t first_bit,
    int32_t first_slot,
    int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
        {
            d_d2_udrei[static_cast<std::size_t>(first_slot + i - 1)] =
                static_cast<int32_t>(read_navigation_data_unsigned(bits, first_bit, i * 4, 4));
        }
}


int32_t Beidou_Dnav_Navigation_Message::d1_subframe_decoder(std::string const& subframe)
{
    flag_crc_test = format_check(subframe);
    if (flag_crc_test == false)
        {
            return 0;
        }

    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> subframe_bits(subframe);
    const auto subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_FRAID));

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

            d_Delta_n = static_cast<double>(read_navigation_signed(subframe_bits, D1_DELTA_N));
            d_Delta_n = d_Delta_n * D1_DELTA_N_LSB;

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

            {
                const auto page_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_PNUM));
                if (page_ID >= 1 && page_ID <= 24)
                    {
                        i_AmEpID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMEPID));
                        decode_almanac(subframe_bits, page_ID, 0, false);
                    }
            }

            // Set system flags for message reception
            flag_d1_sf4 = true;
            flag_new_SOW_available = true;

            break;

        case 5:  // --- It is subframe 5 ---
            int32_t SV_page_5;
            d_SOW_SF5 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_SOW));
            d_SOW = d_SOW_SF5;  // Set transmission time
            SV_page_5 = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_PNUM));

            if (SV_page_5 >= 1 && SV_page_5 <= 6)
                {
                    i_AmEpID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMEPID));
                    decode_almanac(subframe_bits, 24 + SV_page_5, 0, false);
                }
            else if (SV_page_5 == 7)
                {
                    decode_almanac_health(subframe_bits, 19, 1);
                }
            else if (SV_page_5 == 8)
                {
                    decode_almanac_health(subframe_bits, 11, 20);
                    almanac_WN = static_cast<int>(read_navigation_unsigned(subframe_bits, D1_WNA));
                    d_toa2 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOA2)) * D1_TOA_LSB;
                    flag_almanac_week_valid = true;
                }
            else if (SV_page_5 == 9)
                {
                    decode_time_offsets(subframe_bits);
                }
            else if (SV_page_5 == 10)
                {
                    decode_utc_parameters(subframe_bits);
                }
            else if (SV_page_5 >= 11 && SV_page_5 <= 23 && i_AmEpID == 3)
                {
                    const auto amid = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMID));
                    int32_t prn = 0;
                    if (amid == 1)
                        {
                            prn = 31 + SV_page_5 - 11;
                        }
                    else if (amid == 2)
                        {
                            prn = 44 + SV_page_5 - 11;
                        }
                    else if (amid == 3 && SV_page_5 <= 17)
                        {
                            prn = 57 + SV_page_5 - 11;
                        }
                    if (prn != 0)
                        {
                            decode_almanac(subframe_bits, prn, amid, true);
                        }
                }
            else if (SV_page_5 == 24 && i_AmEpID == 3)
                {
                    const auto amid = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMID_HEALTH));
                    if (amid == 1)
                        {
                            decode_almanac_health(subframe_bits, 13, 31);
                        }
                    else if (amid == 2)
                        {
                            decode_almanac_health(subframe_bits, 13, 44);
                        }
                    else if (amid == 3)
                        {
                            decode_almanac_health(subframe_bits, 7, 57);
                        }
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
    flag_crc_test = format_check(subframe);
    if (flag_crc_test == false)
        {
            return 0;
        }

    const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS> subframe_bits(subframe);

    const auto subframe_ID = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_FRAID));
    int32_t page_ID = 0;
    if (subframe_ID == 1)
        {
            page_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_PNUM1));
        }
    else if (subframe_ID == 2)
        {
            page_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_PNUM2));
        }
    else if (subframe_ID == 5)
        {
            page_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_PNUM5));
        }
    const auto sow = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SOW));

    // D2 ephemeris pages must be collected as one contiguous page 1-10 cycle.
    if (subframe_ID == 1 && d2_ephemeris_page_is_expected(page_ID, sow) == false)
        {
            flag_crc_test = false;
            return 0;
        }

    // Decode all 5 sub-frames
    switch (subframe_ID)
        {
        // -- Decode the sub-frame id ------------------------------------------
        case 1:

            switch (page_ID)
                {
                case 1:
                    d_SOW = sow;
                    i_SV_health = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_SAT_H1));
                    d_AODC = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_AODC));
                    i_SV_accuracy = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_URAI));  // (20.3.3.3.1.3)
                    i_BEIDOU_week = static_cast<int>(read_navigation_unsigned(subframe_bits, D2_WN));
                    d_Toc = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_TOC)) * D1_TOC_LSB;
                    d_TGD1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_TGD1)) * D1_TGD1_LSB;
                    d_TGD2 = static_cast<double>(read_navigation_signed(subframe_bits, D2_TGD2)) * D1_TGD2_LSB;

                    // Set system flags for message reception
                    flag_sf1_p1 = true;
                    flag_new_SOW_available = true;

                    break;
                case 2:
                    d_SOW = sow;
                    d_alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA0)) * D1_ALPHA0_LSB;
                    d_alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA1)) * D1_ALPHA1_LSB;
                    d_alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA2)) * D1_ALPHA2_LSB;
                    d_alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, D2_ALPHA3)) * D1_ALPHA3_LSB;
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
                    d_SOW = sow;
                    d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_A0)) * D1_A0_LSB;
                    d_A_f1_msb_bits = (read_navigation_unsigned(subframe_bits, D2_A1_MSB));
                    // Adjust for lsb in next page
                    d_A_f1_msb_bits = d_A_f1_msb_bits << 18ULL;

                    // Set system flags for message reception
                    flag_sf1_p3 = true;
                    flag_new_SOW_available = true;

                    break;
                case 4:
                    d_SOW = sow;
                    d_A_f1_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_A1_LSB));
                    d_A_f1 = static_cast<double>(sign_extend(d_A_f1_msb_bits | d_A_f1_lsb_bits, 22)) * D1_A1_LSB;
                    d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, D2_A2)) * D1_A2_LSB;
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
                    d_SOW = sow;
                    d_Cuc_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_CUC_LSB));
                    d_Cuc = static_cast<double>(sign_extend(d_Cuc_msb_bits | d_Cuc_lsb_bits, 18)) * D1_CUC_LSB;
                    d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_M0)) * D1_M0_LSB;
                    d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, D2_CUS)) * D1_CUS_LSB;
                    d_eccentricity_msb = read_navigation_unsigned(subframe_bits, D2_E_MSB);
                    d_eccentricity_msb_bits = d_eccentricity_msb;
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_eccentricity_msb = static_cast<uint64_t>(d_eccentricity_msb) << 22U;
                    d_eccentricity_msb_bits = d_eccentricity_msb_bits << 22U;

                    // Set system flags for message reception
                    flag_sf1_p5 = true;
                    flag_new_SOW_available = true;

                    break;
                case 6:
                    d_SOW = sow;
                    d_eccentricity_lsb = read_navigation_unsigned(subframe_bits, D2_E_LSB);
                    d_eccentricity_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_E_LSB));
                    d_eccentricity = static_cast<double>(d_eccentricity_msb | d_eccentricity_lsb) * D1_E_LSB;
                    d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, D2_SQRT_A)) * D1_SQRT_A_LSB;
                    d_Cic_msb_bits = (read_navigation_unsigned(subframe_bits, D2_CIC_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_Cic_msb_bits = d_Cic_msb_bits << 8U;

                    // Set system flags for message reception
                    flag_sf1_p6 = true;
                    flag_new_SOW_available = true;

                    break;
                case 7:
                    d_SOW = sow;
                    d_Cic_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_CIC_LSB));
                    d_Cic = static_cast<double>(sign_extend(d_Cic_msb_bits | d_Cic_lsb_bits, 18)) * D1_CIC_LSB;
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
                    d_SOW = sow;
                    d_i_0_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_I0_LSB));
                    d_i_0 = static_cast<double>(sign_extend(d_i_0_msb_bits | d_i_0_lsb_bits, 32)) * D1_I0_LSB;
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
                    d_SOW = sow;
                    d_OMEGA_DOT_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_DOT_LSB));
                    d_OMEGA_DOT = static_cast<double>(sign_extend(d_OMEGA_DOT_msb_bits | d_OMEGA_DOT_lsb_bits, 24)) * D1_OMEGA_DOT_LSB;
                    d_OMEGA0 = static_cast<double>(read_navigation_signed(subframe_bits, D2_OMEGA0)) * D1_OMEGA0_LSB;
                    d_OMEGA_msb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_MSB));
                    // Adjust for lsb in next page (shift number of lsb to the left)
                    d_OMEGA_msb_bits = d_OMEGA_msb_bits << 5U;

                    // Set system flags for message reception
                    flag_sf1_p9 = true;
                    flag_new_SOW_available = true;

                    break;
                case 10:
                    d_SOW = sow;
                    d_OMEGA_lsb_bits = (read_navigation_unsigned(subframe_bits, D2_OMEGA_LSB));
                    d_OMEGA = static_cast<double>(sign_extend(d_OMEGA_msb_bits | d_OMEGA_lsb_bits, 32)) * D1_OMEGA_LSB;
                    d_IDOT = static_cast<double>(read_navigation_signed(subframe_bits, D2_IDOT)) * D1_IDOT_LSB;

                    // Set system flags for message reception
                    flag_sf1_p10 = true;
                    flag_new_SOW_available = true;

                    break;
                default:
                    break;
                }

            advance_d2_ephemeris_page(page_ID, sow);
            break;

        case 2:  // -- It is subframe 2 -------------------
            d_SOW = wrap_dnav_sow(sow + 0.6);
            flag_new_SOW_available = true;
            if (page_ID >= 1 && page_ID <= 6)
                {
                    i_d2_integrity_page = page_ID;
                    d_d2_integrity_sow = sow;
                    flag_d2_sf2_valid = true;
                    flag_d2_sf3_valid = false;
                    d_d2_correction_valid.fill(false);

                    i_SatH2 = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_SAT_H2));
                    update_d2_bdid(read_navigation_unsigned(subframe_bits, D2_BDID1_30), 1, 30);
                    update_d2_udrei(subframe_bits, D2_UDREI1_LOGICAL_FIRST_BIT, 1, 18);

                    const int32_t slot = 3 * (page_ID - 1);
                    d_d2_rurai[static_cast<std::size_t>(slot)] =
                        static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_RURAI_SF2));
                    const int64_t delta_t = read_navigation_signed(subframe_bits, D2_DELTA_T_SF2);
                    d_d2_delta_t[static_cast<std::size_t>(slot)] = static_cast<double>(delta_t) * D2_DELTA_T_LSB;
                    d_d2_delta_t_available[static_cast<std::size_t>(slot)] = delta_t != -4096;
                    d_d2_correction_valid[static_cast<std::size_t>(slot)] = true;
                }
            break;

        case 3:  // --- It is subframe 3 -------------------------------------
            d_SOW = wrap_dnav_sow(sow + 1.2);
            flag_new_SOW_available = true;
            if (flag_d2_sf2_valid && sow == d_d2_integrity_sow)
                {
                    const int32_t first_slot = 3 * (i_d2_integrity_page - 1) + 1;
                    d_d2_rurai[static_cast<std::size_t>(first_slot)] =
                        static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_RURAI_SF3_1));
                    const int64_t delta_t_1 = read_navigation_signed(subframe_bits, D2_DELTA_T_SF3_1);
                    d_d2_delta_t[static_cast<std::size_t>(first_slot)] = static_cast<double>(delta_t_1) * D2_DELTA_T_LSB;
                    d_d2_delta_t_available[static_cast<std::size_t>(first_slot)] = delta_t_1 != -4096;
                    d_d2_correction_valid[static_cast<std::size_t>(first_slot)] = true;

                    const int32_t second_slot = first_slot + 1;
                    d_d2_rurai[static_cast<std::size_t>(second_slot)] =
                        static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_RURAI_SF3_2));
                    const int64_t delta_t_2 = read_navigation_signed(subframe_bits, D2_DELTA_T_SF3_2);
                    d_d2_delta_t[static_cast<std::size_t>(second_slot)] = static_cast<double>(delta_t_2) * D2_DELTA_T_LSB;
                    d_d2_delta_t_available[static_cast<std::size_t>(second_slot)] = delta_t_2 != -4096;
                    d_d2_correction_valid[static_cast<std::size_t>(second_slot)] = true;
                    flag_d2_sf3_valid = true;
                }
            break;

        case 4:  // --- It is subframe 4: expanded integrity and differential corrections ---
            d_SOW = wrap_dnav_sow(sow + 1.8);
            flag_new_SOW_available = true;
            if (flag_d2_sf2_valid && flag_d2_sf3_valid && sow == d_d2_integrity_sow)
                {
                    i_BDEpID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_BDEPID));
                    if (i_BDEpID == 3)
                        {
                            update_d2_bdid(read_navigation_unsigned(subframe_bits, D2_BDID31_63), 31, 33);
                            update_d2_udrei(subframe_bits, D2_UDREI19_LOGICAL_FIRST_BIT, 19, 6);

                            const int32_t slot = 18 + i_d2_integrity_page - 1;
                            d_d2_rurai[static_cast<std::size_t>(slot)] =
                                static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D2_RURAI_SF4));
                            const int64_t delta_t = read_navigation_signed(subframe_bits, D2_DELTA_T_SF4);
                            d_d2_delta_t[static_cast<std::size_t>(slot)] = static_cast<double>(delta_t) * D2_DELTA_T_LSB;
                            d_d2_delta_t_available[static_cast<std::size_t>(slot)] = delta_t != -4096;
                            d_d2_correction_valid[static_cast<std::size_t>(slot)] = true;
                        }
                    else
                        {
                            for (std::size_t i = 30; i < d_d2_bdid.size(); ++i)
                                {
                                    d_d2_bdid[i] = false;
                                }
                            for (std::size_t i = 18; i < d_d2_correction_valid.size(); ++i)
                                {
                                    d_d2_correction_valid[i] = false;
                                }
                        }
                    flag_new_differential_corrections = true;
                }
            flag_d2_sf2_valid = false;
            flag_d2_sf3_valid = false;
            break;

        case 5:  // --- It is subframe 5: ionosphere, almanac and time parameters ---
            d_SOW = wrap_dnav_sow(sow + 2.4);
            flag_new_SOW_available = true;
            decode_d2_iono_grid(subframe_bits, page_ID);

            if (page_ID == 35)
                {
                    decode_almanac_health(subframe_bits, 19, 1);
                }
            else if (page_ID == 36)
                {
                    decode_almanac_health(subframe_bits, 11, 20);
                    almanac_WN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_WNA));
                    d_toa2 = static_cast<double>(read_navigation_unsigned(subframe_bits, D1_TOA2)) * D1_TOA_LSB;
                    flag_almanac_week_valid = true;
                }
            else if (page_ID >= 37 && page_ID <= 60)
                {
                    i_AmEpID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMEPID));
                    decode_almanac(subframe_bits, page_ID - 36, 0, false);
                }
            else if (page_ID >= 95 && page_ID <= 100)
                {
                    i_AmEpID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMEPID));
                    decode_almanac(subframe_bits, page_ID - 70, 0, false);
                }
            else if (page_ID == 101)
                {
                    decode_time_offsets(subframe_bits);
                }
            else if (page_ID == 102)
                {
                    decode_utc_parameters(subframe_bits);
                }
            else if (page_ID >= 103 && page_ID <= 115 && i_AmEpID == 3)
                {
                    const auto amid = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMID));
                    int32_t prn = 0;
                    if (amid == 1)
                        {
                            prn = 31 + page_ID - 103;
                        }
                    else if (amid == 2)
                        {
                            prn = 44 + page_ID - 103;
                        }
                    else if (amid == 3 && page_ID <= 109)
                        {
                            prn = 57 + page_ID - 103;
                        }
                    if (prn != 0)
                        {
                            decode_almanac(subframe_bits, prn, amid, true);
                        }
                }
            else if (page_ID == 116 && i_AmEpID == 3)
                {
                    const auto amid = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, D1_AMID_HEALTH));
                    if (amid == 1)
                        {
                            decode_almanac_health(subframe_bits, 13, 31);
                        }
                    else if (amid == 2)
                        {
                            decode_almanac_health(subframe_bits, 13, 44);
                        }
                    else if (amid == 3)
                        {
                            decode_almanac_health(subframe_bits, 7, 57);
                        }
                }
            break;

        default:
            break;
        }  // switch subframeID ...

    return subframe_ID;
}


double Beidou_Dnav_Navigation_Message::utc_time(double beidoutime_corrected) const
{
    double t_utc;
    double t_utc_daytime;
    const double Delta_t_UTC = i_DeltaT_LS + d_A0UTC + d_A1UTC * (beidoutime_corrected);

    // Determine if the effectivity time of the leap second event is in the past
    const int32_t weeksToLeapSecondEvent = i_WN_LSF - i_BEIDOU_week;

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
        {
            // Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            const int32_t secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                }
            else  // we are in the same week than the leap second event
                {
                    if ((beidoutime_corrected - secondOfLeapSecondEvent) < (static_cast<double>(2) / static_cast<double>(3)) * 24 * 60 * 60)
                        {
                            t_utc_daytime = fmod(beidoutime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            if ((beidoutime_corrected - secondOfLeapSecondEvent) < (static_cast<double>(5) / static_cast<double>(4)) * 24 * 60 * 60)
                                {
                                    const int32_t W = fmod(beidoutime_corrected - Delta_t_UTC - 43200, 86400) + 43200;
                                    t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - i_DeltaT_LS);
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

    const double secondsOfWeekBeforeToday = 43200 * floor(beidoutime_corrected / 43200);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}


Beidou_Dnav_Ephemeris Beidou_Dnav_Navigation_Message::get_ephemeris() const
{
    Beidou_Dnav_Ephemeris eph;

    if ((i_satellite_PRN > 0 && i_satellite_PRN < 6) || i_satellite_PRN > 58)
        {
            // Order as given by eph_t in rtklib
            eph.PRN = i_satellite_PRN;
            eph.AODC = d_AODC;
            eph.AODE = d_AODE;
            eph.SV_accuracy = i_SV_accuracy;
            eph.SV_health = i_SV_health;
            eph.WN = i_BEIDOU_week;
            eph.sig_type = i_signal_type;
            eph.nav_type = 2;

            eph.tow = d_SOW;
            eph.toe = d_Toe;
            eph.toc = d_Toc;

            eph.sqrtA = d_sqrt_A;
            eph.ecc = d_eccentricity;
            eph.i_0 = d_i_0;
            eph.OMEGA_0 = d_OMEGA0;
            eph.omega = d_OMEGA;
            eph.M_0 = d_M_0;
            eph.delta_n = d_Delta_n;

            eph.OMEGAdot = d_OMEGA_DOT;
            eph.idot = d_IDOT;

            eph.Crc = d_Crc;
            eph.Crs = d_Crs;
            eph.Cuc = d_Cuc;
            eph.Cus = d_Cus;
            eph.Cic = d_Cic;
            eph.Cis = d_Cis;

            eph.af0 = d_A_f0;
            eph.af1 = d_A_f1;
            eph.af2 = d_A_f2;

            eph.TGD1 = d_TGD1;
            eph.TGD2 = d_TGD2;
        }
    else
        {
            eph.PRN = i_satellite_PRN;
            eph.AODC = d_AODC;
            eph.AODE = d_AODE;
            eph.SV_accuracy = i_SV_accuracy;
            eph.SV_health = i_SV_health;
            eph.WN = i_BEIDOU_week;
            eph.sig_type = i_signal_type;
            eph.nav_type = 1;  // MEO/IGSO

            eph.tow = d_SOW;
            eph.toe = ((d_Toe_sf2 + d_Toe_sf3) * D1_TOE_LSB);
            eph.toc = d_Toc;

            eph.sqrtA = d_sqrt_A;
            eph.ecc = d_eccentricity;
            eph.i_0 = d_i_0;
            eph.OMEGA_0 = d_OMEGA0;
            eph.omega = d_OMEGA;
            eph.M_0 = d_M_0;
            eph.delta_n = d_Delta_n;
            eph.OMEGAdot = d_OMEGA_DOT;
            eph.idot = d_IDOT;

            eph.Crc = d_Crc;
            eph.Crs = d_Crs;
            eph.Cuc = d_Cuc;
            eph.Cus = d_Cus;
            eph.Cic = d_Cic;
            eph.Cis = d_Cis;

            eph.af0 = d_A_f0;
            eph.af1 = d_A_f1;
            eph.af2 = d_A_f2;

            eph.TGD1 = d_TGD1;
            eph.TGD2 = d_TGD2;
        }

    return eph;
}


Beidou_Dnav_Iono Beidou_Dnav_Navigation_Message::get_iono()
{
    Beidou_Dnav_Iono iono;
    iono.alpha0 = d_alpha0;
    iono.alpha1 = d_alpha1;
    iono.alpha2 = d_alpha2;
    iono.alpha3 = d_alpha3;
    iono.beta0 = d_beta0;
    iono.beta1 = d_beta1;
    iono.beta2 = d_beta2;
    iono.beta3 = d_beta3;
    iono.valid = flag_iono_valid;
    iono.grid_points = d_iono_grid;
    iono.grid_valid = flag_iono_grid_valid;
    // Do not re-send the same information to the ionospheric parameters queue.
    flag_iono_valid = false;
    return iono;
}


Beidou_Dnav_Utc_Model Beidou_Dnav_Navigation_Message::get_utc_model()
{
    Beidou_Dnav_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.A1_UTC = d_A1UTC;
    utc_model.A0_UTC = d_A0UTC;
    utc_model.DeltaT_LS = i_DeltaT_LS;
    utc_model.WN_LSF = i_WN_LSF;
    utc_model.DN = i_DN;
    utc_model.DeltaT_LSF = d_DeltaT_LSF;

    utc_model.A0_GPS = d_A0GPS;
    utc_model.A1_GPS = d_A1GPS;
    utc_model.A0_GAL = d_A0GAL;
    utc_model.A1_GAL = d_A1GAL;
    utc_model.A0_GLO = d_A0GLO;
    utc_model.A1_GLO = d_A1GLO;

    // warning: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_utc_model_valid = false;
    return utc_model;
}


Beidou_Dnav_Almanac Beidou_Dnav_Navigation_Message::get_almanac() const
{
    Beidou_Dnav_Almanac almanac = d_almanac;
    int32_t expanded_week = almanac_WN;
    if (i_BEIDOU_week != 0)
        {
            expanded_week = (i_BEIDOU_week & ~0xFF) | almanac_WN;
            if (expanded_week - i_BEIDOU_week > 128)
                {
                    expanded_week -= 256;
                }
            else if (i_BEIDOU_week - expanded_week > 128)
                {
                    expanded_week += 256;
                }
        }
    almanac.WNa = expanded_week;
    const auto health = almanacHealth.find(static_cast<int32_t>(almanac.PRN));
    if (health != almanacHealth.cend())
        {
            almanac.SV_health = health->second;
        }
    return almanac;
}


Beidou_Dnav_Differential_Corrections Beidou_Dnav_Navigation_Message::get_differential_corrections() const
{
    Beidou_Dnav_Differential_Corrections data;
    data.source_PRN = i_satellite_PRN;
    data.SOW = d_d2_integrity_sow;
    data.Pnum2 = i_d2_integrity_page;
    data.SatH2 = i_SatH2;
    data.BDEpID = i_BDEpID;
    data.range_corrections_valid = (i_SatH2 & 0x2) == 0;
    data.ionospheric_grid_valid = (i_SatH2 & 0x1) == 0;
    data.expanded_corrections = i_BDEpID == 3;

    std::size_t slot = 0;
    for (std::size_t prn_index = 0; prn_index < d_d2_bdid.size() && slot < d_d2_correction_valid.size(); ++prn_index)
        {
            if (d_d2_bdid[prn_index] == false)
                {
                    continue;
                }
            if (d_d2_correction_valid[slot])
                {
                    Beidou_Dnav_Differential_Correction correction;
                    correction.PRN = static_cast<uint32_t>(prn_index + 1);
                    correction.RURAI = d_d2_rurai[slot];
                    correction.UDREI = d_d2_udrei[slot];
                    correction.delta_t = d_d2_delta_t[slot];
                    correction.delta_t_available = d_d2_delta_t_available[slot];
                    data.corrections[static_cast<int32_t>(correction.PRN)] = correction;
                }
            ++slot;
        }
    return data;
}


bool Beidou_Dnav_Navigation_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if ((i_satellite_PRN > 0 && i_satellite_PRN < 6) || i_satellite_PRN > 58)
        {
            if ((flag_sf1_p1 == true) && (flag_sf1_p2 == true) && (flag_sf1_p3 == true) &&
                (flag_sf1_p4 == true) && (flag_sf1_p5 == true) && (flag_sf1_p6 == true) &&
                (flag_sf1_p7 == true) && (flag_sf1_p8 == true) && (flag_sf1_p9 == true) &&
                (flag_sf1_p10 == true))
                {
                    // if all ephemeris pages have the same IOD, then they belong to the same block
                    if (d_previous_aode != d_AODE)
                        {
                            // Clear flags for all received pages
                            clear_d2_ephemeris_page_flags();

                            flag_eph_valid = true;
                            // Update the time of ephemeris information
                            d_previous_aode = d_AODE;

                            return true;
                        }
                }
        }
    else
        {
            if ((flag_d1_sf1 == true) && (flag_d1_sf2 == true) && (flag_d1_sf3 == true) &&
                d1_ephemeris_sow_is_consistent() == true)
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


bool Beidou_Dnav_Navigation_Message::have_new_iono() const
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
    if (flag_d1_sf5_p9 == true && flag_d1_sf5_p10 == true)
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
    if (flag_new_almanac && flag_almanac_week_valid)
        {
            flag_new_almanac = false;
            return true;
        }

    return false;
}


bool Beidou_Dnav_Navigation_Message::have_new_differential_corrections()
{
    if (flag_new_differential_corrections)
        {
            flag_new_differential_corrections = false;
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
    if (d_SOW_SF1 != 0 && d_SOW_SF2 != 0 && d_SOW_SF3 != 0)
        {
            if (d_AODC != -1)
                {
                    flag_data_valid = true;
                    flag_eph_valid = true;
                }
        }
    return flag_data_valid;
}

/*!
 * \file beidou_cnav3_navigation_message.cc
 * \brief B-CNAV3 parser for BeiDou B2b (BDS-SIS-ICD-B2b-1.0)
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
 *
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

#include "beidou_cnav3_navigation_message.h"
#include "Beidou_CNAV1.h"
#include "Beidou_CNAV3.h"
#include "MATH_CONSTANTS.h"
#include <cstdint>

namespace
{
constexpr char PREAMBLE[16] = {'1', '1', '1', '0', '1', '0', '1', '1', '1', '0', '0', '1', '0', '0', '0', '0'};

uint64_t read_unsigned(const uint8_t* bits, int32_t offset, int32_t length)
{
    uint64_t v = 0;
    for (int32_t i = 0; i < length; i++)
        {
            v = (v << 1) | (bits[offset + i] ? 1ULL : 0ULL);
        }
    return v;
}

int64_t sign_extend(uint64_t raw, int32_t length)
{
    const uint64_t sign_bit = 1ULL << (length - 1);
    if (raw & sign_bit)
        {
            return static_cast<int64_t>(raw) - static_cast<int64_t>(1ULL << length);
        }
    return static_cast<int64_t>(raw);
}

int64_t read_signed(const uint8_t* bits, int32_t offset, int32_t length)
{
    return sign_extend(read_unsigned(bits, offset, length), length);
}

uint32_t crc24q(const uint8_t* bits, int32_t num_bits)
{
    uint32_t crc = 0;
    constexpr uint32_t POLY = 0x864CFBU;
    for (int32_t i = 0; i < num_bits; i++)
        {
            const uint32_t msb = (crc >> 23) & 1U;
            const uint32_t bit = (bits[i] != 0U) ? 1U : 0U;
            crc = (crc << 1) & 0xFFFFFFU;
            if (msb ^ bit)
                {
                    crc ^= POLY;
                }
        }
    return crc;
}

bool verify_crc24q(const uint8_t* bits, int32_t data_bits)
{
    const uint32_t computed = crc24q(bits, data_bits);
    const auto received = static_cast<uint32_t>(read_unsigned(bits, data_bits, BEIDOU_CNAV3_CRC_BITS));
    return computed == received;
}

double a_ref_from_sat_type(uint32_t sat_type)
{
    if (sat_type == 1U || sat_type == 2U)
        {
            return BEIDOU_CNAV1_A_REF_IGSO;
        }
    return BEIDOU_CNAV1_A_REF_MEO;
}
}  // namespace


bool Beidou_Cnav3_Navigation_Message::decode_frame_symbols(const float* symbols, int32_t n_symbols, uint32_t prn)
{
    d_last_crc_ok = false;
    d_flag_new_eph = false;
    if (n_symbols < BEIDOU_CNAV3_FRAME_SYMBOLS || symbols == nullptr)
        {
            return false;
        }

    // Hard-decide and match preamble polarity.
    std::array<int, BEIDOU_CNAV3_FRAME_SYMBOLS> hard{};
    int corr = 0;
    for (int32_t i = 0; i < BEIDOU_CNAV3_PREAMBLE_SYMBOLS; i++)
        {
            const int s = (symbols[i] >= 0.0F) ? 1 : 0;
            hard[static_cast<size_t>(i)] = s;
            const int p = (PREAMBLE[i] == '1') ? 1 : 0;
            corr += (s == p) ? 1 : -1;
        }
    const bool invert = corr < 0;
    for (int32_t i = 0; i < BEIDOU_CNAV3_FRAME_SYMBOLS; i++)
        {
            int s = (symbols[i] >= 0.0F) ? 1 : 0;
            if (invert)
                {
                    s ^= 1;
                }
            hard[static_cast<size_t>(i)] = s;
        }
    for (int32_t i = 0; i < BEIDOU_CNAV3_PREAMBLE_SYMBOLS; i++)
        {
            const int p = (PREAMBLE[i] == '1') ? 1 : 0;
            if (hard[static_cast<size_t>(i)] != p)
                {
                    return false;
                }
        }

    const int32_t body0 = BEIDOU_CNAV3_PREAMBLE_SYMBOLS;  // 16
    uint32_t frame_prn = 0;
    for (int32_t i = 0; i < BEIDOU_CNAV3_PRN_SYMBOLS; i++)
        {
            frame_prn = (frame_prn << 1) | static_cast<uint32_t>(hard[static_cast<size_t>(body0 + i)]);
        }
    d_last_frame_prn = frame_prn;
    // After preamble: 6 PRN + 6 reserved + 972 LDPC. Systematic part is first 486 of the 972.
    const int32_t info0 = body0 + BEIDOU_CNAV3_PRN_SYMBOLS + BEIDOU_CNAV3_RESERVED_SYMBOLS;
    std::array<uint8_t, BEIDOU_CNAV3_INFO_BITS> info{};
    for (int32_t i = 0; i < BEIDOU_CNAV3_INFO_BITS; i++)
        {
            info[static_cast<size_t>(i)] = static_cast<uint8_t>(hard[static_cast<size_t>(info0 + i)]);
        }

    if (!verify_crc24q(info.data(), BEIDOU_CNAV3_DATA_BITS))
        {
            return false;
        }
    d_last_crc_ok = true;
    d_last_nav_bits.clear();
    d_last_nav_bits.reserve(static_cast<size_t>(BEIDOU_CNAV3_INFO_BITS));
    for (int32_t i = 0; i < BEIDOU_CNAV3_INFO_BITS; i++)
        {
            d_last_nav_bits.push_back(info[static_cast<size_t>(i)] ? '1' : '0');
        }
    parse_info_bits(info.data(), prn);
    return true;
}


void Beidou_Cnav3_Navigation_Message::parse_info_bits(const uint8_t* bits, uint32_t channel_prn)
{
    const auto mes_type = static_cast<int32_t>(read_unsigned(bits, 0, 6));
    d_last_mes_type = mes_type;
    d_last_sow = static_cast<int32_t>(read_unsigned(bits, 6, 20));
    d_eph.PRN = channel_prn != 0 ? channel_prn : d_last_frame_prn;
    d_eph.tow = d_last_sow;
    d_eph.sig_type = BDS_EPH_SOURCE_CNAV3;

    if (mes_type == BEIDOU_CNAV3_MSG_EPH)
        {
            // ICD / MATLAB ephemeris.m message type 10 (1-based indices converted to 0-based)
            d_eph.toe = static_cast<int32_t>(read_unsigned(bits, 30, 11) * 300);
            const auto sat_type = static_cast<uint32_t>(read_unsigned(bits, 41, 2));
            d_eph.sat_type = static_cast<int32_t>(sat_type);
            d_eph.nav_type = (sat_type == 1U) ? 0 : 1;
            const double delta_a = static_cast<double>(read_signed(bits, 43, 26)) * BEIDOU_CNAV1_DELTA_A_LSB;
            d_eph.A0 = a_ref_from_sat_type(sat_type) + delta_a;
            d_eph.Adot = static_cast<double>(read_signed(bits, 69, 25)) * BEIDOU_CNAV1_A_DOT_LSB;
            d_eph.delta_n = static_cast<double>(read_signed(bits, 94, 17)) * BEIDOU_CNAV1_DELTA_N0_LSB;
            d_eph.delta_ndot = static_cast<double>(read_signed(bits, 111, 23)) * BEIDOU_CNAV1_DELTA_N0_DOT_LSB;
            d_eph.M_0 = static_cast<double>(read_signed(bits, 134, 33)) * BEIDOU_CNAV1_M0_LSB;
            d_eph.ecc = static_cast<double>(read_unsigned(bits, 167, 33)) * BEIDOU_CNAV1_E_LSB;
            d_eph.omega = static_cast<double>(read_signed(bits, 200, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
            d_eph.OMEGA_0 = static_cast<double>(read_signed(bits, 233, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
            d_eph.i_0 = static_cast<double>(read_signed(bits, 266, 33)) * BEIDOU_CNAV1_I0_LSB;
            d_eph.OMEGAdot = static_cast<double>(read_signed(bits, 299, 19)) * BEIDOU_CNAV1_OMEGADOT_LSB;
            d_eph.idot = static_cast<double>(read_signed(bits, 318, 15)) * BEIDOU_CNAV1_IDOT_LSB;
            d_eph.Cis = static_cast<double>(read_signed(bits, 333, 16)) * BEIDOU_CNAV1_CIS_LSB;
            d_eph.Cic = static_cast<double>(read_signed(bits, 349, 16)) * BEIDOU_CNAV1_CIC_LSB;
            d_eph.Crs = static_cast<double>(read_signed(bits, 365, 24)) * BEIDOU_CNAV1_CRS_LSB;
            d_eph.Crc = static_cast<double>(read_signed(bits, 389, 24)) * BEIDOU_CNAV1_CRC_LSB;
            d_eph.Cus = static_cast<double>(read_signed(bits, 413, 21)) * BEIDOU_CNAV1_CUS_LSB;
            d_eph.Cuc = static_cast<double>(read_signed(bits, 434, 21)) * BEIDOU_CNAV1_CUC_LSB;
            d_have_mt10 = true;
        }
    else if (mes_type == BEIDOU_CNAV3_MSG_CLK)
        {
            d_eph.WN = static_cast<int32_t>(read_unsigned(bits, 26, 13));
            d_eph.toc = static_cast<int32_t>(read_unsigned(bits, 43, 11) * 300);
            d_eph.af0 = static_cast<double>(read_signed(bits, 54, 25)) * BEIDOU_CNAV1_AF0_LSB;
            d_eph.af1 = static_cast<double>(read_signed(bits, 79, 22)) * BEIDOU_CNAV1_AF1_LSB;
            d_eph.af2 = static_cast<double>(read_signed(bits, 101, 11)) * BEIDOU_CNAV1_AF2_LSB;
            d_eph.TGD_B2ap = static_cast<double>(read_signed(bits, 112, 12)) * BEIDOU_CNAV1_TGD_LSB;
            d_eph.hs = static_cast<int32_t>(read_unsigned(bits, 460, 2));
            d_have_mt30 = true;
        }

    if (d_have_mt10 && d_have_mt30 && d_eph.PRN < 59)
        {
            d_flag_new_eph = true;
        }
}

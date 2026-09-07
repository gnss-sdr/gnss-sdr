/*!
 * \file beidou_cnav2_navigation_message.cc
 * \brief B-CNAV2 parser for BeiDou B2a (BDS-SIS-ICD-B2a-1.0)
 * \author huangchuhan, 2026. huangchh37(at)mail2.sysu.edu.cn
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

#include "beidou_cnav2_navigation_message.h"
#include "Beidou_CNAV1.h"
#include "Beidou_CNAV2.h"
#include "MATH_CONSTANTS.h"
#include <cstdint>

namespace
{
// 0xE24DE8, MSB first. MATLAB antipodal form maps 1 → −1 (bit 1) and 0 → +1 (bit 0).
constexpr char PREAMBLE[24] = {
    '1', '1', '1', '0', '0', '0', '1', '0', '0', '1', '0', '0',
    '1', '1', '0', '1', '1', '1', '1', '0', '1', '0', '0', '0'};

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
    const auto received = static_cast<uint32_t>(read_unsigned(bits, data_bits, BEIDOU_CNAV2_CRC_BITS));
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


bool Beidou_Cnav2_Navigation_Message::decode_frame_symbols(const float* symbols, int32_t n_symbols, uint32_t prn)
{
    d_last_crc_ok = false;
    d_flag_new_eph = false;
    if (n_symbols < BEIDOU_CNAV2_FRAME_SYMBOLS || symbols == nullptr)
        {
            return false;
        }

    std::array<int, BEIDOU_CNAV2_FRAME_SYMBOLS> hard{};
    int corr = 0;
    for (int32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_SYMBOLS; i++)
        {
            // Positive NAV symbol → bit 0 (MATLAB +1); negative → bit 1.
            const int s = (symbols[i] >= 0.0F) ? 0 : 1;
            hard[static_cast<size_t>(i)] = s;
            const int p = (PREAMBLE[i] == '1') ? 1 : 0;
            corr += (s == p) ? 1 : -1;
        }
    const bool invert = corr < 0;
    for (int32_t i = 0; i < BEIDOU_CNAV2_FRAME_SYMBOLS; i++)
        {
            int s = (symbols[i] >= 0.0F) ? 0 : 1;
            if (invert)
                {
                    s ^= 1;
                }
            hard[static_cast<size_t>(i)] = s;
        }
    for (int32_t i = 0; i < BEIDOU_CNAV2_PREAMBLE_SYMBOLS; i++)
        {
            const int p = (PREAMBLE[i] == '1') ? 1 : 0;
            if (hard[static_cast<size_t>(i)] != p)
                {
                    return false;
                }
        }

    uint32_t frame_prn = 0;
    for (int32_t i = 0; i < 6; i++)
        {
            frame_prn = (frame_prn << 1) | static_cast<uint32_t>(hard[static_cast<size_t>(BEIDOU_CNAV2_PREAMBLE_SYMBOLS + i)]);
        }
    d_last_frame_prn = frame_prn;

    // First cut: skip 64-ary LDPC. Take the first 288 systematic bits of the 576 encoded symbols.
    std::array<uint8_t, BEIDOU_CNAV2_INFO_BITS> info{};
    for (int32_t i = 0; i < BEIDOU_CNAV2_INFO_BITS; i++)
        {
            info[static_cast<size_t>(i)] = static_cast<uint8_t>(hard[static_cast<size_t>(BEIDOU_CNAV2_PREAMBLE_SYMBOLS + i)]);
        }

    if (!verify_crc24q(info.data(), BEIDOU_CNAV2_DATA_BITS))
        {
            return false;
        }
    d_last_crc_ok = true;
    d_last_nav_bits.clear();
    d_last_nav_bits.reserve(static_cast<size_t>(BEIDOU_CNAV2_INFO_BITS));
    for (int32_t i = 0; i < BEIDOU_CNAV2_INFO_BITS; i++)
        {
            d_last_nav_bits.push_back(info[static_cast<size_t>(i)] ? '1' : '0');
        }
    parse_info_bits(info.data(), prn);
    return true;
}


void Beidou_Cnav2_Navigation_Message::parse_clock_common(const uint8_t* bits, int32_t toc_off)
{
    d_eph.toc = static_cast<int32_t>(read_unsigned(bits, toc_off, 11) * 300);
    d_eph.af0 = static_cast<double>(read_signed(bits, toc_off + 11, 25)) * BEIDOU_CNAV1_AF0_LSB;
    d_eph.af1 = static_cast<double>(read_signed(bits, toc_off + 36, 22)) * BEIDOU_CNAV1_AF1_LSB;
    d_eph.af2 = static_cast<double>(read_signed(bits, toc_off + 58, 11)) * BEIDOU_CNAV1_AF2_LSB;
    const auto iodc_msb = static_cast<int32_t>(read_unsigned(bits, toc_off + 69, 2));
    const auto iodc_lsb = static_cast<int32_t>(read_unsigned(bits, toc_off + 71, 8));
    d_eph.IODC = static_cast<double>((iodc_msb << 8) | iodc_lsb);
    d_eph.IODE = d_eph.IODC;
    d_have_clk = true;
}


void Beidou_Cnav2_Navigation_Message::parse_info_bits(const uint8_t* bits, uint32_t channel_prn)
{
    d_last_frame_prn = static_cast<uint32_t>(read_unsigned(bits, 0, 6));
    const auto mes_type = static_cast<int32_t>(read_unsigned(bits, 6, 6));
    d_last_mes_type = mes_type;
    d_last_sow = static_cast<int32_t>(read_unsigned(bits, 12, 18) * BEIDOU_CNAV2_SOW_LSB_S);
    d_eph.PRN = channel_prn != 0 ? channel_prn : d_last_frame_prn;
    d_eph.tow = d_last_sow;
    d_eph.sig_type = BDS_EPH_SOURCE_CNAV2;

    if (mes_type == BEIDOU_CNAV2_MSG_EPH1)
        {
            // MATLAB include/ephemeris.m case 10, 1-based indices converted to 0-based.
            d_eph.WN = static_cast<int32_t>(read_unsigned(bits, 30, 13));
            d_eph.toe = static_cast<int32_t>(read_unsigned(bits, 61, 11) * 300);
            const auto sat_type = static_cast<uint32_t>(read_unsigned(bits, 72, 2));
            d_eph.sat_type = static_cast<int32_t>(sat_type);
            d_eph.nav_type = (sat_type == 1U) ? 0 : 1;
            const double delta_a = static_cast<double>(read_signed(bits, 74, 26)) * BEIDOU_CNAV1_DELTA_A_LSB;
            d_eph.A0 = a_ref_from_sat_type(sat_type) + delta_a;
            d_eph.Adot = static_cast<double>(read_signed(bits, 100, 25)) * BEIDOU_CNAV1_A_DOT_LSB;
            d_eph.delta_n = static_cast<double>(read_signed(bits, 125, 17)) * BEIDOU_CNAV1_DELTA_N0_LSB;
            d_eph.delta_ndot = static_cast<double>(read_signed(bits, 142, 23)) * BEIDOU_CNAV1_DELTA_N0_DOT_LSB;
            d_eph.M_0 = static_cast<double>(read_signed(bits, 165, 33)) * BEIDOU_CNAV1_M0_LSB;
            d_eph.ecc = static_cast<double>(read_unsigned(bits, 198, 33)) * BEIDOU_CNAV1_E_LSB;
            d_eph.omega = static_cast<double>(read_signed(bits, 231, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
            d_have_mt10 = true;
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_EPH2)
        {
            d_eph.hs = static_cast<int32_t>(read_unsigned(bits, 30, 2));
            d_eph.OMEGA_0 = static_cast<double>(read_signed(bits, 42, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
            d_eph.i_0 = static_cast<double>(read_signed(bits, 75, 33)) * BEIDOU_CNAV1_I0_LSB;
            d_eph.OMEGAdot = static_cast<double>(read_signed(bits, 108, 19)) * BEIDOU_CNAV1_OMEGADOT_LSB;
            d_eph.idot = static_cast<double>(read_signed(bits, 127, 15)) * BEIDOU_CNAV1_IDOT_LSB;
            d_eph.Cis = static_cast<double>(read_signed(bits, 142, 16)) * BEIDOU_CNAV1_CIS_LSB;
            d_eph.Cic = static_cast<double>(read_signed(bits, 158, 16)) * BEIDOU_CNAV1_CIC_LSB;
            d_eph.Crs = static_cast<double>(read_signed(bits, 174, 24)) * BEIDOU_CNAV1_CRS_LSB;
            d_eph.Crc = static_cast<double>(read_signed(bits, 198, 24)) * BEIDOU_CNAV1_CRC_LSB;
            d_eph.Cus = static_cast<double>(read_signed(bits, 222, 21)) * BEIDOU_CNAV1_CUS_LSB;
            d_eph.Cuc = static_cast<double>(read_signed(bits, 243, 21)) * BEIDOU_CNAV1_CUC_LSB;
            d_have_mt11 = true;
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_CLK_IONO)
        {
            parse_clock_common(bits, 42);
            d_eph.TGD_B2ap = static_cast<double>(read_signed(bits, 121, 12)) * BEIDOU_CNAV1_TGD_LSB;
            d_eph.ISC_B2ad = static_cast<double>(read_signed(bits, 133, 12)) * BEIDOU_CNAV1_ISC_LSB;
            d_eph.TGD_B1Cp = static_cast<double>(read_signed(bits, 219, 12)) * BEIDOU_CNAV1_TGD_LSB;
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_CLK_ALM ||
             mes_type == BEIDOU_CNAV2_MSG_CLK_EOP ||
             mes_type == BEIDOU_CNAV2_MSG_CLK_UTC)
        {
            parse_clock_common(bits, 42);
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_CLK_DC)
        {
            parse_clock_common(bits, 64);
        }

    if (d_have_mt10 && d_have_mt11 && d_have_clk && d_eph.PRN < 59 && d_eph.sat_type != 1 && d_eph.hs == 0)
        {
            d_flag_new_eph = true;
        }
}

/*!
 * \file beidou_cnav1_navigation_message.cc
 * \brief B-CNAV1 navigation message parser (BDS-SIS-ICD-B1C-1.0 §6.2)
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#include "beidou_cnav1_navigation_message.h"
#include "Beidou_B1C.h"
#include "Beidou_CNAV1.h"
#include "beidou_cnav1_ldpc.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace
{
struct BchCodebook
{
    int32_t n = 0;
    int32_t k = 0;
    int32_t corr_threshold = 0;
    std::vector<int8_t> bipolar_codewords;
    std::vector<uint16_t> messages;
};

std::vector<int8_t> encode_bch_hypothesis_bipolar(
    uint16_t msg_bits,
    int32_t n,
    int32_t k,
    const std::vector<int32_t>& feedback_pos_1based)
{
    std::vector<int8_t> register_state(static_cast<size_t>(k), 1);
    std::vector<int8_t> encoded(static_cast<size_t>(n), 1);

    for (int32_t i = 0; i < k; i++)
        {
            const auto bit = static_cast<int32_t>((msg_bits >> static_cast<uint32_t>(k - 1 - i)) & 1U);
            register_state[static_cast<size_t>(k - 1 - i)] = (bit == 0) ? 1 : -1;
        }

    for (int32_t ind = 0; ind < n; ind++)
        {
            encoded[static_cast<size_t>(ind)] = register_state.back();
            int8_t feedback = 1;
            for (const int32_t pos : feedback_pos_1based)
                {
                    feedback = static_cast<int8_t>(feedback * register_state[static_cast<size_t>(pos - 1)]);
                }
            for (int32_t j = k - 1; j > 0; j--)
                {
                    register_state[static_cast<size_t>(j)] = register_state[static_cast<size_t>(j - 1)];
                }
            register_state[0] = feedback;
        }

    return encoded;
}

BchCodebook build_bch_codebook(
    int32_t n,
    int32_t k,
    const std::vector<int32_t>& feedback_pos_1based,
    int32_t corr_threshold)
{
    BchCodebook cb{};
    cb.n = n;
    cb.k = k;
    cb.corr_threshold = corr_threshold;
    const uint32_t num_messages = 1U << static_cast<uint32_t>(k);
    cb.bipolar_codewords.resize(static_cast<size_t>(num_messages) * static_cast<size_t>(n));
    cb.messages.resize(num_messages);
    for (uint32_t m = 0; m < num_messages; m++)
        {
            const auto encoded = encode_bch_hypothesis_bipolar(static_cast<uint16_t>(m), n, k, feedback_pos_1based);
            std::copy(encoded.begin(), encoded.end(),
                cb.bipolar_codewords.begin() + static_cast<ptrdiff_t>(m) * static_cast<ptrdiff_t>(n));
            cb.messages[m] = static_cast<uint16_t>(m);
        }
    return cb;
}

bool decode_bch_codeword(const int32_t* in, int32_t* out_msg, const BchCodebook& cb)
{
    int32_t best_corr = std::numeric_limits<int32_t>::min();
    uint16_t best_message = 0U;
    for (size_t i = 0; i < cb.messages.size(); i++)
        {
            int32_t corr = 0;
            const auto* cw = cb.bipolar_codewords.data() + static_cast<ptrdiff_t>(i) * cb.n;
            for (int32_t j = 0; j < cb.n; j++)
                {
                    const int32_t bit = in[j] > 0 ? 1 : 0;
                    const int32_t bipolar = (bit == 0) ? 1 : -1;
                    corr += static_cast<int32_t>(cw[j]) * bipolar;
                }
            if (corr > best_corr)
                {
                    best_corr = corr;
                    best_message = cb.messages[i];
                }
        }
    if (best_corr < cb.corr_threshold)
        {
            return false;
        }
    for (int32_t bit = 0; bit < cb.k; bit++)
        {
            out_msg[bit] = static_cast<int32_t>((best_message >> static_cast<uint32_t>(cb.k - 1 - bit)) & 1U);
        }
    return true;
}


static bool decode_bch_21_6(const int32_t* in, int32_t* out_msg)
{
    static const std::vector<int32_t> feedback_pos = {2, 4, 5, 6};
    static const BchCodebook cb = build_bch_codebook(21, 6, feedback_pos, 20);
    return decode_bch_codeword(in, out_msg, cb);
}


static bool decode_bch_51_8(const int32_t* in, int32_t* out_msg)
{
    static const std::vector<int32_t> feedback_pos = {1, 4, 5, 6, 7, 8};
    static const BchCodebook cb = build_bch_codebook(51, 8, feedback_pos, 50);
    return decode_bch_codeword(in, out_msg, cb);
}


uint32_t bits_to_unsigned(const int32_t* bits, int32_t length)
{
    uint32_t value = 0;
    for (int32_t i = 0; i < length; i++)
        {
            value = (value << 1) | static_cast<uint32_t>(bits[i] & 1);
        }
    return value;
}

template <typename UIntT>
UIntT read_bits_be(const uint8_t* bits, int32_t offset, int32_t length)
{
    UIntT value = 0;
    for (int32_t i = 0; i < length; i++)
        {
            value = static_cast<UIntT>((value << 1U) | static_cast<UIntT>(bits[offset + i] & 0x1U));
        }
    return value;
}

template <typename IntT>
IntT sign_extend(uint64_t raw, int32_t bit_width)
{
    const uint64_t sign_mask = 1ULL << static_cast<uint64_t>(bit_width - 1);
    if ((raw & sign_mask) == 0ULL)
        {
            return static_cast<IntT>(raw);
        }
    const uint64_t extend_mask = ~((1ULL << static_cast<uint64_t>(bit_width)) - 1ULL);
    return static_cast<IntT>(raw | extend_mask);
}

double twos_pow(int32_t exponent)
{
    return std::ldexp(1.0, exponent);
}


void deinterleave_sf2_sf3(const float* interleaved, float* sf2, float* sf3)
{
    int32_t sf2_idx = 0;
    int32_t sf3_idx = 0;
    for (int32_t in_idx = 0; in_idx < BEIDOU_CNAV1_INTERLEAVED_SYMBOLS; in_idx++)
        {
            // Received stream is column-major readout of a 36x48 block.
            const int32_t row = in_idx % BEIDOU_CNAV1_INTERLEAVE_ROWS;
            const int32_t col = in_idx / BEIDOU_CNAV1_INTERLEAVE_ROWS;
            if (row < 33)
                {
                    const int32_t group = row / 3;  // 0..10
                    const int32_t rem = row % 3;    // 0,1 => SF2 rows ; 2 => SF3 row
                    if (rem < 2)
                        {
                            const int32_t sf2_row = group * 2 + rem;  // 0..21
                            sf2[sf2_row * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = interleaved[in_idx];
                            sf2_idx++;
                        }
                    else
                        {
                            const int32_t sf3_row = group;  // 0..10
                            sf3[sf3_row * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = interleaved[in_idx];
                            sf3_idx++;
                        }
                }
            else
                {
                    const int32_t sf2_row = 22 + (row - 33);  // 22..24
                    sf2[sf2_row * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = interleaved[in_idx];
                    sf2_idx++;
                }
        }
    (void)sf2_idx;
    (void)sf3_idx;
}

uint64_t read_unsigned(const uint8_t* bits, int32_t offset, int32_t length)
{
    return read_bits_be<uint64_t>(bits, offset, length);
}

int64_t read_signed(const uint8_t* bits, int32_t offset, int32_t length)
{
    const uint64_t raw = read_unsigned(bits, offset, length);
    return sign_extend<int64_t>(raw, length);
}
uint32_t crc24q(const uint8_t* bits, int32_t num_bits)
{
    // 寄存器初始值设为全 0
    uint32_t crc = 0;

    // 生成多项式去除了最高位 x^24 后的低 24 位掩码 (0x1864CFB & 0xFFFFFF)
    const uint32_t POLY = 0x864CFBU;

    for (int32_t i = 0; i < num_bits; i++)
        {
            // 提取当前最高位（第24位，索引为23）
            uint32_t msb = (crc >> 23) & 1U;
            uint32_t bit = (bits[i] != 0U) ? 1U : 0U;

            // 寄存器左移，保持 24 位位宽
            crc = (crc << 1) & 0xFFFFFFU;

            // MSB 反馈与输入信息比特异或，决定是否执行多项式除法（异或生成多项式）
            if (msb ^ bit)
                {
                    crc ^= POLY;
                }
        }

    // 依次输出构成 CRC 校验序列
    return crc;
}

bool verify_crc24q(const uint8_t* bits, int32_t data_bits)
{
    const uint32_t computed = crc24q(bits, data_bits);
    const auto received = static_cast<uint32_t>(read_unsigned(bits, data_bits, BEIDOU_CNAV1_CRC_BITS));
    return computed == received;
}

double a_ref_from_sat_type(uint32_t sat_type)
{
    if (sat_type == 0b01U || sat_type == 0b10U)
        {
            return BEIDOU_CNAV1_A_REF_IGSO;
        }
    return BEIDOU_CNAV1_A_REF_MEO;
}

int32_t nav_type_from_sat_type(uint32_t sat_type)
{
    if (sat_type == 0b01U)
        {
            return 0;
        }
    return 1;
}

void parse_subframe2(const uint8_t* bits, Beidou_Cnav1_Ephemeris& eph, double soh_seconds)
{
    int32_t offset = 0;
    eph.WN = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    const auto how = static_cast<int32_t>(read_unsigned(bits, offset, 8));
    offset += 8;
    eph.IODC = static_cast<double>(read_unsigned(bits, offset, 10));
    offset += 10;
    eph.IODE = static_cast<double>(read_unsigned(bits, offset, 8));
    offset += 8;

    const auto toe_raw = static_cast<int32_t>(read_unsigned(bits, offset, 11));
    offset += 11;
    const auto sat_type = static_cast<uint32_t>(read_unsigned(bits, offset, 2));
    offset += 2;
    const auto delta_a = static_cast<double>(read_signed(bits, offset, 26)) * BEIDOU_CNAV1_DELTA_A_LSB;
    offset += 26;
    eph.Adot = static_cast<double>(read_signed(bits, offset, 25)) * BEIDOU_CNAV1_A_DOT_LSB;
    offset += 25;
    eph.delta_n = static_cast<double>(read_signed(bits, offset, 17)) * BEIDOU_CNAV1_DELTA_N0_LSB;
    offset += 17;
    eph.delta_ndot = static_cast<double>(read_signed(bits, offset, 23)) * BEIDOU_CNAV1_DELTA_N0_DOT_LSB;
    offset += 23;
    eph.M_0 = static_cast<double>(read_signed(bits, offset, 33)) * BEIDOU_CNAV1_M0_LSB;
    offset += 33;
    eph.ecc = static_cast<double>(read_unsigned(bits, offset, 33)) * BEIDOU_CNAV1_E_LSB;
    offset += 33;
    eph.omega = static_cast<double>(read_signed(bits, offset, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
    offset += 33;

    eph.OMEGA_0 = static_cast<double>(read_signed(bits, offset, 33)) * BEIDOU_CNAV1_OMEGA_LSB;
    offset += 33;
    eph.i_0 = static_cast<double>(read_signed(bits, offset, 33)) * BEIDOU_CNAV1_I0_LSB;
    offset += 33;
    eph.OMEGAdot = static_cast<double>(read_signed(bits, offset, 19)) * BEIDOU_CNAV1_OMEGADOT_LSB;
    offset += 19;
    eph.idot = static_cast<double>(read_signed(bits, offset, 15)) * BEIDOU_CNAV1_IDOT_LSB;
    offset += 15;
    eph.Cis = static_cast<double>(read_signed(bits, offset, 16)) * BEIDOU_CNAV1_CIS_LSB;
    offset += 16;
    eph.Cic = static_cast<double>(read_signed(bits, offset, 16)) * BEIDOU_CNAV1_CIC_LSB;
    offset += 16;
    eph.Crs = static_cast<double>(read_signed(bits, offset, 24)) * BEIDOU_CNAV1_CRS_LSB;
    offset += 24;
    eph.Crc = static_cast<double>(read_signed(bits, offset, 24)) * BEIDOU_CNAV1_CRC_LSB;
    offset += 24;
    eph.Cus = static_cast<double>(read_signed(bits, offset, 21)) * BEIDOU_CNAV1_CUS_LSB;
    offset += 21;
    eph.Cuc = static_cast<double>(read_signed(bits, offset, 21)) * BEIDOU_CNAV1_CUC_LSB;
    offset += 21;

    eph.toc = static_cast<int32_t>(read_unsigned(bits, offset, 11));
    offset += 11;
    eph.af0 = static_cast<double>(read_signed(bits, offset, 25)) * BEIDOU_CNAV1_AF0_LSB;
    offset += 25;
    eph.af1 = static_cast<double>(read_signed(bits, offset, 22)) * BEIDOU_CNAV1_AF1_LSB;
    offset += 22;
    eph.af2 = static_cast<double>(read_signed(bits, offset, 11)) * BEIDOU_CNAV1_AF2_LSB;
    offset += 11;
    eph.TGD_B2ap = static_cast<double>(read_signed(bits, offset, 12)) * BEIDOU_CNAV1_TGD_LSB;
    offset += 12;
    eph.ISC_B1Cd = static_cast<double>(read_signed(bits, offset, 12)) * BEIDOU_CNAV1_ISC_LSB;
    offset += 12;
    eph.TGD_B1Cp = static_cast<double>(read_signed(bits, offset, 12)) * BEIDOU_CNAV1_TGD_LSB;
    offset += 12;

    const double a_ref = a_ref_from_sat_type(sat_type);
    eph.A0 = a_ref + delta_a;
    eph.toe = static_cast<int32_t>(toe_raw * BEIDOU_CNAV1_TOE_TOC_LSB);
    eph.toc = static_cast<int32_t>(eph.toc * BEIDOU_CNAV1_TOE_TOC_LSB);
    eph.tow = static_cast<int32_t>(how * 3600 + soh_seconds);
    eph.nav_type = nav_type_from_sat_type(sat_type);
    eph.sig_type = 7;
}

void parse_page_common(
    const uint8_t* bits,
    bool has_sisaioe,
    bool has_sisaioc,
    Bds3_B1c_PageData& page_data,
    int32_t& offset)
{
    offset = 0;
    page_data.common.page_id = static_cast<int32_t>(read_unsigned(bits, offset, 6));
    offset += 6;
    page_data.common.hs = static_cast<int32_t>(read_unsigned(bits, offset, 2));
    offset += 2;
    page_data.common.dif = read_unsigned(bits, offset, 1) != 0U;
    offset += 1;
    page_data.common.sif = read_unsigned(bits, offset, 1) != 0U;
    offset += 1;
    page_data.common.aif = read_unsigned(bits, offset, 1) != 0U;
    offset += 1;
    page_data.common.sismai = static_cast<int32_t>(read_unsigned(bits, offset, 4));
    offset += 4;
    page_data.sisaioe = 0;
    page_data.sisaioc = 0;
    if (has_sisaioe)
        {
            page_data.sisaioe = static_cast<int32_t>(read_unsigned(bits, offset, 5));
            offset += 5;
        }
    if (has_sisaioc)
        {
            page_data.sisaioc = static_cast<int32_t>(read_unsigned(bits, offset, 22));
            offset += 22;
        }
}

void parse_reduced_almanac(const uint8_t* bits, int32_t& offset, Bds3_B1c_AlmanacReduced& almanac)
{
    almanac.prn = static_cast<int32_t>(read_unsigned(bits, offset, 6));
    offset += 6;
    almanac.sat_type = static_cast<int32_t>(read_unsigned(bits, offset, 2));
    offset += 2;
    almanac.delta_a_m = static_cast<double>(read_signed(bits, offset, 8)) * twos_pow(9);
    offset += 8;
    almanac.omega0_rad = static_cast<double>(read_signed(bits, offset, 7)) * twos_pow(-6) * M_PI;
    offset += 7;
    almanac.phi0_rad = static_cast<double>(read_signed(bits, offset, 7)) * twos_pow(-6) * M_PI;
    offset += 7;
    almanac.health = static_cast<int32_t>(read_unsigned(bits, offset, 8));
    offset += 8;
}

void parse_medium_almanac(const uint8_t* bits, int32_t& offset, Bds3_B1c_AlmanacMedium& almanac)
{
    almanac.prn = static_cast<int32_t>(read_unsigned(bits, offset, 6));
    offset += 6;
    almanac.sat_type = static_cast<int32_t>(read_unsigned(bits, offset, 2));
    offset += 2;
    almanac.wna = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    almanac.toa_s = static_cast<int32_t>(read_unsigned(bits, offset, 8)) * 4096;
    offset += 8;
    almanac.eccentricity = static_cast<double>(read_unsigned(bits, offset, 11)) * twos_pow(-16);
    offset += 11;
    almanac.delta_i_rad = static_cast<double>(read_signed(bits, offset, 11)) * twos_pow(-14) * M_PI;
    offset += 11;
    almanac.sqrt_a_m_sqrt = static_cast<double>(read_unsigned(bits, offset, 17)) * twos_pow(-4);
    offset += 17;
    almanac.omega0_rad = static_cast<double>(read_signed(bits, offset, 16)) * twos_pow(-15) * M_PI;
    offset += 16;
    almanac.omega_dot_rad_s = static_cast<double>(read_signed(bits, offset, 11)) * twos_pow(-33) * M_PI;
    offset += 11;
    almanac.omega_rad = static_cast<double>(read_signed(bits, offset, 16)) * twos_pow(-15) * M_PI;
    offset += 16;
    almanac.m0_rad = static_cast<double>(read_signed(bits, offset, 16)) * twos_pow(-15) * M_PI;
    offset += 16;
    almanac.af0_s = static_cast<double>(read_signed(bits, offset, 11)) * twos_pow(-20);
    offset += 11;
    almanac.af1_s_s = static_cast<double>(read_signed(bits, offset, 10)) * twos_pow(-37);
    offset += 10;
    almanac.health = static_cast<int32_t>(read_unsigned(bits, offset, 8));
    offset += 8;
}

void parse_eop(const uint8_t* bits, int32_t& offset, Bds3_B1c_Eop& eop)
{
    eop.t_eop_s = static_cast<int32_t>(read_unsigned(bits, offset, 16)) * 16;
    offset += 16;
    eop.pm_x_arcsec = static_cast<double>(read_signed(bits, offset, 21)) * twos_pow(-20);
    offset += 21;
    eop.pm_x_dot_arcsec_day = static_cast<double>(read_signed(bits, offset, 15)) * twos_pow(-21);
    offset += 15;
    eop.pm_y_arcsec = static_cast<double>(read_signed(bits, offset, 21)) * twos_pow(-20);
    offset += 21;
    eop.pm_y_dot_arcsec_day = static_cast<double>(read_signed(bits, offset, 15)) * twos_pow(-21);
    offset += 15;
    eop.delta_ut1_s = static_cast<double>(read_signed(bits, offset, 31)) * twos_pow(-24);
    offset += 31;
    eop.delta_ut1_dot_s_day = static_cast<double>(read_signed(bits, offset, 19)) * twos_pow(-25);
    offset += 19;
}

void parse_bgto(const uint8_t* bits, int32_t& offset, Bds3_B1c_Bgto& bgto)
{
    bgto.gnss_id = static_cast<int32_t>(read_unsigned(bits, offset, 3));
    offset += 3;
    bgto.wn0_bgto = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    bgto.t0_bgto_s = static_cast<int32_t>(read_unsigned(bits, offset, 16)) * 16;
    offset += 16;
    bgto.a0_bgto_s = static_cast<double>(read_signed(bits, offset, 16)) * twos_pow(-35);
    offset += 16;
    bgto.a1_bgto_s_s = static_cast<double>(read_signed(bits, offset, 13)) * twos_pow(-51);
    offset += 13;
    bgto.a2_bgto_s_s2 = static_cast<double>(read_signed(bits, offset, 7)) * twos_pow(-68);
    offset += 7;
}

void parse_page1(
    const uint8_t* bits,
    Beidou_Cnav1_Iono& iono,
    Beidou_Cnav1_Utc_Model& utc,
    Bds3_B1c_PageData& page_data)
{
    int32_t offset = 0;
    parse_page_common(bits, true, true, page_data, offset);
    // ICD Table 7-10
    iono.alpha1 = static_cast<double>(read_unsigned(bits, offset, 10)) * 0.125;
    offset += 10;
    iono.alpha2 = static_cast<double>(read_signed(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha3 = static_cast<double>(read_unsigned(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha4 = static_cast<double>(read_unsigned(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha5 = static_cast<double>(read_unsigned(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha6 = static_cast<double>(read_signed(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha7 = static_cast<double>(read_signed(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha8 = static_cast<double>(read_signed(bits, offset, 8)) * 0.125;
    offset += 8;
    iono.alpha9 = static_cast<double>(read_signed(bits, offset, 8)) * 0.125;
    offset += 8;

    // ICD Table 7-20 / Figure 6-17
    utc.A0 = static_cast<double>(read_signed(bits, offset, 16)) * GNSS_SDR_TWO_N35;
    offset += 16;
    utc.A1 = static_cast<double>(read_signed(bits, offset, 13)) * GNSS_SDR_TWO_N51;
    offset += 13;
    utc.A2 = static_cast<double>(read_signed(bits, offset, 7)) * GNSS_SDR_TWO_N68;
    offset += 7;
    utc.delta_t_LS = static_cast<int32_t>(read_signed(bits, offset, 8));
    offset += 8;
    utc.tot = static_cast<int32_t>(read_unsigned(bits, offset, 16)) * 16;  // scale 2^4
    offset += 16;
    utc.WN_t = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    utc.WN_LSF = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    utc.DN = static_cast<int32_t>(read_unsigned(bits, offset, 3));
    offset += 3;
    utc.delta_t_LSF = static_cast<int32_t>(read_signed(bits, offset, 8));
}

void parse_page2(const uint8_t* bits, Bds3_B1c_PageData& page_data)
{
    int32_t offset = 0;
    parse_page_common(bits, false, true, page_data, offset);
    page_data.wna = static_cast<int32_t>(read_unsigned(bits, offset, 13));
    offset += 13;
    page_data.toa_s = static_cast<int32_t>(read_unsigned(bits, offset, 8)) * 4096;
    offset += 8;
    for (auto& alm : page_data.reduced_almanac)
        {
            parse_reduced_almanac(bits, offset, alm);
        }
}

void parse_page3(const uint8_t* bits, Bds3_B1c_PageData& page_data)
{
    int32_t offset = 0;
    parse_page_common(bits, true, false, page_data, offset);
    parse_eop(bits, offset, page_data.eop);
    parse_bgto(bits, offset, page_data.bgto);
    // Consume trailing reserved block and CRC for layout completeness.
    (void)read_unsigned(bits, offset, 14);
    offset += 14;
    (void)read_unsigned(bits, offset, 24);
}

void parse_page4(const uint8_t* bits, Bds3_B1c_PageData& page_data)
{
    int32_t offset = 0;
    parse_page_common(bits, false, true, page_data, offset);
    parse_medium_almanac(bits, offset, page_data.medium_almanac);
    // Consume trailing reserved block and CRC for layout completeness.
    (void)read_unsigned(bits, offset, 47);
    offset += 47;
    (void)read_unsigned(bits, offset, 24);
}
}  // namespace


bool Beidou_Cnav1_Navigation_Message::decode_frame(
    const float* symbols,
    int32_t num_symbols,
    int32_t expected_prn,
    int32_t* fail_stage)
{
    auto set_fail = [&](int32_t stage) {
        if (fail_stage != nullptr)
            {
                *fail_stage = stage;
            }
    };

    if (fail_stage != nullptr)
        {
            *fail_stage = -1;
        }
    if (num_symbols < BEIDOU_CNAV1_FRAME_SYMBOLS)
        {
            set_fail(0);
            return false;
        }

    std::array<float, BEIDOU_CNAV1_FRAME_SYMBOLS> bit_llr{};
    for (int32_t i = 0; i < BEIDOU_CNAV1_FRAME_SYMBOLS; i++)
        {
            bit_llr[i] = symbols[i];
        }

    std::array<int32_t, BEIDOU_CNAV1_SUBFRAME1_SYMBOLS> hard_bits{};
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME1_SYMBOLS; i++)
        {
            hard_bits[i] = (symbols[i] >= 0.0F) ? 1 : 0;
        }

    int32_t prn_bits[6];
    int32_t soh_bits[8];
    if (!decode_bch_21_6(hard_bits.data(), prn_bits))
        {
            set_fail(1);
            return false;
        }
    if (!decode_bch_51_8(hard_bits.data() + 21, soh_bits))
        {
            set_fail(2);
            return false;
        }

    const uint32_t prn = bits_to_unsigned(prn_bits, 6);
    if (prn < 1U || prn > static_cast<uint32_t>(BEIDOU_B1C_NUMBER_OF_PRNS))
        {
            set_fail(3);
            return false;
        }
    if (expected_prn > 0 && static_cast<uint32_t>(expected_prn) != prn)
        {
            set_fail(4);
            return false;
        }

    const uint32_t soh = bits_to_unsigned(soh_bits, 8);
    const auto soh_seconds = static_cast<double>(soh * BEIDOU_CNAV1_SOH_LSB_S);
    ephemeris_.PRN = static_cast<int32_t>(prn);

    std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> sf2_llr{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> sf3_llr{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> sf2_llr_inv{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> sf2_llr_bitrev{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> sf2_llr_bitrev_inv{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> sf3_llr_inv{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> sf3_llr_bitrev{};
    std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> sf3_llr_bitrev_inv{};
    deinterleave_sf2_sf3(bit_llr.data() + BEIDOU_CNAV1_SUBFRAME1_SYMBOLS, sf2_llr.data(), sf3_llr.data());
    // LDPC works on GF(64): 6 bit-LLRs per symbol. Build four polarity/order variants
    // (normal, inverted, per-symbol bit-reversed, both) and try each below.
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME2_SYMBOLS; i++)
        {
            sf2_llr_inv[static_cast<size_t>(i)] = -sf2_llr[static_cast<size_t>(i)];
        }
    // Reverse bit order within each 6-LLR GF(64) symbol; inv applies the same polarity flip.
    for (int32_t symbol = 0; symbol < BEIDOU_CNAV1_SUBFRAME2_SYMBOLS / 6; symbol++)
        {
            const auto base = static_cast<size_t>(symbol) * 6U;
            for (int32_t bit = 0; bit < 6; bit++)
                {
                    const auto bit_u = static_cast<size_t>(bit);
                    sf2_llr_bitrev[base + bit_u] = sf2_llr[base + (5U - bit_u)];
                    sf2_llr_bitrev_inv[base + bit_u] = -sf2_llr_bitrev[base + bit_u];
                }
        }
    // SF3: same four LLR variants as SF2.
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME3_SYMBOLS; i++)
        {
            sf3_llr_inv[static_cast<size_t>(i)] = -sf3_llr[static_cast<size_t>(i)];
        }
    for (int32_t symbol = 0; symbol < BEIDOU_CNAV1_SUBFRAME3_SYMBOLS / 6; symbol++)
        {
            const auto base = static_cast<size_t>(symbol) * 6U;
            for (int32_t bit = 0; bit < 6; bit++)
                {
                    const auto bit_u = static_cast<size_t>(bit);
                    sf3_llr_bitrev[base + bit_u] = sf3_llr[base + (5U - bit_u)];
                    sf3_llr_bitrev_inv[base + bit_u] = -sf3_llr_bitrev[base + bit_u];
                }
        }

    std::array<uint8_t, BEIDOU_CNAV1_SF2_DATA_BITS> sf2_data{};
    std::array<uint8_t, BEIDOU_CNAV1_SF3_DATA_BITS> sf3_data{};
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> sf2_codeword_bits{};
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> sf3_codeword_bits{};
    bool sf2_ldpc_ok = beidou_cnav1_ldpc_decode_200_100_codeword(
        sf2_llr.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, sf2_codeword_bits.data());
    if (!sf2_ldpc_ok)
        {
            sf2_ldpc_ok = beidou_cnav1_ldpc_decode_200_100_codeword(
                sf2_llr_inv.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, sf2_codeword_bits.data());
        }
    if (!sf2_ldpc_ok)
        {
            sf2_ldpc_ok = beidou_cnav1_ldpc_decode_200_100_codeword(
                sf2_llr_bitrev.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, sf2_codeword_bits.data());
        }
    if (!sf2_ldpc_ok)
        {
            sf2_ldpc_ok = beidou_cnav1_ldpc_decode_200_100_codeword(
                sf2_llr_bitrev_inv.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, sf2_codeword_bits.data());
        }
    if (!sf2_ldpc_ok)
        {
            // SF2 LDPC fallback: after normal/inverted/bit-reversed attempts all fail,
            // hard-slice the deinterleaved LLRs (sign of sf2_llr) as information bits.
            for (int32_t i = 0; i < BEIDOU_CNAV1_SF2_DATA_BITS; i++)
                {
                    sf2_data[static_cast<size_t>(i)] = (sf2_llr[static_cast<size_t>(i)] >= 0.0F) ? 1U : 0U;
                }
        }
    else
        {
            // Some matrix descriptions/orderings are not guaranteed to keep information symbols
            // in the first K symbols. Prefer the half that satisfies CRC if available.
            std::copy_n(sf2_codeword_bits.begin(), BEIDOU_CNAV1_SF2_DATA_BITS, sf2_data.begin());
            if (!verify_crc24q(sf2_data.data(), BEIDOU_CNAV1_SF2_DATA_BITS - BEIDOU_CNAV1_CRC_BITS))
                {
                    std::copy_n(
                        sf2_codeword_bits.begin() + BEIDOU_CNAV1_SF2_DATA_BITS,
                        BEIDOU_CNAV1_SF2_DATA_BITS,
                        sf2_data.begin());
                }
        }
    bool sf3_ldpc_ok = beidou_cnav1_ldpc_decode_88_44_codeword(
        sf3_llr.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, sf3_codeword_bits.data());
    if (!sf3_ldpc_ok)
        {
            sf3_ldpc_ok = beidou_cnav1_ldpc_decode_88_44_codeword(
                sf3_llr_inv.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, sf3_codeword_bits.data());
        }
    if (!sf3_ldpc_ok)
        {
            sf3_ldpc_ok = beidou_cnav1_ldpc_decode_88_44_codeword(
                sf3_llr_bitrev.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, sf3_codeword_bits.data());
        }
    if (!sf3_ldpc_ok)
        {
            sf3_ldpc_ok = beidou_cnav1_ldpc_decode_88_44_codeword(
                sf3_llr_bitrev_inv.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, sf3_codeword_bits.data());
        }
    if (!sf3_ldpc_ok)
        {
            for (int32_t i = 0; i < BEIDOU_CNAV1_SF3_DATA_BITS; i++)
                {
                    sf3_data[static_cast<size_t>(i)] = (sf3_llr[static_cast<size_t>(i)] >= 0.0F) ? 1U : 0U;
                }
        }
    else
        {
            std::copy_n(sf3_codeword_bits.begin(), BEIDOU_CNAV1_SF3_DATA_BITS, sf3_data.begin());
            if (!verify_crc24q(sf3_data.data(), BEIDOU_CNAV1_SF3_DATA_BITS - BEIDOU_CNAV1_CRC_BITS))
                {
                    std::copy_n(
                        sf3_codeword_bits.begin() + BEIDOU_CNAV1_SF3_DATA_BITS,
                        BEIDOU_CNAV1_SF3_DATA_BITS,
                        sf3_data.begin());
                }
        }

    const bool sf2_crc_ok = verify_crc24q(sf2_data.data(), BEIDOU_CNAV1_SF2_DATA_BITS - BEIDOU_CNAV1_CRC_BITS);
    const bool sf3_crc_ok = verify_crc24q(sf3_data.data(), BEIDOU_CNAV1_SF3_DATA_BITS - BEIDOU_CNAV1_CRC_BITS);

    if (!sf2_crc_ok)
        {
            set_fail(sf2_ldpc_ok ? 6 : 5);
            return false;
        }

    // Parse SF2 into a candidate so a mismatched IODE/IODC pair cannot overwrite a good set.
    Beidou_Cnav1_Ephemeris candidate{};
    parse_subframe2(sf2_data.data(), candidate, soh_seconds);
    candidate.PRN = static_cast<int32_t>(prn);
    // ICD B1C §7.4.3: usable only when IODE equals IODC low 8 bits.
    const auto iode = static_cast<uint32_t>(candidate.IODE);
    const auto iodc_lo = static_cast<uint32_t>(candidate.IODC) & 0xFFU;
    tow_s_ = static_cast<double>(candidate.tow);
    if (iode != iodc_lo)
        {
            // Frame CRC is OK; do not publish unmatched eph/clock (keep prior ephemeris_).
        }
    else
        {
            ephemeris_ = candidate;
            flag_new_ephemeris_ = true;
        }

    if (sf3_crc_ok)
        {
            const auto page_id = static_cast<int32_t>(read_unsigned(sf3_data.data(), 0, 6));
            switch (page_id)
                {
                case 1:
                    parse_page1(sf3_data.data(), iono_, utc_model_, page_data_);
                    flag_new_iono_ = true;
                    flag_new_utc_ = true;
                    flag_new_page_data_ = true;
                    break;
                case 2:
                    parse_page2(sf3_data.data(), page_data_);
                    flag_new_page_data_ = true;
                    break;
                case 3:
                    parse_page3(sf3_data.data(), page_data_);
                    flag_new_page_data_ = true;
                    break;
                case 4:
                    parse_page4(sf3_data.data(), page_data_);
                    flag_new_page_data_ = true;
                    break;
                default:
                    break;
                }
        }

    return true;
}


bool Beidou_Cnav1_Navigation_Message::decode_frame_symbols(
    const float* symbols,
    int32_t num_symbols,
    int32_t expected_prn,
    int32_t* fail_stage)
{
    return decode_frame(symbols, num_symbols, expected_prn, fail_stage);
}


bool Beidou_Cnav1_Navigation_Message::probe_subframe1_prn(const float* symbols, int32_t num_symbols, int32_t expected_prn) const
{
    if (num_symbols < BEIDOU_CNAV1_SUBFRAME1_SYMBOLS)
        {
            return false;
        }

    std::array<int32_t, BEIDOU_CNAV1_SUBFRAME1_SYMBOLS> hard_bits{};
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME1_SYMBOLS; i++)
        {
            hard_bits[i] = (symbols[i] >= 0.0F) ? 1 : 0;
        }

    int32_t prn_bits[6];
    int32_t soh_bits[8];
    if (!decode_bch_21_6(hard_bits.data(), prn_bits))
        {
            return false;
        }
    if (!decode_bch_51_8(hard_bits.data() + 21, soh_bits))
        {
            return false;
        }

    const uint32_t prn = bits_to_unsigned(prn_bits, 6);
    if (prn < 1U || prn > static_cast<uint32_t>(BEIDOU_B1C_NUMBER_OF_PRNS))
        {
            return false;
        }
    return expected_prn <= 0 || static_cast<uint32_t>(expected_prn) == prn;
}


bool Beidou_Cnav1_Navigation_Message::have_new_ephemeris() const
{
    return flag_new_ephemeris_;
}

bool Beidou_Cnav1_Navigation_Message::have_new_iono() const
{
    return flag_new_iono_;
}

bool Beidou_Cnav1_Navigation_Message::have_new_utc_model() const
{
    return flag_new_utc_;
}

bool Beidou_Cnav1_Navigation_Message::have_new_page_data() const
{
    return flag_new_page_data_;
}

void Beidou_Cnav1_Navigation_Message::clear_flags()
{
    flag_new_ephemeris_ = false;
    flag_new_iono_ = false;
    flag_new_utc_ = false;
    flag_new_page_data_ = false;
}

const Beidou_Cnav1_Ephemeris& Beidou_Cnav1_Navigation_Message::get_ephemeris() const
{
    return ephemeris_;
}

const Beidou_Cnav1_Iono& Beidou_Cnav1_Navigation_Message::get_iono() const
{
    return iono_;
}

const Beidou_Cnav1_Utc_Model& Beidou_Cnav1_Navigation_Message::get_utc_model() const
{
    return utc_model_;
}

const Bds3_B1c_PageData& Beidou_Cnav1_Navigation_Message::get_page_data() const
{
    return page_data_;
}

double Beidou_Cnav1_Navigation_Message::get_tow_s() const
{
    return tow_s_;
}

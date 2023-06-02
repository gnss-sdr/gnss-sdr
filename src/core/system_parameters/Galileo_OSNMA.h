/*!
 * \file Galileo_OSNMA.h
 * \brief Galileo OSNMA mesage constants
 * \author Carles Fernandez, 2023. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_OSNMA_H
#define GNSS_SDR_GALILEO_OSNMA_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 2
const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_2 = {
    {0, std::string("Reserved")},
    {1, std::string("Nominal")},
    {2, std::string("End of Chain (EOC)")},
    {3, std::string("Chain Revoked (CREV)")},
    {4, std::string("New Public Key (NPK)")},
    {5, std::string("Public Key Revoked (PKREV)")},
    {6, std::string("Reserved")},
    {7, std::string("Reserved")}};  // key: cpks, value: Chain and Public Key Status

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 3
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_3 = {
    {0, {0, 0}},
    {1, {0, 0}},
    {2, {0, 0}},
    {3, {0, 0}},
    {4, {0, 0}},
    {5, {0, 0}},
    {6, {0, 0}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {15, 1560}},
    {10, {16, 1664}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dp, value: {num_blocks, l_dp_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_5 = {
    {0, std::string("Reserved")},
    {1, std::string("ECDSA P-256")},
    {2, std::string("Reserved")},
    {3, std::string("ECDSA P-521")},
    {4, std::string("OAM")},
    {5, std::string("Reserved")},
    {6, std::string("Reserved")},
    {7, std::string("Reserved")},
    {8, std::string("Reserved")},
    {9, std::string("Reserved")},
    {10, std::string("Reserved")},
    {11, std::string("Reserved")},
    {12, std::string("Reserved")},
    {13, std::string("Reserved")},
    {14, std::string("Reserved")},
    {15, std::string("Reserved")}};  // key: nptk, value: message

const std::unordered_map<std::string, uint16_t> OSNMA_TABLE_6 = {
    {std::string("ECDSA P-256"), 264},
    {std::string("ECDSA P-521"), 536}};

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 7
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_7 = {
    {0, {0, 0}},
    {1, {7, 728}},
    {2, {8, 832}},
    {3, {9, 936}},
    {4, {10, 1040}},
    {5, {11, 1144}},
    {6, {12, 1248}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {0, 0}},
    {10, {0, 0}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dk, value: {num_blocks, l_dk_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_8 = {
    {0, std::string("SHA-256")},
    {1, std::string("Reserved")},
    {2, std::string("SHA3-256")},
    {3, std::string("Reserved")}};  // key: hs, value: hash_function

const std::unordered_map<uint8_t, uint16_t> OSNMA_TABLE_10 = {
    {0, 96},
    {1, 104},
    {2, 112},
    {3, 120},
    {4, 128},
    {5, 160},
    {6, 192},
    {7, 224},
    {8, 256},
    {9, 0},
    {10, 0},
    {11, 0},
    {12, 0},
    {13, 0},
    {15, 0},
    {15, 0}};  // key: ks, value: lk_bits

const std::unordered_map<uint8_t, uint8_t> OSNMA_TABLE_11 = {
    {0, 0},
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0},
    {5, 20},
    {6, 24},
    {7, 26},
    {8, 32},
    {9, 40},
    {10, 0},
    {11, 0},
    {12, 0},
    {13, 0},
    {14, 0},
    {15, 0},
};

const std::unordered_map<std::string, uint16_t> OSNMA_TABLE_15 = {
    {std::string("ECDSA P-256"), 512},
    {std::string("ECDSA P-521"), 1056},
    {std::string("SHA-256"), 512},
    {std::string("SHA-512"), 1056}};  // key: ECDSA Curve and hash function, value: {l_ds_bits}

#if __cplusplus == 201103L
constexpr std::uint8_t mask_nmas{0xC0};
constexpr std::uint8_t mask_cid{0x30};
constexpr std::uint8_t mask_cpks{0x07};
constexpr std::uint8_t mask_nma_header_reserved{0x01};
constexpr std::uint8_t mask_dsm_id{0xF0};
constexpr std::uint8_t mask_dsm_block_id{0x0F};
constexpr std::uint8_t mask_dsm_number_blocks{0xF0};
constexpr std::uint8_t mask_dsm_pkid{0x0F};
constexpr std::uint8_t mask_dsm_cidkr{0xC0};
constexpr std::uint8_t mask_dsm_reserved1{0x30};
constexpr std::uint8_t mask_dsm_hf{0x0C};
constexpr std::uint8_t mask_dsm_mf{0x03};
constexpr std::uint8_t mask_dsm_ks{0xF0};
constexpr std::uint8_t mask_dsm_ts{0x0F};
constexpr std::uint8_t mask_dsm_reserved{0xF0};
constexpr std::uint8_t mask_dsm_wk_k_msbyte{0x0F};
constexpr std::uint8_t mask_dsm_mid{0x0F};
constexpr std::uint8_t mask_dsm_npkt{0xF0};
constexpr std::uint8_t mask_dsm_npktid{0x0F};
#else
constexpr std::uint8_t mask_nmas{0b1100'0000};
constexpr std::uint8_t mask_cid{0b0011'0000};
constexpr std::uint8_t mask_cpks{0b0000'1110};
constexpr std::uint8_t mask_nma_header_reserved{0b0000'0001};
constexpr std::uint8_t mask_dsm_id{0b1111'0000};
constexpr std::uint8_t mask_dsm_block_id{0b0000'1111};
constexpr std::uint8_t mask_dsm_number_blocks{0b1111'0000};
constexpr std::uint8_t mask_dsm_pkid{0b0000'1111};
constexpr std::uint8_t mask_dsm_cidkr{0b1100'0000};
constexpr std::uint8_t mask_dsm_reserved1{0b0011'0000};
constexpr std::uint8_t mask_dsm_hf{0b0000'1100};
constexpr std::uint8_t mask_dsm_mf{0b0000'0011};
constexpr std::uint8_t mask_dsm_ks{0b1111'0000};
constexpr std::uint8_t mask_dsm_ts{0b0000'1111};
constexpr std::uint8_t mask_dsm_reserved{0b1111'0000};
constexpr std::uint8_t mask_dsm_wk_k_msbyte{0b0000'1111};
constexpr std::uint8_t mask_dsm_mid{0b0000'1111};
constexpr std::uint8_t mask_dsm_npkt{0b1111'0000};
constexpr std::uint8_t mask_dsm_npktid{0b0000'1111};
#endif

// // hf = (dsm_msg[1] & 0b00001100) >> 2;
uint8_t get_nmas(uint8_t nma_header)
{
    return (nma_header & mask_nmas) >> 6;
}

uint8_t get_cid(uint8_t nma_header)
{
    return (nma_header & mask_cid) >> 4;
}

uint8_t get_cpks(uint8_t nma_header)
{
    return (nma_header & mask_cpks) >> 1;
}

bool get_nma_header_reserved(uint8_t nma_header)
{
    return ((nma_header & mask_nma_header_reserved) ? true : false);
}

uint8_t get_dsm_id(uint8_t dsm_header)
{
    return (dsm_header & mask_dsm_id) >> 4;
}

uint8_t get_dsm_block_id(uint8_t dsm_header)
{
    return dsm_header & mask_dsm_block_id;
}

uint8_t get_number_blocks_index(uint8_t dsm_msg_0)
{
    return (dsm_msg_0 & mask_dsm_number_blocks) >> 4;
}

uint8_t get_pkid(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[0] & mask_dsm_pkid);
}

uint8_t get_cidkr(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[1] & mask_dsm_cidkr) >> 6;
}

uint8_t get_dsm_reserved1(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[1] & mask_dsm_reserved1) >> 4;
}

uint8_t get_hf(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[1] & mask_dsm_hf) >> 2;
}

uint8_t get_mf(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[1] & mask_dsm_mf);
}

uint8_t get_ks(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[2] & mask_dsm_ks) >> 4;
}

uint8_t get_ts(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[2] & mask_dsm_ts);
}

uint8_t get_maclt(const std::vector<uint8_t>& dsm_msg)
{
    return dsm_msg[3];
}

uint8_t get_dsm_reserved(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[4] & mask_dsm_reserved) >> 4;
}

uint16_t get_wn_k(const std::vector<uint8_t>& dsm_msg)
{
    return (static_cast<uint16_t>((dsm_msg[4] & mask_dsm_wk_k_msbyte) << 8) + static_cast<uint16_t>(dsm_msg[5]));
}

uint8_t get_towh_k(const std::vector<uint8_t>& dsm_msg)
{
    return dsm_msg[6];
}

uint64_t get_alpha(const std::vector<uint8_t>& dsm_msg)
{
    uint64_t alpha = (static_cast<uint64_t>(dsm_msg[8]) << 32) +
                     (static_cast<uint64_t>(dsm_msg[9]) << 24) +
                     (static_cast<uint64_t>(dsm_msg[10]) << 16) +
                     (static_cast<uint64_t>(dsm_msg[11]) << 8) +
                     static_cast<uint64_t>(dsm_msg[12]);
    return alpha;
}

uint16_t get_l_dk_bits(uint8_t nb_dk)
{
    const auto it = OSNMA_TABLE_7.find(nb_dk);
    if (it != OSNMA_TABLE_7.cend())
        {
            return it->second.second;
        }
    return 0;
}

uint16_t get_lk_bits(uint8_t ks)
{
    const auto it = OSNMA_TABLE_10.find(ks);
    if (it != OSNMA_TABLE_10.cend())
        {
            return it->second;
        }
    return 0;
}

std::vector<uint8_t> get_kroot(const std::vector<uint8_t>& dsm_msg, uint16_t bytes_lk)
{
    std::vector<uint8_t> kroot = std::vector<uint8_t>(bytes_lk, 0);
    if (dsm_msg.size() > static_cast<uint64_t>(13 + bytes_lk))
        {
            for (uint16_t k = 0; k < bytes_lk; k++)
                {
                    kroot[k] = dsm_msg[13 + k];
                }
        }
    return kroot;
}

std::string get_hash_function(uint8_t hf)
{
    std::string hash_;
    const auto it = OSNMA_TABLE_8.find(hf);
    if (it != OSNMA_TABLE_8.cend())
        {
            hash_ = it->second;
        }
    return hash_;
}

uint8_t get_mid(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[0] & mask_dsm_mid);
}

uint8_t get_npkt(const std::vector<uint8_t>& dsm_msg)
{
    return ((dsm_msg[129] & mask_dsm_npkt) >> 4);
}

uint8_t get_npktid(const std::vector<uint8_t>& dsm_msg)
{
    return (dsm_msg[129] & mask_dsm_npktid);
}


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_OSNMA_H
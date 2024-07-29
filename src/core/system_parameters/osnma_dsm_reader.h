/*!
 * \file osnma_dsm_reader.h
 * \brief Class for reading OSNMA DSM messages
 * \author Carles Fernandez-Prades, 2023 cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_OSNMA_DSM_READER_H
#define GNSS_SDR_OSNMA_DSM_READER_H

#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class OSNMA_DSM_Reader
{
public:
    OSNMA_DSM_Reader() = default;
    uint8_t get_nmas(uint8_t nma_header) const;
    uint8_t get_cid(uint8_t nma_header) const;
    uint8_t get_cpks(uint8_t nma_header) const;
    bool get_nma_header_reserved(uint8_t nma_header) const;

    uint8_t get_dsm_id(uint8_t dsm_header) const;
    uint8_t get_dsm_block_id(uint8_t dsm_header) const;

    uint8_t get_number_blocks_index(uint8_t dsm_msg_0) const;
    uint8_t get_pkid(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_cidkr(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_dsm_reserved1(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_hf(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_mf(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_ks(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_ts(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_maclt(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_dsm_reserved(const std::vector<uint8_t>& dsm_msg) const;
    uint16_t get_wn_k(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_towh_k(const std::vector<uint8_t>& dsm_msg) const;
    uint64_t get_alpha(const std::vector<uint8_t>& dsm_msg) const;
    uint16_t get_l_dk_bits(uint8_t nb_dk) const;
    uint16_t get_lk_bits(uint8_t ks) const;
    std::vector<uint8_t> get_kroot(const std::vector<uint8_t>& dsm_msg, uint16_t bytes_lk) const;
    std::string get_hash_function(uint8_t hf) const;
    std::string get_nmas_status(uint8_t nmas) const;
    std::string get_cpks_status(uint8_t cpks) const;

    uint8_t get_mid(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_npkt(const std::vector<uint8_t>& dsm_msg) const;
    uint8_t get_npktid(const std::vector<uint8_t>& dsm_msg) const;

private:
    static constexpr std::uint8_t mask_nmas{0xC0};
    static constexpr std::uint8_t mask_cid{0x30};
    static constexpr std::uint8_t mask_cpks{0x0E};
    static constexpr std::uint8_t mask_nma_header_reserved{0x01};
    static constexpr std::uint8_t mask_dsm_id{0xF0};
    static constexpr std::uint8_t mask_dsm_block_id{0x0F};
    static constexpr std::uint8_t mask_dsm_number_blocks{0xF0};
    static constexpr std::uint8_t mask_dsm_pkid{0x0F};
    static constexpr std::uint8_t mask_dsm_cidkr{0xC0};
    static constexpr std::uint8_t mask_dsm_reserved1{0x30};
    static constexpr std::uint8_t mask_dsm_hf{0x0C};
    static constexpr std::uint8_t mask_dsm_mf{0x03};
    static constexpr std::uint8_t mask_dsm_ks{0xF0};
    static constexpr std::uint8_t mask_dsm_ts{0x0F};
    static constexpr std::uint8_t mask_dsm_reserved{0xF0};
    static constexpr std::uint8_t mask_dsm_wk_k_msbyte{0x0F};
    static constexpr std::uint8_t mask_dsm_mid{0x0F};
    static constexpr std::uint8_t mask_dsm_npkt{0xF0};
    static constexpr std::uint8_t mask_dsm_npktid{0x0F};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_DSM_READER_H
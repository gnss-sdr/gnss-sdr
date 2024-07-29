/*!
 * \file osnma_dsm_reader.cc
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

#include "osnma_dsm_reader.h"
#include "Galileo_OSNMA.h"


uint8_t OSNMA_DSM_Reader::get_nmas(uint8_t nma_header) const
{
    return (nma_header & mask_nmas) >> 6;
}


uint8_t OSNMA_DSM_Reader::get_cid(uint8_t nma_header) const
{
    return (nma_header & mask_cid) >> 4;
}


uint8_t OSNMA_DSM_Reader::get_cpks(uint8_t nma_header) const
{
    return (nma_header & mask_cpks) >> 1;
}


bool OSNMA_DSM_Reader::get_nma_header_reserved(uint8_t nma_header) const
{
    return (nma_header & mask_nma_header_reserved) != 0;
}


uint8_t OSNMA_DSM_Reader::get_dsm_id(uint8_t dsm_header) const
{
    return (dsm_header & mask_dsm_id) >> 4;
}


uint8_t OSNMA_DSM_Reader::get_dsm_block_id(uint8_t dsm_header) const
{
    return dsm_header & mask_dsm_block_id;
}


uint8_t OSNMA_DSM_Reader::get_number_blocks_index(uint8_t dsm_msg_0) const
{
    return (dsm_msg_0 & mask_dsm_number_blocks) >> 4;
}


uint8_t OSNMA_DSM_Reader::get_pkid(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[0] & mask_dsm_pkid);
}


uint8_t OSNMA_DSM_Reader::get_cidkr(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[1] & mask_dsm_cidkr) >> 6;
}


uint8_t OSNMA_DSM_Reader::get_dsm_reserved1(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[1] & mask_dsm_reserved1) >> 4;
}


uint8_t OSNMA_DSM_Reader::get_hf(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[1] & mask_dsm_hf) >> 2;
}


uint8_t OSNMA_DSM_Reader::get_mf(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[1] & mask_dsm_mf);
}


uint8_t OSNMA_DSM_Reader::get_ks(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[2] & mask_dsm_ks) >> 4;
}


uint8_t OSNMA_DSM_Reader::get_ts(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[2] & mask_dsm_ts);
}


uint8_t OSNMA_DSM_Reader::get_maclt(const std::vector<uint8_t>& dsm_msg) const
{
    return dsm_msg[3];
}


uint8_t OSNMA_DSM_Reader::get_dsm_reserved(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[4] & mask_dsm_reserved) >> 4;
}


uint16_t OSNMA_DSM_Reader::get_wn_k(const std::vector<uint8_t>& dsm_msg) const
{
    return (static_cast<uint16_t>((dsm_msg[4] & mask_dsm_wk_k_msbyte) << 8) + static_cast<uint16_t>(dsm_msg[5]));
}


uint8_t OSNMA_DSM_Reader::get_towh_k(const std::vector<uint8_t>& dsm_msg) const
{
    return dsm_msg[6];
}


uint64_t OSNMA_DSM_Reader::get_alpha(const std::vector<uint8_t>& dsm_msg) const
{
    uint64_t alpha = (static_cast<uint64_t>(dsm_msg[7]) << 40) +
                     (static_cast<uint64_t>(dsm_msg[8]) << 32) +
                     (static_cast<uint64_t>(dsm_msg[9]) << 24) +
                     (static_cast<uint64_t>(dsm_msg[10]) << 16) +
                     (static_cast<uint64_t>(dsm_msg[11]) << 8) +
                     (static_cast<uint64_t>(dsm_msg[12]));
    return alpha;
}


uint16_t OSNMA_DSM_Reader::get_l_dk_bits(uint8_t nb_dk) const
{
    const auto it = OSNMA_TABLE_7.find(nb_dk);
    if (it != OSNMA_TABLE_7.cend())
        {
            return it->second.second;
        }
    return 0;
}


uint16_t OSNMA_DSM_Reader::get_lk_bits(uint8_t ks) const
{
    const auto it = OSNMA_TABLE_10.find(ks);
    if (it != OSNMA_TABLE_10.cend())
        {
            return it->second;
        }
    return 0;
}


std::vector<uint8_t> OSNMA_DSM_Reader::get_kroot(const std::vector<uint8_t>& dsm_msg, uint16_t bytes_lk) const
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


std::string OSNMA_DSM_Reader::get_hash_function(uint8_t hf) const
{
    std::string hash_;
    const auto it = OSNMA_TABLE_8.find(hf);
    if (it != OSNMA_TABLE_8.cend())
        {
            hash_ = it->second;
        }
    return hash_;
}


uint8_t OSNMA_DSM_Reader::get_mid(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[0] & mask_dsm_mid);
}


uint8_t OSNMA_DSM_Reader::get_npkt(const std::vector<uint8_t>& dsm_msg) const
{
    return ((dsm_msg[129] & mask_dsm_npkt) >> 4);
}


uint8_t OSNMA_DSM_Reader::get_npktid(const std::vector<uint8_t>& dsm_msg) const
{
    return (dsm_msg[129] & mask_dsm_npktid);
}


std::string OSNMA_DSM_Reader::get_nmas_status(uint8_t nmas) const
{
    std::string status_;
    const auto it = OSNMA_TABLE_1.find(nmas);
    if (it != OSNMA_TABLE_1.cend())
        {
            status_ = it->second;
        }
    return status_;
}


std::string OSNMA_DSM_Reader::get_cpks_status(uint8_t cpks) const
{
    std::string status_;
    const auto it = OSNMA_TABLE_2.find(cpks);
    if (it != OSNMA_TABLE_2.cend())
        {
            status_ = it->second;
        }
    return status_;
}
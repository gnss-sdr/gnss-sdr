/*!
 * \file osnma_data.h
 * \brief Class for Galileo OSNMA data storage
 * \author Carles Fernandez-Prades, 2020-2023 cfernandez(at)cttc.es
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


#ifndef GNSS_SDR_OSNMA_DATA_H
#define GNSS_SDR_OSNMA_DATA_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class DSM_nma_header
{
public:
    DSM_nma_header() = default;
    uint8_t nmas{};
    uint8_t cid{};
    uint8_t cpks{};
    bool reserved{};
};

class DSM_dsm_header
{
public:
    DSM_dsm_header() = default;
    uint8_t dsm_id{};
    uint8_t dsm_block_id{};
};

class MACK_header
{
public:
    MACK_header() = default;
    uint64_t tag0{};
    uint16_t macseq{};
    uint8_t cop{};
};

class MACK_tag_info
{
public:
    MACK_tag_info() = default;
    uint8_t PRN_d{};
    uint8_t ADKD{};
    uint8_t cop{};
};

class MACK_tag_and_info
{
public:
    MACK_tag_and_info() = default;
    uint64_t tag;
    MACK_tag_info tag_info;
};

class DSM_PKR_message
{
public:
    DSM_PKR_message() = default;

    std::array<uint8_t, 128> itn;  // bitset<1024>
    std::vector<uint8_t> npk;
    std::vector<uint8_t> p_dp;
    uint8_t nb_dp{};
    uint8_t mid{};
    uint8_t npkt{};
    uint8_t npktid{};
};

class DSM_KROOT_message
{
public:
    DSM_KROOT_message() = default;

    std::vector<uint8_t> kroot;
    std::vector<uint8_t> ds;
    std::vector<uint8_t> p_dk;
    uint64_t alpha{};
    uint16_t wn_k{};
    uint8_t nb_dk{};
    uint8_t pkid{};
    uint8_t cidkr{};
    uint8_t reserved1{};
    uint8_t hf{};
    uint8_t mf{};
    uint8_t ks{};
    uint8_t ts{};
    uint8_t maclt{};
    uint8_t reserved{};
    uint8_t towh_k{};
};

class MACK_message
{
public:
    MACK_message() = default;
    MACK_header header;
    std::vector<MACK_tag_and_info> tag_and_info;
    std::vector<uint8_t> key;
};

/*!
 * \brief This class handles ONSMA data
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OSNMA_User_ICD_for_Test_Phase_v1.0.pdf
 */
class OSNMA_data
{
public:
    OSNMA_data() = default;
    DSM_nma_header d_nma_header;
    DSM_dsm_header d_dsm_header;
    DSM_PKR_message d_dsm_pkr_message;
    DSM_KROOT_message d_dsm_kroot_message;
    MACK_message d_mack_message;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_DATA_H

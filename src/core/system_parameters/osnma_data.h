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

struct nma_header
{
    uint8_t nmas;
    uint8_t cid;
    uint8_t cpks;
    bool reserved;
};

struct dsm_header
{
    uint8_t dsm_id;
    uint8_t dsm_block_id;
};

struct mack_header
{
    std::vector<uint8_t> tag0;
    uint16_t macsec;
    uint8_t cop;
};

struct tag
{
    std::vector<uint8_t> tag;
    uint16_t tag_info;
};

struct tag_and_info
{
    std::vector<tag> tags;
};

struct DSM_PKR_message
{
    uint8_t nb_dp;
    uint8_t mid;
    std::array<uint8_t, 128> itn;  // bitset<1024>
    uint8_t npkt;
    uint8_t npktid;
    std::vector<uint8_t> npk;
    std::vector<uint8_t> p_dp;
};

struct DSM_KROOT_message
{
    uint8_t nb_dk;
    uint8_t pkid;
    uint8_t cidkr;
    uint8_t reserved1;
    uint8_t hf;
    uint8_t mf;
    uint8_t ks;
    uint8_t ts;
    uint8_t maclt;
    uint8_t reserved;
    uint16_t wn_k;
    uint8_t towh_k;
    uint64_t alpha;
    std::vector<uint8_t> kroot;
    std::vector<uint8_t> ds;
    std::vector<uint8_t> p_dk;
};

struct MACK_message
{
    mack_header header;
    tag_and_info tag_info;
    std::vector<uint8_t> key;
    std::vector<uint8_t> padding;
};

/*!
 * \brief This class handles ONSMA data
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OSNMA_User_ICD_for_Test_Phase_v1.0.pdf
 */
class OSNMA_data
{
public:
    OSNMA_data() = default;
    nma_header d_nma_header;
    dsm_header d_dsm_header;
    DSM_PKR_message d_dsm_pkr_message;
    DSM_KROOT_message d_dsm_kroot_message;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_DATA_H

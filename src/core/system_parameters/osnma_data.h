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

#include "galileo_inav_message.h"  // for OSNMA_msg
#include <array>
#include <cstdint>
#include <memory>
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
    uint64_t tag;  // C: 20-40 bits
    MACK_tag_info tag_info;
    uint32_t counter;  // CTR
};


class DSM_PKR_message
{
public:
    DSM_PKR_message() = default;

    std::array<uint8_t, 128> itn{};  // bitset<1024>
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
    uint8_t ks{};  // key size, in bits
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
    uint32_t TOW;  // TODO duplicated variable, also in OSNMA_NavData
    uint32_t WN;
    uint32_t PRNa;
};


class OSNMA_NavData
{
public:
    OSNMA_NavData() : nav_data_id(id_counter++) {}
    const uint32_t nav_data_id;

    std::string get_utc_data() const;
    std::string get_ephemeris_data() const;
    uint32_t get_last_received_TOW() const { return d_last_received_TOW; }
    uint32_t get_tow_sf0() const { return d_TOW_sf0; }

    bool add_nav_data(const std::string& nav_data);
    void set_tow_sf0(int value) { d_TOW_sf0 = value; }
    void set_ephemeris_data(std::string value) { d_ephemeris_iono = value; }
    void set_utc_data(std::string value) { d_utc = value; }
    void update_last_received_timestamp(uint32_t TOW);

    uint32_t verified_bits{0};
    uint32_t IOD_nav{0};
    uint32_t PRNd{0};
    uint32_t ADKD{};
    bool verified{false};

private:
    std::string d_ephemeris_iono{""};
    std::string d_utc{""};
    uint32_t d_TOW_sf0{0};
    uint32_t d_last_received_TOW{0};
    static uint32_t id_counter;
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
    OSNMA_NavData d_nav_data;
};


class Tag
{
public:
    enum e_verification_status
    {
        SUCCESS,
        FAIL,
        UNVERIFIED
    };
    Tag(const MACK_tag_and_info& MTI, uint32_t TOW, uint32_t WN, uint32_t PRNa, uint8_t CTR)  // standard tag constructor, for tags within Tag&Info field
        : tag_id(id_counter++),
          TOW(TOW),  // TODO missing for build_message WN for GST computation, CTR, NMAS, OSNMA_NavData missing
          WN(WN),
          PRNa(PRNa),
          CTR(CTR),
          status(UNVERIFIED),
          received_tag(MTI.tag),
          computed_tag(0),
          PRN_d(MTI.tag_info.PRN_d),
          ADKD(MTI.tag_info.ADKD),
          cop(MTI.tag_info.cop),
          skipped(0)
    {
    }
    explicit Tag(const MACK_message& mack)  // constructor for Tag0
        : tag_id(id_counter++),
          TOW(mack.TOW),  // TODO missing for build_message WN for GST computation, CTR, NMAS, OSNMA_NavData missing
          WN(mack.WN),
          PRNa(mack.PRNa),
          CTR(1),
          status(UNVERIFIED),
          received_tag(mack.header.tag0),
          computed_tag(0),
          PRN_d(mack.PRNa),  // Tag0 are self-authenticating
          ADKD(0),
          cop(mack.header.cop),
          skipped(0)
    {
    }
    const uint32_t tag_id;
    uint32_t static id_counter;
    uint32_t TOW;
    uint32_t WN;
    uint32_t PRNa;
    uint8_t CTR;
    e_verification_status status;
    uint64_t received_tag;
    uint64_t computed_tag;
    uint8_t PRN_d;
    uint8_t ADKD;
    uint8_t cop;
    uint32_t skipped;
    std::string nav_data;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_OSNMA_DATA_H

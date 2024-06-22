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

#include "galileo_ephemeris.h"
#include "galileo_inav_message.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
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
    uint64_t tag; // C: 20-40 bits
    MACK_tag_info tag_info;
    // TODO - std::vector<uint8_t> with complete Tag
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
    uint8_t ks{}; // key size, in bits
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
    uint32_t TOW; // TODO duplicated variable, also in NavData
    uint32_t WN;
    uint32_t PRNa;
};

class NavData
{
public:
    NavData()=default;
    void init(const std::shared_ptr<OSNMA_msg> &osnma_msg);
    std::vector<uint8_t> ephemeris_iono_vector{};
    std::string ephemeris_iono_vector_2{};
    std::vector<uint8_t> utc_vector{};
    std::string utc_vector_2{};
    uint32_t PRNa{};
    uint32_t WN_sf0{};
    uint32_t TOW_sf0{};
private:
    Galileo_Ephemeris EphemerisData;
    Galileo_Iono IonoData;
    Galileo_Utc_Model UtcData;
    void generate_eph_iono_vector(); // TODO pass data directly fro Telemetry Decoder (if bits are in the needed order)
    void generate_utc_vector(); // TODO


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
    NavData d_nav_data;
};

class Tag
{
public:
    enum e_verification_status{
        SUCCESS,
        FAIL,
        UNVERIFIED};
    Tag(const MACK_tag_and_info& MTI, uint32_t TOW,uint32_t WN, uint32_t PRNa,uint8_t CTR)
        : tag_id(id_counter++),
          TOW(TOW), // TODO missing for build_message WN for GST computation, CTR, NMAS, NavData missing
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

    const uint32_t tag_id;
    uint32_t TOW;
    uint32_t WN;
    uint32_t PRNa;
    uint8_t CTR;
    e_verification_status status;
    uint64_t received_tag;

    uint32_t static id_counter;
    uint64_t computed_tag;

    uint8_t PRN_d;
    uint8_t ADKD;
    uint8_t cop;
    uint32_t skipped;
};
/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_DATA_H

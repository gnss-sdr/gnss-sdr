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
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

namespace osnma
{
constexpr uint64_t GALILEO_SECONDS_PER_WEEK = 604800;
constexpr uint64_t OSNMA_AUTH_IOD_MAX_AGE_SECONDS = 4 * 60 * 60;
constexpr uint64_t OSNMA_AUTH_IOD_FUTURE_TOLERANCE_SECONDS = 10 * 60;

uint64_t galileo_gst_seconds(uint32_t WN, uint32_t TOW);
std::pair<uint32_t, uint32_t> galileo_week_tow_with_offset(uint32_t WN, uint32_t TOW, int32_t offset_seconds);
uint32_t galileo_week_to_uint(int32_t WN);
uint32_t galileo_tow_to_uint(int32_t TOW);
bool auth_gst_matches_nav_data(uint64_t auth_gst, uint64_t nav_data_gst);
bool auth_gst_is_stale(uint64_t auth_gst, uint64_t nav_data_gst);
}  // namespace osnma

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
    bool tag0_valid{false};
    bool macseq_valid{false};
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
    uint64_t tag{};  // C: 20-40 bits
    MACK_tag_info tag_info;
    uint32_t counter{};  // CTR
    bool valid{true};
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
    bool verified{false};
};


class MACK_message
{
public:
    MACK_message() = default;
    MACK_header header;
    std::vector<MACK_tag_and_info> tag_and_info;
    std::vector<uint8_t> key;
    std::vector<uint8_t> allowed_adkds{0, 4, 12};
    uint32_t TOW{};  // TODO duplicated variable, also in OSNMA_NavData
    uint32_t WN{};
    uint32_t PRNa{};
    uint8_t nmas{};
};


class OSNMA_NavData
{
public:
    OSNMA_NavData() : nav_data_id(id_counter++) {}
    const uint32_t nav_data_id;
    std::string get_utc_data() const;
    std::string get_ephemeris_data() const;
    uint32_t get_verified_bits() const { return verified_bits; }
    uint32_t get_prn_d() const { return PRNd; }
    uint32_t get_IOD_nav() const { return IOD_nav; }
    uint32_t get_wn_sf0() const { return d_WN_sf0; }
    uint32_t get_last_received_WN() const { return d_last_received_WN; }
    uint32_t get_last_received_TOW() const { return d_last_received_TOW; }
    uint32_t get_tow_sf0() const { return d_TOW_sf0; }
    uint64_t get_first_accumulated_tag_gst() const { return d_first_accumulated_tag_gst; }
    uint64_t get_last_accumulated_tag_gst() const { return d_last_accumulated_tag_gst; }
    uint8_t get_accumulated_tag_adkd() const { return d_accumulated_tag_adkd; }
    bool have_this_bits(std::string nav_data);
    bool get_verified_status() const { return verified; }
    bool has_tag_accumulation() const { return d_has_tag_accumulation; }
    bool add_nav_data(const std::string& nav_data);
    void set_tow_sf0(int value) { d_TOW_sf0 = value; }
    void set_wn_sf0(uint32_t WN) { d_WN_sf0 = WN; }
    void set_ephemeris_data(std::string value) { d_ephemeris_iono = value; }
    void set_utc_data(std::string value) { d_utc = value; }
    void update_last_received_timestamp(uint32_t TOW);
    void set_prn_d(uint32_t value) { PRNd = value; }
    void set_last_received_WN(uint32_t WN) { d_last_received_WN = WN; }
    void set_last_received_TOW(uint32_t TOW) { d_last_received_TOW = TOW; };
    void set_update_verified_bits(uint32_t morebits) { verified_bits += morebits; }
    void set_verified_status(bool value) { verified = value; }
    void set_IOD_nav(uint32_t value) { IOD_nav = value; }
    void start_tag_accumulation(uint32_t bits, uint8_t adkd, uint64_t tag_gst)
    {
        verified_bits = bits;
        d_accumulated_tag_adkd = adkd;
        d_first_accumulated_tag_gst = tag_gst;
        d_last_accumulated_tag_gst = tag_gst;
        d_has_tag_accumulation = true;
    }
    void continue_tag_accumulation(uint32_t morebits, uint64_t tag_gst)
    {
        verified_bits += morebits;
        d_last_accumulated_tag_gst = tag_gst;
    }
    void clear_tag_accumulation_metadata()
    {
        d_accumulated_tag_adkd = 0;
        d_first_accumulated_tag_gst = 0;
        d_last_accumulated_tag_gst = 0;
        d_has_tag_accumulation = false;
    }
    void reset_tag_accumulation()
    {
        verified_bits = 0;
        clear_tag_accumulation_metadata();
    }

private:
    static uint32_t id_counter;
    std::string d_ephemeris_iono{""};
    std::string d_utc{""};
    uint32_t d_WN_sf0{0};
    uint32_t d_last_received_WN{0};
    uint32_t d_TOW_sf0{0};
    uint32_t d_last_received_TOW{0};
    uint64_t d_first_accumulated_tag_gst{0};
    uint64_t d_last_accumulated_tag_gst{0};
    uint32_t PRNd{0};
    uint32_t verified_bits{0};
    uint32_t IOD_nav{0};
    uint8_t d_accumulated_tag_adkd{0};
    bool d_has_tag_accumulation{false};
    bool verified{false};
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
    DSM_KROOT_message d_dsm_kroot_new_message;
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
        UNVERIFIED,
        AUTHENTICATED_DONT_USE
    };
    Tag(const MACK_tag_and_info& MTI,
        uint32_t TOW,
        uint32_t WN,
        uint32_t PRNa,
        uint8_t CTR,
        uint8_t nmas,
        std::vector<uint8_t> allowed_adkds = {0, 4, 12})  // standard tag constructor, for tags within Tag&Info field
        : tag_id(id_counter++),
          TOW(TOW),
          WN(WN),
          PRNa(PRNa),
          CTR(CTR),
          nmas(nmas),
          status(UNVERIFIED),
          received_tag(MTI.tag),
          computed_tag(0),
          PRN_d(MTI.tag_info.PRN_d),
          ADKD(MTI.tag_info.ADKD),
          cop(MTI.tag_info.cop),
          skipped(0),
          allowed_adkds(std::move(allowed_adkds))
    {
    }
    explicit Tag(const MACK_message& mack)  // constructor for Tag0
        : tag_id(id_counter++),
          TOW(mack.TOW),
          WN(mack.WN),
          PRNa(mack.PRNa),
          CTR(1),
          nmas(mack.nmas),
          status(UNVERIFIED),
          received_tag(mack.header.tag0),
          computed_tag(0),
          PRN_d(mack.PRNa),  // Tag0 are self-authenticating
          ADKD(0),
          cop(mack.header.cop),
          skipped(0),
          allowed_adkds(mack.allowed_adkds)
    {
    }
    const uint32_t tag_id;
    static uint32_t id_counter;
    uint32_t TOW;
    uint32_t WN;
    uint32_t PRNa;
    uint8_t CTR;
    uint8_t nmas;
    e_verification_status status;
    uint64_t received_tag;
    uint64_t computed_tag;
    uint8_t PRN_d;
    uint8_t ADKD;
    uint8_t cop;
    uint32_t skipped;
    std::vector<uint8_t> allowed_adkds{0, 4, 12};
    std::string nav_data;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_OSNMA_DATA_H

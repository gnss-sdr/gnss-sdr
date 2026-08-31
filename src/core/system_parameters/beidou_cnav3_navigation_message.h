/*!
 * \file beidou_cnav3_navigation_message.h
 * \brief B-CNAV3 frame parser (BDS-SIS-ICD-B2b-1.0). First cut skips 64-ary LDPC
 * and takes the systematic 486 information bits after the preamble.
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
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

#ifndef GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H

#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_utc_model.h"
#include <array>
#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class Beidou_Cnav3_Navigation_Message
{
public:
    bool decode_frame_symbols(const float* symbols, int32_t n_symbols, uint32_t prn);
    bool have_new_ephemeris() const { return d_flag_new_eph; }
    Beidou_Cnav1_Ephemeris get_ephemeris() const { return d_eph; }
    void clear_flags() { d_flag_new_eph = false; }
    bool last_crc_ok() const { return d_last_crc_ok; }
    int32_t last_mes_type() const { return d_last_mes_type; }
    int32_t last_sow() const { return d_last_sow; }
    uint32_t last_frame_prn() const { return d_last_frame_prn; }
    const std::string& get_last_nav_bits() const { return d_last_nav_bits; }

private:
    void parse_info_bits(const uint8_t* bits, uint32_t channel_prn);
    Beidou_Cnav1_Ephemeris d_eph{};
    std::string d_last_nav_bits;
    int32_t d_last_mes_type{-1};
    int32_t d_last_sow{-1};
    uint32_t d_last_frame_prn{};
    bool d_have_mt10{false};
    bool d_have_mt30{false};
    bool d_flag_new_eph{false};
    bool d_last_crc_ok{false};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV3_NAVIGATION_MESSAGE_H

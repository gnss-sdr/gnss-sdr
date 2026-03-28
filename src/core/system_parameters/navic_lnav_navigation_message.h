/*!
 * \file navic_lnav_navigation_message.h
 * \brief  Interface of a NavIC LNAV Data message decoder
 * \author Pradyumna Krishna, 2026. pradyumnakrishna(at)gmail.com
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


#ifndef GNSS_SDR_NAVIC_LNAV_NAVIGATION_MESSAGE_H
#define GNSS_SDR_NAVIC_LNAV_NAVIGATION_MESSAGE_H


#include "NAVIC_LNAV.h"
#include "navic_lnav_almanac.h"
#include "navic_lnav_ephemeris.h"
#include "navic_lnav_iono.h"
#include "navic_lnav_utc_model.h"
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes a NavIC LNAV Data message as described
 * in IRNSS SIS ICD for SPS, Version 1.1 (ISRO-IRNSS-ICD-SPS-1.1)
 */
class Navic_Lnav_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Navic_Lnav_Navigation_Message();

    /*!
     * \brief Decodes a NavIC LNAV subframe (292 decoded data bits)
     */
    void decode_subframe(const std::string& subframe_bits);

    /*!
     * \brief Obtain a NavIC SV Ephemeris class filled with current SV data
     */
    Navic_Lnav_Ephemeris get_ephemeris() const;

    /*!
     * \brief Obtain a NavIC Almanac class filled with current SV data
     */
    Navic_Lnav_Almanac get_almanac() const;

    /*!
     * \brief Obtain a NavIC ionospheric correction parameters class filled with current SV data
     */
    Navic_Lnav_Iono get_iono() const;

    /*!
     * \brief Obtain a NavIC UTC model parameters class filled with current SV data
     */
    Navic_Lnav_Utc_Model get_utc_model() const;

    /*!
     * \brief Returns true if new Ephemeris has arrived (both SF1 and SF2 decoded with matching IODEC).
     * The flag is set to false when the function is executed.
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Almanac has arrived.
     * The flag is set to false when the function is executed.
     */
    bool have_new_almanac();

    /*!
     * \brief Returns true if new Iono model has arrived.
     * The flag is set to false when the function is executed.
     */
    bool have_new_iono();

    /*!
     * \brief Returns true if new UTC model has arrived.
     * The flag is set to false when the function is executed.
     */
    bool have_new_utc_model();

    /*!
     * \brief Sets satellite PRN number
     */
    inline void set_satellite_PRN(uint32_t prn)
    {
        i_satellite_PRN = prn;
    }

    inline bool get_flag_CRC_test() const
    {
        return flag_crc_test;
    }

    // Channel and satellite identification
    int32_t i_channel_ID{};
    uint32_t i_satellite_PRN{};

    // Subframe decoded flags
    bool flag_sf1_decoded{};
    bool flag_sf2_decoded{};
    bool flag_sf3_decoded{};
    bool flag_sf4_decoded{};

    // New data flags
    bool flag_new_ephemeris{};
    bool flag_new_almanac{};
    bool flag_new_utc_model{};
    bool flag_new_iono{};
    bool flag_crc_test{};

private:
    /*!
     * \brief Extracts an unsigned integer from a bit string given a field definition
     * \param bits The bit string (1-based indexing in field_def)
     * \param field_def Vector of (start_bit, num_bits) pairs
     * \return Unsigned integer value
     */
    uint64_t read_navigation_unsigned(const std::string& bits, const std::vector<std::pair<int32_t, int32_t>>& field_def) const;

    /*!
     * \brief Extracts a signed (2's complement) integer from a bit string given a field definition
     * \param bits The bit string (1-based indexing in field_def)
     * \param field_def Vector of (start_bit, num_bits) pairs
     * \return Signed integer value
     */
    int64_t read_navigation_signed(const std::string& bits, const std::vector<std::pair<int32_t, int32_t>>& field_def) const;

    /*!
     * \brief Verifies CRC-24Q over bits 1-262, checking against CRC in bits 263-286
     * \param bits The 292-bit subframe string
     * \return true if CRC passes
     */
    bool crc24q_check(const std::string& bits) const;

    // Ephemeris storage
    Navic_Lnav_Ephemeris navic_ephemeris;

    // Almanac storage
    Navic_Lnav_Almanac navic_almanac;

    // Ionosphere storage
    Navic_Lnav_Iono navic_iono;

    // UTC model storage
    Navic_Lnav_Utc_Model navic_utc_model;

    // IODEC tracking for ephemeris consistency
    int32_t d_sf1_iodec{-1};
    int32_t d_sf2_iodec{-1};
    int32_t d_previous_iodec{-1};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_LNAV_NAVIGATION_MESSAGE_H

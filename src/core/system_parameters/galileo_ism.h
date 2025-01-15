/*!
 * \file galileo_ism.h
 * \brief  Interface of a Galileo Integrity Support Message
 * \author Carles Fernandez, 2024. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_ISM_H
#define GNSS_SDR_GALILEO_ISM_H

#include "Galileo_INAV.h"
#include <boost/crc.hpp>
#include <bitset>
#include <cstdint>
#include <unordered_map>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GALILEO Integrity Support Message as described
 * in Galileo ICD paragraph 5.2
 *
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf
 */
class Galileo_ISM
{
public:
    /*!
     * Default constructor
     */
    Galileo_ISM() = default;

    void set_ism_constellation_id(uint8_t const_id);
    void set_ism_service_level_id(uint8_t sl_id);
    void set_ism_wn(uint16_t wn_ism);
    void set_ism_t0(uint16_t t0);
    void set_ism_mask_msb(bool mask_msb);
    void set_ism_mask(uint32_t mask);
    void set_ism_pconst(uint8_t pconst);
    void set_ism_psat(uint8_t psat);
    void set_ism_ura(uint8_t ura);
    void set_ism_ure(uint8_t ure);
    void set_ism_bnom(uint8_t bnom);
    void set_ism_Tvalidity(uint8_t tvalidity);

    bool check_ism_crc(const std::bitset<GALILEO_DATA_JK_BITS>& bits);

    double get_pconst_value() const;
    double get_psat_value() const;
    float get_ura_m() const;
    float get_ure_m() const;
    float get_bnom_m() const;
    uint32_t get_mask_ISM() const;
    uint16_t get_WN_ISM() const;
    uint16_t get_t0_ISM() const;
    uint16_t get_Tvalidity_hours() const;
    bool get_ism_mask_msb() const;
    bool ism_parameters_apply(uint32_t prn) const;

private:
    uint32_t compute_crc(const std::vector<uint8_t>& data);
    boost::crc_optimal<32, 0x814141AB, 0, 0, false, false> d_crc32_ism;

    // ICD 2.1 Table 97
    std::unordered_map<uint8_t, double> d_ISM_PCONST_MAP = {
        {0, 1.0e-8},
        {1, 1.0e-7},
        {2, 1.0e-6},
        {3, 3.0e-6},
        {4, 6.0e-6},
        {5, 8.0e-6},
        {6, 1.0e-5},
        {7, 2.0e-5},
        {8, 4.0e-5},
        {9, 6.0e-5},
        {10, 8.0e-5},
        {11, 1.0e-4},
        {12, 1.25e-4},
        {13, 1.5e-4},
        {14, 1.75e-4},
        {15, 2.0e-4}};

    // ICD 2.1 Table 98
    std::unordered_map<uint8_t, double> d_ISM_PSAT_MAP = {
        {0, 1.0e-7},
        {1, 3.0e-7},
        {2, 6.0e-7},
        {3, 1.0e-6},
        {4, 2.0e-6},
        {5, 3.0e-6},
        {6, 5.0e-6},
        {7, 7.0e-6},
        {8, 1.0e-5},
        {9, 1.2e-5},
        {10, 1.4e-5},
        {11, 1.7e-5},
        {12, 2.05e-5},
        {13, 2.4e-5},
        {14, 2.8e-5},
        {15, 3.0e-5}};

    // ICD 2.1 Table 99
    std::unordered_map<uint8_t, float> d_ISM_URA_MAP = {
        {0, 0.75},
        {1, 1.0},
        {2, 1.5},
        {3, 2.0},
        {4, 2.25},
        {5, 2.50},
        {6, 2.75},
        {7, 3.0},
        {8, 3.25},
        {9, 3.50},
        {10, 3.75},
        {11, 4.0},
        {12, 4.50},
        {13, 5.0},
        {14, 5.50},
        {15, 6.0}};

    // ICD 2.1 Table 100
    std::unordered_map<uint8_t, float> d_ISM_URE_MAP = {
        {0, 0.25},
        {1, 0.50},
        {2, 0.75},
        {3, 1.00},
        {4, 1.25},
        {5, 1.50},
        {6, 1.75},
        {7, 2.0},
        {8, 2.25},
        {9, 2.50},
        {10, 2.75},
        {11, 3.0},
        {12, 3.25},
        {13, 3.50},
        {14, 3.75},
        {15, 4.00}};

    // ICD 2.1 Table 101
    std::unordered_map<uint8_t, float> d_ISM_BNOM_MAP = {
        {0, 0.0},
        {1, 0.10},
        {2, 0.20},
        {3, 0.30},
        {4, 0.40},
        {5, 0.50},
        {6, 0.60},
        {7, 0.75},
        {8, 0.85},
        {9, 1.0},
        {10, 1.20},
        {11, 1.40},
        {12, 1.60},
        {13, 1.80},
        {14, 2.0},
        {15, 2.4}};

    // ICD 2.1 Table 102
    std::unordered_map<uint8_t, uint16_t> d_ISM_TVALIDITY_MAP = {
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4},
        {4, 6},
        {5, 8},
        {6, 12},
        {7, 18},
        {8, 24},
        {9, 36},
        {10, 48},
        {11, 72},
        {12, 120},
        {13, 168},
        {14, 720},
        {15, 1440}};

    uint32_t d_ism_crc{};
    uint32_t d_ism_mask{};
    uint16_t d_ism_wn{};
    uint16_t d_ism_t0{};
    uint8_t d_ism_constellation_id{};
    uint8_t d_ism_service_level_id{};
    uint8_t d_ism_pconst{};
    uint8_t d_ism_psat{};
    uint8_t d_ism_ura{};
    uint8_t d_ism_ure{};
    uint8_t d_ism_bnom{};
    uint8_t d_ism_Tvalidity{};
    bool d_ism_mask_msb{};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_ISM_H
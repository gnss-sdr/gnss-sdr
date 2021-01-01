/*!
 * \file galileo_has_data.h
 * \brief  Class for Galileo HAS message type 1 data storage
 * \author Carles Fernandez-Prades, 2020 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_HAS_DATA_H
#define GNSS_SDR_GALILEO_HAS_DATA_H

#include <cstdint>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

struct mt1_header
{
    uint16_t toh;
    uint8_t mask_id;
    uint8_t iod_id;
    bool mask_flag;
    bool orbit_correction_flag;
    bool clock_fullset_flag;
    bool clock_subset_flag;
    bool code_bias_flag;
    bool phase_bias_flag;
    bool ura_flag;
};

/*!
 * \brief This class is a storage for Galileo HAS message type 1, as defined in
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020).
 */
class Galileo_HAS_data
{
public:
    Galileo_HAS_data() = default;

    mt1_header header;

    // Mask
    uint8_t Nsys;
    std::vector<uint8_t> gnss_id_mask;
    std::vector<uint64_t> satellite_mask;
    std::vector<uint16_t> signal_mask;
    std::vector<bool> cell_mask_availability_flag;
    std::vector<std::vector<std::vector<bool>>> cell_mask;
    std::vector<uint8_t> nav_message;

    // Orbit corrections
    uint8_t validity_interval_index_orbit_corrections;
    std::vector<uint16_t> gnss_iod;
    std::vector<int16_t> delta_radial;
    std::vector<int16_t> delta_along_track;
    std::vector<int16_t> delta_cross_track;

    // Clock full-set corrections
    uint8_t validity_interval_index_clock_fullset_corrections;
    std::vector<uint8_t> delta_clock_c0_multiplier;
    std::vector<bool> iod_change_flag;
    std::vector<int16_t> delta_clock_c0;

    // Clock subset corrections
    uint8_t validity_interval_index_clock_subset_corrections;
    uint8_t Nsysprime;
    std::vector<uint8_t> gnss_id_clock_subset;
    std::vector<uint8_t> delta_clock_c0_multiplier_clock_subset;
    std::vector<std::vector<uint64_t>> satellite_submask;
    std::vector<bool> iod_change_flag_clock_subset;
    std::vector<int16_t> delta_clock_c0_clock_subset;

    // Code bias
    uint8_t validity_interval_index_code_bias_corrections;
    std::vector<std::vector<int16_t>> code_bias;

    // Phase bias
    uint8_t validity_interval_index_phase_bias_corrections;
    std::vector<std::vector<int16_t>> phase_bias;
    std::vector<std::vector<uint8_t>> phase_discontinuity_indicator;

    // URA
    uint8_t validity_interval_index_ura_corrections;
    std::vector<uint8_t> ura;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_HAS_DATA_H

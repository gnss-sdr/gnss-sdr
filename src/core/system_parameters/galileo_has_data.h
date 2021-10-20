/*!
 * \file galileo_has_data.h
 * \brief Class for Galileo HAS message type 1 data storage
 * \author Carles Fernandez-Prades, 2020-2021 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_HAS_DATA_H
#define GNSS_SDR_GALILEO_HAS_DATA_H

#include <cstdint>
#include <string>
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

    std::vector<std::string> get_signals_in_mask(uint8_t nsys) const;         //!< Get a vector of Nsys std::string with signals in mask for system nsys, with 0 <= nsys < Nsys
    std::vector<std::string> get_systems_string() const;                      //!< Get Nsys system name strings
    std::vector<std::vector<float>> get_code_bias_m() const;                  //!< Get Nsat x Ncodes code biases in [m]
    std::vector<std::vector<float>> get_phase_bias_cycle() const;             //!< Get Nsat x Nphases phase biases in [cycles]
    std::vector<float> get_delta_radial_m() const;                            //!< Get Nsat delta radial corrections in [m]
    std::vector<float> get_delta_radial_m(uint8_t nsys) const;                //!< Get delta radial corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_along_track_m() const;                       //!< Get Nsat delta along_track corrections in [m]
    std::vector<float> get_delta_along_track_m(uint8_t nsys) const;           //!< Get alog-track corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_cross_track_m() const;                       //!< Get Nsat delta cross_track corrections in [m]
    std::vector<float> get_delta_cross_track_m(uint8_t nsys) const;           //!< Get delta cross_track corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_clock_c0_m() const;                          //!< Get Nsat delta clock C0 corrections in [m]
    std::vector<float> get_delta_clock_c0_m(uint8_t nsys) const;              //!< Get delta clock C0 corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<int> get_PRNs_in_mask(uint8_t nsys) const;                    //!< Get PRNs in mask for system nsys, with 0 <= nsys < Nsys
    std::vector<int> get_PRNs_in_submask(uint8_t nsys) const;                 //!< Get PRNs in submask for system nsys, with 0 <= nsys < Nsys
    std::vector<uint16_t> get_gnss_iod(uint8_t nsys) const;                   //!< Get GNSS IODs for for system nsys, with 0 <= nsys < Nsys
    std::vector<uint8_t> get_num_satellites() const;                          //!< Get Nsys number of satellites
    uint16_t get_nsat() const;                                                //!< Get total number of satellites with corrections
    uint16_t get_nsatprime() const;                                           //!< Get number of satellites in clock subset corrections
    uint16_t get_validity_interval_s(uint8_t validity_interval_index) const;  //!< Get validity interbal in [s] from the validity_interval_index
    uint8_t get_gnss_id(int nsat) const;                                      //!< Get GNSS ID from the nsat satellite

    // Mask
    std::vector<uint8_t> gnss_id_mask;
    std::vector<uint64_t> satellite_mask;
    std::vector<uint16_t> signal_mask;
    std::vector<bool> cell_mask_availability_flag;
    std::vector<std::vector<std::vector<bool>>> cell_mask;
    std::vector<uint8_t> nav_message;

    // Orbit corrections
    std::vector<uint16_t> gnss_iod;
    std::vector<int16_t> delta_radial;
    std::vector<int16_t> delta_along_track;
    std::vector<int16_t> delta_cross_track;

    // Clock full-set corrections
    std::vector<uint8_t> delta_clock_c0_multiplier;
    std::vector<bool> iod_change_flag;
    std::vector<int16_t> delta_clock_c0;

    // Clock subset corrections
    std::vector<uint8_t> gnss_id_clock_subset;
    std::vector<uint8_t> delta_clock_c0_multiplier_clock_subset;
    std::vector<uint64_t> satellite_submask;
    std::vector<std::vector<bool>> iod_change_flag_clock_subset;
    std::vector<std::vector<int16_t>> delta_clock_c0_clock_subset;

    // Code bias
    std::vector<std::vector<int16_t>> code_bias;

    // Phase bias
    std::vector<std::vector<int16_t>> phase_bias;
    std::vector<std::vector<uint8_t>> phase_discontinuity_indicator;

    // URA
    std::vector<uint8_t> ura;

    mt1_header header;
    uint8_t has_status;
    uint8_t message_id;

    uint8_t Nsys;                                               //!< Number of GNSS for which corrections are provided
    uint8_t validity_interval_index_orbit_corrections;          // in Orbit corrections
    uint8_t validity_interval_index_clock_fullset_corrections;  // in Clock full-set corrections
    uint8_t validity_interval_index_clock_subset_corrections;   // in Clock subset corrections
    uint8_t Nsysprime;                                          // in Clock subset corrections
    uint8_t validity_interval_index_code_bias_corrections;      // in Code bias
    uint8_t validity_interval_index_phase_bias_corrections;     // in Phase bias
    uint8_t validity_interval_index_ura_corrections;            // in URA
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_HAS_DATA_H

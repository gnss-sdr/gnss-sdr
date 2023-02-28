/*!
 * \file galileo_has_data.h
 * \brief Class for Galileo HAS message type 1 data storage
 * \author Carles Fernandez-Prades, 2020-2022 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
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
    uint8_t iod_set_id;
    uint8_t reserved;
    bool mask_flag;
    bool orbit_correction_flag;
    bool clock_fullset_flag;
    bool clock_subset_flag;
    bool code_bias_flag;
    bool phase_bias_flag;
};

/*!
 * \brief This class is a storage for Galileo HAS message type 1, as defined in
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022.
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_HAS_SIS_ICD_v1.0.pdf
 */
class Galileo_HAS_data
{
public:
    Galileo_HAS_data() = default;

    std::vector<std::string> get_signals_in_mask(uint8_t nsys) const;                    //!< Get a vector of Nsys std::string with signals in mask for system nsys, with 0 <= nsys < Nsys
    std::vector<std::string> get_signals_in_mask(const std::string& system) const;       //!< Get a vector of Nsys std::string with signals in mask for system ("GPS"/"Galileo")
    std::vector<std::string> get_systems_string() const;                                 //!< Get Nsys system name strings
    std::vector<std::string> get_systems_subset_string() const;                          //!< Get Nsat system name strings present in clock corrections subset
    std::vector<std::vector<float>> get_code_bias_m() const;                             //!< Get Nsat x Ncodes code biases in [m]
    std::vector<std::vector<float>> get_phase_bias_cycle() const;                        //!< Get Nsat x Nphases phase biases in [cycles]
    std::vector<std::vector<float>> get_delta_clock_subset_correction_m() const;         //!< Get Nsys_sub vectors with Nsat_sub delta clock C0 corrections in [m]
    std::vector<float> get_delta_radial_m() const;                                       //!< Get Nsat delta radial corrections in [m]
    std::vector<float> get_delta_radial_m(uint8_t nsys) const;                           //!< Get delta radial corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_in_track_m() const;                                     //!< Get Nsat delta in-track corrections in [m]
    std::vector<float> get_delta_in_track_m(uint8_t nsys) const;                         //!< Get delta in-track corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_cross_track_m() const;                                  //!< Get Nsat delta cross-track corrections in [m]
    std::vector<float> get_delta_cross_track_m(uint8_t nsys) const;                      //!< Get delta cross-track corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_clock_correction_m() const;                             //!< Get Nsat delta clock C0 corrections in [m]
    std::vector<float> get_delta_clock_correction_m(uint8_t nsys) const;                 //!< Get delta clock C0 corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<float> get_delta_clock_subset_correction_m(uint8_t nsys) const;          //!< Get delta clock C0 subset corrections in [m] for system nsys, with 0 <= nsys < Nsys
    std::vector<int> get_PRNs_in_mask(uint8_t nsys) const;                               //!< Get PRNs in mask for system nsys, with 0 <= nsys < Nsys
    std::vector<int> get_PRNs_in_mask(const std::string& system) const;                  //!< Get PRNs in mask for system ("GPS"/"Galileo")
    std::vector<int> get_PRNs_in_submask(uint8_t nsys) const;                            //!< Get PRNs in submask for system nsys, with 0 <= nsys < Nsys
    std::vector<uint16_t> get_gnss_iod(uint8_t nsys) const;                              //!< Get GNSS IODs for for system nsys, with 0 <= nsys < Nsys
    std::vector<uint8_t> get_num_satellites() const;                                     //!< Get Nsys number of satellites
    std::vector<uint8_t> get_num_subset_satellites() const;                              //!< Get Nsys_sub number of satellites
    float get_code_bias_m(const std::string& signal, int PRN) const;                     //!< Get code bias in [m] for a given signal and PRN satellite
    float get_phase_bias_cycle(const std::string& signal, int PRN) const;                //!< Get phase bias in [cycles] for a given signal and PRN satellite
    float get_delta_radial_m(const std::string& system, int prn) const;                  //!< Get orbital radial correction in [m] for a given system ("GPS"/"Galileo") and PRN
    float get_delta_in_track_m(const std::string& system, int prn) const;                //!< Get orbital in_track correction in [m] for a given system ("GPS"/"Galileo") and PRN
    float get_delta_cross_track_m(const std::string& system, int prn) const;             //!< Get orbital cross_track correction in [m] for a given system ("GPS"/"Galileo") and PRN
    float get_clock_correction_mult_m(const std::string& system, int prn) const;         //!< Get clock correction in [m], already multiplied by its Delta Clock Multiplier, for a given system ("GPS"/"Galileo") and PRN
    float get_clock_subset_correction_mult_m(const std::string& system, int prn) const;  //!< Get clock correction subset in [m], already multiplied by its Delta Clock Multiplier
    uint16_t get_nsat() const;                                                           //!< Get total number of satellites with corrections
    uint16_t get_nsat_sub() const;                                                       //!< Get number of satellites in clock subset corrections
    uint16_t get_validity_interval_s(uint8_t validity_interval_index) const;             //!< Get validity interval in [s] from the validity_interval_index
    uint16_t get_gnss_iod(const std::string& system, int prn) const;                     //!< Get GNSS IOD from a given system ("GPS"/"Galileo") and PRN
    uint8_t get_gnss_id(int nsat) const;                                                 //!< Get GNSS ID from the nsat satellite

    // Mask
    std::vector<uint8_t> gnss_id_mask;                      //!< GNSS ID. See HAS SIS ICD 1.0 Section 5.2.1.1
    std::vector<uint64_t> satellite_mask;                   //!< SatM - Satellite Mask. See HAS SIS ICD 1.0 Section 5.2.1.2
    std::vector<uint16_t> signal_mask;                      //!< SigM - Signal Mask. See HAS SIS ICD 1.0 Section 5.2.1.3
    std::vector<bool> cell_mask_availability_flag;          //!< CMAF - Cell Mask Availability Flag. See HAS SIS ICD 1.0 Section 5.2.1.4
    std::vector<std::vector<std::vector<bool>>> cell_mask;  //!< CM - Cell Mask. See HAS SIS ICD 1.0 Section 5.2.1.5
    std::vector<uint8_t> nav_message;                       //!< NM - Navigation Message Index. See HAS SIS ICD 1.0 Section 5.2.1.6

    // Orbit corrections
    std::vector<uint16_t> gnss_iod;          //!< IODref - Reference Issue of Data. See HAS SIS ICD 1.0 Table 26
    std::vector<int16_t> delta_radial;       //!< DR - Delta Radial Correction. See HAS SIS ICD 1.0 Table 25
    std::vector<int16_t> delta_in_track;     //!< DIT - Delta In-Track Correction. See HAS SIS ICD 1.0 Table 25
    std::vector<int16_t> delta_cross_track;  //!< DCT - Delta Cross Correction. See HAS SIS ICD 1.0 Table 25

    // Clock full-set corrections
    std::vector<uint8_t> delta_clock_multiplier;  //!< DCM - Delta Clock Multipliers. See HAS SIS ICD 1.0 Section 5.2.3.1
    std::vector<int16_t> delta_clock_correction;  //!< DCC - Delta Clock Corrections. See HAS SIS ICD 1.0 Section 5.2.3.2

    // Clock subset corrections
    std::vector<uint8_t> gnss_id_clock_subset;                              //!< GNSS ID. Specific GNSS to which the corrections refer. See HAS SIS ICD 1.0 Section 5.2.1.1
    std::vector<uint8_t> delta_clock_multiplier_clock_subset;               //!< DCM. Multiplier for all Delta Clock corrections. See HAS SIS ICD 1.0 Section 5.2.3.1
    std::vector<uint64_t> satellite_submask;                                //!< SatMsub - Satellite Subset Mask. See HAS SIS ICD 1.0 Section 5.2.4.1
    std::vector<std::vector<int16_t>> delta_clock_correction_clock_subset;  //!< DCCsub - Delta Clock Subset Corrections. See HAS SIS ICD 1.0 Section 5.2.4.1

    // Code bias
    std::vector<std::vector<int16_t>> code_bias;  //!< CB - Code bias for the m-th signal of the n-th SV. See HAS SIS ICD 1.0 Section 5.2.5

    // Phase bias
    std::vector<std::vector<int16_t>> phase_bias;                     //!< PB - Phase bias for the m-th signal of the n-th SV. See HAS SIS ICD 1.0 Section 5.2.6
    std::vector<std::vector<uint8_t>> phase_discontinuity_indicator;  //!< PDI - Phase Discontinuity Indicator. See HAS SIS ICD 1.0 Section 5.2.6.

    uint32_t tow;  //!< Time of Week

    mt1_header header;   //!< MT1 Header parameters. See HAS SIS ICD 1.0 Section 5.1.1
    uint8_t has_status;  //!< HASS - HAS Status (from HAS page header). See HAS SIS ICD 1.0 Section 3.1.1
    uint8_t message_id;  //!< MID - Message ID (from HAS page header). See HAS SIS ICD 1.0 Section 3.1

    uint8_t Nsys;      //!< Number of GNSS for which corrections are provided. See HAS SIS ICD 1.0 Setion 5.2.1
    uint8_t Nsys_sub;  //!< Number of GNSS for which corrections are provided in clock subset corrections. See HAS SIS ICD 1.0 Section 5.2.2.1

    uint8_t validity_interval_index_orbit_corrections;          //!< VI - Validity Interval Index for Orbit corrections. See HAS SIS ICD 1.0 Section 5.2.2.1
    uint8_t validity_interval_index_clock_fullset_corrections;  //!< VI - Validity Interval Index for Clock full-set corrections. See HAS SIS ICD 1.0 Section 5.2.2.1
    uint8_t validity_interval_index_clock_subset_corrections;   //!< VI - Validity Interval Index for Clock subset corrections. See HAS SIS ICD 1.0 Section 5.2.2.1
    uint8_t validity_interval_index_code_bias_corrections;      //!< VI - Validity Interval Index for Code bias. See HAS SIS ICD 1.0 Section 5.2.2.1
    uint8_t validity_interval_index_phase_bias_corrections;     //!< VI - Validity Interval Index for Phase bias. See HAS SIS ICD 1.0 Section 5.2.2.1
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_HAS_DATA_H

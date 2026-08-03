/*!
 * \file beidou_dnav_navigation_message.h
 * \brief  Interface of a BeiDou DNAV Data message decoder
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * \author Carles Fernandez-Prades, 2018-2026. cfernandez(at)cttc.es
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


#ifndef GNSS_SDR_BEIDOU_DNAV_NAVIGATION_MESSAGE_H
#define GNSS_SDR_BEIDOU_DNAV_NAVIGATION_MESSAGE_H


#include "Beidou_B1I.h"
#include "Beidou_B3I.h"
#include "Beidou_DNAV.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_differential_corrections.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes BeiDou D1 and D2 NAV data messages
 */
class Beidou_Dnav_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Beidou_Dnav_Navigation_Message();

    /*!
     * \brief Obtain a BDS SV Ephemeris class filled with current SV data
     */
    Beidou_Dnav_Ephemeris get_ephemeris() const;

    /*!
     * \brief Obtain a BDS ionospheric correction parameters class filled with current SV data
     */
    Beidou_Dnav_Iono get_iono();

    /*!
     * \brief Obtain a BDS UTC model parameters class filled with current SV data
     */
    Beidou_Dnav_Utc_Model get_utc_model();

    /*!
     * \brief Obtain the last decoded BeiDou almanac record
     */
    Beidou_Dnav_Almanac get_almanac() const;

    /*!
     * \brief Obtain the latest D2 integrity and differential corrections
     */
    Beidou_Dnav_Differential_Corrections get_differential_corrections() const;

    /*!
     * \brief Decodes the BDS D1 NAV message
     */
    int32_t d1_subframe_decoder(std::string const& subframe);

    /*!
     * \brief Decodes the BDS D2 NAV message
     */
    int32_t d2_subframe_decoder(std::string const& subframe);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s]
     */
    double utc_time(double beidoutime_corrected) const;

    bool satellite_validation();

    /*!
     * \brief Returns true if new Ephemeris has arrived. The flag is set to false when the function is executed
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Iono model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_iono() const;

    /*!
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_utc_model();

    /*!
     * \brief Returns true if new UTC model has arrived. The flag is set to false when the function is executed
     */
    bool have_new_almanac();

    /*!
     * \brief Returns true when a complete D2 subframes 2-4 correction page has arrived
     */
    bool have_new_differential_corrections();

    /*!
     * \brief Sets satellite PRN number
     */
    inline void set_satellite_PRN(uint32_t prn)
    {
        i_satellite_PRN = prn;
    }

    inline void set_signal_type(int32_t signal_type)
    {
        i_signal_type = signal_type;
    }

    inline bool get_flag_CRC_test() const
    {
        return flag_crc_test;
    }

    inline bool get_flag_new_SOW_available() const
    {
        return flag_new_SOW_available;
    }

    inline void set_flag_new_SOW_available(bool new_SOW_available)
    {
        flag_new_SOW_available = new_SOW_available;
    }

    inline double get_SOW() const
    {
        return d_SOW;
    }

private:
    uint64_t read_navigation_unsigned(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    uint64_t read_navigation_data_unsigned(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, int32_t first_bit, int32_t logical_offset, int32_t length) const;
    int64_t sign_extend(uint64_t value, int32_t length) const;
    bool read_navigation_bool(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    void print_beidou_word_bytes(uint32_t BEIDOU_word) const;
    double wrap_dnav_sow(double sow) const;
    bool d1_ephemeris_sow_is_consistent() const;
    bool format_check(std::string const& subframe) const;
    void clear_d2_ephemeris_page_flags();
    bool d2_ephemeris_page_is_expected(int32_t page_ID, double sow);
    void advance_d2_ephemeris_page(int32_t page_ID, double sow);
    void decode_almanac(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, int32_t prn, int32_t amid, bool expanded);
    void decode_almanac_health(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, int32_t count, int32_t first_prn);
    void decode_time_offsets(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits);
    void decode_utc_parameters(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits);
    void decode_d2_iono_grid(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, int32_t page_ID);
    void update_d2_bdid(uint64_t value, int32_t first_prn, int32_t count);
    void update_d2_udrei(const std::bitset<BEIDOU_DNAV_SUBFRAME_DATA_BITS>& bits, int32_t first_bit, int32_t first_slot, int32_t count);

    // broadcast orbit 1
    double d_SOW{};      // Time of BeiDou Week of the ephemeris set (taken from subframes SOW) [s]
    double d_SOW_SF1{};  // Time of BeiDou Week from HOW word of Subframe 1 [s]
    double d_SOW_SF2{};  // Time of BeiDou Week from HOW word of Subframe 2 [s]
    double d_SOW_SF3{};  // Time of BeiDou Week from HOW word of Subframe 3 [s]
    double d_SOW_SF4{};  // Time of BeiDou Week from HOW word of Subframe 4 [s]
    double d_SOW_SF5{};  // Time of BeiDou Week from HOW word of Subframe 5 [s]

    double d_AODE{};
    double d_Crs{};      // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n{};  // Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0{};      // Mean Anomaly at Reference Time [semi-circles]

    // broadcast orbit 2
    double d_Cuc{};           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_eccentricity{};  // Eccentricity [dimensionless]
    double d_Cus{};           // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A{};        // Square Root of the Semi-Major Axis [sqrt(m)]

    // broadcast orbit 3
    double d_Toe_sf2{};  // Ephemeris data reference time of week in subframe 2, D1 Message
    double d_Toe_sf3{};  // Ephemeris data reference time of week in subframe 3, D1 Message
    double d_Toe{};      // Ephemeris data reference time of week in subframe 1, D2 Message
    double d_Toc{};      // clock data reference time [s]
    double d_Cic{};      // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0{};   // Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis{};      // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]

    // broadcast orbit 4
    double d_i_0{};        // Inclination Angle at Reference Time [semi-circles]
    double d_Crc{};        // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA{};      // Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT{};  // Rate of Right Ascension [semi-circles/s]

    // broadcast orbit 5
    double d_IDOT{};          // Rate of Inclination Angle [semi-circles/s]
    int32_t i_BEIDOU_week{};  // BeiDou week number, aka WN [week]

    // broadcast orbit 6
    int32_t i_SV_accuracy{};  // User Range Accuracy (URA) index of the SV
    int32_t i_SV_health{};
    double d_TGD1{};  // Estimated Group Delay Differential in B1 [s]
    double d_TGD2{};  // Estimated Group Delay Differential in B2 [s]
    double d_AODC{};  // Age of Data, Clock

    // broadcast orbit 7
    // int32_t i_AODO{};              // Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    // bool b_fit_interval_flag{};  // indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    // double d_spare1{};
    // double d_spare2{};

    double d_A_f0{};  // Clock correction parameters. Coefficient 0 of code phase offset model [s]
    double d_A_f1{};  // Clock correction parameters. Coefficient 1 of code phase offset model [s/s]
    double d_A_f2{};  // Clock correction parameters. Coefficient 2 of code phase offset model [s/s^2]

    // D2 NAV Message Decoding
    uint64_t d_A_f1_msb_bits{};          // Clock correction parameters, D2 NAV MSB
    uint64_t d_A_f1_lsb_bits{};          // Clock correction parameters, D2 NAV LSB
    uint64_t d_Cuc_msb_bits{};           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    uint64_t d_Cuc_lsb_bits{};           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    uint64_t d_eccentricity_msb{};       // Eccentricity [dimensionless]
    uint64_t d_eccentricity_lsb{};       // Eccentricity [dimensionless]
    uint64_t d_Cic_msb_bits{};           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    uint64_t d_Cic_lsb_bits{};           // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    uint64_t d_eccentricity_msb_bits{};  // Eccentricity [dimensionless]
    uint64_t d_eccentricity_lsb_bits{};
    uint64_t d_i_0_msb_bits{};        // Inclination Angle at Reference Time [semi-circles]
    uint64_t d_i_0_lsb_bits{};        // Inclination Angle at Reference Time [semi-circles]
    uint64_t d_OMEGA_msb_bits{};      // Argument of Perigee [semi-cicles]
    uint64_t d_OMEGA_lsb_bits{};      // Argument of Perigee [semi-cicles]
    uint64_t d_OMEGA_DOT_msb_bits{};  // Rate of Right Ascension [semi-circles/s]
    uint64_t d_OMEGA_DOT_lsb_bits{};  // Rate of Right Ascension [semi-circles/s]

    // D2 integrity and differential corrections
    std::array<bool, 63> d_d2_bdid{};
    std::array<int32_t, 24> d_d2_rurai{};
    std::array<int32_t, 24> d_d2_udrei{};
    std::array<double, 24> d_d2_delta_t{};
    std::array<bool, 24> d_d2_delta_t_available{};
    std::array<bool, 24> d_d2_correction_valid{};
    int32_t i_SatH2{};
    int32_t i_BDEpID{};
    int32_t i_d2_integrity_page{};
    double d_d2_integrity_sow{};
    bool flag_d2_sf2_valid{};
    bool flag_d2_sf3_valid{};
    bool flag_new_differential_corrections{};

    // Almanac
    // double d_Toa{};                            // Almanac reference time [s]
    // int32_t i_WN_A{};                          // Modulo 256 of the GPS week number to which the almanac reference time (d_Toa) is referenced
    std::map<int32_t, int32_t> almanacHealth;  // Map that stores the health information stored in the almanac
    Beidou_Dnav_Almanac d_almanac;
    int32_t i_AmEpID{};
    bool flag_new_almanac{};
    bool flag_almanac_week_valid{};

    std::map<int32_t, std::string> satelliteBlock;  // Map that stores to which block the PRN belongs

    // satellite identification info
    int32_t i_signal_type{};  // BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q)
    uint32_t i_satellite_PRN{};

    // Ionospheric parameters
    double d_alpha0{};  // Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1{};  // Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2{};  // Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3{};  // Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0{};   // Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1{};   // Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2{};   // Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3{};   // Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]
    std::map<int, Beidou_Dnav_Iono_Grid_Point> d_iono_grid;
    bool flag_iono_grid_valid{};

    // UTC parameters
    double d_A1UTC{};       // 1st order term of a model that relates GPS and UTC time [s/s]
    double d_A0UTC{};       // Constant of a model that relates GPS and UTC time [s]
    int32_t i_DeltaT_LS{};  // BDT-UTC delta time due to leap seconds [s]
    int32_t i_WN_LSF{};     // Week number at the end of which the leap second becomes effective [weeks]
    int32_t i_DN{};         // Day number (DN) at the end of which the leap second becomes effective [days]
    double d_DeltaT_LSF{};  // Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]
    double d_A1GPS{};
    double d_A0GPS{};
    double d_A1GAL{};
    double d_A0GAL{};
    double d_A1GLO{};
    double d_A0GLO{};

    double d_SQRT_A_ALMANAC{};
    double d_A1_ALMANAC{};
    double d_A0_ALMANAC{};
    double d_OMEGA0_ALMANAC{};
    double d_E_ALMANAC{};
    double d_DELTA_I{};
    double d_TOA{};
    double d_OMEGA_DOT_ALMANAC{};
    double d_OMEGA_ALMANAC{};
    double d_M0_ALMANAC{};
    int32_t almanac_WN{};
    double d_toa2{};

    // System flags for data processing
    bool flag_eph_valid{};
    bool flag_utc_model_valid{};
    bool flag_iono_valid{};
    bool flag_d1_sf1{};
    bool flag_d1_sf2{};
    bool flag_d1_sf3{};
    bool flag_d1_sf4{};
    bool flag_d1_sf5{};
    bool flag_new_SOW_available{};
    bool flag_crc_test{};
    double d_previous_aode{-1.0};

    // bool flag_d1_sf5_p7{};   // D1 NAV Message, Subframe 5, Page 09 decoded indicator
    // bool flag_d1_sf5_p8{};   // D1 NAV Message, Subframe 5, Page 09 decoded indicator
    bool flag_d1_sf5_p9{};   // D1 NAV Message, Subframe 5, Page 09 decoded indicator
    bool flag_d1_sf5_p10{};  // D1 NAV Message, Subframe 5, Page 10 decoded indicator

    bool flag_sf1_p1{};   // D2 NAV Message, Subframe 1, Page 1 decoded indicator
    bool flag_sf1_p2{};   // D2 NAV Message, Subframe 1, Page 2 decoded indicator
    bool flag_sf1_p3{};   // D2 NAV Message, Subframe 1, Page 3 decoded indicator
    bool flag_sf1_p4{};   // D2 NAV Message, Subframe 1, Page 4 decoded indicator
    bool flag_sf1_p5{};   // D2 NAV Message, Subframe 1, Page 5 decoded indicator
    bool flag_sf1_p6{};   // D2 NAV Message, Subframe 1, Page 6 decoded indicator
    bool flag_sf1_p7{};   // D2 NAV Message, Subframe 1, Page 7 decoded indicator
    bool flag_sf1_p8{};   // D2 NAV Message, Subframe 1, Page 8 decoded indicator
    bool flag_sf1_p9{};   // D2 NAV Message, Subframe 1, Page 9 decoded indicator
    bool flag_sf1_p10{};  // D2 NAV Message, Subframe 1, Page 10 decoded indicator
    bool flag_d2_ephemeris_collection_started{};
    int32_t d_d2_expected_page{1};
    double d_d2_expected_sow{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_NAVIGATION_MESSAGE_H

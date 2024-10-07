/*!
 * \file gps_navigation_message.h
 * \brief  Interface of a GPS NAV Data message decoder
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
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


#ifndef GNSS_SDR_GPS_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GPS_NAVIGATION_MESSAGE_H


#include "GPS_L1_CA.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <utility>  // for pair
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class decodes a GPS NAV Data message as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 */
class Gps_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Gps_Navigation_Message();

    /*!
     * \brief Obtain a GPS SV Ephemeris class filled with current SV data
     */
    Gps_Ephemeris get_ephemeris() const;

    /*!
     * \brief Obtain a GPS ionospheric correction parameters class filled with current SV data
     */
    Gps_Iono get_iono();

    /*!
     * \brief Obtain a GPS UTC model parameters class filled with current SV data
     */
    Gps_Utc_Model get_utc_model();

    /*!
     * \brief Decodes the GPS NAV message
     */
    int32_t subframe_decoder(const char* subframe);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (IS-GPS-200M, 20.3.3.5.2.4)
     */
    double utc_time(double gpstime_corrected) const;

    /*!
     * \brief Gets Time of Week, in seconds
     */
    inline int32_t get_TOW() const
    {
        return d_TOW;
    }

    /*!
     * \brief Sets Time of Week, in seconds
     */
    inline int32_t get_GPS_week() const
    {
        return i_GPS_week;
    }

    /*!
     * \brief Sets satellite PRN number
     */
    inline void set_satellite_PRN(uint32_t prn)
    {
        i_satellite_PRN = prn;
    }

    /*!
     * \brief Gets satellite PRN number
     */
    inline uint32_t get_satellite_PRN() const
    {
        return i_satellite_PRN;
    }

    /*!
     * \brief Sets channel ID
     */
    inline void set_channel(int32_t channel_id)
    {
        i_channel_ID = channel_id;
    }

    /*!
     * \brief Gets flag_iono_valid
     */
    inline bool get_flag_iono_valid() const
    {
        return flag_iono_valid;
    }

    /*!
     * \brief Gets flag_utc_model_valid
     */
    inline bool get_flag_utc_model_valid() const
    {
        return flag_utc_model_valid;
    }

    bool satellite_validation();

private:
    uint64_t read_navigation_unsigned(const std::bitset<GPS_SUBFRAME_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    int64_t read_navigation_signed(const std::bitset<GPS_SUBFRAME_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    bool read_navigation_bool(const std::bitset<GPS_SUBFRAME_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const;
    void print_gps_word_bytes(uint32_t GPS_word) const;

    std::map<int32_t, int32_t> almanacHealth;  //!< Map that stores the health information stored in the almanac

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs https://www.navcen.uscg.gov/?Do=constellationStatus

    // broadcast orbit 1
    int32_t d_TOW{};      // Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    int32_t d_TOW_SF1{};  // Time of GPS Week from HOW word of Subframe 1 [s]
    int32_t d_TOW_SF2{};  // Time of GPS Week from HOW word of Subframe 2 [s]
    int32_t d_TOW_SF3{};  // Time of GPS Week from HOW word of Subframe 3 [s]
    int32_t d_TOW_SF4{};  // Time of GPS Week from HOW word of Subframe 4 [s]
    int32_t d_TOW_SF5{};  // Time of GPS Week from HOW word of Subframe 5 [s]
    int32_t d_IODE_SF2{};
    int32_t d_IODE_SF3{};
    double d_Crs{};      // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n{};  // Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0{};      // Mean Anomaly at Reference Time [semi-circles]
    // broadcast orbit 2
    double d_Cuc{};             // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_e_eccentricity{};  // Eccentricity [dimensionless]
    double d_Cus{};             // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A{};          // Square Root of the Semi-Major Axis [sqrt(m)]
    // broadcast orbit 3
    int32_t d_Toe{};    // Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200M) [s]
    int32_t d_Toc{};    // clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200M) [s]
    double d_Cic{};     // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0{};  // Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis{};     // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    // broadcast orbit 4
    double d_i_0{};        // Inclination Angle at Reference Time [semi-circles]
    double d_Crc{};        // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA{};      // Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT{};  // Rate of Right Ascension [semi-circles/s]
    // broadcast orbit 5
    double d_IDOT{};          // Rate of Inclination Angle [semi-circles/s]
    int32_t i_code_on_L2{};   // If 1, P code ON in L2;  if 2, C/A code ON in L2;
    int32_t i_GPS_week{};     // GPS week number, aka WN [week]
    bool b_L2_P_data_flag{};  // When true, indicates that the NAV data stream was commanded OFF on the P-code of the L2 channel
    // broadcast orbit 6
    int32_t i_SV_accuracy{};  // User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200M)
    int32_t i_SV_health{};
    double d_TGD{};    // Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    int32_t d_IODC{};  // Issue of Data, Clock
    // broadcast orbit 7
    int32_t i_AODO{};            // Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]
    bool b_fit_interval_flag{};  // indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double d_spare1{};
    double d_spare2{};
    double d_A_f0{};  // Coefficient 0 of code phase offset model [s]
    double d_A_f1{};  // Coefficient 1 of code phase offset model [s/s]
    double d_A_f2{};  // Coefficient 2 of code phase offset model [s/s^2]

    // Almanac
    int32_t i_Toa{};   // Almanac reference time [s]
    int32_t i_WN_A{};  // Modulo 256 of the GPS week number to which the almanac reference time (i_Toa) is referenced

    // satellite identification info
    int32_t i_channel_ID{};
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

    // UTC parameters
    double d_A0{};  // Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200M) [s]
    double d_A1{};  // 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200M) [s/s]

    int32_t d_t_OT{};        // Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200M) [s]
    int32_t i_WN_T{};        // UTC reference week number [weeks]
    int32_t d_DeltaT_LS{};   // delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int32_t i_WN_LSF{};      // Week number at the end of which the leap second becomes effective [weeks]
    int32_t i_DN{};          // Day number (DN) at the end of which the leap second becomes effective [days]
    int32_t d_DeltaT_LSF{};  // Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]

    // Flags
    bool b_valid_ephemeris_set_flag{};  // flag indicating that this ephemeris set have passed the validation check
    bool flag_iono_valid{};             // If set, it indicates that the ionospheric parameters are filled (page 18 has arrived and decoded)
    bool flag_utc_model_valid{};        // If set, it indicates that the UTC model parameters are filled

    /*  If true, enhanced level of integrity assurance.
     *
     *  If false, indicates that the conveying signal is provided with the legacy level of integrity assurance.
     *  That is, the probability that the instantaneous URE of the conveying signal exceeds 4.42 times the upper bound
     *  value of the current broadcast URA index, for more than 5.2 seconds, without an accompanying alert, is less
     *  than 1E-5 per hour. If true, indicates that the conveying signal is provided with an enhanced level of
     *  integrity assurance. That is, the probability that the instantaneous URE of the conveying signal exceeds 5.73
     *  times the upper bound value of the current broadcast URA index, for more than 5.2 seconds, without an
     *  accompanying alert, is less than 1E-8 per hour.
     */
    bool b_integrity_status_flag{};
    bool b_alert_flag{};         // If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool b_antispoofing_flag{};  // If true, the AntiSpoofing mode is ON in that SV
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_NAVIGATION_MESSAGE_H

/*!
 * \file beidou_dnav_ephemeris.h
 * \brief  Interface of a BEIDOU EPHEMERIS storage
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H
#define GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H

#include <boost/serialization/nvp.hpp>
#include <map>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage and orbital model functions for the GPS SV ephemeris data as described in
 * BeiDou Navigation Satellite System Signal In Space Interface Control Document
 * Open Service Signal B1I (Version 3.0)
 *
 * See http://en.beidou.gov.cn/SYSTEMS/Officialdocument/201902/P020190227601370045731.pdf
 */
class Beidou_Dnav_Ephemeris
{
public:
    /*!
     * Default constructor
     */
    Beidou_Dnav_Ephemeris();

    /*!
     * \brief Compute the ECEF SV coordinates and ECEF velocity
     * Implementation of Table 20-IV (IS-GPS-200K)
     * and compute the clock bias term including relativistic effect (return value)
     */
    double satellitePosition(double transmitTime);

    /*!
     * \brief Sets (\a d_satClkDrift)and returns the clock drift in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200K,  20.3.3.3.3.1)
     */
    double sv_clock_drift(double transmitTime);

    /*!
     * \brief Sets (\a d_dtr) and returns the clock relativistic correction term in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200K,  20.3.3.3.3.1)
     */
    double sv_clock_relativistic_term(double transmitTime);

    unsigned int i_satellite_PRN{};  //!< SV PRN NUMBER
    double d_TOW{};                  //!< Time of BEIDOU Week of the ephemeris set (taken from subframes TOW) [s]
    double d_Crs{};                  //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n{};              //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0{};                  //!< Mean Anomaly at Reference Time [semi-circles]
    double d_Cuc{};                  //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_eccentricity{};         //!< Eccentricity [dimensionless]
    double d_Cus{};                  //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A{};               //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double d_Toe{};                  //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200K) [s]
    double d_Toc{};                  //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200K) [s]
    double d_Cic{};                  //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0{};               //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis{};                  //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_i_0{};                  //!< Inclination Angle at Reference Time [semi-circles]
    double d_Crc{};                  //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA{};                //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT{};            //!< Rate of Right Ascension [semi-circles/s]
    double d_IDOT{};                 //!< Rate of Inclination Angle [semi-circles/s]
    int i_BEIDOU_week{};             //!< BEIDOU week number, aka WN [week]
    int i_SV_accuracy{};             //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200K)
    int i_SV_health{};
    double d_TGD1{};  //!< Estimated Group Delay Differential on B1I [s]
    double d_TGD2{};  //!< Estimated Group Delay Differential on B2I [s]
    double d_AODC{};  //!< Age of Data, Clock
    double d_AODE{};  //!< Age of Data, Ephemeris
    int i_AODO{};     //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    int i_sig_type{};  //!< BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
    int i_nav_type{};  //!< BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */

    bool b_fit_interval_flag{};  //!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double d_spare1{};
    double d_spare2{};

    double d_A_f0{};  //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1{};  //!< Coefficient 1 of code phase offset model [s/s]
    double d_A_f2{};  //!< Coefficient 2 of code phase offset model [s/s^2]

    /*! \brief If true, enhanced level of integrity assurance.
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
    bool b_alert_flag{};         //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool b_antispoofing_flag{};  //!<  If true, the AntiSpoofing mode is ON in that SV

    // clock terms derived from ephemeris data
    double d_satClkDrift{};  //!< GPS clock error
    double d_dtr{};          //!< relativistic clock correction term

    // satellite positions
    double d_satpos_X{};  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y{};  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z{};  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // Satellite velocity
    double d_satvel_X{};  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y{};  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z{};  //!< Earth-fixed velocity coordinate z of the satellite [m]

    std::map<int, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs


    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };

        archive& make_nvp("i_satellite_PRN", i_satellite_PRN);  // SV PRN NUMBER
        archive& make_nvp("d_TOW", d_TOW);                      //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
        archive& make_nvp("d_AODE", d_AODE);
        archive& make_nvp("d_Crs", d_Crs);                      //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
        archive& make_nvp("d_Delta_n", d_Delta_n);              //!< Mean Motion Difference From Computed Value [semi-circles/s]
        archive& make_nvp("d_M_0", d_M_0);                      //!< Mean Anomaly at Reference Time [semi-circles]
        archive& make_nvp("d_Cuc", d_Cuc);                      //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
        archive& make_nvp("d_e_eccentricity", d_eccentricity);  //!< Eccentricity [dimensionless]
        archive& make_nvp("d_Cus", d_Cus);                      //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
        archive& make_nvp("d_sqrt_A", d_sqrt_A);                //!< Square Root of the Semi-Major Axis [sqrt(m)]
        archive& make_nvp("d_Toe", d_Toe);                      //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200K) [s]
        archive& make_nvp("d_Toc", d_Toe);                      //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200K) [s]
        archive& make_nvp("d_Cic", d_Cic);                      //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
        archive& make_nvp("d_OMEGA0", d_OMEGA0);                //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
        archive& make_nvp("d_Cis", d_Cis);                      //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
        archive& make_nvp("d_i_0", d_i_0);                      //!< Inclination Angle at Reference Time [semi-circles]
        archive& make_nvp("d_Crc", d_Crc);                      //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
        archive& make_nvp("d_OMEGA", d_OMEGA);                  //!< Argument of Perigee [semi-cicles]
        archive& make_nvp("d_OMEGA_DOT", d_OMEGA_DOT);          //!< Rate of Right Ascension [semi-circles/s]
        archive& make_nvp("d_IDOT", d_IDOT);                    //!< Rate of Inclination Angle [semi-circles/s]
        archive& make_nvp("i_BEIDOU_week", i_BEIDOU_week);      //!< GPS week number, aka WN [week]
        archive& make_nvp("i_SV_accuracy", i_SV_accuracy);      //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200K)
        archive& make_nvp("i_SV_health", i_SV_health);
        archive& make_nvp("d_AODC", d_AODC);  //!< Issue of Data, Clock
        archive& make_nvp("d_TGD1", d_TGD1);  //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("d_TGD2", d_TGD2);  //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("i_AODO", i_AODO);  //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        archive& make_nvp("b_fit_interval_flag", b_fit_interval_flag);  //!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        archive& make_nvp("d_spare1", d_spare1);
        archive& make_nvp("d_spare2", d_spare2);

        archive& make_nvp("d_A_f0", d_A_f0);  //!< Coefficient 0 of code phase offset model [s]
        archive& make_nvp("d_A_f1", d_A_f1);  //!< Coefficient 1 of code phase offset model [s/s]
        archive& make_nvp("d_A_f2", d_A_f2);  //!< Coefficient 2 of code phase offset model [s/s^2]

        archive& make_nvp("b_integrity_status_flag", b_integrity_status_flag);
        archive& make_nvp("b_alert_flag", b_alert_flag);                //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
        archive& make_nvp("b_antispoofing_flag", b_antispoofing_flag);  //!<  If true, the AntiSpoofing mode is ON in that SV
    }

private:
    /*
     * Accounts for the beginning or end of week crossover
     *
     * See paragraph 20.3.3.3.3.1 (IS-GPS-200K)
     * \param[in]  -  time in seconds
     * \param[out] -  corrected time, in seconds
     */
    double check_t(double time);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_EPHEMERIS_H

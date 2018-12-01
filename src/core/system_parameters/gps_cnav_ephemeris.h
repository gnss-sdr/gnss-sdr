/*!
 * \file gps_cnav_ephemeris.h
 * \brief  Interface of a GPS CNAV EPHEMERIS storage
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_CNAV_EPHEMERIS_H_
#define GNSS_SDR_GPS_CNAV_EPHEMERIS_H_

#include "GPS_CNAV.h"
#include <boost/assign.hpp>
#include <boost/serialization/nvp.hpp>


/*!
 * \brief This class is a storage and orbital model functions for the GPS SV ephemeris data as described in IS-GPS-200H
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf Appendix III
 */
class Gps_CNAV_Ephemeris
{
private:
    double check_t(double time);

public:
    uint32_t i_satellite_PRN;  // SV PRN NUMBER

    // Message Types 10 and 11 Parameters (1 of 2)
    int32_t i_GPS_week;        //!< GPS week number, aka WN [week]
    int32_t i_URA;             //!< ED Accuracy Index
    int32_t i_signal_health;   //!< Signal health (L1/L2/L5)
    double d_Top;              //!< Data predict time of week
    double d_DELTA_A;          //!< Semi-major axis difference at reference time
    double d_A_DOT;            //!< Change rate in semi-major axis
    double d_Delta_n;          //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_DELTA_DOT_N;      //!< Rate of mean motion difference from computed value
    double d_M_0;              //!< Mean Anomaly at Reference Time [semi-circles]
    double d_e_eccentricity;   //!< Eccentricity
    double d_OMEGA;            //!< Argument of Perigee [semi-cicles]
    double d_OMEGA0;           //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-cicles]
    double d_Toe1;             //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
    double d_Toe2;             //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
    double d_DELTA_OMEGA_DOT;  //!< Rate of Right Ascension  difference [semi-circles/s]
    double d_i_0;              //!< Inclination Angle at Reference Time [semi-circles]
    double d_IDOT;             //!< Rate of Inclination Angle [semi-circles/s]
    double d_Cis;              //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_Cic;              //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_Crs;              //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Crc;              //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_Cus;              //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_Cuc;              //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]

    // Clock Correction and Accuracy Parameters
    double d_Toc;   //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
    double d_A_f0;  //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1;  //!< Coefficient 1 of code phase offset model [s/s]
    double d_A_f2;  //!< Coefficient 2 of code phase offset model [s/s^2]

    double d_URA0;  //!<NED Accuracy Index
    double d_URA1;  //!<NED Accuracy Change Index
    double d_URA2;  //!< NED Accuracy Change Rate Index

    // Group Delay Differential Parameters
    double d_TGD;  //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    double d_ISCL1;
    double d_ISCL2;
    double d_ISCL5I;
    double d_ISCL5Q;

    double d_TOW;  //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]

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
    bool b_integrity_status_flag;
    bool b_l2c_phasing_flag;
    bool b_alert_flag;         //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool b_antispoofing_flag;  //!<  If true, the AntiSpoofing mode is ON in that SV

    // clock terms derived from ephemeris data
    double d_satClkDrift;  //!< GPS clock error
    double d_dtr;          //!< relativistic clock correction term

    // satellite positions
    double d_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // Satellite velocity
    double d_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };

        archive& make_nvp("i_satellite_PRN", i_satellite_PRN);    // SV PRN NUMBER
        archive& make_nvp("d_TOW", d_TOW);                        //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
        archive& make_nvp("d_Crs", d_Crs);                        //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
        archive& make_nvp("d_M_0", d_M_0);                        //!< Mean Anomaly at Reference Time [semi-circles]
        archive& make_nvp("d_Cuc", d_Cuc);                        //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
        archive& make_nvp("d_e_eccentricity", d_e_eccentricity);  //!< Eccentricity [dimensionless]
        archive& make_nvp("d_Cus", d_Cus);                        //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
        archive& make_nvp("d_Toe1", d_Toe1);                      //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
        archive& make_nvp("d_Toe2", d_Toe2);                      //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
        archive& make_nvp("d_Toc", d_Toc);                        //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
        archive& make_nvp("d_Cic", d_Cic);                        //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
        archive& make_nvp("d_OMEGA0", d_OMEGA0);                  //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
        archive& make_nvp("d_Cis", d_Cis);                        //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
        archive& make_nvp("d_i_0", d_i_0);                        //!< Inclination Angle at Reference Time [semi-circles]
        archive& make_nvp("d_Crc", d_Crc);                        //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
        archive& make_nvp("d_OMEGA", d_OMEGA);                    //!< Argument of Perigee [semi-cicles]
        archive& make_nvp("d_IDOT", d_IDOT);                      //!< Rate of Inclination Angle [semi-circles/s]
        archive& make_nvp("i_GPS_week", i_GPS_week);              //!< GPS week number, aka WN [week]
        archive& make_nvp("d_TGD", d_TGD);                        //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("d_ISCL1", d_ISCL1);                    //!< Estimated Group Delay Differential: L1P(Y)-L1C/A correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("d_ISCL2", d_ISCL2);                    //!< Estimated Group Delay Differential: L1P(Y)-L2C correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("d_ISCL5I", d_ISCL5I);                  //!< Estimated Group Delay Differential: L1P(Y)-L5i correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive& make_nvp("d_ISCL5Q", d_ISCL5Q);                  //!< Estimated Group Delay Differential: L1P(Y)-L5q correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]

        archive& make_nvp("d_DELTA_A", d_DELTA_A);                  //!< Semi-major axis difference at reference time [m]
        archive& make_nvp("d_A_DOT", d_A_DOT);                      //!< Change rate in semi-major axis [m/s]
        archive& make_nvp("d_DELTA_OMEGA_DOT", d_DELTA_OMEGA_DOT);  //!< Rate of Right Ascension  difference [semi-circles/s]
        archive& make_nvp("d_A_f0", d_A_f0);                        //!< Coefficient 0 of code phase offset model [s]
        archive& make_nvp("d_A_f1", d_A_f1);                        //!< Coefficient 1 of code phase offset model [s/s]
        archive& make_nvp("d_A_f2", d_A_f2);                        //!< Coefficient 2 of code phase offset model [s/s^2]

        archive& make_nvp("b_integrity_status_flag", b_integrity_status_flag);
        archive& make_nvp("b_alert_flag", b_alert_flag);                //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
        archive& make_nvp("b_antispoofing_flag", b_antispoofing_flag);  //!<  If true, the AntiSpoofing mode is ON in that SV
    }

    /*!
     * \brief Compute the ECEF SV coordinates and ECEF velocity
     * Implementation of Table 20-IV (IS-GPS-200E)
     */
    double satellitePosition(double transmitTime);

    /*!
     * \brief Sets (\a d_satClkDrift)and returns the clock drift in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_drift(double transmitTime);

    /*!
     * \brief Sets (\a d_dtr) and returns the clock relativistic correction term in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_relativistic_term(double transmitTime);
    /*!
     * Default constructor
     */
    Gps_CNAV_Ephemeris();
};

#endif

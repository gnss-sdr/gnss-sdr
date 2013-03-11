/*!
 * \file gps_navigation_message.h
 * \brief  Interface of a GPS EPHEMERIS storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GPS_EPHEMERIS_H_
#define GNSS_SDR_GPS_EPHEMERIS_H_

#include <iostream>
#include <map>
#include "boost/assign.hpp"

#include "GPS_L1_CA.h"


/*!
 * \brief This class is a storage for the GPS SV ephemeris data as described in IS-GPS-200E
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 */
class Gps_Ephemeris
{
private:

public:

    unsigned int i_satellite_PRN; // SV PRN NUMBER
    double d_TOW; //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    double d_Crs;            //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n;        //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0;            //!< Mean Anomaly at Reference Time [semi-circles]
    double d_Cuc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_e_eccentricity; //!< Eccentricity [dimensionless]
    double d_Cus;            //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A;         //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double d_Toe;            //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
    double d_Toc;            //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
    double d_Cic;            //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0;         //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis;            //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_i_0;            //!< Inclination Angle at Reference Time [semi-circles]
    double d_Crc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA;          //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;      //!< Rate of Right Ascension [semi-circles/s]
    double d_IDOT;           //!< Rate of Inclination Angle [semi-circles/s]
    int i_code_on_L2;        //!< If 1, P code ON in L2;  if 2, C/A code ON in L2;
    int i_GPS_week;          //!< GPS week number, aka WN [week]
    bool b_L2_P_data_flag;   //!< When true, indicates that the NAV data stream was commanded OFF on the P-code of the L2 channel
    int i_SV_accuracy;       //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200E)
    int i_SV_health;
    double d_TGD;            //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
    double d_IODC;           //!< Issue of Data, Clock
    int i_AODO;              //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

    bool b_fit_interval_flag;//!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double d_spare1;
    double d_spare2;

    double d_A_f0;          //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1;          //!< Coefficient 1 of code phase offset model [s/s]
    double d_A_f2;          //!< Coefficient 2 of code phase offset model [s/s^2]


    // Flags

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
    bool b_alert_flag;      //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    bool b_antispoofing_flag;  //!<  If true, the AntiSpoofing mode is ON in that SV


    /*!
     * Default constructor
     */
    Gps_Ephemeris();
};

#endif

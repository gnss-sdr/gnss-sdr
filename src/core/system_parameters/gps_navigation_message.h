/*!
 * \file gps_navigation_message.h
 * \brief  Interface of a GPS NAV Data message decoder
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GPS_NAVIGATION_MESSAGE_H
#define GNSS_SDR_GPS_NAVIGATION_MESSAGE_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <bitset>
#include "boost/assign.hpp"
#include <math.h>
#include "GPS_L1_CA.h"

//using namespace boost::assign;



/*!
 * \brief This class decodes a GPS NAV Data message as described in IS-GPS-200E
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 */
class gps_navigation_message
{

private:
  unsigned long int read_navigation_unsigned(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices);
  signed long int read_navigation_signed(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices);
  bool read_navigation_bool(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices);
  void print_gps_word_bytes(unsigned int GPS_word);

  /*
   * Accounts for the beginning or end of week crossover
   *
   * See paragraph 20.3.3.3.3.1 (IS-GPS-200E)
   * \param[in]  -  time in seconds
   * \param[out] -  corrected time, in seconds
   */
  double check_t(double time);

public:
    //broadcast orbit 1
    double d_TOW;
    double d_IODE_SF2;
    double d_IODE_SF3;
    double d_Crs;            //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n;        //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0;            //!< Mean Anomaly at Reference Time [semi-circles]
    //broadcast orbit 2
    double d_Cuc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_e_eccentricity; //!< Eccentricity [dimensionless]
    double d_Cus;            //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A;         //!< Square Root of the Semi-Major Axis [sqrt(m)]
    //broadcast orbit 3
    double d_Toe;            //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
    double d_Toc;            //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
    double d_Cic;            //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0;         //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis;            //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    //broadcast orbit 4
    double d_i_0;            //!< Inclination Angle at Reference Time [semi-circles]
    double d_Crc;            //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA;          //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;      //!< Rate of Right Ascension [semi-circles/s]
    //broadcast orbit 5
    double d_IDOT;           //!< Rate of Inclination Angle [semi-circles/s]
    double d_codes_on_L2;    //!<
    double d_GPS_week;       //!< GPS week number, aka WN [week]
    bool   b_L2_P_data_flag; //!< When true, indicates that the NAV data stream was commanded OFF on the P-code of the L2 channel
    //broadcast orbit 6
    double d_SV_accuracy;    //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200E)
    double d_SV_health;
    double d_TGD;            //!< Estimated Group Delay Differential: L1-L2 correction term for the benefit of "L1 only" or "L2 only" users [s]
    double d_IODC;           //!< Issue of Data, Clock
    //broadcast orbit 7

    double d_fit_interval;
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


    // clock terms
    double d_master_clock;   // GPS transmission time
    double d_satClkCorr;     // GPS clock error
    double d_dtr;            // relativistic clock correction term

    // satellite positions
    double d_satpos_X;       //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;       //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;       //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // satellite identification info

    int d_channel_ID;
    int d_satellite_PRN;

    // time synchro
    double d_subframe1_timestamp_ms; //[ms]

    // Ionospheric parameters

    double d_alpha0;      //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;      //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;      //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;      //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;       //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;       //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;       //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;       //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    // UTC parameters

    double d_A1;          //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s/s]
    double d_A0;          //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200E) [s]
    double d_t_OT;        //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200E) [s]
    double d_WN_T;        //!< UTC reference week number [weeks]
    double d_DeltaT_LS;   //!< delta time due to leap seconds [s]
    double d_WN_LSF;      //!< Week number at the end of which the leap second becomes effective [weeks]
    double d_DN;          //!< Day number (DN) at the end of which the leap second becomes effective [days]
    double d_DeltaT_LSF;  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]


    // public functions
    void reset();

    /*!
     * \brief Decodes the GPS NAV message
     */
    int subframe_decoder(char *subframe);

    /*!
     * \brief User Algorithm for SV Clock Correction
     *
     * Implementation of paragraph 20.3.3.3.3.1 (IS-GPS-200E)
     */
    void master_clock(double transmitTime);

    /*!
     * \brief Computes the position of the satellite
     *
     * Implementation of Table 20-IV (IS-GPS-200E)
     */
    void satpos();
    void relativistic_clock_correction(double transmitTime);
    bool satellite_validation();

    /*!
     * Default constructor
     */
    gps_navigation_message();
};

#endif

/*!
 * \file galileo_navigation_message.h
 * \brief  Interface of a Galileo EPHEMERIS storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
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


#ifndef GNSS_SDR_GALILEO_EPHEMERIS_H_
#define GNSS_SDR_GALILEO_EPHEMERIS_H_

#include <iostream>
#include <map>
#include "boost/assign.hpp"
#include <boost/serialization/nvp.hpp>
#include "Galileo_E1.h"

/*!
 * \brief This class is a storage and orbital model functions for the Galileo SV ephemeris data as described in Galileo ICD paragraph 5.1.1
 *
 */
class Galileo_Ephemeris
{
public:
    /* Galileo ephemeris are 16 parameters and here are reported following the ICD order, paragraph 5.1.1.
       The number in the name after underscore (_1, _2, _3 and so on) refers to the page were we can find that parameter */
    bool flag_all_ephemeris;
    int IOD_ephemeris;
    int IOD_nav_1;
    int SV_ID_PRN_4;
    double M0_1;        //!< Mean anomaly at reference time [semi-circles]
    double delta_n_3;   //!< Mean motion difference from computed value [semi-circles/sec]
    double e_1;         //!< Eccentricity
    double A_1;         //!< Square root of the semi-major axis [metres^1/2]
    double OMEGA_0_2;   //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    double i_0_2;       //!< Inclination angle at reference time  [semi-circles]
    double omega_2;     //!< Argument of perigee [semi-circles]
    double OMEGA_dot_3; //!< Rate of right ascension [semi-circles/sec]
    double iDot_2;      //!< Rate of inclination angle [semi-circles/sec]
    double C_uc_3;      //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    double C_us_3;      //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    double C_rc_3;      //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    double C_rs_3;      //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
    double C_ic_4;      //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    double C_is_4;      //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    double t0e_1;       //!< Ephemeris reference time [s]

    /*Clock correction parameters*/
    double t0c_4;       //!< Clock correction data reference Time of Week [sec]
    double af0_4;       //!< SV clock bias correction coefficient [s]
    double af1_4;       //!< SV clock drift correction coefficient [s/s]
    double af2_4;       //!< SV clock drift rate correction coefficient [s/s^2]

    /*GST*/
    //Not belong to ephemeris set (page 1 to 4)
    double WN_5;        //!< Week number
    double TOW_5;       //!< Time of Week
    double Galileo_satClkDrift;
    double Galileo_dtr; //!< relativistic clock correction term

    // satellite positions
    double d_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // Satellite velocity
    double d_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]

    unsigned int i_satellite_PRN; //!< SV PRN NUMBER

    void satellitePosition(double transmitTime);            //!< Computes the ECEF SV coordinates and ECEF velocity
    double Galileo_System_Time(double WN, double TOW);      //!< Galileo System Time (GST), ICD paragraph 5.1.2
    double sv_clock_drift(double transmitTime);             //!< Satellite Time Correction Algorithm, ICD 5.1.4
    double sv_clock_relativistic_term(double transmitTime); //!< Satellite Time Correction Algorithm, ICD 5.1.4
    Galileo_Ephemeris();

    /*
    template<class Archive>

     \\brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.

    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;

        archive & make_nvp("i_satellite_PRN",i_satellite_PRN); // SV PRN NUMBER
        archive & make_nvp("d_TOW",d_TOW);         //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
        archive & make_nvp("d_Crs",d_Crs);         //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
        archive & make_nvp("d_Delta_n",d_Delta_n); //!< Mean Motion Difference From Computed Value [semi-circles/s]
        archive & make_nvp("d_M_0",d_M_0);         //!< Mean Anomaly at Reference Time [semi-circles]
        archive & make_nvp("d_Cuc",d_Cuc);         //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
        archive & make_nvp("d_e_eccentricity",d_e_eccentricity); //!< Eccentricity [dimensionless]
        archive & make_nvp("d_Cus",d_Cus);         //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
        archive & make_nvp("d_sqrt_A",d_sqrt_A);   //!< Square Root of the Semi-Major Axis [sqrt(m)]
        archive & make_nvp("d_Toe",d_Toe);         //!< Ephemeris data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200E) [s]
        archive & make_nvp("d_Toc",d_Toe);         //!< clock data reference time (Ref. 20.3.3.3.3.1 IS-GPS-200E) [s]
        archive & make_nvp("d_Cic",d_Cic);         //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
        archive & make_nvp("d_OMEGA0",d_OMEGA0);   //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
        archive & make_nvp("d_Cis",d_Cis);         //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
        archive & make_nvp("d_i_0",d_i_0);         //!< Inclination Angle at Reference Time [semi-circles]
        archive & make_nvp("d_Crc",d_Crc);         //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
        archive & make_nvp("d_OMEGA",d_OMEGA);     //!< Argument of Perigee [semi-cicles]
        archive & make_nvp("d_OMEGA_DOT",d_OMEGA_DOT);      //!< Rate of Right Ascension [semi-circles/s]
        archive & make_nvp("d_IDOT",d_IDOT);       //!< Rate of Inclination Angle [semi-circles/s]
        archive & make_nvp("i_code_on_L2",i_code_on_L2);        //!< If 1, P code ON in L2;  if 2, C/A code ON in L2;
        archive & make_nvp("i_GPS_week",i_GPS_week);          //!< GPS week number, aka WN [week]
        archive & make_nvp("b_L2_P_data_flag",b_L2_P_data_flag);   //!< When true, indicates that the NAV data stream was commanded OFF on the P-code of the L2 channel
        archive & make_nvp("i_SV_accuracy",i_SV_accuracy);       //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1) for the standard positioning service user (Ref 20.3.3.3.1.3 IS-GPS-200E)
        archive & make_nvp("i_SV_health",i_SV_health);
        archive & make_nvp("d_TGD",d_TGD);         //!< Estimated Group Delay Differential: L1-L2 correction term only for the benefit of "L1 P(Y)" or "L2 P(Y)" s users [s]
        archive & make_nvp("d_IODC",d_IODC);       //!< Issue of Data, Clock
        archive & make_nvp("i_AODO",i_AODO);       //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]

        archive & make_nvp("b_fit_interval_flag",b_fit_interval_flag);//!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
        archive & make_nvp("d_spare1",d_spare1);
        archive & make_nvp("d_spare2",d_spare2);

        archive & make_nvp("d_A_f0",d_A_f0);       //!< Coefficient 0 of code phase offset model [s]
        archive & make_nvp("d_A_f1",d_A_f1);       //!< Coefficient 1 of code phase offset model [s/s]
        archive & make_nvp("d_A_f2",d_A_f2);       //!< Coefficient 2 of code phase offset model [s/s^2]

        archive & make_nvp("b_integrity_status_flag",b_integrity_status_flag);
        archive & make_nvp("b_alert_flag",b_alert_flag);     //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
        archive & make_nvp("b_antispoofing_flag",b_antispoofing_flag); //!<  If true, the AntiSpoofing mode is ON in that SV
    }
     */
};

#endif

/*!
 * \file galileo_ephemeris.h
 * \brief  Interface of a Galileo EPHEMERIS storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es,
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
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


#ifndef GNSS_SDR_GALILEO_EPHEMERIS_H_
#define GNSS_SDR_GALILEO_EPHEMERIS_H_

#include <boost/assign.hpp>
#include <boost/serialization/nvp.hpp>
#include <cstdint>


/*!
 * \brief This class is a storage and orbital model functions for the Galileo SV ephemeris data as described in Galileo ICD paragraph 5.1.1
 *  (See https://www.gsc-europa.eu/system/files/galileo_documents/Galileo_OS_SIS_ICD.pdf )
 *
 */
class Galileo_Ephemeris
{
public:
    /* Galileo ephemeris are 16 parameters and here are reported following the ICD order, paragraph 5.1.1.
       The number in the name after underscore (_1, _2, _3 and so on) refers to the page were we can find that parameter */
    bool flag_all_ephemeris;
    int32_t IOD_ephemeris;
    int32_t IOD_nav_1;
    int32_t SV_ID_PRN_4;
    double M0_1;         //!< Mean anomaly at reference time [semi-circles]
    double delta_n_3;    //!< Mean motion difference from computed value [semi-circles/sec]
    double e_1;          //!< Eccentricity
    double A_1;          //!< Square root of the semi-major axis [meters^1/2]
    double OMEGA_0_2;    //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    double i_0_2;        //!< Inclination angle at reference time  [semi-circles]
    double omega_2;      //!< Argument of perigee [semi-circles]
    double OMEGA_dot_3;  //!< Rate of right ascension [semi-circles/sec]
    double iDot_2;       //!< Rate of inclination angle [semi-circles/sec]
    double C_uc_3;       //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    double C_us_3;       //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    double C_rc_3;       //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    double C_rs_3;       //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
    double C_ic_4;       //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    double C_is_4;       //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    double t0e_1;        //!< Ephemeris reference time [s]

    /*Clock correction parameters*/
    double t0c_4;  //!< Clock correction data reference Time of Week [sec]
    double af0_4;  //!< SV clock bias correction coefficient [s]
    double af1_4;  //!< SV clock drift correction coefficient [s/s]
    double af2_4;  //!< SV clock drift rate correction coefficient [s/s^2]

    /*GST*/
    //Not belong to ephemeris set (page 1 to 4)
    double WN_5;   //!< Week number
    double TOW_5;  //!< Time of Week
    double Galileo_satClkDrift;
    double Galileo_dtr;  //!< relativistic clock correction term

    // SV status
    double SISA_3;
    uint32_t E5a_HS;   //!< E5a Signal Health Status
    double E5b_HS_5;   //!< E5b Signal Health Status
    double E1B_HS_5;   //!< E1B Signal Health Status
    bool E5a_DVS;      //!< E5a Data Validity Status
    double E5b_DVS_5;  //!< E5b Data Validity Status
    double E1B_DVS_5;  //!< E1B Data Validity Status

    double BGD_E1E5a_5;  //!< E1-E5a Broadcast Group Delay [s]
    double BGD_E1E5b_5;  //!< E1-E5b Broadcast Group Delay [s]

    // satellite positions
    double d_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // Satellite velocity
    double d_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]

    uint32_t i_satellite_PRN;  //!< SV PRN NUMBER

    void satellitePosition(double transmitTime);             //!< Computes the ECEF SV coordinates and ECEF velocity
    double Galileo_System_Time(double WN, double TOW);       //!< Galileo System Time (GST), ICD paragraph 5.1.2
    double sv_clock_drift(double transmitTime);              //!< Satellite Time Correction Algorithm, ICD 5.1.4
    double sv_clock_relativistic_term(double transmitTime);  //!< Satellite Time Correction Algorithm, ICD 5.1.4
    Galileo_Ephemeris();

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        if (version)
            {
            };

        archive& BOOST_SERIALIZATION_NVP(flag_all_ephemeris);
        archive& BOOST_SERIALIZATION_NVP(IOD_ephemeris);
        archive& BOOST_SERIALIZATION_NVP(IOD_nav_1);
        archive& BOOST_SERIALIZATION_NVP(SV_ID_PRN_4);

        archive& BOOST_SERIALIZATION_NVP(M0_1);
        archive& BOOST_SERIALIZATION_NVP(delta_n_3);
        archive& BOOST_SERIALIZATION_NVP(e_1);
        archive& BOOST_SERIALIZATION_NVP(A_1);
        archive& BOOST_SERIALIZATION_NVP(OMEGA_0_2);
        archive& BOOST_SERIALIZATION_NVP(i_0_2);
        archive& BOOST_SERIALIZATION_NVP(omega_2);
        archive& BOOST_SERIALIZATION_NVP(OMEGA_dot_3);
        archive& BOOST_SERIALIZATION_NVP(iDot_2);
        archive& BOOST_SERIALIZATION_NVP(C_uc_3);
        archive& BOOST_SERIALIZATION_NVP(C_us_3);
        archive& BOOST_SERIALIZATION_NVP(C_rc_3);
        archive& BOOST_SERIALIZATION_NVP(C_rs_3);
        archive& BOOST_SERIALIZATION_NVP(C_ic_4);
        archive& BOOST_SERIALIZATION_NVP(C_is_4);
        archive& BOOST_SERIALIZATION_NVP(t0e_1);

        archive& BOOST_SERIALIZATION_NVP(t0c_4);
        archive& BOOST_SERIALIZATION_NVP(af0_4);
        archive& BOOST_SERIALIZATION_NVP(af1_4);
        archive& BOOST_SERIALIZATION_NVP(af2_4);

        archive& BOOST_SERIALIZATION_NVP(WN_5);
        archive& BOOST_SERIALIZATION_NVP(TOW_5);
        archive& BOOST_SERIALIZATION_NVP(Galileo_satClkDrift);
        archive& BOOST_SERIALIZATION_NVP(Galileo_dtr);

        archive& BOOST_SERIALIZATION_NVP(SISA_3);
        archive& BOOST_SERIALIZATION_NVP(E5a_HS);
        archive& BOOST_SERIALIZATION_NVP(E5b_HS_5);
        archive& BOOST_SERIALIZATION_NVP(E1B_HS_5);
        archive& BOOST_SERIALIZATION_NVP(E5a_DVS);
        archive& BOOST_SERIALIZATION_NVP(E5b_DVS_5);
        archive& BOOST_SERIALIZATION_NVP(E1B_DVS_5);

        archive& BOOST_SERIALIZATION_NVP(BGD_E1E5a_5);
        archive& BOOST_SERIALIZATION_NVP(BGD_E1E5b_5);

        archive& BOOST_SERIALIZATION_NVP(i_satellite_PRN);
    }
};

#endif

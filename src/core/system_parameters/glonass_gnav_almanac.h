/*!
 * \file glonass_gnav_almanac.h
 * \brief  Interface of a GLONASS GNAV ALMANAC storage
 * \author Damian Miralles, 2017. dmiralles2009@gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GLONASS_ALMANAC_H_
#define GNSS_SDR_GLONASS_ALMANAC_H_

#include <map>
#include <string>
#include "boost/assign.hpp"
#include <boost/serialization/nvp.hpp>

/*!
 * \brief This class is a storage for the GLONASS SV ALMANAC data as described GLONASS ICD (Edition 5.1)
 *
 * See http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf
 */
class Glonass_Gnav_Almanac
{
public:
    int i_satellite_freq_channel; //!< SV Frequency Channel NUMBER
    double d_tau_c;             //!< GLONASS time scale correction to UTC(SU) time. [s]
    double d_tau_gps;           //!< Correction to GPS time to GLONASS time [day]
    double d_N_4;               //!< Four year interval number starting from 1996 [4 year interval]
    double d_N_A;               //!< Calendar day number within the four-year period beginning since the leap year [days]
    double d_n_A;               //!< Conventional number of satellite within GLONASS space segment [dimensionless]
    double d_H_n_A;             //!< Carrier frequency number of navigation RF signal transmitted by d_nA satellite [dimensionless]
    double d_lambda_n_A;        //!< Longitude of the first (within the d_NA day) ascending node of d_nA  [semi-circles]
    double d_t_lambda_n_A;      //!< Time of first ascending node passage [s]
    double d_Delta_i_n_A;       //!< Correction of the mean value of inclination of d_n_A satellite at instant t_lambda_n_A [semi-circles]
    double d_Delta_T_n_A;       //!< Correction to the mean value of Draconian period of d_n_A satellite at instant t_lambda_n_A[s / orbital period]
    double d_Delta_T_n_A_dot;   //!< Rate of change of Draconian period of d_n_A satellite at instant t_lambda_n_A [s / orbital period^2]
    double d_epsilon_n_A;       //!< Eccentricity of d_n_A satellite at instant t_lambda_n_A [dimensionless]
    double d_omega_n_A;         //!< Argument of preigree of d_n_A satellite at instant t_lambdan_A [semi-circles]
    double d_M_n_A;             //!< Type of satellite n_A [dimensionless]
    double d_KP;                //!< Notification on forthcoming leap second correction of UTC [dimensionless]
    double d_tau_n_A;           //!< Coarse value of d_n_A satellite time correction to GLONASS time at instant  t_lambdan_A[s]
    double d_C_n;             //!< Generalized “unhealthy flag” of n_A satellite at instant of almanac upload [dimensionless]

    // satellite positions
    double d_satpos_Xo;        //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    double d_satpos_Yo;        //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    double d_satpos_Zo;        //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    // Satellite velocity
    double d_satvel_Xo;        //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Yo;        //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Zo;        //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]

    // satellite positions
    double d_satpos_X;        //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    double d_satpos_Y;        //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    double d_satpos_Z;        //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    // Satellite velocity
    double d_satvel_X;        //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Y;        //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    double d_satvel_Z;        //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]

    template<class Archive>
    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the almanac data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if(version){};

        archive & make_nvp("i_satellite_freq_channel", i_satellite_freq_channel);
        archive & make_nvp("d_tau_c", d_tau_c);
        archive & make_nvp("d_tau_gps", d_tau_gps);
        archive & make_nvp("d_N_4", d_N_4);
        archive & make_nvp("d_N_A", d_N_A);
        archive & make_nvp("d_n_A", d_n_A);
        archive & make_nvp("d_H_n_A", d_H_n_A);
        archive & make_nvp("d_lambda_n_A", d_lambda_n_A);
        archive & make_nvp("d_t_lambda_n_A", d_t_lambda_n_A);
        archive & make_nvp("d_Delta_i_n_A", d_Delta_i_n_A);
        archive & make_nvp("d_Delta_T_n_A", d_Delta_T_n_A);
        archive & make_nvp("d_Delta_T_n_A_dot", d_Delta_T_n_A_dot);
        archive & make_nvp("d_epsilon_n_A", d_epsilon_n_A);
        archive & make_nvp("d_omega_n_A", d_omega_n_A);
        archive & make_nvp("d_M_n_A", d_M_n_A);
        archive & make_nvp("d_KP", d_KP);
        archive & make_nvp("d_tau_n_A", d_tau_n_A);
        archive & make_nvp("d_C_n", d_C_n);
    }

    void satellite_position(double N_i, double t_i);
    /*!
     * Default constructor
     */
    Glonass_Gnav_Almanac();
};

#endif

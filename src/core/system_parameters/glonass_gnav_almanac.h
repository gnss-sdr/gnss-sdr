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


/*!
 * \brief This class is a storage for the GLONASS SV ALMANAC data as described in IS-GPS-200E
 * \todo Add proper links for GLONASS ICD
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 */
class Glonass_Gnav_Almanac
{
public:
    int i_satellite_freq_channel; //!< SV PRN NUMBER
    double d_tau_c;             //!< GLONASS time scale correction to UTC(SU) time. [s]
    double d_tau_gps;           //!< Correction to GPS time to GLONASS time [day]
    double d_N_4;               //!< Four year interval number starting from 1996 [4 year interval]
    double d_N_A;               //!< Calendar day number within the four-year period beginning since the leap year [days]
    double d_n_A;               //!< Conventional number of satellite within GLONASS space segment [dimensionless]
    double d_H_n_A;             //!< Carrier frequency number of navigation RF signal transmitted by d_nA satellite [dimensionless]
    double d_lambda_n_A;        //!< Longitude of the first (within the d_NA day) ascending node of d_nA  [semi-circles]
    double d_t_lambda_n_A;      //!< Time of first ascending node passage [s]
    double d_Delta_i_n_A        //!< Correction of the mean value of inclination of d_n_A satellite at instant t_lambda_n_A [semi-circles]
    double d_Delta_T_n_A;       //!< Correction to the mean value of Draconian period of d_n_A satellite at instant t_lambda_n_A[s / orbital period]
    double d_Delta_T_n_A_dot;   //!< Rate of change of Draconian period of d_n_A satellite at instant t_lambda_n_A [s / orbital period^2]
    double d_epsilon_n_A;       //!< Eccentricity of d_n_A satellite at instant t_lambda_n_A [dimensionless]
    double d_omega_n_A;         //!< Argument of preigree of d_n_A satellite at instant t_lambdan_A [semi-circles]
    double d_M_n_A;             //!< Type of satellite n_A [dimensionless]
    double d_B1;                //!< Coefficient  to  determine DeltaUT1 [s]
    double d_B2;                //!< Coefficient  to  determine DeltaUT1 [s/msd]
    double d_KP;                //!< Notification on forthcoming leap second correction of UTC [dimensionless]
    double d_tau_n_A;           //!< Coarse value of d_n_A satellite time correction to GLONASS time at instant  t_lambdan_A[s]
    double d_C_n_A;             //!< Generalized “unhealthy flag” of n_A satellite at instant of almanac upload [dimensionless]
    /*!
     * Default constructor
     */
    Glonass_Gnav_Almanac();
};

#endif

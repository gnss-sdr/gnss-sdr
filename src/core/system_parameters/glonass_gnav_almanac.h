/*!
 * \file glonass_gnav_almanac.h
 * \brief  Interface of a GLONASS GNAV ALMANAC storage
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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


#ifndef GNSS_SDR_GLONASS_ALMANAC_H
#define GNSS_SDR_GLONASS_ALMANAC_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GLONASS SV ALMANAC data as described GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Almanac
{
public:
    /*!
     * Default constructor
     */
    Glonass_Gnav_Almanac() = default;

    double d_n_A{};              //!< Conventional number of satellite within GLONASS space segment [dimensionless]
    double d_H_n_A{};            //!< Carrier frequency number of navigation RF signal transmitted by d_nA satellite as table 4.10 (0-31) [dimensionless]
    double d_lambda_n_A{};       //!< Longitude of the first (within the d_NA day) ascending node of d_nA [radians]
    double d_t_lambda_n_A{};     //!< Time of first ascending node passage [s]
    double d_Delta_i_n_A{};      //!< Correction of the mean value of inclination of d_n_A satellite at instant t_lambda_n_A [radians]
    double d_Delta_T_n_A{};      //!< Correction to the mean value of Draconian period of d_n_A satellite at instant t_lambda_n_A [s / orbital period]
    double d_Delta_T_n_A_dot{};  //!< Rate of change of Draconian period of d_n_A satellite at instant t_lambda_n_A [s / orbital period^2]
    double d_epsilon_n_A{};      //!< Eccentricity of d_n_A satellite at instant t_lambda_n_A [dimensionless]
    double d_omega_n_A{};        //!< Argument of perigee of d_n_A satellite at instant t_lambdan_A [radians]
    double d_M_n_A{};            //!< Type of satellite n_A [dimensionless]
    double d_KP{};               //!< Notification on forthcoming leap second correction of UTC [dimensionless]
    double d_tau_n_A{};          //!< Coarse value of d_n_A satellite time correction to GLONASS time at instant  t_lambdan_A[s]
    bool d_C_n{};                //!< Generalized “unhealthy flag” of n_A satellite at instant of almanac upload [dimensionless]
    bool d_l_n{};                //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // Satellite Identification Information
    int32_t i_satellite_freq_channel{};  //!< SV Frequency Channel Number
    uint32_t i_satellite_PRN{};          //!< SV PRN Number, equivalent to slot number for compatibility with GPS
    uint32_t i_satellite_slot_number{};  //!< SV Slot Number

    template <class Archive>
    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the almanac data on disk file.
     */
    void serialize(Archive& archive, const uint32_t version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };

        archive& make_nvp("i_satellite_freq_channel", i_satellite_freq_channel);
        archive& make_nvp("i_satellite_PRN", i_satellite_PRN);
        archive& make_nvp("i_satellite_slot_number", i_satellite_slot_number);
        archive& make_nvp("d_n_A", d_n_A);
        archive& make_nvp("d_H_n_A", d_H_n_A);
        archive& make_nvp("d_lambda_n_A", d_lambda_n_A);
        archive& make_nvp("d_t_lambda_n_A", d_t_lambda_n_A);
        archive& make_nvp("d_Delta_i_n_A", d_Delta_i_n_A);
        archive& make_nvp("d_Delta_T_n_A", d_Delta_T_n_A);
        archive& make_nvp("d_Delta_T_n_A_dot", d_Delta_T_n_A_dot);
        archive& make_nvp("d_epsilon_n_A", d_epsilon_n_A);
        archive& make_nvp("d_omega_n_A", d_omega_n_A);
        archive& make_nvp("d_M_n_A", d_M_n_A);
        archive& make_nvp("d_KP", d_KP);
        archive& make_nvp("d_tau_n_A", d_tau_n_A);
        archive& make_nvp("d_C_n", d_C_n);
        archive& make_nvp("d_l_n", d_l_n);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_ALMANAC_H

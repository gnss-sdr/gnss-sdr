/*!
 * \file glonass_gnav_utc_model.h
 * \brief  Interface of a GLONASS GNAV UTC MODEL storage
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


#ifndef GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H
#define GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GLONASS GNAV UTC MODEL data as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Utc_Model
{
public:
    /*!
     * Default constructor
     */
    Glonass_Gnav_Utc_Model() = default;

    bool valid{};
    // Clock Parameters
    double d_tau_c{};    //!< GLONASS time scale correction to UTC(SU) time. [s]
    double d_tau_gps{};  //!< Correction to GPS time to GLONASS time [day]
    double d_N_4{};      //!< Four year interval number starting from 1996 [4 year interval]
    double d_N_A{};      //!< Calendar day number within the four-year period beginning since the leap year for Almanac data [days]
    double d_B1{};       //!< Coefficient  to  determine DeltaUT1 [s]
    double d_B2{};       //!< Coefficient  to  determine DeltaUT1 [s/msd]

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (GLONASS ICD (Edition 5.1) Section 3.3.3 GLONASS Time)
     */
    double utc_time(double glonass_time_corrected) const;

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
        archive& make_nvp("valid", valid);
        archive& make_nvp("d_tau_c", d_tau_c);
        archive& make_nvp("d_tau_gps", d_tau_gps);
        archive& make_nvp("d_N_4", d_N_4);
        archive& make_nvp("d_N_A", d_N_A);
        archive& make_nvp("d_B1", d_B1);
        archive& make_nvp("d_B2", d_B2);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H

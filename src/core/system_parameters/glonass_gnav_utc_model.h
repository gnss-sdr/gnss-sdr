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
    double d_KP{};       //!< Notification of forthcoming leap second correction of UTC (GLONASS ICD Table 4.12) [dimensionless]

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (GLONASS ICD (Edition 5.1) Section 3.3.3 GLONASS Time)
     */
    double utc_time(double glonass_time_corrected) const;

    /*!
     * \brief Returns the UTC leap second correction announced by the KP word
     * for the end of the current quarter: +1 s, -1 s, or 0 if no correction
     * is planned (GLONASS ICD Edition 5.1, Table 4.12)
     */
    inline int32_t announced_leap_second() const
    {
        if (static_cast<int32_t>(d_KP) == 1)
            {
                return 1;
            }
        if (static_cast<int32_t>(d_KP) == 3)
            {
                return -1;
            }
        return 0;
    }

    template <class Archive>
    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the almanac data on disk file.
     */
    void serialize(Archive& archive, const uint32_t version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(valid);
        archive& BOOST_SERIALIZATION_NVP(d_tau_c);
        archive& BOOST_SERIALIZATION_NVP(d_tau_gps);
        archive& BOOST_SERIALIZATION_NVP(d_N_4);
        archive& BOOST_SERIALIZATION_NVP(d_N_A);
        archive& BOOST_SERIALIZATION_NVP(d_B1);
        archive& BOOST_SERIALIZATION_NVP(d_B2);
        archive& BOOST_SERIALIZATION_NVP(d_KP);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H

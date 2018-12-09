/*!
 * \file glonass_gnav_utc_model.h
 * \brief  Interface of a GLONASS GNAV UTC MODEL storage
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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


#ifndef GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H_
#define GNSS_SDR_GLONASS_GNAV_UTC_MODEL_H_

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/*!
 * \brief This class is a storage for the GLONASS GNAV UTC MODEL data as described in GLONASS ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 */
class Glonass_Gnav_Utc_Model
{
public:
    bool valid;
    // Clock Parameters
    double d_tau_c;    //!< GLONASS time scale correction to UTC(SU) time. [s]
    double d_tau_gps;  //!< Correction to GPS time to GLONASS time [day]
    double d_N_4;      //!< Four year interval number starting from 1996 [4 year interval]
    double d_N_A;      //!< Calendar day number within the four-year period beginning since the leap year for Almanac data [days]
    double d_B1;       //!< Coefficient  to  determine DeltaUT1 [s]
    double d_B2;       //!< Coefficient  to  determine DeltaUT1 [s/msd]

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

    /*!
     * Default constructor
     */
    Glonass_Gnav_Utc_Model();

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] (GLONASS ICD (Edition 5.1) Section 3.3.3 GLONASS Time)
     */
    double utc_time(double glonass_time_corrected);
};

#endif

/*!
 * \file galileo_almanac.h
 * \brief  Interface of a Galileo ALMANAC storage
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.cat
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


#ifndef GNSS_SDR_GALILEO_ALMANAC_H_
#define GNSS_SDR_GALILEO_ALMANAC_H_

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/*!
 * \brief This class is a storage for the Galileo SV ALMANAC data
 */
class Galileo_Almanac
{
public:
    uint32_t i_satellite_PRN;  //!< SV PRN NUMBER
    double d_Toa;
    double d_WNa;
    double d_IODa;
    double d_Delta_i;
    double d_M_0;             //!< Mean Anomaly at Reference Time [semi-circles]
    double d_e_eccentricity;  //!< Eccentricity [dimensionless]
    double d_Delta_sqrt_A;    //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double d_OMEGA0;          //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_OMEGA;           //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;       //!< Rate of Right Ascension [semi-circles/s]
    double d_A_f0;            //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1;            //!< Coefficient 1 of code phase offset model [s/s]
    double E5b_HS;
    double E1B_HS;
    double E5a_HS;

    /*!
     * Default constructor
     */
    Galileo_Almanac();

    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        ar& BOOST_SERIALIZATION_NVP(i_satellite_PRN);
        ar& BOOST_SERIALIZATION_NVP(d_Toa);
        ar& BOOST_SERIALIZATION_NVP(d_WNa);
        ar& BOOST_SERIALIZATION_NVP(d_IODa);
        ar& BOOST_SERIALIZATION_NVP(d_Delta_i);
        ar& BOOST_SERIALIZATION_NVP(d_M_0);
        ar& BOOST_SERIALIZATION_NVP(d_e_eccentricity);
        ar& BOOST_SERIALIZATION_NVP(d_Delta_sqrt_A);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA0);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA_DOT);
        ar& BOOST_SERIALIZATION_NVP(d_A_f0);
        ar& BOOST_SERIALIZATION_NVP(d_A_f1);
        ar& BOOST_SERIALIZATION_NVP(E5b_HS);
        ar& BOOST_SERIALIZATION_NVP(E1B_HS);
        ar& BOOST_SERIALIZATION_NVP(E5a_HS);
    }
};

#endif

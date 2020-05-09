/*!
 * \file galileo_almanac.h
 * \brief  Interface of a Galileo ALMANAC storage
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.cat
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_ALMANAC_H
#define GNSS_SDR_GALILEO_ALMANAC_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/*!
 * \brief This class is a storage for the Galileo SV ALMANAC data
 */
class Galileo_Almanac
{
public:
    uint32_t i_satellite_PRN;  //!< SV PRN NUMBER
    int32_t i_Toa;
    int32_t i_WNa;
    int32_t i_IODa;
    double d_Delta_i;         //!< Inclination at reference time relative to i0 = 56º [semi-circles]
    double d_M_0;             //!< Mean Anomaly at Reference Time [semi-circles]
    double d_e_eccentricity;  //!< Eccentricity [dimensionless]
    double d_Delta_sqrt_A;    //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double d_OMEGA0;          //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_OMEGA;           //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;       //!< Rate of Right Ascension [semi-circles/s]
    double d_A_f0;            //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1;            //!< Coefficient 1 of code phase offset model [s/s]
    int32_t E5b_HS;
    int32_t E1B_HS;
    int32_t E5a_HS;

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
        ar& BOOST_SERIALIZATION_NVP(i_Toa);
        ar& BOOST_SERIALIZATION_NVP(i_WNa);
        ar& BOOST_SERIALIZATION_NVP(i_IODa);
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

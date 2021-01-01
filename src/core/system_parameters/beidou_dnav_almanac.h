/*!
 * \file beidou_dnav_almanac.h
 * \brief  Interface of a Beidou DNAV Almanac storage
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_DNAV_ALMANAC_H
#define GNSS_SDR_BEIDOU_DNAV_ALMANAC_H

#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the BeiDou D1 almanac
 */
class Beidou_Dnav_Almanac
{
public:
    /*!
     * Default constructor
     */
    Beidou_Dnav_Almanac() = default;

    unsigned int i_satellite_PRN{};  //!< SV PRN NUMBER
    double d_Delta_i{};
    double d_Toa{};             //!< Almanac data reference time of week [s]
    double d_M_0{};             //!< Mean Anomaly at Reference Time [semi-circles]
    double d_e_eccentricity{};  //!< Eccentricity [dimensionless]
    double d_sqrt_A{};          //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double d_OMEGA0{};          //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_OMEGA{};           //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT{};       //!< Rate of Right Ascension [semi-circles/s]
    int i_SV_health{};          //!< SV Health
    double d_A_f0{};            //!< Coefficient 0 of code phase offset model [s]
    double d_A_f1{};            //!< Coefficient 1 of code phase offset model [s/s]

    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        ar& BOOST_SERIALIZATION_NVP(i_satellite_PRN);
        ar& BOOST_SERIALIZATION_NVP(d_Delta_i);
        ar& BOOST_SERIALIZATION_NVP(d_Toa);
        // ar& BOOST_SERIALIZATION_NVP(i_WNa);
        ar& BOOST_SERIALIZATION_NVP(d_M_0);
        ar& BOOST_SERIALIZATION_NVP(d_e_eccentricity);
        ar& BOOST_SERIALIZATION_NVP(d_sqrt_A);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA0);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA);
        ar& BOOST_SERIALIZATION_NVP(d_OMEGA_DOT);
        ar& BOOST_SERIALIZATION_NVP(i_SV_health);
        // ar& BOOST_SERIALIZATION_NVP(i_AS_status);
        ar& BOOST_SERIALIZATION_NVP(d_A_f0);
        ar& BOOST_SERIALIZATION_NVP(d_A_f1);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_ALMANAC_H

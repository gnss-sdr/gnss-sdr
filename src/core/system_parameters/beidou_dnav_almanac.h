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

    unsigned int PRN{};  //!< SV PRN NUMBER
    double delta_i{};
    double toa{};       //!< Almanac data reference time of week [s]
    double M_0{};       //!< Mean Anomaly at Reference Time [semi-circles]
    double ecc{};       //!< Eccentricity [dimensionless]
    double sqrtA{};     //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double OMEGA_0{};   //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double omega{};     //!< Argument of Perigee [semi-cicles]
    double OMEGAdot{};  //!< Rate of Right Ascension [semi-circles/s]
    int SV_health{};    //!< SV Health
    double af0{};       //!< Coefficient 0 of code phase offset model [s]
    double af1{};       //!< Coefficient 1 of code phase offset model [s/s]

    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        ar& BOOST_SERIALIZATION_NVP(PRN);
        ar& BOOST_SERIALIZATION_NVP(delta_i);
        ar& BOOST_SERIALIZATION_NVP(toa);
        // ar& BOOST_SERIALIZATION_NVP(WNa);
        ar& BOOST_SERIALIZATION_NVP(M_0);
        ar& BOOST_SERIALIZATION_NVP(ecc);
        ar& BOOST_SERIALIZATION_NVP(sqrtA);
        ar& BOOST_SERIALIZATION_NVP(OMEGA_0);
        ar& BOOST_SERIALIZATION_NVP(omega);
        ar& BOOST_SERIALIZATION_NVP(OMEGAdot);
        ar& BOOST_SERIALIZATION_NVP(SV_health);
        // ar& BOOST_SERIALIZATION_NVP(AS_status);
        ar& BOOST_SERIALIZATION_NVP(af0);
        ar& BOOST_SERIALIZATION_NVP(af1);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_ALMANAC_H

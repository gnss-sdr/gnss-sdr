/*!
 * \file gps_almanac.h
 * \brief  Interface of a GPS ALMANAC storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_ALMANAC_H
#define GNSS_SDR_GPS_ALMANAC_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS SV ALMANAC data as described in IS-GPS-200L
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200L.pdf Appendix II
 */
class Gps_Almanac
{
public:
    /*!
     * Default constructor
     */
    Gps_Almanac() = default;

    uint32_t PRN{};       //!< SV PRN NUMBER
    double delta_i{};     //!< Inclination Angle at Reference Time (relative to i_0 = 0.30 semi-circles)
    int32_t toa{};        //!< Almanac data reference time of week (Ref. 20.3.3.4.3 IS-GPS-200L) [s]
    int32_t WNa{};        //!< Almanac week number
    double M_0{};         //!< Mean Anomaly at Reference Time [semi-circles]
    double ecc{};         //!< Eccentricity [dimensionless]
    double sqrtA{};       //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double OMEGA_0{};     //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double omega{};       //!< Argument of Perigee [semi-cicles]
    double OMEGAdot{};    //!< Rate of Right Ascension [semi-circles/s]
    int32_t SV_health{};  //!< SV Health
    int32_t AS_status{};  //!< Anti-Spoofing Flags and SV Configuration
    double af0{};         //!< Coefficient 0 of code phase offset model [s]
    double af1{};         //!< Coefficient 1 of code phase offset model [s/s]

    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        ar& BOOST_SERIALIZATION_NVP(PRN);
        ar& BOOST_SERIALIZATION_NVP(delta_i);
        ar& BOOST_SERIALIZATION_NVP(toa);
        ar& BOOST_SERIALIZATION_NVP(WNa);
        ar& BOOST_SERIALIZATION_NVP(M_0);
        ar& BOOST_SERIALIZATION_NVP(ecc);
        ar& BOOST_SERIALIZATION_NVP(sqrtA);
        ar& BOOST_SERIALIZATION_NVP(OMEGA_0);
        ar& BOOST_SERIALIZATION_NVP(omega);
        ar& BOOST_SERIALIZATION_NVP(OMEGAdot);
        ar& BOOST_SERIALIZATION_NVP(SV_health);
        ar& BOOST_SERIALIZATION_NVP(AS_status);
        ar& BOOST_SERIALIZATION_NVP(af0);
        ar& BOOST_SERIALIZATION_NVP(af1);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_ALMANAC_H

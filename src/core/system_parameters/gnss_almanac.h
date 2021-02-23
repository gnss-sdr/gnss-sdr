/*!
 * \file gnss_almanac.h
 * \brief Base class for GNSS almanac storage
 * \author Carles Fernandez, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_ALMANAC_H
#define GNSS_SDR_GNSS_ALMANAC_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief Base class for GNSS almanac storage
 */
class Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Gnss_Almanac() = default;

    uint32_t PRN{};     //!< SV PRN NUMBER
    double delta_i{};   //!< Inclination Angle at Reference Time (relative to i_0 = 0.30 semi-circles)
    int32_t toa{};      //!< Almanac data reference time of week [s]
    int32_t WNa{};      //!< Almanac week number
    double M_0{};       //!< Mean Anomaly at Reference Time [semi-circles]
    double ecc{};       //!< Eccentricity [dimensionless]
    double sqrtA{};     //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double OMEGA_0{};   //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double omega{};     //!< Argument of Perigee [semi-cicles]
    double OMEGAdot{};  //!< Rate of Right Ascension [semi-circles/s]
    double af0{};       //!< Coefficient 0 of code phase offset model [s]
    double af1{};       //!< Coefficient 1 of code phase offset model [s/s]
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_ALMANAC_H

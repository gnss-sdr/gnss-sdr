/*!
 * \file pvt_interface.h
 * \brief This class represents an interface to a PVT block.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Abstract class for PVT solvers. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
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


#ifndef GNSS_SDR_PVT_INTERFACE_H
#define GNSS_SDR_PVT_INTERFACE_H

#include "galileo_almanac.h"
#include "galileo_ephemeris.h"
#include "gnss_block_interface.h"
#include "gps_almanac.h"
#include "gps_ephemeris.h"
#include <map>

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */


/*!
 * \brief This class represents an interface to a PVT block.
 *
 * Abstract class for PVT interfaces, derived from GNSSBlockInterface.
 * Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class PvtInterface : public GNSSBlockInterface
{
public:
    /*!
     * \brief Resets the PVT block state.
     */
    virtual void reset() = 0;

    /*!
     * \brief Clears all stored ephemeris data.
     */
    virtual void clear_ephemeris() = 0;

    /*!
     * \brief Returns a map of GPS ephemeris data keyed by PRN.
     * \return Map of PRN to Gps_Ephemeris objects.
     */
    virtual std::map<int, Gps_Ephemeris> get_gps_ephemeris() const = 0;

    /*!
     * \brief Returns a map of Galileo ephemeris data keyed by PRN.
     * \return Map of PRN to Galileo_Ephemeris objects.
     */
    virtual std::map<int, Galileo_Ephemeris> get_galileo_ephemeris() const = 0;

    /*!
     * \brief Returns a map of GPS almanac data keyed by PRN.
     * \return Map of PRN to Gps_Almanac objects.
     */
    virtual std::map<int, Gps_Almanac> get_gps_almanac() const = 0;

    /*!
     * \brief Returns a map of Galileo almanac data keyed by PRN.
     * \return Map of PRN to Galileo_Almanac objects.
     */
    virtual std::map<int, Galileo_Almanac> get_galileo_almanac() const = 0;

    /*!
     * \brief Retrieves the latest computed PVT solution.
     * \param[out] longitude_deg Longitude in degrees.
     * \param[out] latitude_deg Latitude in degrees.
     * \param[out] height_m Height above ellipsoid in meters.
     * \param[out] ground_speed_kmh Ground speed in km/h.
     * \param[out] course_over_ground_deg Course over ground in degrees.
     * \param[out] UTC_time UTC time as a time_t value.
     * \return True if a valid PVT solution is available, false otherwise.
     */
    virtual bool get_latest_PVT(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        double* ground_speed_kmh,
        double* course_over_ground_deg,
        time_t* UTC_time) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_INTERFACE_H

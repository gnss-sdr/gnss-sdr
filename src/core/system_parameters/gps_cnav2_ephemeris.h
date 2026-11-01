/*!
 * \file gps_cnav2_ephemeris.h
 * \brief Interface of a GPS CNAV-2 ephemeris storage
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_CNAV2_EPHEMERIS_H
#define GNSS_SDR_GPS_CNAV2_EPHEMERIS_H

#include "gps_cnav_ephemeris.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This is a storage class for the GPS CNAV-2 ephemeris data broadcast
 * on L1C, as described in IS-GPS-800J, Section 3.5.3.
 *
 * Orbit and clock parameters share the CNAV definitions, with toc = toe.
 * The ISCL1, ISCL2, ISCL5I and ISCL5Q members inherited from
 * Gps_CNAV_Ephemeris hold the ISC_L1C/A, ISC_L2C, ISC_L5I5 and ISC_L5Q5
 * values of subframe 3, page 1, when available.
 */
class Gps_CNAV2_Ephemeris : public Gps_CNAV_Ephemeris
{
public:
    Gps_CNAV2_Ephemeris() = default;

    double ISCL1CP{};       //!< Inter-signal correction between L1 P(Y) and L1CP [s]
    double ISCL1CD{};       //!< Inter-signal correction between L1 P(Y) and L1CD [s]
    int32_t ITOW{};         //!< Number of two-hour epochs since the start of the week
    int32_t l1c_health{};   //!< L1C signal health (0 = signal OK, 1 = signal bad or unavailable)
    bool sf3_page1_iscs{};  //!< True if ISCL1, ISCL2, ISCL5I and ISCL5Q come from subframe 3, page 1

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t /* version */)
    {
        archive& boost::serialization::make_nvp("Gps_CNAV_Ephemeris", boost::serialization::base_object<Gps_CNAV_Ephemeris>(*this));
        archive& BOOST_SERIALIZATION_NVP(ISCL1CP);
        archive& BOOST_SERIALIZATION_NVP(ISCL1CD);
        archive& BOOST_SERIALIZATION_NVP(ITOW);
        archive& BOOST_SERIALIZATION_NVP(l1c_health);
        archive& BOOST_SERIALIZATION_NVP(sf3_page1_iscs);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_EPHEMERIS_H

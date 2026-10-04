/*!
 * \file beidou_dnav_iono.h
 * \brief  Interface of a BEIDOU IONOSPHERIC MODEL storage
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


#ifndef GNSS_SDR_BEIDOU_DNAV_IONO_H
#define GNSS_SDR_BEIDOU_DNAV_IONO_H

#include "gps_iono.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <map>

struct Beidou_Dnav_Iono_Grid_Point
{
    double vertical_delay{};  //!< B1I vertical delay [m]
    int GIVEI{};              //!< Grid Ionospheric Vertical Error Index
    bool monitored{};         //!< False when the broadcast delay code is 510
    bool available{};         //!< False when the broadcast delay code is 511

    template <class Archive>
    void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(vertical_delay);
        archive& BOOST_SERIALIZATION_NVP(GIVEI);
        archive& BOOST_SERIALIZATION_NVP(monitored);
        archive& BOOST_SERIALIZATION_NVP(available);
    }
};

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the BEIDOU IONOSPHERIC data as described
 * in ICD v2.1
 */
class Beidou_Dnav_Iono : public Gps_Iono
{
public:
    Beidou_Dnav_Iono() = default;  //!< Default constructor

    std::map<int, Beidou_Dnav_Iono_Grid_Point> grid_points;  //!< D2 grid corrections, keyed by IGP number
    bool grid_valid{};

    template <class Archive>
    void serialize(Archive& archive, const unsigned int version)
    {
        if (version > 0)
            {
                archive& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Gps_Iono);
                archive& BOOST_SERIALIZATION_NVP(grid_points);
                archive& BOOST_SERIALIZATION_NVP(grid_valid);
            }
        else
            {
                // Version 0 archives stored the inherited model directly.
                archive& BOOST_SERIALIZATION_NVP(alpha0);
                archive& BOOST_SERIALIZATION_NVP(alpha1);
                archive& BOOST_SERIALIZATION_NVP(alpha2);
                archive& BOOST_SERIALIZATION_NVP(alpha3);
                archive& BOOST_SERIALIZATION_NVP(beta0);
                archive& BOOST_SERIALIZATION_NVP(beta1);
                archive& BOOST_SERIALIZATION_NVP(beta2);
                archive& BOOST_SERIALIZATION_NVP(beta3);
            }
    }
};

BOOST_CLASS_VERSION(Beidou_Dnav_Iono, 1)


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_IONO_H

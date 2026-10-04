/*!
 * \file agnss_ref_location.h
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
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


#ifndef GNSS_SDR_AGNSS_REF_LOCATION_H
#define GNSS_SDR_AGNSS_REF_LOCATION_H

#include <boost/serialization/nvp.hpp>
#include <sstream>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * Classes containing info about system parameters for the different GNSS.
 * \{ */

/*!
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
 *
 */
class Agnss_Ref_Location
{
public:
    /*!
     * Default constructor
     */
    Agnss_Ref_Location() = default;

    double lat{};
    double lon{};
    double uncertainty{};
    bool valid{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the Ref location on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(lat);
        archive& BOOST_SERIALIZATION_NVP(lon);
        archive& BOOST_SERIALIZATION_NVP(uncertainty);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/*!
 * \brief Parses GNSS-SDR.AGNSS_ref_location ("lat,lon" or "lat lon", degrees).
 * valid is false if the string is empty or not a plausible WGS84 position.
 * Single parsing point for ControlThread and SatelliteVisibility.
 */
inline Agnss_Ref_Location parse_agnss_ref_location(const std::string& ref_location_str)
{
    Agnss_Ref_Location result{};
    if (ref_location_str.empty())
        {
            return result;
        }
    std::vector<double> vect;
    std::stringstream ss(ref_location_str);
    double d;
    while (ss >> d)
        {
            vect.push_back(d);
            if ((ss.peek() == ',') || (ss.peek() == ' '))
                {
                    ss.ignore();
                }
        }
    if (vect.size() >= 2)
        {
            if ((vect[0] < 90.0) && (vect[0] > -90) && (vect[1] < 180.0) && (vect[1] > -180.0))
                {
                    result.lat = vect[0];
                    result.lon = vect[1];
                    result.valid = true;
                }
        }
    return result;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_AGNSS_REF_LOCATION_H

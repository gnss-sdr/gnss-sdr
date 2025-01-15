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


/** \} */
/** \} */
#endif  // GNSS_SDR_AGNSS_REF_LOCATION_H

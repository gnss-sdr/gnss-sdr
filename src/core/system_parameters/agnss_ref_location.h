/*!
 * \file agnss_ref_location.h
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_AGNSS_REF_LOCATION_H
#define GNSS_SDR_AGNSS_REF_LOCATION_H

#include <boost/serialization/nvp.hpp>


/*!
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
 *
 */
class Agnss_Ref_Location
{
public:
    bool valid;
    double lat;
    double lon;
    double uncertainty;
    /*!
     * Default constructor
     */
    Agnss_Ref_Location();

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the Ref location on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("valid", valid);
        archive& make_nvp("lat", lat);
        archive& make_nvp("lon", lon);
        archive& make_nvp("uncertainty", uncertainty);
    }
};

#endif

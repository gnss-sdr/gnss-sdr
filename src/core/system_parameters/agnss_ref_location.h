/*!
 * \file agnss_ref_location.h
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_AGNSS_REF_LOCATION_H_
#define GNSS_SDR_AGNSS_REF_LOCATION_H_

#include <boost/assign.hpp>
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

/*!
 * \file gps_ref_time.h
 * \brief  Interface of a GPS REFERENCE TIME storage
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


#ifndef GNSS_SDR_GPS_REF_TIME_H_
#define GNSS_SDR_GPS_REF_TIME_H_

#include <boost/assign.hpp>
#include <boost/serialization/nvp.hpp>


/*!
 * \brief  Interface of a GPS REFERENCE TIME storage
 *
 */
class Gps_Ref_Time
{
public:
    bool valid;
    double d_TOW;
    double d_Week;
    double d_tv_sec;
    double d_tv_usec;
    /*!
     * Default constructor
     */
    Gps_Ref_Time();

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ref time data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("valid", valid);
        archive& make_nvp("d_TOW", d_TOW);
        archive& make_nvp("d_Week", d_Week);
        archive& make_nvp("d_tv_sec", d_tv_sec);
        archive& make_nvp("d_tv_usec", d_tv_usec);
    }
};

#endif

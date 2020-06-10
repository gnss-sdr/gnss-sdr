/*!
 * \file agnss_ref_time.h
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
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


#ifndef GNSS_SDR_AGNSS_REF_TIME_H
#define GNSS_SDR_AGNSS_REF_TIME_H

#include <boost/serialization/nvp.hpp>


/*!
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
 *
 */
class Agnss_Ref_Time
{
public:
    /*!
     * Default constructor
     */
    Agnss_Ref_Time() = default;

    bool valid{};
    double d_TOW{};
    double d_Week{};
    double d_tv_sec{};
    double d_tv_usec{};

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

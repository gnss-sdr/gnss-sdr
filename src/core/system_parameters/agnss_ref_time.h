/*!
 * \file agnss_ref_time.h
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
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


#ifndef GNSS_SDR_AGNSS_REF_TIME_H
#define GNSS_SDR_AGNSS_REF_TIME_H

#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


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

    double tow{};
    double week{};
    double seconds{};
    double microseconds{};
    bool valid{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ref time data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(tow);
        archive& BOOST_SERIALIZATION_NVP(week);
        archive& BOOST_SERIALIZATION_NVP(seconds);
        archive& BOOST_SERIALIZATION_NVP(microseconds);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_AGNSS_REF_TIME_H

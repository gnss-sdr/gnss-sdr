/*!
 * \file beidou_dnav_almanac.h
 * \brief  Interface of a Beidou DNAV Almanac storage
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


#ifndef GNSS_SDR_BEIDOU_DNAV_ALMANAC_H
#define GNSS_SDR_BEIDOU_DNAV_ALMANAC_H

#include "gnss_almanac.h"
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the BeiDou D1/D2 almanac
 */
class Beidou_Dnav_Almanac : public Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Beidou_Dnav_Almanac()
    {
        this->System = 'C';
    };

    int SV_health{};  //!< SV Health
    int AmEpID{};     //!< Identification of the expanded-almanac sequence
    int AmID{};       //!< Identification of the expanded-almanac time-sharing group
    bool expanded{};  //!< True for almanacs of SV ID 31 through 63

    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        ar& BOOST_SERIALIZATION_NVP(PRN);
        ar& BOOST_SERIALIZATION_NVP(delta_i);
        ar& BOOST_SERIALIZATION_NVP(toa);
        ar& BOOST_SERIALIZATION_NVP(WNa);
        ar& BOOST_SERIALIZATION_NVP(M_0);
        ar& BOOST_SERIALIZATION_NVP(ecc);
        ar& BOOST_SERIALIZATION_NVP(sqrtA);
        ar& BOOST_SERIALIZATION_NVP(OMEGA_0);
        ar& BOOST_SERIALIZATION_NVP(omega);
        ar& BOOST_SERIALIZATION_NVP(OMEGAdot);
        ar& BOOST_SERIALIZATION_NVP(af0);
        ar& BOOST_SERIALIZATION_NVP(af1);
        ar& BOOST_SERIALIZATION_NVP(SV_health);
        if (version > 0)
            {
                ar& BOOST_SERIALIZATION_NVP(AmEpID);
                ar& BOOST_SERIALIZATION_NVP(AmID);
                ar& BOOST_SERIALIZATION_NVP(expanded);
            }
    }
};

BOOST_CLASS_VERSION(Beidou_Dnav_Almanac, 1)


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_ALMANAC_H

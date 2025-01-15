/*!
 * \file gps_almanac.h
 * \brief  Interface of a GPS ALMANAC storage
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


#ifndef GNSS_SDR_GPS_ALMANAC_H
#define GNSS_SDR_GPS_ALMANAC_H

#include "gnss_almanac.h"
#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS SV ALMANAC data as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 */
class Gps_Almanac : public Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Gps_Almanac()
    {
        this->System = 'G';
    };

    int32_t SV_health{};  //!< SV Health
    int32_t AS_status{};  //!< Anti-Spoofing Flags and SV Configuration

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
        ar& BOOST_SERIALIZATION_NVP(AS_status);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_ALMANAC_H

/*!
 * \file navic_lnav_almanac.h
 * \brief  Interface of a NavIC (IRNSS) LNAV Almanac storage
 * \author Pradyumna Byppanahalli Suresha, 2025.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_NAVIC_LNAV_ALMANAC_H
#define GNSS_SDR_NAVIC_LNAV_ALMANAC_H

#include "gnss_almanac.h"
#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the NavIC (IRNSS) LNAV almanac
 * as described in the IRNSS SIS ICD for SPS, Version 1.1, August 2017
 * (ISRO-IRNSS-ICD-SPS-1.1), Message Type 7
 */
class Navic_Lnav_Almanac : public Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Navic_Lnav_Almanac()
    {
        this->System = 'I';
    };

    int PRN_ID{};                    //!< PRN ID of the almanac satellite (6 bits)
    double inter_signal_corr{};      //!< Inter-signal correction [s], 8-bit signed, scale 2^-31

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
        ar& BOOST_SERIALIZATION_NVP(PRN_ID);
        ar& BOOST_SERIALIZATION_NVP(inter_signal_corr);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_LNAV_ALMANAC_H

/*!
 * \file galileo_almanac.h
 * \brief  Interface of a Galileo ALMANAC storage
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.cat
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


#ifndef GNSS_SDR_GALILEO_ALMANAC_H
#define GNSS_SDR_GALILEO_ALMANAC_H

#include "gnss_almanac.h"
#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the Galileo SV ALMANAC data
 */
class Galileo_Almanac : public Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Galileo_Almanac() = default;

    int32_t IODa{};
    int32_t E5b_HS{};
    int32_t E1B_HS{};
    int32_t E5a_HS{};

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
        ar& BOOST_SERIALIZATION_NVP(IODa);
        ar& BOOST_SERIALIZATION_NVP(E5b_HS);
        ar& BOOST_SERIALIZATION_NVP(E1B_HS);
        ar& BOOST_SERIALIZATION_NVP(E5a_HS);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_ALMANAC_H

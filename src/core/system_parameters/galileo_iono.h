/*!
 * \file galileo_iono.h
 * \brief  Interface of a Galileo Ionospheric Model storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
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


#ifndef GNSS_SDR_GALILEO_IONO_H
#define GNSS_SDR_GALILEO_IONO_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GALILEO IONOSPHERIC data as described
 * in Galileo ICD paragraph 5.1.6
 *
 * See https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.0.pdf
 */
class Galileo_Iono
{
public:
    /*!
     * Default constructor
     */
    Galileo_Iono() = default;

    // Ionospheric correction
    double ai0{};  //!< Effective Ionisation Level 1st order parameter [sfu]
    double ai1{};  //!< Effective Ionisation Level 2st order parameter [sfu/degree]
    double ai2{};  //!< Effective Ionisation Level 3st order parameter [sfu/degree]

    // from page 5 (UTC) to have a timestamp
    int32_t tow{};  //!< UTC data reference Time of Week [s]
    int32_t WN{};   //!< UTC data reference Week number [week]

    // Ionospheric disturbance flag
    bool Region1_flag{};  //!< Ionospheric Disturbance Flag for region 1
    bool Region2_flag{};  //!< Ionospheric Disturbance Flag for region 2
    bool Region3_flag{};  //!< Ionospheric Disturbance Flag for region 3
    bool Region4_flag{};  //!< Ionospheric Disturbance Flag for region 4
    bool Region5_flag{};  //!< Ionospheric Disturbance Flag for region 5

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization.
     Here is used to save the iono data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(ai0);
        archive& BOOST_SERIALIZATION_NVP(ai1);
        archive& BOOST_SERIALIZATION_NVP(ai2);
        archive& BOOST_SERIALIZATION_NVP(tow);
        archive& BOOST_SERIALIZATION_NVP(WN);
        archive& BOOST_SERIALIZATION_NVP(Region1_flag);
        archive& BOOST_SERIALIZATION_NVP(Region2_flag);
        archive& BOOST_SERIALIZATION_NVP(Region3_flag);
        archive& BOOST_SERIALIZATION_NVP(Region4_flag);
        archive& BOOST_SERIALIZATION_NVP(Region5_flag);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_IONO_H

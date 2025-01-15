/*!
 * \file gps_iono.h
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
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


#ifndef GNSS_SDR_GPS_IONO_H
#define GNSS_SDR_GPS_IONO_H


#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS IONOSPHERIC data as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 */
class Gps_Iono
{
public:
    Gps_Iono() = default;  //!< Default constructor

    // Ionospheric parameters
    double alpha0{};  //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double alpha1{};  //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double alpha2{};  //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double alpha3{};  //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double beta0{};   //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double beta1{};   //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double beta2{};   //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double beta3{};   //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    bool valid{};  //!< Valid flag

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(alpha0);
        archive& BOOST_SERIALIZATION_NVP(alpha1);
        archive& BOOST_SERIALIZATION_NVP(alpha2);
        archive& BOOST_SERIALIZATION_NVP(alpha3);
        archive& BOOST_SERIALIZATION_NVP(beta0);
        archive& BOOST_SERIALIZATION_NVP(beta1);
        archive& BOOST_SERIALIZATION_NVP(beta2);
        archive& BOOST_SERIALIZATION_NVP(beta3);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_IONO_H

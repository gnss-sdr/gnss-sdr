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
 * \brief This class is a storage for the GPS IONOSPHERIC data as described in IS-GPS-200K
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf Appendix II
 */
class Gps_Iono
{
public:
    bool valid{};  //!< Valid flag
    // Ionospheric parameters
    double d_alpha0{};  //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1{};  //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2{};  //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3{};  //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0{};   //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1{};   //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2{};   //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3{};   //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    Gps_Iono() = default;  //!< Default constructor

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("d_alpha0", d_alpha0);
        archive& make_nvp("d_alpha1", d_alpha1);
        archive& make_nvp("d_alpha2", d_alpha2);
        archive& make_nvp("d_alpha3", d_alpha3);
        archive& make_nvp("d_beta0", d_beta0);
        archive& make_nvp("d_beta1", d_beta1);
        archive& make_nvp("d_beta2", d_beta2);
        archive& make_nvp("d_beta3", d_beta3);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_IONO_H

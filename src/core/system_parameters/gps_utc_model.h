/*!
 * \file gps_utc_model.h
 * \brief  Interface of a GPS UTC MODEL storage
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


#ifndef GNSS_SDR_GPS_UTC_MODEL_H
#define GNSS_SDR_GPS_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS UTC MODEL data as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 */
class Gps_Utc_Model
{
public:
    /*!
     * Default constructor
     */
    Gps_Utc_Model() = default;

    // UTC parameters
    double A0{};           //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200M) [s]
    double A1{};           //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200M) [s/s]
    double A2{};           //!< 2nd order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200M) [s/s]
    int32_t tot{};         //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200M) [s]
    int32_t WN_T{};        //!< UTC reference week number [weeks]
    int32_t DeltaT_LS{};   //!< Delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int32_t WN_LSF{};      //!< Week number at the end of which the leap second becomes effective [weeks]
    int32_t DN{};          //!< Day number (DN) at the end of which the leap second becomes effective [days]
    int32_t DeltaT_LSF{};  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]

    bool valid{};

    template <class Archive>
    /*
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(A0);
        archive& BOOST_SERIALIZATION_NVP(A1);
        archive& BOOST_SERIALIZATION_NVP(A2);
        archive& BOOST_SERIALIZATION_NVP(tot);
        archive& BOOST_SERIALIZATION_NVP(WN_T);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LS);
        archive& BOOST_SERIALIZATION_NVP(WN_LSF);
        archive& BOOST_SERIALIZATION_NVP(DN);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LSF);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_UTC_MODEL_H

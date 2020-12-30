/*!
 * \file gps_cnav_utc_model.h
 * \brief  Interface of a GPS CNAV UTC MODEL storage
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


#ifndef GNSS_SDR_GPS_CNAV_UTC_MODEL_H
#define GNSS_SDR_GPS_CNAV_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GPS UTC MODEL data as described in in IS-GPS-200K
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf Appendix III
 */
class Gps_CNAV_Utc_Model
{
public:
    /*!
     * Default constructor
     */
    Gps_CNAV_Utc_Model() = default;

    // UTC parameters
    double d_A2{};           //!< 2nd order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200K) [s/s]
    double d_A1{};           //!< 1st order term of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200K) [s/s]
    double d_A0{};           //!< Constant of a model that relates GPS and UTC time (ref. 20.3.3.5.2.4 IS-GPS-200K) [s]
    int32_t d_t_OT{};        //!< Reference time for UTC data (reference 20.3.4.5 and 20.3.3.5.2.4 IS-GPS-200K) [s]
    int32_t i_WN_T{};        //!< UTC reference week number [weeks]
    int32_t d_DeltaT_LS{};   //!< delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the GPS almanac.
    int32_t i_WN_LSF{};      //!< Week number at the end of which the leap second becomes effective [weeks]
    int32_t i_DN{};          //!< Day number (DN) at the end of which the leap second becomes effective [days]
    int32_t d_DeltaT_LSF{};  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]
    bool valid{};

    template <class Archive>
    /*
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("d_A1", d_A1);
        archive& make_nvp("d_A0", d_A0);
        archive& make_nvp("d_t_OT", d_t_OT);
        archive& make_nvp("i_WN_T", i_WN_T);
        archive& make_nvp("d_DeltaT_LS", d_DeltaT_LS);
        archive& make_nvp("i_WN_LSF", i_WN_LSF);
        archive& make_nvp("i_DN", i_DN);
        archive& make_nvp("d_DeltaT_LSF", d_DeltaT_LSF);
        archive& make_nvp("valid", valid);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_UTC_MODEL_H

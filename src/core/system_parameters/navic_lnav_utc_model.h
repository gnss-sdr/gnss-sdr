/*!
 * \file navic_lnav_utc_model.h
 * \brief  Interface of a NavIC UTC MODEL storage
 * \author Pradyumna Krishna, 2026. pradyumnakrishna(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_NAVIC_LNAV_UTC_MODEL_H
#define GNSS_SDR_NAVIC_LNAV_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the NavIC LNAV UTC Model.
 * \details Implementation follows the interface described in
 * IRNSS ICD (ISRO-IRNSS-ICD-SPS-1.1), Message Types 9 and 26
 *
 */
class Navic_Lnav_Utc_Model
{
public:
    Navic_Lnav_Utc_Model() = default;

    // IRNSS-UTC parameters (Message Type 9)
    double A0_UTC{};       //!< IRNSS clock bias relative to UTC [s]
    double A1_UTC{};       //!< IRNSS clock rate relative to UTC [s/s]
    double A2_UTC{};       //!< IRNSS clock acceleration relative to UTC [s/s^2]
    int32_t DeltaT_LS{};   //!< Delta time due to leap seconds before the new leap second effective [s]
    double t_OT_UTC{};    //!< Reference time of UTC parameters [s]
    int32_t WN_OT_UTC{};   //!< Reference week number of UTC parameters [weeks]
    int32_t WN_LSF{};      //!< Week number of the new leap second [weeks]
    int32_t DN{};           //!< Day number of week of the new leap second [days]
    int32_t DeltaT_LSF{};  //!< Delta time due to leap seconds after the new leap second effective [s]

    // IRNSS-GPS time offset parameters (Message Type 26)
    double A0{};           //!< IRNSS-GNSS time offset bias [s]
    double A1{};           //!< IRNSS-GNSS time offset rate [s/s]
    double A2{};           //!< IRNSS-GNSS time offset acceleration [s/s^2]
    double t_OT{};         //!< Reference time of GNSS time offset parameters [s]
    int32_t WN_OT{};       //!< Reference week number of GNSS time offset parameters [weeks]
    int32_t GNSS_ID{};     //!< GNSS identifier (0=GPS, 1=Galileo, 2=GLONASS, 7=UTC(NPLI))

    bool valid{};

    template <class Archive>
    /*
     * \brief Serialize is a boost standard method to be called by the boost XML
     * serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(A0_UTC);
        archive& BOOST_SERIALIZATION_NVP(A1_UTC);
        archive& BOOST_SERIALIZATION_NVP(A2_UTC);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LS);
        archive& BOOST_SERIALIZATION_NVP(t_OT_UTC);
        archive& BOOST_SERIALIZATION_NVP(WN_OT_UTC);
        archive& BOOST_SERIALIZATION_NVP(WN_LSF);
        archive& BOOST_SERIALIZATION_NVP(DN);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LSF);
        archive& BOOST_SERIALIZATION_NVP(A0);
        archive& BOOST_SERIALIZATION_NVP(A1);
        archive& BOOST_SERIALIZATION_NVP(A2);
        archive& BOOST_SERIALIZATION_NVP(t_OT);
        archive& BOOST_SERIALIZATION_NVP(WN_OT);
        archive& BOOST_SERIALIZATION_NVP(GNSS_ID);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_LNAV_UTC_MODEL_H

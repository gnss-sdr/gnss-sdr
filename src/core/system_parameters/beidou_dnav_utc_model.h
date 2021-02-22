/*!
 * \file beidou_dnav_utc_model.h
 * \brief  Interface of a BeiDou UTC MODEL storage
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
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


#ifndef GNSS_SDR_BEIDOU_DNAV_UTC_MODEL_H
#define GNSS_SDR_BEIDOU_DNAV_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the BeiDou DNAV UTC Model.
 * \details Implementation follows the interface described in the Open Service Signal (Version 2.1)
 *
 */
class Beidou_Dnav_Utc_Model
{
public:
    Beidou_Dnav_Utc_Model() = default;

    // BeiDou UTC parameters
    double A0_UTC{};      //!< BDT clock bias relative to UTC [s]
    double A1_UTC{};      //!< BDT clock rate relative to UTC [s/s]
    int DeltaT_LS{};      //!< Delta time due to leap seconds before the new leap second effective
    int WN_LSF{};         //!< Week number of the new leap second
    int DN{};             //!< Day number of week of the new leap second
    double DeltaT_LSF{};  //!< Delta time due to leap seconds after the new leap second effective [s]

    // BeiDou to GPS time corrections
    double A0_GPS{};  //!< BDT clock bias relative to GPS time [s]
    double A1_GPS{};  //!< BDT clock rate relative to GPS time [s/s]

    // BeiDou to Galileo time corrections
    double A0_GAL{};  //!< BDT clock bias relative to GAL time [s]
    double A1_GAL{};  //!< BDT clock rate relative to GAL time [s/s]

    // BeiDou to GLONASS time corrections
    double A0_GLO{};  //!< BDT clock bias relative to GLO time [s]
    double A1_GLO{};  //!< BDT clock rate relative to GLO time [s/s]

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
        archive& BOOST_SERIALIZATION_NVP(A1_UTC);
        archive& BOOST_SERIALIZATION_NVP(A0_UTC);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LS);
        archive& BOOST_SERIALIZATION_NVP(WN_LSF);
        archive& BOOST_SERIALIZATION_NVP(DN);
        archive& BOOST_SERIALIZATION_NVP(DeltaT_LSF);
        archive& BOOST_SERIALIZATION_NVP(A0_GPS);
        archive& BOOST_SERIALIZATION_NVP(A1_GPS);
        archive& BOOST_SERIALIZATION_NVP(A0_GAL);
        archive& BOOST_SERIALIZATION_NVP(A1_GAL);
        archive& BOOST_SERIALIZATION_NVP(A0_GLO);
        archive& BOOST_SERIALIZATION_NVP(A1_GLO);
        archive& BOOST_SERIALIZATION_NVP(valid);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_UTC_MODEL_H

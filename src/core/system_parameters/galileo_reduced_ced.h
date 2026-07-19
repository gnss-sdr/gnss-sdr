/*!
 * \file galileo_reduced_ced.h
 * \brief Galileo Reduced Clock and Ephemeris Data storage class
 * \author Carles Fernandez, 2021. cfernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_REDUCED_CED_H
#define GNSS_SDR_GALILEO_REDUCED_CED_H

#include "galileo_ephemeris.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class manages the Galileo Reduced Clock and Ephemeris Data
 */
class Galileo_Reduced_CED
{
public:
    /*!
     * Default constructor
     */
    Galileo_Reduced_CED() = default;

    /*!
     * Convert to Galileo_Ephemeris
     */
    Galileo_Ephemeris compute_eph() const;

    /*!
     * \brief Return true when this Reduced CED set is applicable at the given GST epoch.
     */
    bool is_valid_at(uint32_t week, uint32_t tow) const;

    /*!
     * \brief Return the Reduced CED orbit and clock reference time of week.
     */
    uint32_t reference_time() const;

    static constexpr uint32_t CED_VALIDITY_SECONDS = 600U;

    uint32_t PRN{};        //!< Satellite ID
    uint32_t WN{};         //!< GST week containing the start of transmission
    uint32_t TOTRedCED{};  //!< Start time of transmission of the Reduced CED word as GST TOW [s]
    double DeltaAred{};    //!< Difference between the Reduced CED semi-major axis and the nominal semi-major axis [meters]
    double exred{};        //!< Reduced CED eccentricity vector component x
    double eyred{};        //!< Reduced CED eccentricity vector component y
    double Deltai0red{};   //!< Difference between the Reduced CED inclination angle at reference time and the nominal inclination [rad]
    double Omega0red{};    //!< Reduced CED longitude of ascending node at weekly epoch [rad]
    double lambda0red{};   //!< Reduced CED mean argument of latitude [rad]
    double af0red{};       //!< Reduced CED satellite clock bias correction coefficient [seconds]
    double af1red{};       //!< Reduced CED satellite clock drift correction coefficient [seconds/seconds]

    // Word 5 data used by the E1/E5b user algorithms. These are not combined
    // with full-precision CED orbit or clock parameters.
    double BGD_E1E5b{};  //!< E1-E5b Broadcast Group Delay [s]
    int32_t E5b_HS{};    //!< E5b Signal Health Status
    int32_t E1B_HS{};    //!< E1B Signal Health Status
    bool E5b_DVS{};      //!< E5b Data Validity Status
    bool E1B_DVS{};      //!< E1B Data Validity Status
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_REDUCED_CED_H

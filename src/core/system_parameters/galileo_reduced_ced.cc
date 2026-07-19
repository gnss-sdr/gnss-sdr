/*!
 * \file galileo_reduced_ced.cc
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

#include "galileo_reduced_ced.h"
#include "MATH_CONSTANTS.h"
#include <cmath>


Galileo_Ephemeris Galileo_Reduced_CED::compute_eph() const
{
    Galileo_Ephemeris eph{};
    eph.nav_message_type = Galileo_Nav_Message_Type::INAV;

    const double A_nominal = 29600000;  // meters (Table 1 Galileo ICD 2.0)
    const double Ared = DeltaAred + A_nominal;
    eph.sqrtA = std::sqrt(Ared);    // Square root of the semi-major axis [meters^1/2]
    const double i_nominal = 56.0;  // degrees (Table 1 Galileo ICD 2.0)
    const double i0red = Deltai0red + i_nominal / 180.0;
    eph.i_0 = i0red * GNSS_PI;                           // Inclination angle at reference time [rad]
    eph.ecc = std::sqrt(exred * exred + eyred * eyred);  // Eccentricity
    eph.omega = std::atan2(eyred, exred);                // Argument of perigee [rad]
    eph.M_0 = lambda0red * GNSS_PI - eph.omega;          // Mean anomaly at reference time [rad]
    eph.OMEGA_0 = Omega0red * GNSS_PI;                   // Longitude of ascending node of orbital plane at weekly epoch [rad]

    // Word 16 has no IODnav and is not a full-precision ephemeris set.
    eph.flag_all_ephemeris = false;
    eph.IOD_ephemeris = -1;
    eph.IOD_nav = -1;
    eph.PRN = PRN;

    const uint32_t t0r = reference_time();
    eph.toe = t0r;  // Ephemeris reference time [s]

    // Clock correction parameters
    eph.toc = t0r;     // Clock correction data reference Time of Week [sec]
    eph.af0 = af0red;  // SV clock bias correction coefficient [s]
    eph.af1 = af1red;  // SV clock drift correction coefficient [s/s]

    // GST
    eph.WN = WN;
    eph.tow = TOTRedCED;

    eph.BGD_E1E5b = BGD_E1E5b;
    eph.E5b_HS = E5b_HS;
    eph.E1B_HS = E1B_HS;
    eph.E5b_DVS = E5b_DVS;
    eph.E1B_DVS = E1B_DVS;

    return eph;
}


uint32_t Galileo_Reduced_CED::reference_time() const
{
    return (30U * (TOTRedCED / 30U) + 1U) % 604800U;
}


bool Galileo_Reduced_CED::is_valid_at(uint32_t week, uint32_t tow) const
{
    if (TOTRedCED >= 604800U || tow >= 604800U)
        {
            return false;
        }

    const uint64_t reference_epoch = static_cast<uint64_t>(WN) * 604800ULL + reference_time();
    const uint64_t observation_epoch = static_cast<uint64_t>(week) * 604800ULL + tow;
    return observation_epoch >= reference_epoch &&
           observation_epoch - reference_epoch < static_cast<uint64_t>(CED_VALIDITY_SECONDS);
}

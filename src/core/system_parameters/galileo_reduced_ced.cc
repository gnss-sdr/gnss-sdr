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

    eph.flag_all_ephemeris = true;
    eph.IOD_ephemeris = IODnav;
    eph.IOD_nav = IODnav;
    eph.PRN = PRN;

    int32_t t0r = (30 * (TOTRedCED / 30) + 1) % 604800;
    eph.toe = t0r;  // Ephemeris reference time [s]

    // Clock correction parameters
    eph.toc = t0r;     // Clock correction data reference Time of Week [sec]
    eph.af0 = af0red;  // SV clock bias correction coefficient [s]
    eph.af1 = af1red;  // SV clock drift correction coefficient [s/s]

    // GST
    eph.WN = TOTRedCED / 604800;   // Week number
    eph.tow = TOTRedCED % 604800;  // Time of Week

    return eph;
}

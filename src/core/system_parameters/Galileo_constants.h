/*!
 * \file Galileo_constants.h
 * \brief  Defines constants for Galileo
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CONSTANTS_H
#define GNSS_SDR_GALILEO_CONSTANTS_H

// Physical constants for Galileo
constexpr double GALILEO_PI = 3.1415926535898;               //!< Pi as defined in GALILEO ICD
constexpr double GALILEO_TWO_PI = 6.283185307179600;         //!< 2*Pi as defined in GALILEO ICD
constexpr double GALILEO_GM = 3.986004418e14;                //!< Geocentric gravitational constant[m^3/s^2]
constexpr double GALILEO_OMEGA_EARTH_DOT = 7.2921151467e-5;  //!< Mean angular velocity of the Earth [rad/s]
constexpr double GALILEO_C_M_S = 299792458.0;                //!< The speed of light, [m/s]
constexpr double GALILEO_C_M_MS = 299792.4580;               //!< The speed of light, [m/ms]
constexpr double GALILEO_F = -4.442807309e-10;               //!< Constant, [s/(m)^(1/2)]

#endif  // GNSS_SDR_GALILEO_CONSTANTS_H

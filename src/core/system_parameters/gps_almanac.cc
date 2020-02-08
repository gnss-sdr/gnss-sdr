/*!
 * \file gps_almanac.cc
 * \brief  Interface of a GPS ALMANAC storage
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "gps_almanac.h"

Gps_Almanac::Gps_Almanac()
{
    i_satellite_PRN = 0U;
    d_Delta_i = 0.0;
    i_Toa = 0;
    i_WNa = 0;
    d_M_0 = 0.0;
    d_e_eccentricity = 0.0;
    d_sqrt_A = 0.0;
    d_OMEGA0 = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    i_SV_health = 0;
    i_AS_status = 0;
    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
}

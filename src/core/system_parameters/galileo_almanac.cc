/*!
 * \file galileo_almanac.cc
 * \brief  Interface of a Galileo ALMANAC storage
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.cat
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

#include "galileo_almanac.h"


Galileo_Almanac::Galileo_Almanac()
{
    i_satellite_PRN = 0U;
    i_Toa = 0;
    i_WNa = 0;
    i_IODa = 0;
    d_Delta_i = 0.0;
    d_M_0 = 0.0;
    d_e_eccentricity = 0.0;
    d_Delta_sqrt_A = 0.0;
    d_OMEGA0 = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
    E5b_HS = 0;
    E1B_HS = 0;
    E5a_HS = 0;
}

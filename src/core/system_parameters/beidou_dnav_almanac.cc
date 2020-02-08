/*!
 * \file beidou_dnav_almanac.cc
 * \brief  Interface of a Beidou DNAV Almanac storage
 *
 * See http://en.beidou.gov.cn/SYSTEMS/Officialdocument/201902/P020190227601370045731.pdf
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_dnav_almanac.h"

Beidou_Dnav_Almanac::Beidou_Dnav_Almanac()
{
    i_satellite_PRN = 0;
    d_Delta_i = 0.0;
    d_Toa = 0.0;
    d_M_0 = 0.0;
    d_e_eccentricity = 0.0;
    d_sqrt_A = 0.0;
    d_OMEGA0 = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    i_SV_health = 0;
    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
}

/*!
 * \file beidou_dnav_iono.cc
 * \brief  Interface of a BEIDOU IONOSPHERIC MODEL storage
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

#include "beidou_dnav_iono.h"

Beidou_Dnav_Iono::Beidou_Dnav_Iono()
{
    valid = false;
    d_alpha0 = 0.0;
    d_alpha1 = 0.0;
    d_alpha2 = 0.0;
    d_alpha3 = 0.0;
    d_beta0 = 0.0;
    d_beta1 = 0.0;
    d_beta2 = 0.0;
    d_beta3 = 0.0;
}

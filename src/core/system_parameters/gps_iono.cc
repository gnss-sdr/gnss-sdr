/*!
 * \file gps_iono.cc
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
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

#include "gps_iono.h"

Gps_Iono::Gps_Iono()
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

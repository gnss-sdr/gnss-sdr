/*!
 * \file galileo_iono.cc
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
 *
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

#include "galileo_iono.h"

Galileo_Iono::Galileo_Iono()
{
    //  Ionospheric correction
    ai0_5 = 0.0;  // Effective Ionisation Level 1st order parameter [sfu]
    ai1_5 = 0.0;  // Effective Ionisation Level 2st order parameter [sfu/degree]
    ai2_5 = 0.0;  // Effective Ionisation Level 3st order parameter [sfu/degree]

    // Ionospheric disturbance flag
    Region1_flag_5 = false;  // Ionospheric Disturbance Flag for region 1
    Region2_flag_5 = false;  // Ionospheric Disturbance Flag for region 2
    Region3_flag_5 = false;  // Ionospheric Disturbance Flag for region 3
    Region4_flag_5 = false;  // Ionospheric Disturbance Flag for region 4
    Region5_flag_5 = false;  // Ionospheric Disturbance Flag for region 5

    TOW_5 = 0;
    WN_5 = 0;
}

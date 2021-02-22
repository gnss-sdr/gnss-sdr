/*!
 * \file beidou_dnav_ephemeris.cc
 * \brief  Interface of a BeiDou EPHEMERIS storage and orbital model functions
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "beidou_dnav_ephemeris.h"
#include "gnss_satellite.h"
#include <string>

Beidou_Dnav_Ephemeris::Beidou_Dnav_Ephemeris()
{
    auto gnss_sat = Gnss_Satellite();
    const std::string _system("Beidou");
    for (unsigned int i = 1; i < 36; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
    this->System = 'B';
}

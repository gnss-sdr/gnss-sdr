/*!
 * \file gps_ephemeris.cc
 * \brief  Interface of a GPS EPHEMERIS storage and orbital model functions
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_ephemeris.h"
#include "gnss_satellite.h"
#include <string>


Gps_Ephemeris::Gps_Ephemeris()
{
    auto gnss_sat = Gnss_Satellite();
    const std::string _system("GPS");
    for (uint32_t i = 1; i < 33; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
    this->System = 'G';
}

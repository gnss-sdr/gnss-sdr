/*!
 * \file pvt_sd_conf.cc
 * \brief Class that contains all the configuration parameters for spoofing detection techniques that are a part of PVT block
 * \author Harshad Sathaye sathaye.h(at)northeastern.edu
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

#include "pvt_sd_conf.h"

Pvt_SD_Conf::Pvt_SD_Conf()
{
    // ####### Position consistency check variables

    int max_jump_distance = 100;   // meters
    int geo_fence_radius = 15;     // meters
    int velocity_difference = 15;  // meters/s

    double static_lat = 0;  //degrees
    double static_lon = 0;  //degrees
    double static_alt = 0;  //meters

    position_check = false;
    dump_pos_checks_results = false;

    std::string filename = "./pos_consistency_results.mat";
}

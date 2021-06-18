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

#include "spoofing_detector_conf.h"

SpoofingDetectorConf::SpoofingDetectorConf()
{
    // ####### Position consistency check variables

    max_jump_distance = 100;   // meters
    geo_fence_radius = 15;     // meters
    velocity_difference = 15;  // m/s
    pos_error_threshold = 10;  // meters
    min_altitude = -10;        // meters
    max_altitude = 20000;      // meters
    min_ground_speed = 0;      // m/s
    max_ground_speed = 200;    // m/s
    static_lat = 0;            //degrees
    static_lon = 0;            //degrees
    static_alt = 0;            //meters

    position_check = false;
    dump_pvt_checks_results = false;
    static_pos_check = false;

    std::string filename = "./pos_consistency_results.mat";
}

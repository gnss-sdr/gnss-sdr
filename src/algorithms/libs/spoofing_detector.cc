/*!
 * \file spoofing_detector.cc
 * \brief Library of anti-spoofing functions
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

#include "spoofing_detector.h"
#include "GPS_L1_CA.h"
#include <cmath>    // for floor, fmod, rint, ceil
#include <cstring>  // for memcpy
#include <iostream>
#include <map>

SpoofingDetector::SpoofingDetector()
{

}

SpoofingDetector::SpoofingDetector(const ConfigurationInterface* configuration)
{
    d_dump_results = configuration->property("Spoofing.dump_results", true);

    // ####### Position consistency check configuration
    d_position_check = configuration->property("Spoofing.position_check", false);
    d_max_jump_distance = configuration->property("Spoofing.max_jump_distance", 100);
    d_geo_fence_radius = configuration->property("Spoofing.geo_fence_radius", 15);
    d_velocity_difference = configuration->property("Spoofing.geo_fence_radius", 15);

    DLOG(INFO) << "Spoofing detector constructor";
}

SpoofingDetector::~SpoofingDetector()
{
    DLOG(INFO) << "Spoofing detector destructor";
}

void SpoofingDetector::check_position_consistency(
    double lat, 
    double lon, 
    double alt, 
    const Gnss_Synchro **in)
{
    DLOG(INFO) << "Check position consistency function";
}

void SpoofingDetector::compare_velocity()
{
    DLOG(INFO) << "Compare velocity function";
}

void SpoofingDetector::static_pos_check(
    double lat, 
    double lon, 
    double alt)
{
    DLOG(INFO) << "Check static position function";
}
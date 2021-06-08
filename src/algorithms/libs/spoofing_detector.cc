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
#include <cmath>  // for floor, fmod, rint, ceil
#include <map>

SpoofingDetector::SpoofingDetector()
{
}

SpoofingDetector::SpoofingDetector(const Pvt_SD_Conf* conf_)
{
    DLOG(INFO) << "Spoofing detector initialized";
    d_max_jump_distance = conf_->max_jump_distance;
    d_geo_fence_radius = conf_->geo_fence_radius;
    d_velocity_difference = conf_->velocity_difference;

    d_static_lat = conf_->static_lat;
    d_static_lon = conf_->static_lon;
    d_static_alt = conf_->static_alt;

    d_dump_pos_checks_results = conf_->dump_pos_checks_results;

    d_position_check = conf_->position_check;
    d_spoofer_score = 0;
}

// ####### Position consistency functions
void SpoofingDetector::check_position_consistency(double lat, double lon, double alt,
    const Gnss_Synchro** in)
{
    static_pos_check(lat, lon, alt);
}

void SpoofingDetector::compare_velocity()
{
    //DLOG(INFO) << "Compare velocity function";
}

void SpoofingDetector::static_pos_check(double lat, double lon, double alt)
{
    DLOG(INFO) << "Check static position function";
    double distance = calculate_distance(lat, lon, d_static_lat, d_static_lon);

    if (distance > d_geo_fence_radius)
        {
            d_score.static_pos_check_score = 1;
            d_spoofer_score = d_score.total_score();
            DLOG(INFO) << "Spoofer score: " << d_spoofer_score << " - Distance: " << distance;
        }
    else
        {
            if (d_score.static_pos_check_score == 1)
                {
                    DLOG(INFO) << "Received location within geo-fence, resetting static position score";
                }

            d_score.static_pos_check_score = 0;
            d_spoofer_score = d_score.total_score();
        }
}

void SpoofingDetector::position_jump(double lat, double lon, double alt)
{
    //DLOG(INFO) << "Check position jump function";
}

// ####### General functions
int SpoofingDetector::get_spoofer_score()
{
    int d_spoofer_score = d_score.total_score();
    DLOG(INFO) << "Total spoofer score: " << d_spoofer_score;
    return d_spoofer_score;
}

long double SpoofingDetector::to_radians(double degree)
{
    // Convert degrees to radians
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

long double SpoofingDetector::calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    // Calculate distance between l1 and l2 using Haversine formula
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    // Haversine Formula
    long double dlong = lon2 - lon2;
    long double dlat = lat2 - lat1;

    long double distance = pow(sin(dlat / 2), 2) +
                           cos(lat1) * cos(lat2) *
                               pow(sin(dlong / 2), 2);

    distance = 2 * asin(sqrt(distance));

    long double R = 6371000;

    // Calculate the result
    distance = distance * R;
    return distance;
}
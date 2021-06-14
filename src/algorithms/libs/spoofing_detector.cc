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

SpoofingDetector::SpoofingDetector(const Pvt_SD_Conf *conf_)
{
    DLOG(INFO) << "Spoofing detector for PVT initialized";

    d_max_jump_distance = conf_->max_jump_distance;
    d_geo_fence_radius = conf_->geo_fence_radius;
    d_velocity_difference = conf_->velocity_difference;
    d_pos_jump_recovery = conf_->pos_jump_recovery;

    // Set predetermined location
    d_static_pvt.lat = conf_->static_lat;
    d_static_pvt.lon = conf_->static_lon;
    d_static_pvt.alt = conf_->static_alt;

    DLOG(INFO) << "Static pos check: " << d_static_pvt.lat << ", " << d_static_pvt.lon;

    d_dump_pos_checks_results = conf_->dump_pos_checks_results;
    d_position_check = conf_->position_check;
    d_static_pos_check = conf_->static_pos_check;

    d_spoofer_score = 0;

    // Used to set old coordinates and last known good location
    d_first_record = true;

    // Used to decide whether to update LKGL
    d_update_lkgl = true;

    boost::posix_time::ptime d_pvt_epoch(boost::gregorian::date(1970, 1, 1));
}

// ####### Position consistency functions
void SpoofingDetector::check_position_consistency()
{
    // Need to check this for dynamic scenarios.
    if (d_static_pos_check)
        {
            static_pos_check();
        }

    position_jump();

    if (d_first_record)
        {
            d_first_record = false;
        }
}

void SpoofingDetector::compare_velocity()
{
    //DLOG(INFO) << "Compare velocity function";
}

void SpoofingDetector::static_pos_check()
{
    long double distance = calculate_distance(d_new_pvt.lat, d_new_pvt.lon, d_static_pvt.lat, d_static_pvt.lon);
    DLOG(INFO) << "STATIC_POS: static_coords (" << d_static_pvt.lat << "," << d_static_pvt.lon << ") received (" << d_new_pvt.lat << "," << d_new_pvt.lon << ") Distance: " << distance;
    if (distance > d_geo_fence_radius)
        {
            d_score.static_pos_check_score = 1;
            d_spoofer_score = d_score.total_score();

            DLOG(INFO) << "STATIC_POS: Spoofer score: " << d_spoofer_score << " - Distance: " << distance;
        }
    else
        {
            if (d_score.static_pos_check_score == 1)
                {
                    DLOG(INFO) << "STATIC_POS: Received location within geo-fence, resetting static position score";
                }

            d_score.static_pos_check_score = 0;
            d_spoofer_score = d_score.total_score();
        }
}

void SpoofingDetector::position_jump()
{
    double distance_to_lkgl;  // LKGL - Last Known Good Location
    double jump_distance;

    bool is_spoofing = false;

    DLOG(INFO) << "POS_JUMP: check";

    if (d_first_record)
        {
            update_old_pvt();

            // In case of a cold start there is no way to know if the received location is legit. Hence a naive way is to check for spoofer score.
            // The score will be 0 if no other technique detects spoofing
            if (get_spoofer_score() == 0)
                {
                    // Set last known good location to current coordinates
                    update_lkg_pvt(false);  // Set old = false, hence set new location as lkg
                    DLOG(INFO) << "POS_JUMP: location update: " << d_new_pvt.lat << ", " << d_new_pvt.lon << ", " << d_new_pvt.alt;
                }

            d_first_record = false;
            return;
        }

    jump_distance = calculate_distance(d_old_pvt.lat, d_old_pvt.lon, d_new_pvt.lat, d_new_pvt.lon);
    DLOG(INFO) << "POS_JUMP: Old (" << d_old_pvt.lat << "," << d_old_pvt.lon << ") received ("
               << d_new_pvt.lat << "," << d_new_pvt.lon << ") Distance: "
               << jump_distance << " Spoofer score: " << get_spoofer_score();

    if (jump_distance > d_max_jump_distance)
        {
            distance_to_lkgl = calculate_distance(d_lkg_pvt.lat, d_lkg_pvt.lon, d_new_pvt.lat, d_new_pvt.lon);

            if (distance_to_lkgl < d_pos_jump_recovery)
                {
                    // Reset jump check when the receiver is back to the last known good location
                    reset_pos_jump_check();
                }
            else
                {
                    DLOG(INFO) << "POS_JUMP: Jump distance: " << jump_distance;

                    // A naive way of checking if the jump is legitimate jump or is caused by spoofing.
                    // (A legitimate jump will occur if the receiver looses lock and re-acquires it)
                    //uint32_t dt = d_new_pvt.tstamp - d_lkg_pvt.tstamp;

                    //double est_dist_travelled = dt * d_lkg_pvt.speed_over_ground;

                    if (check_propagated_pos())
                        {
                            DLOG(INFO) << "POS_JUMP: Propagated position detects spoofing";
                            is_spoofing = true;
                        }
                    else
                        {
                            DLOG(INFO) << "POS_JUMP: Propagated position did not detect spoofing";
                        }

                    // if (abs(est_dist_travelled - distance_to_lkgl) > 10)
                    //     {
                    //         if (abs(d_lkg_pvt.heading - d_new_pvt.heading) == 5)
                    //             {
                    //             }
                    //         DLOG(INFO) << "POS_JUMP: Estimated travel distance and heading: " << est_dist_travelled << ", " << d_new_pvt.heading;
                    //     }

                    if (is_spoofing)
                        {
                            d_score.position_jump_score = 2;
                            d_spoofer_score = d_score.total_score();
                            DLOG(INFO) << "POS_JUMP: Spoofer score: " << d_spoofer_score << " - Jump distance: " << jump_distance;

                            // Set last known good location to old coordinates
                            if (d_update_lkgl)
                                {
                                    update_lkg_pvt(true);  // Set old = true, hence set old location as lkg
                                    d_update_lkgl = false;
                                }
                        }
                }
        }
    else
        {
            if (d_score.position_jump_score == 2)
                {
                    distance_to_lkgl = calculate_distance(d_lkg_pvt.lat, d_lkg_pvt.lon, d_new_pvt.lat, d_new_pvt.lon);
                    if (distance_to_lkgl < d_pos_jump_recovery)
                        {
                            // Reset jump check when the receiver is back to the last known good location
                            reset_pos_jump_check();
                            DLOG(INFO) << "POS_JUMP: Received location within geo-fence, resetting static position score";
                        }
                }
            else
                {
                    update_lkg_pvt(false);
                }
        }

    DLOG(INFO) << "POS_JUMP: Old location updated to: " << d_new_pvt.lat << ", " << d_new_pvt.lon << ", " << d_new_pvt.alt;
    DLOG(INFO) << "POS_JUMP: Last known good location: " << d_lkg_pvt.lat << ", " << d_lkg_pvt.lon;
    DLOG(INFO) << "POS_JUMP: Distance to LKGL: " << distance_to_lkgl;
}

bool SpoofingDetector::check_propagated_pos()
{
    double metersPerDegLat = 111111.0;
    double metersPerRadLat = metersPerDegLat * 180 / M_PI;
    double metersPerDegLon = metersPerDegLat * cos(d_old_pvt.lat);
    double metersPerRadLon = metersPerDegLon * 180 / M_PI;

    PvtSol temp_pvt;
    uint32_t dt = (d_new_pvt.tstamp - d_old_pvt.tstamp) / 1000;

    temp_pvt.lat = d_old_pvt.lat + d_old_pvt.vel_x * dt / metersPerRadLat;
    temp_pvt.lon = d_old_pvt.lon + d_old_pvt.vel_y * dt / metersPerRadLon;
    temp_pvt.alt = d_old_pvt.alt + d_old_pvt.vel_z * dt;

    double distance_error = calculate_distance(temp_pvt.lat, temp_pvt.lon, d_new_pvt.lat, d_new_pvt.lon);

    DLOG(INFO) << "PROPAGATE_POS: Pro " << temp_pvt.lat << "," << temp_pvt.lon << ", " << temp_pvt.alt;
    DLOG(INFO) << "PROPAGATE_POS: Recv " << d_new_pvt.lat << "," << d_new_pvt.lon << ", " << d_new_pvt.alt;
    return distance_error > d_pos_jump_recovery;
}

// ####### General functions
void SpoofingDetector::update_pvt(double lat, double lon, double alt, double vel_x, double vel_y, double vel_z, double speed_over_ground, double heading, uint32_t tstamp)
{
    d_new_pvt.lat = lat;
    d_new_pvt.lon = lon;
    d_new_pvt.alt = alt;
    d_new_pvt.vel_x = vel_x;
    d_new_pvt.vel_y = vel_y;
    d_new_pvt.vel_z = vel_z;
    d_new_pvt.speed_over_ground = sqrt(pow(vel_x, 2) + pow(vel_y, 2) + pow(vel_y, 2));
    d_new_pvt.heading = 180 + 180 / M_PI * (atan2(-vel_x, -vel_z));
    d_new_pvt.tstamp = tstamp;

    DLOG(INFO) << "New PVT: " << lat << ", " << lon << ", " << alt
               << ", " << vel_x << ", " << vel_y << ", " << vel_z
               << ", " << speed_over_ground << ", " << heading << ", " << tstamp;
}

void SpoofingDetector::update_old_pvt()
{
    d_old_pvt.lat = d_new_pvt.lat;
    d_old_pvt.lon = d_new_pvt.lon;
    d_old_pvt.alt = d_new_pvt.alt;
    d_old_pvt.vel_x = d_new_pvt.vel_x;
    d_old_pvt.vel_y = d_new_pvt.vel_y;
    d_old_pvt.vel_z = d_new_pvt.vel_z;
    d_old_pvt.speed_over_ground = d_new_pvt.speed_over_ground;
    d_old_pvt.heading = d_new_pvt.heading;
    d_old_pvt.tstamp = d_new_pvt.tstamp;

    DLOG(INFO) << "Old pvt updated to: " << d_old_pvt.lat << ", " << d_old_pvt.lon << ", " << d_old_pvt.alt;
}

void SpoofingDetector::update_lkg_pvt(bool set_old)
{
    if (set_old)
        {
            d_lkg_pvt.lat = d_old_pvt.lat;
            d_lkg_pvt.lon = d_old_pvt.lon;
            d_lkg_pvt.alt = d_old_pvt.alt;
            d_lkg_pvt.vel_x = d_old_pvt.vel_x;
            d_lkg_pvt.vel_y = d_old_pvt.vel_y;
            d_lkg_pvt.vel_z = d_old_pvt.vel_z;
            d_lkg_pvt.speed_over_ground = d_old_pvt.speed_over_ground;
            d_lkg_pvt.heading = d_old_pvt.heading;
            d_lkg_pvt.tstamp = d_old_pvt.tstamp;
        }
    else
        {
            d_lkg_pvt.lat = d_new_pvt.lat;
            d_lkg_pvt.lon = d_new_pvt.lon;
            d_lkg_pvt.alt = d_new_pvt.alt;
            d_lkg_pvt.vel_x = d_new_pvt.vel_x;
            d_lkg_pvt.vel_y = d_new_pvt.vel_y;
            d_lkg_pvt.vel_z = d_new_pvt.vel_z;
            d_lkg_pvt.speed_over_ground = d_new_pvt.speed_over_ground;
            d_lkg_pvt.heading = d_new_pvt.heading;
            d_lkg_pvt.tstamp = d_new_pvt.tstamp;
        }


    DLOG(INFO) << "LKG updated to: " << d_lkg_pvt.lat << ", " << d_lkg_pvt.lon << ", " << d_lkg_pvt.alt;
}

void SpoofingDetector::reset_pos_jump_check()
{
    d_score.position_jump_score = 0;

    // Set last known good location to current coordinates
    update_lkg_pvt(false);  // Set old = false, hence set new location as lkg

    d_score.position_jump_score = 0;
    d_spoofer_score = d_score.total_score();

    d_update_lkgl = true;
}

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
    long double dlong = lon2 - lon1;
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
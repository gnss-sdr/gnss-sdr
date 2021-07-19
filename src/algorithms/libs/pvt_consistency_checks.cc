/*!
 * \file pvt_consistency_checks.cc
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

PVTConsistencyChecks::PVTConsistencyChecks()
{
}

PVTConsistencyChecks::PVTConsistencyChecks(const PVTConsistencyChecksConf* conf_)
{
    DLOG(INFO) << "Spoofing detector for PVT initialized";

    d_max_jump_distance = conf_->max_jump_distance;
    d_geo_fence_radius = conf_->geo_fence_radius;
    d_velocity_difference = conf_->velocity_difference;
    d_pos_error_threshold = conf_->pos_error_threshold;

    // Set predetermined location
    d_static_pvt.ecef_y = conf_->static_lat;
    d_static_pvt.ecef_x = conf_->static_lon;
    d_static_pvt.ecef_z = conf_->static_alt;

    d_dump_pvt_checks_results = conf_->dump_pvt_checks_results;
    d_position_check = conf_->position_check;
    d_static_pos_check = conf_->static_pos_check;

    // Position abnormalities check paramaters
    d_min_altitude = conf_->min_altitude;
    d_max_altitude = conf_->max_altitude;
    d_min_ground_speed = conf_->min_ground_speed;
    d_max_ground_speed = conf_->max_ground_speed;

    d_spoofer_score = 0;
    d_checked_velocity_pairs = 0;  // Number of velocity measurements checked. Velocity measurements are processed in pairs (old and new)

    // Used to set old coordinates and last known good location
    d_first_record = true;

    // Used to decide whether to update LKGL
    d_update_lkgl = true;


    boost::posix_time::ptime d_pvt_epoch(boost::gregorian::date(1970, 1, 1));
}

// ####### Position consistency functions
void PVTConsistencyChecks::check_PVT_consistency()
{
    // Public function
    // PVT block calls this function. Actual consistency checks are triggered from here
    if (d_static_pos_check)
        {
            static_pos_check();
        }

    position_jump();
    compare_velocity();
    abnormal_position_checks();
    check_time();

    if (d_first_record)
        {
            d_first_record = false;
        }
}

void PVTConsistencyChecks::abnormal_position_checks()
{
    // A collection of model based position abnormalities check Min/Max altitude and speed
    d_score.abnormal_position_score = 0;

    if (d_new_pvt.ecef_z < d_min_altitude) d_score.abnormal_position_score += 0.25;
    if (d_new_pvt.ecef_z > d_max_altitude) d_score.abnormal_position_score += 0.25;
    if (d_new_pvt.speed_over_ground < d_min_ground_speed) d_score.abnormal_position_score += 0.25;
    if (d_new_pvt.speed_over_ground > d_max_ground_speed) d_score.abnormal_position_score += 0.25;

    DLOG(INFO) << "ABNORMAL_CHECK: " << d_score.abnormal_position_score;
}

void PVTConsistencyChecks::compare_velocity()
{
    /// Compares the reported velocity with the position pairs. If the projected coordinates do not match the received coordinate velocity error is increased
    ++d_checked_velocity_pairs;

    if (check_propagated_pos())
        {
            ++d_velocity_error;
        }
    d_score.velocity_check_score = d_velocity_error / d_checked_velocity_pairs;
    DLOG(INFO) << "VELOCITY_COMPARE: Confidence " << d_velocity_error << "/" << d_checked_velocity_pairs;
    update_old_pvt();
}

void PVTConsistencyChecks::static_pos_check()
{
    long double distance = calculate_distance_ECEF(&d_static_pvt, &d_new_pvt);
    DLOG(INFO) << "STATIC_POS: static_coords (" << d_static_pvt.ecef_y << "," << d_static_pvt.ecef_x << ") received (" << d_new_pvt.ecef_y << "," << d_new_pvt.ecef_x << ") Distance: " << distance;
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

void PVTConsistencyChecks::position_jump()
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
                    DLOG(INFO) << "POS_JUMP: location update: " << d_new_pvt.ecef_y << ", " << d_new_pvt.ecef_x << ", " << d_new_pvt.ecef_z;
                }

            d_first_record = false;
            return;
        }

    jump_distance = calculate_distance_ECEF(&d_old_pvt, &d_new_pvt);
    DLOG(INFO) << "POS_JUMP: Old (" << d_old_pvt.ecef_y << "," << d_old_pvt.ecef_x << ") received ("
               << d_new_pvt.ecef_y << "," << d_new_pvt.ecef_x << ") Distance: "
               << jump_distance << " Spoofer score: " << get_spoofer_score();

    if (jump_distance > d_max_jump_distance)
        {
            distance_to_lkgl = calculate_distance_ECEF(&d_lkg_pvt, &d_new_pvt);

            if (distance_to_lkgl < d_pos_error_threshold)
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
                    distance_to_lkgl = calculate_distance_ECEF(&d_lkg_pvt, &d_new_pvt);
                    if (distance_to_lkgl < d_pos_error_threshold)
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

    DLOG(INFO) << "POS_JUMP: Old location updated to: " << d_new_pvt.ecef_y << ", " << d_new_pvt.ecef_x << ", " << d_new_pvt.ecef_z;
    DLOG(INFO) << "POS_JUMP: Last known good location: " << d_lkg_pvt.ecef_y << ", " << d_lkg_pvt.ecef_x;
    DLOG(INFO) << "POS_JUMP: Distance to LKGL: " << distance_to_lkgl;
}

bool PVTConsistencyChecks::check_propagated_pos()
{
    PvtSol temp_pvt;
    double dt = (d_new_pvt.tstamp - d_old_pvt.tstamp) / 1000;

    temp_pvt.ecef_y = d_old_pvt.ecef_y + d_old_pvt.vel_x * dt;  // / metersPerRadLat;
    temp_pvt.ecef_x = d_old_pvt.ecef_x + d_old_pvt.vel_y * dt;  // / metersPerRadLon;
    temp_pvt.ecef_z = d_old_pvt.ecef_z + d_old_pvt.vel_z * dt;

    double distance_error = calculate_distance_ECEF(&temp_pvt, &d_new_pvt);

    DLOG(INFO) << "PROPAGATE_POS: Pro " << temp_pvt.ecef_y << "," << temp_pvt.ecef_x << ", " << temp_pvt.ecef_z;
    DLOG(INFO) << "PROPAGATE_POS: Recv " << d_new_pvt.ecef_y << "," << d_new_pvt.ecef_x << ", " << d_new_pvt.ecef_z;
    DLOG(INFO) << "PROPAGATE_POS: Error: " << distance_error;

    return distance_error > d_pos_error_threshold;
}

void PVTConsistencyChecks::check_time()
{
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());

    if ((now - d_new_pvt.utc_time).total_seconds() < -18)
        {
            DLOG(INFO) << "UTC_TIME_CHECK: Calculated UTC time is " << (now - d_new_pvt.utc_time).total_seconds() << " in future";
        }
    else if ((now - d_new_pvt.utc_time).total_seconds() > 18)
        {
            DLOG(INFO) << "UTC_TIME_CHECK: Calculated UTC time is " << (now - d_new_pvt.utc_time).total_seconds() << " in past";
        }
}

// ####### General functions
void PVTConsistencyChecks::update_pvt(const std::array<double, 3>& pos,
    const std::array<double, 3>& vel,
    double speed_over_ground, double heading,
    uint32_t tstamp, boost::posix_time::ptime utc_time)
{
    d_new_pvt.ecef_y = pos[1];
    d_new_pvt.ecef_x = pos[0];
    d_new_pvt.ecef_z = pos[2];

    d_new_pvt.vel_x = vel[0];
    d_new_pvt.vel_y = vel[1];
    d_new_pvt.vel_z = vel[2];

    d_new_pvt.speed_over_ground = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[1], 2));
    d_new_pvt.heading = 180 + 180 / M_PI * (atan2(-vel[0], -vel[2]));
    d_new_pvt.tstamp = tstamp;
    d_new_pvt.utc_time = utc_time;

    DLOG(INFO)
        << "New PVT: " << pos[1] << ", " << pos[0] << ", " << pos[2]
        << ", " << vel[0] << ", " << vel[1] << ", " << vel[2]
        << ", " << speed_over_ground << ", " << heading << ", " << tstamp;
}

void PVTConsistencyChecks::update_old_pvt()
{
    d_old_pvt.ecef_y = d_new_pvt.ecef_y;
    d_old_pvt.ecef_x = d_new_pvt.ecef_x;
    d_old_pvt.ecef_z = d_new_pvt.ecef_z;
    d_old_pvt.vel_x = d_new_pvt.vel_x;
    d_old_pvt.vel_y = d_new_pvt.vel_y;
    d_old_pvt.vel_z = d_new_pvt.vel_z;
    d_old_pvt.speed_over_ground = d_new_pvt.speed_over_ground;
    d_old_pvt.heading = d_new_pvt.heading;
    d_old_pvt.tstamp = d_new_pvt.tstamp;
    d_old_pvt.utc_time = d_new_pvt.utc_time;

    DLOG(INFO) << "Old pvt updated to: " << d_old_pvt.ecef_y << ", " << d_old_pvt.ecef_x << ", " << d_old_pvt.ecef_z;
}

void PVTConsistencyChecks::update_lkg_pvt(bool set_old)
{
    if (set_old)
        {
            d_lkg_pvt.ecef_y = d_old_pvt.ecef_y;
            d_lkg_pvt.ecef_x = d_old_pvt.ecef_x;
            d_lkg_pvt.ecef_z = d_old_pvt.ecef_z;
            d_lkg_pvt.vel_x = d_old_pvt.vel_x;
            d_lkg_pvt.vel_y = d_old_pvt.vel_y;
            d_lkg_pvt.vel_z = d_old_pvt.vel_z;
            d_lkg_pvt.speed_over_ground = d_old_pvt.speed_over_ground;
            d_lkg_pvt.heading = d_old_pvt.heading;
            d_lkg_pvt.tstamp = d_old_pvt.tstamp;
            d_lkg_pvt.utc_time = d_old_pvt.utc_time;
        }
    else
        {
            d_lkg_pvt.ecef_y = d_new_pvt.ecef_y;
            d_lkg_pvt.ecef_x = d_new_pvt.ecef_x;
            d_lkg_pvt.ecef_z = d_new_pvt.ecef_z;
            d_lkg_pvt.vel_x = d_new_pvt.vel_x;
            d_lkg_pvt.vel_y = d_new_pvt.vel_y;
            d_lkg_pvt.vel_z = d_new_pvt.vel_z;
            d_lkg_pvt.speed_over_ground = d_new_pvt.speed_over_ground;
            d_lkg_pvt.heading = d_new_pvt.heading;
            d_lkg_pvt.tstamp = d_new_pvt.tstamp;
            d_lkg_pvt.utc_time = d_new_pvt.utc_time;
        }


    DLOG(INFO) << "LKG updated to: " << d_lkg_pvt.ecef_y << ", " << d_lkg_pvt.ecef_x << ", " << d_lkg_pvt.ecef_z;
}

void PVTConsistencyChecks::reset_pos_jump_check()
{
    d_score.position_jump_score = 0;

    // Set last known good location to current coordinates
    update_lkg_pvt(false);  // Set old = false, hence set new location as lkg

    d_score.position_jump_score = 0;
    d_spoofer_score = d_score.total_score();

    d_update_lkgl = true;
}

int PVTConsistencyChecks::get_spoofer_score()
{
    int d_spoofer_score = d_score.total_score();
    DLOG(INFO) << "Total spoofer score: " << d_spoofer_score;
    return d_spoofer_score;
}

long double PVTConsistencyChecks::to_radians(double degree)
{
    // Convert degrees to radians
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

long double PVTConsistencyChecks::calculate_distance(double lat1, double lon1, double lat2, double lon2)
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

long double PVTConsistencyChecks::calculate_distance_ECEF(const PvtSol* pvtsol1, const PvtSol* pvtsol2)
{
    return sqrt(pow((pvtsol1->ecef_x - pvtsol2->ecef_x), 2) + pow((pvtsol1->ecef_y - pvtsol2->ecef_y), 2) + pow((pvtsol1->ecef_z - pvtsol2->ecef_z), 2));
}
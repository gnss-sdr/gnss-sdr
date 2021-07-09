/*!
 * \file spoofing_detector.h
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

#ifndef GNSS_SDR_SPOOFING_DETECTOR_H_
#define GNSS_SDR_SPOOFING_DETECTOR_H_

#include "configuration_interface.h"
#include "gnss_synchro.h"
#include "spoofing_detector_conf.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <map>


// Collection of PVT consistency checks
class PVTConsistencyChecks
{
public:
    // ####### Structure to store assurance score - total score and individual scores
    struct Score
    {
        int position_jump_score = 0;
        double velocity_check_score = 0;
        int static_pos_check_score = 0;
        int aux_peak_score = 0;
        int cno_score = 0;
        int agc_score = 0;
        double abnormal_position_score = 0;
        double total_score()
        {
            return (position_jump_score + velocity_check_score + static_pos_check_score + aux_peak_score + cno_score + agc_score + abnormal_position_score);
        }
    };

    // ####### Structure to store position solution
    struct PvtSol
    {
        double lat;
        double lon;
        double alt;

        double vel_x;
        double vel_y;
        double vel_z;

        double speed_over_ground;
        double heading;

        uint32_t tstamp;

        boost::posix_time::ptime utc_time;
    };

    PVTConsistencyChecks();
    PVTConsistencyChecks(const PVTConsistencyChecksConf* conf_);

    void update_pvt(double lat, double lon, double alt, double vel_x, double vel_y, double vel_z,
        double speed_over_ground, double heading,
        uint32_t tstamp, boost::posix_time::ptime utc_time);

    void check_PVT_consistency();
    void dump_results(int check_id);

    void check_RX_time();

    bool d_position_check;

    int d_spoofer_score;
    double d_assurance_score;

    const Gnss_Synchro** d_gnss_synchro;

private:
    bool d_dump_results;

    // ####### Map of last known good location. key is the check type's enumeration
    std::map<unsigned int, PvtSol> last_known_good_location;

    Score d_score;

    // ####### Position consistency
    int d_max_jump_distance;
    int d_geo_fence_radius;
    int d_velocity_difference;
    int d_pos_error_threshold;  // Spoofing detector will tolerate position error within the specified radius.
    int d_checked_velocity_pairs;
    int d_velocity_error;

    int d_min_altitude;
    int d_max_altitude;

    int d_min_ground_speed;
    int d_max_ground_speed;

    bool d_update_lkgl;
    bool d_first_record;
    bool d_static_pos_check;
    bool d_dump_pvt_checks_results;

    // Compare with new coordinates with these coordinates for position jumps
    PvtSol d_new_pvt;
    PvtSol d_old_pvt;
    PvtSol d_lkg_pvt;     // Last known good PVT sol
    PvtSol d_static_pvt;  // Static surveyed coordinates to compare received coordinates with

    void position_jump();     // Jump check, recheck with a known good location - increase score if close to known location.
    void compare_velocity();  // velocity consistency.
    void static_pos_check();  // Static position check with a known pre-determined location
    bool check_propagated_pos();
    void abnormal_position_checks();  // Check for abnormal positions

    void check_time();

    // ####### General Functions
    void update_old_pvt();
    void update_lkg_pvt(bool set_old);
    void reset_pos_jump_check();

    long double calculate_distance(double lat1, double lon1, double lat2, double lon2);
    long double to_radians(double degree);
    int get_spoofer_score();

    boost::posix_time::ptime d_pvt_epoch;
};
#endif
/*
Position Jump - 1
Compare Velocity - 2
Static Position - 3
*/
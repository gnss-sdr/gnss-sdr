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
#include "pvt_sd_conf.h"
#include <glog/logging.h>
#include <map>


struct score
{
    // ####### Structure to store assurance score - total score and individual scores
    int position_jump_score = 0;
    int velocity_check_score = 0;
    int static_pos_check_score = 0;
    int aux_peak_score = 0;
    int cno_score = 0;
    int agc_score = 0;

    int total_score()
    {
        return (position_jump_score + velocity_check_score + static_pos_check_score + aux_peak_score + cno_score + agc_score);
    }
};

struct pos_sol
{
    // ####### Structure to store position solution
    double lat;
    double lon;
    double alt;
};

class SpoofingDetector
{
public:
    SpoofingDetector();
    SpoofingDetector(const Pvt_SD_Conf* conf_);

    void check_position_consistency(double lat, double lon, double alt, const Gnss_Synchro** in);
    void dump_results(int check_id);

    bool d_position_check;

    int d_spoofer_score;
    double d_assurance_score;

private:
    bool d_dump_results;

    // ####### Map of last known good location. key is the check type's enumeration
    std::map<unsigned int, pos_sol> last_known_good_location;

    score d_score;

    // ####### Position consistency check variables
    int d_max_jump_distance;
    int d_geo_fence_radius;
    int d_velocity_difference;
    double d_static_lat;
    double d_static_lon;
    double d_static_alt;
    bool d_dump_pos_checks_results;

    int get_spoofer_score();

    // ####### Position consistency check functions
    void position_jump(double lat, double lon, double alt);  // Jump check, recheck with a known good location - increase score if close to known location.
    void compare_velocity();
    void static_pos_check(double lat, double lon, double alt);
    void set_last_known_good_location(double lat, double lon, double alt, int check_enum_id);

    // ####### General Functions
    long double calculate_distance(double lat1, double lon1, double lat2, double lon2);
    long double to_radians(double degree);
};

#endif
/*
Position Jump - 1
Compare Velocity - 2
Static Position - 3
*/
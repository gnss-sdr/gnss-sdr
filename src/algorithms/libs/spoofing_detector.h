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
#define	GNSS_SDR_SPOOFING_DETECTOR_H_

#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <vector>
#include <set>
#include "gnss_synchro.h"
#include <boost/circular_buffer.hpp>
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

struct pvt_solution_assurance
{
    // ####### Structure to store assurance score - total score and individual scores
    int position_jump_score;
    int velocity_check_score;
    int static_pos_check_score;
    int aux_peak_score;
    int cno_score;
    int agc_score;
    int total_score;
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
        SpoofingDetector(const ConfigurationInterface* configuration);
        
        void check_position_consistency(double lat, double lon, double alt, const Gnss_Synchro **in);
        void dump_results(int check_id);

        ~SpoofingDetector();

    private:
        bool d_print_assurance_score;
        bool d_dump_results;
        
        // ####### Position consistency check variables
        bool d_position_check;

        int d_max_jump_distance;
        int d_geo_fence_radius;
        int d_velocity_difference;
        int d_observation_count;

        // ####### Map of last known good location. key is the check type's enumeration
        std::map<unsigned int, pos_sol> last_known_good_location;

        // ####### Position consistency check functions
        void position_jump(double lat, double lon, double alt); // Jump check, recheck with a known good location - increase score if close to known location.
        void compare_velocity(); 
        void static_pos_check(double lat, double lon, double alt);
        void set_last_known_good_location(double lat, double lon, double alt, int check_enum_id);

        // ####### General Functions
        double distance(double lat1, double lon1, double lat2, double lon2);
};

#endif
/*
Position Jump - 1
Compare Velocity - 2
Static Position - 3
*/
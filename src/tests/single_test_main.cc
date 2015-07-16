/*!
 * \file single_test_main.cc
 * \brief  This file contains the main function for tests (used with CTest).
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include <cmath>
#include <iostream>
#include <queue>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gnuradio/msg_queue.h>
#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "gps_navigation_message.h"
#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_acq_assist.h"
#include "gps_ref_location.h"
#include "gps_ref_time.h"
#include "galileo_navigation_message.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_telemetry_data.h"
#include "sbas_ephemeris.h"
#include "sbas_satellite_correction.h"


concurrent_queue<Gps_Ephemeris> global_gps_ephemeris_queue;
concurrent_queue<Gps_Iono> global_gps_iono_queue;
concurrent_queue<Gps_Utc_Model> global_gps_utc_model_queue;
concurrent_queue<Gps_Almanac> global_gps_almanac_queue;
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_queue<Gps_Ref_Location> global_gps_ref_location_queue;
concurrent_queue<Gps_Ref_Time> global_gps_ref_time_queue;

concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
concurrent_map<Gps_Iono> global_gps_iono_map;
concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
concurrent_map<Gps_Almanac> global_gps_almanac_map;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;
concurrent_map<Gps_Ref_Location> global_gps_ref_location_map;
concurrent_map<Gps_Ref_Time> global_gps_ref_time_map;

concurrent_queue<Gps_CNAV_Ephemeris> global_gps_cnav_ephemeris_queue;
concurrent_map<Gps_CNAV_Ephemeris> global_gps_cnav_ephemeris_map;
concurrent_queue<Gps_CNAV_Iono> global_gps_cnav_iono_queue;
concurrent_map<Gps_CNAV_Iono> global_gps_cnav_iono_map;

concurrent_queue<Galileo_Ephemeris> global_galileo_ephemeris_queue;
concurrent_queue<Galileo_Iono> global_galileo_iono_queue;
concurrent_queue<Galileo_Utc_Model> global_galileo_utc_model_queue;
concurrent_queue<Galileo_Almanac> global_galileo_almanac_queue;

concurrent_map<Galileo_Ephemeris> global_galileo_ephemeris_map;
concurrent_map<Galileo_Iono> global_galileo_iono_map;
concurrent_map<Galileo_Utc_Model> global_galileo_utc_model_map;
concurrent_map<Galileo_Almanac> global_galileo_almanac_map;

concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
concurrent_queue<Sbas_Ionosphere_Correction> global_sbas_iono_queue;
concurrent_queue<Sbas_Satellite_Correction> global_sbas_sat_corr_queue;
concurrent_queue<Sbas_Ephemeris> global_sbas_ephemeris_queue;

concurrent_map<Sbas_Ionosphere_Correction> global_sbas_iono_map;
concurrent_map<Sbas_Satellite_Correction> global_sbas_sat_corr_map;
concurrent_map<Sbas_Ephemeris> global_sbas_ephemeris_map;


int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    google::InitGoogleLogging(argv[0]);
    return RUN_ALL_TESTS();
}

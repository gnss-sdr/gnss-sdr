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
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <queue>




concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

using google::LogMessage;

DECLARE_string(log_dir);

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    try
    {
            testing::InitGoogleTest(&argc, argv);
    }
    catch(...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
    google::InitGoogleLogging(argv[0]);
    int res = 0;
    try
    {
            res = RUN_ALL_TESTS();
    }
    catch(...)
    {
            LOG(WARNING) << "Unexpected catch";
    }
    google::ShutDownCommandLineFlags();
    return res;
}

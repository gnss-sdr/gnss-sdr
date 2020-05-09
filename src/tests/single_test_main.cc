/*!
 * \file single_test_main.cc
 * \brief  This file contains the main function for tests (used with CTest).
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "gps_acq_assist.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <iostream>


Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;

DECLARE_string(log_dir);

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
    google::InitGoogleLogging(argv[0]);
    int res = 0;
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
    google::ShutDownCommandLineFlags();
    return res;
}

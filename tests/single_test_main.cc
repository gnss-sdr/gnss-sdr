/*!
 * \file single_test_main.cc
 * \brief  This file contains the main function for tests (used with CTest).
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "gnss_sdr_flags.h"
#include "gps_acq_assist.h"
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>

#if USE_GLOG_AND_GFLAGS
#include <gflags/gflags.h>
#include <glog/logging.h>
#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
DECLARE_string(log_dir);
#endif
#else
#include "gnss_sdr_log_sink.h"
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/log/flags.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#endif


Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;


int main(int argc, char** argv)
try
    {
#if USE_GLOG_AND_GFLAGS
        try
            {
                testing::InitGoogleTest(&argc, argv);
                gflags::ParseCommandLineFlags(&argc, &argv, true);
            }
        catch (...)
            {
            }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
#else
        GnssSdrLogSinkGuard log_sink;
        absl::ParseCommandLine(argc, argv);
        try
            {
                testing::InitGoogleTest(&argc, argv);
            }
        catch (...)
            {
            }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
        absl::InitializeLog();
        log_sink.Register(absl::GetFlag(FLAGS_log_dir), "test");
#endif
        int res = 0;
        try
            {
                res = RUN_ALL_TESTS();
            }
        catch (...)
            {
                LOG(WARNING) << "Unexpected catch";
            }
#if USE_GLOG_AND_GFLAGS
        gflags::ShutDownCommandLineFlags();
#else
        log_sink.Shutdown();
#endif
        return res;
    }
catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
catch (...)
    {
        std::cerr << "Unexpected error\n";
        return 1;
    }

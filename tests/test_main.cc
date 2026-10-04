/*!
 * \file test_main.cc
 * \brief Entry point of the run_tests executable. The unit tests themselves
 * are aggregated in the unit_tests_*.cc translation units.
 * \author Carles Fernandez-Prades, 2012-2026 cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "unit_tests_common.h"

#if !USE_GLOG_AND_GFLAGS
#include "gnss_sdr_log_sink.h"
#endif

// For GPS NAVIGATION (L1)
Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;

int main(int argc, char** argv)
try
    {
        std::cout << "Running GNSS-SDR Tests...\n";
        int res = 0;
        try
            {
                testing::InitGoogleTest(&argc, argv);
            }
        catch (...)
            {
            }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
#if USE_GLOG_AND_GFLAGS
        gflags::ParseCommandLineFlags(&argc, &argv, true);
        google::InitGoogleLogging(argv[0]);
#else
        GnssSdrLogSinkGuard log_sink;
        absl::ParseCommandLine(argc, argv);
        absl::InitializeLog();
        log_sink.Register(absl::GetFlag(FLAGS_log_dir), "run_tests");
#endif
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

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
#include <fstream>
#include <iostream>
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
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/log/flags.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <absl/log/log_sink.h>
#include <absl/log/log_sink_registry.h>
class TestLogSink : public absl::LogSink
{
public:
    TestLogSink()
    {
        if (!absl::GetFlag(FLAGS_log_dir).empty())
            {
                logfile.open(absl::GetFlag(FLAGS_log_dir) + "/test.log");
            }
        else
            {
                logfile.open(GetTempDir() + "/test.log");
            }
    }
    void Send(const absl::LogEntry &entry) override
    {
        logfile << entry.text_message_with_prefix_and_newline() << std::flush;
    }

private:
    std::ofstream logfile;
};
#endif


Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;


int main(int argc, char **argv)
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
    absl::ParseCommandLine(argc, argv);
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
    absl::LogSink *testLogSink = new TestLogSink;
    absl::AddLogSink(testLogSink);
    absl::InitializeLog();
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
    absl::FlushLogSinks();
#endif
    return res;
}

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
class TestingLogSink : public absl::LogSink
{
public:
    TestingLogSink()
    {
        if (!absl::GetFlag(FLAGS_log_dir).empty())
            {
                filename = std::string(absl::GetFlag(FLAGS_log_dir) + "/run_tests.log");
            }
        else
            {
                filename = std::string(GetTempDir() + "/run_tests.log");
            }
        logfile.open(filename);
    }
    void Send(const absl::LogEntry& entry) override
    {
        std::lock_guard<std::mutex> lock(logfile_mutex);
        logfile << entry.text_message_with_prefix_and_newline() << std::flush;
    }

private:
    std::mutex logfile_mutex;
    std::ofstream logfile;
    std::string filename;
};

class TestingLogSinkGuard
{
public:
    TestingLogSinkGuard() = default;
    TestingLogSinkGuard(const TestingLogSinkGuard&) = delete;
    TestingLogSinkGuard& operator=(const TestingLogSinkGuard&) = delete;
    TestingLogSinkGuard(TestingLogSinkGuard&&) = delete;
    TestingLogSinkGuard& operator=(TestingLogSinkGuard&&) = delete;
    ~TestingLogSinkGuard() noexcept
    {
        Shutdown();
    }

    void Register()
    {
        log_sink.reset(new TestingLogSink);
        absl::AddLogSink(log_sink.get());
        registered = true;
        absl::InitializeLog();
    }

    void Shutdown() noexcept
    {
        if (registered)
            {
                try
                    {
                        absl::FlushLogSinks();
                    }
                catch (...)
                    {
                    }
                try
                    {
                        absl::RemoveLogSink(log_sink.get());
                    }
                catch (...)
                    {
                    }
                registered = false;
            }
        log_sink.reset();
    }

private:
    std::unique_ptr<TestingLogSink> log_sink;
    bool registered = false;
};
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
        TestingLogSinkGuard log_sink;
        absl::ParseCommandLine(argc, argv);
        log_sink.Register();
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

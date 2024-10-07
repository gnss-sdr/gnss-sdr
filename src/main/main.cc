/*!
 * \file main.cc
 * \brief Main file of the GNSS-SDR program.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * It sets up the logging system, creates a ControlThread object,
 * makes it run, and releases memory back when the main thread has ended.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2013 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VERSION
#define GNSS_SDR_VERSION "0.0.19"
#endif

#ifndef GOOGLE_STRIP_LOG
#define GOOGLE_STRIP_LOG 0
#endif

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_make_unique.h"
#include "gps_acq_assist.h"
#include <boost/exception/diagnostic_information.hpp>  // for diagnostic_information
#include <boost/exception/exception.hpp>               // for exception
#include <boost/thread/exceptions.hpp>                 // for thread_resource_error
#include <chrono>                                      // for time_point
#include <exception>                                   // for exception
#include <fstream>                                     // for ofstream
#include <iostream>                                    // for operator<<
#include <memory>                                      // for unique_ptr
#include <ostream>                                     // fro std::flush
#include <string>                                      // for string

#if USE_GLOG_AND_GFLAGS
#include <gflags/gflags.h>  // for ShutDownCommandLineFlags
#include <glog/logging.h>
#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
#endif
#else
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/flags/usage.h>
#include <absl/flags/usage_config.h>
#include <absl/log/flags.h>
#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <absl/log/log_sink.h>
#include <absl/log/log_sink_registry.h>
std::string GnssSdrVersionString() { return std::string("gnss-sdr version ") + std::string(GNSS_SDR_VERSION) + "\n"; }
class GnssSdrLogSink : public absl::LogSink
{
public:
    GnssSdrLogSink()
    {
        if (!absl::GetFlag(FLAGS_log_dir).empty())
            {
                filename = absl::GetFlag(FLAGS_log_dir) + "/gnss-sdr.log";
            }
        else
            {
                filename = GetTempDir() + "/gnss-sdr.log";
            }
        logfile.open(filename);
    }
    void Send(const absl::LogEntry& entry) override
    {
        logfile << entry.text_message_with_prefix_and_newline() << std::flush;
    }

private:
    std::ofstream logfile;
    std::string filename;
};
#endif

#if CUDA_GPU_ACCEL
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#endif


/*
 * Concurrent queues that communicates the Telemetry Decoder
 * to the Observables modules
 */

// For GPS NAVIGATION (L1)
Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;

int main(int argc, char** argv)
{
    try
        {
            const std::string intro_help(
                std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n") +
                "Copyright (C) 2010-2024 (see AUTHORS file for a list of contributors)\n" +
                "This program comes with ABSOLUTELY NO WARRANTY;\n" +
                "See COPYING file to see a copy of the General Public License\n \n");
            const std::string gnss_sdr_version(GNSS_SDR_VERSION);
#if USE_GLOG_AND_GFLAGS
            gflags::SetUsageMessage(intro_help);
            gflags::SetVersionString(gnss_sdr_version);
            gflags::ParseCommandLineFlags(&argc, &argv, true);
#else
            absl::FlagsUsageConfig empty_config;
            empty_config.version_string = &GnssSdrVersionString;
            absl::SetFlagsUsageConfig(empty_config);
            absl::SetProgramUsageMessage(intro_help);
            absl::ParseCommandLine(argc, argv);
            if (!ValidateFlags())
                {
                    std::cout << "GNSS-SDR program ended.\n";
                    return 1;
                }

#endif
            std::cout << "Initializing GNSS-SDR v" << gnss_sdr_version << " ... Please wait.\n";
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "GNSS-SDR program ended.\n";
            return 1;
        }

#if CUDA_GPU_ACCEL
    // Reset the device
    // cudaDeviceReset causes the driver to clean up all state. While
    // not mandatory in normal operation, it is good practice.  It is also
    // needed to ensure correct operation when the application is being
    // profiled. Calling cudaDeviceReset causes all profile data to be
    // flushed before the application exits
    cudaDeviceReset();
    std::cout << "Reset CUDA device done.\n";
#endif

    if (GOOGLE_STRIP_LOG == 0)
        {
#if USE_GLOG_AND_GFLAGS
            google::InitGoogleLogging(argv[0]);
            if (FLAGS_log_dir.empty())
#else
            absl::LogSink* theSink = new GnssSdrLogSink;
            absl::AddLogSink(theSink);
            absl::InitializeLog();
            if (absl::GetFlag(FLAGS_log_dir).empty())
#endif
                {
                    std::cout << "Logging will be written at "
                              << fs::temp_directory_path()
                              << '\n'
                              << "Use gnss-sdr --log_dir=/path/to/log to change that.\n";
                }
            else
                {
                    try
                        {
#if USE_GLOG_AND_GFLAGS
                            const fs::path p(FLAGS_log_dir);
#else
                            const fs::path p(absl::GetFlag(FLAGS_log_dir));
#endif
                            if (!fs::exists(p))
                                {
                                    std::cout << "The path "
#if USE_GLOG_AND_GFLAGS
                                              << FLAGS_log_dir
#else
                                              << absl::GetFlag(FLAGS_log_dir)
#endif
                                              << " does not exist, attempting to create it.\n";
                                    errorlib::error_code ec;
                                    if (!fs::create_directory(p, ec))
                                        {
#if USE_GLOG_AND_GFLAGS
                                            std::cerr << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended.\n";
                                            gflags::ShutDownCommandLineFlags();
#else
                                            std::cerr << "Could not create the " << absl::GetFlag(FLAGS_log_dir) << " folder. GNSS-SDR program ended.\n";
#endif
                                            return 1;
                                        }
                                }
#if USE_GLOG_AND_GFLAGS
                            std::cout << "Logging will be written at " << FLAGS_log_dir << '\n';
#else
                            std::cout << "Logging will be written at " << absl::GetFlag(FLAGS_log_dir) << '\n';
#endif
                        }
                    catch (const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
#if USE_GLOG_AND_GFLAGS
                            std::cerr << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended.\n";
                            gflags::ShutDownCommandLineFlags();
#else
                            std::cerr << "Could not create the " << absl::GetFlag(FLAGS_log_dir) << " folder. GNSS-SDR program ended.\n";
#endif
                            return 1;
                        }
                }
        }
#if USE_GLOG_AND_GFLAGS
#else
    else
        {
            absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfinity);  // do not log
        }
#endif
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    start = std::chrono::system_clock::now();
    int return_code = 0;
    try
        {
            auto control_thread = std::make_unique<ControlThread>();
            // record startup time
            start = std::chrono::system_clock::now();
            return_code = control_thread->run();
        }
    catch (const boost::thread_resource_error& e)
        {
            std::cerr << "Failed to create boost thread.\n";
            return_code = 1;
        }
    catch (const boost::exception& e)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                    std::cerr << boost::diagnostic_information(e) << '\n';
                }
            else
                {
                    std::cerr << "Boost exception: " << boost::diagnostic_information(e) << '\n';
                }
            return_code = 1;
        }
    catch (const std::exception& ex)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "C++ Standard Library exception: " << ex.what();
                    std::cerr << ex.what() << '\n';
                }
            else
                {
                    std::cerr << "C++ Standard Library exception: " << ex.what() << '\n';
                }
            return_code = 1;
        }
    catch (...)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Unexpected catch. This should not happen.";
                    std::cerr << "Unexpected error.\n";
                }
            else
                {
                    std::cerr << "Unexpected catch. This should not happen.\n";
                }
            return_code = 1;
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    std::cout << "Total GNSS-SDR run time: "
              << elapsed_seconds.count()
              << " [seconds]\n";

#if USE_GLOG_AND_GFLAGS
    gflags::ShutDownCommandLineFlags();
#else
    absl::FlushLogSinks();
#endif
    std::cout << "GNSS-SDR program ended.\n";
    return return_code;
}

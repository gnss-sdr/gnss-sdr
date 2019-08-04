/*!
* \file main.cc
* \brief Main file of the GNSS-SDR program.
* \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
*
* It sets up the logging system, creates a ControlThread object,
* makes it run, and releases memory back when the main thread has ended.
*
* -------------------------------------------------------------------------
*
* Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
*
* GNSS-SDR is a software defined Global Navigation
* Satellite Systems receiver
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
*/

#ifndef GNSS_SDR_VERSION
#define GNSS_SDR_VERSION "0.0.11"
#endif

#ifndef GOOGLE_STRIP_LOG
#define GOOGLE_STRIP_LOG 0
#endif

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "gps_acq_assist.h"
#include <boost/exception/diagnostic_information.hpp>  // for diagnostic_information
#include <boost/exception/exception.hpp>               // for exception
#include <boost/thread/exceptions.hpp>                 // for thread_resource_error
#include <gflags/gflags.h>                             // for ShutDownCommandLineFlags
#include <glog/logging.h>                              // for FLAGS_log_dir
#include <chrono>                                      // for time_point
#include <exception>                                   // for exception
#include <iostream>                                    // for operator<<, endl
#include <memory>                                      // for unique_ptr
#include <string>                                      // for string

#if CUDA_GPU_ACCEL
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#endif

#if HAS_STD_FILESYSTEM
#include <system_error>
namespace errorlib = std;
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <boost/system/error_code.hpp>       // for error_code
namespace fs = boost::filesystem;
namespace errorlib = boost::system;
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
    const std::string intro_help(
        std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n") +
        "Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)\n" +
        "This program comes with ABSOLUTELY NO WARRANTY;\n" +
        "See COPYING file to see a copy of the General Public License\n \n");

    const std::string gnss_sdr_version(GNSS_SDR_VERSION);
    google::SetUsageMessage(intro_help);
    google::SetVersionString(gnss_sdr_version);
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "Initializing GNSS-SDR v" << gnss_sdr_version << " ... Please wait." << std::endl;

#if CUDA_GPU_ACCEL
    // Reset the device
    // cudaDeviceReset causes the driver to clean up all state. While
    // not mandatory in normal operation, it is good practice.  It is also
    // needed to ensure correct operation when the application is being
    // profiled. Calling cudaDeviceReset causes all profile data to be
    // flushed before the application exits
    cudaDeviceReset();
    std::cout << "Reset CUDA device done " << std::endl;
#endif

    if (GOOGLE_STRIP_LOG == 0)
        {
            google::InitGoogleLogging(argv[0]);
            if (FLAGS_log_dir.empty())
                {
                    std::cout << "Logging will be written at "
                              << fs::temp_directory_path()
                              << std::endl
                              << "Use gnss-sdr --log_dir=/path/to/log to change that."
                              << std::endl;
                }
            else
                {
                    const fs::path p(FLAGS_log_dir);
                    if (!fs::exists(p))
                        {
                            std::cout << "The path "
                                      << FLAGS_log_dir
                                      << " does not exist, attempting to create it."
                                      << std::endl;
                            errorlib::error_code ec;
                            if (!fs::create_directory(p, ec))
                                {
                                    std::cerr << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended." << std::endl;
                                    google::ShutDownCommandLineFlags();
                                    return 1;
                                }
                        }
                    std::cout << "Logging will be written at " << FLAGS_log_dir << std::endl;
                }
        }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    int return_code = 0;
    try
        {
            std::unique_ptr<ControlThread> control_thread(new ControlThread());
            // record startup time
            start = std::chrono::system_clock::now();
            return_code = control_thread->run();
        }
    catch (const boost::thread_resource_error& e)
        {
            std::cerr << "Failed to create boost thread." << std::endl;
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR program ended." << std::endl;
            return 1;
        }
    catch (const boost::exception& e)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                    std::cerr << boost::diagnostic_information(e) << std::endl;
                }
            else
                {
                    std::cerr << "Boost exception: " << boost::diagnostic_information(e) << std::endl;
                }
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR program ended." << std::endl;
            return 1;
        }
    catch (const std::exception& ex)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "C++ Standard Library exception: " << ex.what();
                    std::cerr << ex.what() << std::endl;
                }
            else
                {
                    std::cerr << "C++ Standard Library exception: " << ex.what() << std::endl;
                }
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR program ended." << std::endl;
            return 1;
        }
    catch (...)
        {
            if (GOOGLE_STRIP_LOG == 0)
                {
                    LOG(WARNING) << "Unexpected catch. This should not happen.";
                    std::cerr << "Unexpected error." << std::endl;
                }
            else
                {
                    std::cerr << "Unexpected catch. This should not happen." << std::endl;
                }
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR program ended." << std::endl;
            return 1;
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    std::cout << "Total GNSS-SDR run time: "
              << elapsed_seconds.count()
              << " [seconds]" << std::endl;

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended." << std::endl;
    return return_code;
}

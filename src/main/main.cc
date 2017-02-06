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
* Copyright (C) 2010-2017 (see AUTHORS file for a list of contributors)
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
* along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
*/
#ifndef GNSS_SDR_VERSION
#define GNSS_SDR_VERSION "0.0.9"
#endif

#ifndef GOOGLE_STRIP_LOG
#define GOOGLE_STRIP_LOG 0
#endif

#include <ctime>
#include <cstdlib>
#include <memory>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception_ptr.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "control_thread.h"
#include "concurrent_queue.h"
#include "concurrent_map.h"

#if CUDA_GPU_ACCEL
    // For the CUDA runtime routines (prefixed with "cuda_")
    #include <cuda_runtime.h>
#endif


using google::LogMessage;

DECLARE_string(log_dir);

/*
* Concurrent queues that communicates the Telemetry Decoder
* to the Observables modules
*/

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

int main(int argc, char** argv)
{
    const std::string intro_help(
            std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n")
    +
    "Copyright (C) 2010-2017 (see AUTHORS file for a list of contributors)\n"
    +
    "This program comes with ABSOLUTELY NO WARRANTY;\n"
    +
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

    if(GOOGLE_STRIP_LOG == 0)
        {
            google::InitGoogleLogging(argv[0]);
            if (FLAGS_log_dir.empty())
                {
                    std::cout << "Logging will be done at "
                              << boost::filesystem::temp_directory_path()
                              << std::endl
                              << "Use gnss-sdr --log_dir=/path/to/log to change that."
                              << std::endl;
                }
            else
                {
                    const boost::filesystem::path p (FLAGS_log_dir);
                    if (!boost::filesystem::exists(p))
                        {
                            std::cout << "The path "
                                      << FLAGS_log_dir
                                      << " does not exist, attempting to create it."
                                      << std::endl;
                            boost::system::error_code ec;
                            boost::filesystem::create_directory(p, ec);
                            if(ec != 0)
                                {
                                    std::cout << "Could not create the " << FLAGS_log_dir << " folder. GNSS-SDR program ended." << std::endl;
                                    google::ShutDownCommandLineFlags();
                                    std::exit(0);
                                }
                        }
                    std::cout << "Logging with be done at " << FLAGS_log_dir << std::endl;
                }
        }

    std::unique_ptr<ControlThread> control_thread(new ControlThread());

    // record startup time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    try
    {
            control_thread->run();
    }
    catch( boost::exception & e )
    {
            LOG(FATAL) << "Boost exception: " << boost::diagnostic_information(e);
    }
    catch(std::exception const&  ex)
    {
            LOG(FATAL) << "STD exception: " << ex.what();
    }
    catch(...)
    {
            LOG(INFO) << "Unexpected catch";
    }
    // report the elapsed time
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Total GNSS-SDR run time "
              << (static_cast<double>(end - begin)) / 1000000.0
              << " [seconds]" << std::endl;

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended." << std::endl;
    return 0;
}

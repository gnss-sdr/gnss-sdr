/*!
 * \file main.cc
 * \brief Main file of the Front-end calibration program.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#ifndef FRONT_END_CAL_VERSION
#define FRONT_END_CAL_VERSION "0.0.1"
#endif

#include "GPS_L1_CA.h"  // for GPS_L1_CA_COD...
#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "configuration_interface.h"  // for Configuration...
#include "file_configuration.h"
#include "front_end_cal.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"  // for GNSSBlockInte...
#include "gnss_sdr_flags.h"
#include "gnss_synchro.h"
#include "gps_acq_assist.h"  // for Gps_Acq_Assist
#include "gps_almanac.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_utc_model.h"
#include <boost/any.hpp>  // for bad_any_cast
#include <boost/exception/exception.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gnuradio/block.h>  // for block
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/gr_complex.h>     // for gr_complex
#include <gnuradio/io_signature.h>   // for io_signature
#include <gnuradio/runtime_types.h>  // for block_sptr
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>        // for pmt_t, to_long
#include <pmt/pmt_sugar.h>  // for mp
#include <chrono>
#include <cmath>  // for round
#include <cstdint>
#include <cstdlib>
#include <ctime>  // for ctime
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>  // for logic_error
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
#endif

DECLARE_string(log_dir);

Concurrent_Map<Gps_Ephemeris> global_gps_ephemeris_map;
Concurrent_Map<Gps_Iono> global_gps_iono_map;
Concurrent_Map<Gps_Utc_Model> global_gps_utc_model_map;
Concurrent_Map<Gps_Almanac> global_gps_almanac_map;
Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;
Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

bool stop;
Concurrent_Queue<int> channel_internal_queue;
std::vector<Gnss_Synchro> gnss_sync_vector;
Gnss_Synchro gnss_synchro{};

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class FrontEndCal_msg_rx;

using FrontEndCal_msg_rx_sptr = gnss_shared_ptr<FrontEndCal_msg_rx>;

FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make();


class FrontEndCal_msg_rx : public gr::block
{
private:
    friend FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t& msg);
    FrontEndCal_msg_rx();

public:
    int rx_message;
};


FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make()
{
    return FrontEndCal_msg_rx_sptr(new FrontEndCal_msg_rx());
}


void FrontEndCal_msg_rx::msg_handler_channel_events(const pmt::pmt_t& msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (const boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!\n";
            rx_message = 0;
        }
}


FrontEndCal_msg_rx::FrontEndCal_msg_rx() : gr::block("FrontEndCal_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&FrontEndCal_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&FrontEndCal_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


void wait_message()
{
    while (!stop)
        {
            int message;
            channel_internal_queue.wait_and_pop(message);
            // std::cout<<"Acq message rx="<<message<< '\n';
            switch (message)
                {
                case 1:  // Positive acq
                    gnss_sync_vector.push_back(gnss_synchro);
                    // acquisition->reset();
                    break;
                case 2:  // negative acq
                    // acquisition->reset();
                    break;
                case 3:
                    stop = true;
                    break;
                default:
                    break;
                }
        }
}


bool front_end_capture(const std::shared_ptr<ConfigurationInterface>& configuration)
{
    gr::top_block_sptr top_block;
    GNSSBlockFactory block_factory;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;

    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    top_block = gr::make_top_block("Acquisition test");

    std::shared_ptr<GNSSBlockInterface> source;
    try
        {
            source = block_factory.GetSignalSource(configuration.get(), queue.get());
        }
    catch (const boost::exception_ptr& e)
        {
            std::cout << "Exception caught in creating source " << e << '\n';
            return false;
        }

    std::shared_ptr<GNSSBlockInterface> conditioner;
    try
        {
            conditioner = block_factory.GetSignalConditioner(configuration.get());
        }
    catch (const boost::exception_ptr& e)
        {
            std::cout << "Exception caught in creating signal conditioner " << e << '\n';
            return false;
        }
    gr::block_sptr sink;
    sink = gr::blocks::file_sink::make(sizeof(gr_complex), "tmp_capture.dat");

    // -- Find number of samples per spreading code ---
    int64_t fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", 2048000);
    int samples_per_code = round(fs_in_ / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS));
    int nsamples = samples_per_code * 50;

    int skip_samples = fs_in_ * 5;  // skip 5 seconds

    gr::block_sptr head = gr::blocks::head::make(sizeof(gr_complex), nsamples);

    gr::block_sptr skiphead = gr::blocks::skiphead::make(sizeof(gr_complex), skip_samples);

    try
        {
            source->connect(top_block);
            conditioner->connect(top_block);
            top_block->connect(source->get_right_block(), 0, conditioner->get_left_block(), 0);
            top_block->connect(conditioner->get_right_block(), 0, skiphead, 0);
            top_block->connect(skiphead, 0, head, 0);
            top_block->connect(head, 0, sink, 0);
            top_block->run();
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the GNU Radio blocks " << e.what() << '\n';
            return false;
        }

    return true;
}


static time_t utc_time(int week, int64_t tow)
{
    time_t t;

    /* Jan 5/6 midnight 1980 - beginning of GPS time as Unix time */
    t = 315964801;

    /* soon week will wrap again, uh oh... */
    /* TS 44.031: GPSTOW, range 0-604799.92, resolution 0.08 sec, 23-bit presentation */
    /* The factor 0.08 was applied in the ephemeris SUPL class */
    /* here the tow is in [s] */
    t += (1024 + week) * 604800 + tow;

    return t;
}


int main(int argc, char** argv)
{
    const std::string intro_help(
        std::string("\n RTL-SDR E4000 RF front-end center frequency and sampling rate calibration tool that uses GPS signals\n") +
        "Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)\n" +
        "This program comes with ABSOLUTELY NO WARRANTY;\n" +
        "See COPYING file to see a copy of the General Public License\n \n");

    gflags::SetUsageMessage(intro_help);
    google::SetVersionString(FRONT_END_CAL_VERSION);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "Initializing... Please wait.\n";

    google::InitGoogleLogging(argv[0]);
    if (FLAGS_log_dir.empty())
        {
            std::cout << "Logging will be done at "
                      << "/tmp"
                      << '\n'
                      << "Use front-end-cal --log_dir=/path/to/log to change that."
                      << '\n';
        }
    else
        {
            const fs::path p(FLAGS_log_dir);
            if (!fs::exists(p))
                {
                    std::cout << "The path "
                              << FLAGS_log_dir
                              << " does not exist, attempting to create it"
                              << '\n';
                    fs::create_directory(p);
                }
            std::cout << "Logging with be done at "
                      << FLAGS_log_dir << '\n';
        }

    // 0. Instantiate the FrontEnd Calibration class
    FrontEndCal front_end_cal;

    // 1. Load configuration parameters from config file
    std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<FileConfiguration>(FLAGS_config_file);
    front_end_cal.set_configuration(configuration);

    // 2. Get SUPL information from server: Ephemeris record, assistance info and TOW
    try
        {
            if (front_end_cal.get_ephemeris() == true)
                {
                    std::cout << "SUPL data received OK!\n";
                }
            else
                {
                    std::cout << "Failure connecting to SUPL server\n";
                }
        }
    catch (const boost::exception& e)
        {
            std::cout << "Failure connecting to SUPL server\n";
        }

    // 3. Capture some front-end samples to hard disk
    try
        {
            if (front_end_capture(configuration))
                {
                    std::cout << "Front-end RAW samples captured\n";
                }
            else
                {
                    std::cout << "Failure capturing front-end samples\n";
                }
        }
    catch (const boost::bad_lexical_cast& e)
        {
            std::cout << "Exception caught while capturing samples (bad lexical cast)\n";
        }
    catch (const boost::io::too_few_args& e)
        {
            std::cout << "Exception caught while capturing samples (too few args)\n";
        }
    catch (...)
        {
            std::cout << "Unexpected exception\n";
        }

    // 4. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    gnss_synchro = Gnss_Synchro();
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;

    int64_t fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", 2048000);
    configuration->set_property("Acquisition.max_dwells", "10");

    auto acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFineDoppler>(configuration.get(), "Acquisition", 1, 1);

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->set_threshold(configuration->property("Acquisition.threshold", 2.0));
    acquisition->set_doppler_max(configuration->property("Acquisition.doppler_max", 10000));
    acquisition->set_doppler_step(configuration->property("Acquisition.doppler_step", 250));

    gr::block_sptr source;
    source = gr::blocks::file_source::make(sizeof(gr_complex), "tmp_capture.dat");
#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<FrontEndCal_msg_rx> msg_rx;
#else
    boost::shared_ptr<FrontEndCal_msg_rx> msg_rx;
#endif
    try
        {
            msg_rx = FrontEndCal_msg_rx_make();
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the message port system: " << e.what() << '\n';
            exit(0);
        }

    try
        {
            acquisition->connect(top_block);
            top_block->connect(source, 0, acquisition->get_left_block(), 0);
            top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the GNU Radio blocks: " << e.what() << '\n';
        }

    // 5. Run the flowgraph
    // Get visible GPS satellites (positive acquisitions with Doppler measurements)
    // Compute Doppler estimations

    // todo: Fix the front-end cal to support new channel internal message system (no more external queues)
    std::map<int, double> doppler_measurements_map;
    std::map<int, double> cn0_measurements_map;

    std::thread ch_thread;

    // record startup time
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> elapsed_seconds{};
    start = std::chrono::system_clock::now();

    bool start_msg = true;

    for (unsigned int PRN = 1; PRN < 33; PRN++)
        {
            gnss_synchro.PRN = PRN;
            acquisition->set_gnss_synchro(&gnss_synchro);
            acquisition->init();
            acquisition->set_local_code();
            acquisition->reset();
            stop = false;
            try
                {
                    ch_thread = std::thread(wait_message);
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Exception caught (thread resource error)";
                }
            top_block->run();
            if (start_msg == true)
                {
                    std::cout << "Searching for GPS Satellites in L1 band...\n";
                    std::cout << "[";
                    start_msg = false;
                }
            if (!gnss_sync_vector.empty())
                {
                    std::cout << " " << PRN << " ";
                    double doppler_measurement_hz = 0;
                    for (auto& it : gnss_sync_vector)
                        {
                            doppler_measurement_hz += it.Acq_doppler_hz;
                        }
                    doppler_measurement_hz = doppler_measurement_hz / gnss_sync_vector.size();
                    doppler_measurements_map.insert(std::pair<int, double>(PRN, doppler_measurement_hz));
                }
            else
                {
                    std::cout << " . ";
                }
            try
                {
                    channel_internal_queue.push(3);
                }
            catch (const boost::exception& e)
                {
                    LOG(INFO) << "Exception caught while pushing to the internal queue.";
                }
            try
                {
                    ch_thread.join();
                }
            catch (const std::exception& e)
                {
                    LOG(INFO) << "Exception caught while joining threads.";
                }
            gnss_sync_vector.clear();
#if GNURADIO_USES_STD_POINTERS
            std::dynamic_pointer_cast<gr::blocks::file_source>(source)->seek(0, 0);
#else
            boost::dynamic_pointer_cast<gr::blocks::file_source>(source)->seek(0, 0);
#endif
            std::cout.flush();
        }
    std::cout << "]\n";

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]\n";

    // 6. find TOW from SUPL assistance
    double current_TOW = 0;
    try
        {
            if (global_gps_ephemeris_map.size() > 0)
                {
                    std::map<int, Gps_Ephemeris> Eph_map;
                    Eph_map = global_gps_ephemeris_map.get_map_copy();
                    current_TOW = Eph_map.begin()->second.d_TOW;

                    time_t t = utc_time(Eph_map.begin()->second.i_GPS_week, static_cast<int64_t>(current_TOW));

                    std::cout << "Reference Time:\n";
                    std::cout << "  GPS Week: " << Eph_map.begin()->second.i_GPS_week << '\n';
                    std::cout << "  GPS TOW:  " << static_cast<int64_t>(current_TOW) << " " << static_cast<int64_t>(current_TOW) * 0.08 << '\n';
                    std::cout << "  ~ UTC:    " << ctime(&t) << '\n';
                    std::cout << "Current TOW obtained from SUPL assistance = " << current_TOW << '\n';
                }
            else
                {
                    std::cout << "Unable to get Ephemeris SUPL assistance. TOW is unknown!\n";
                    gflags::ShutDownCommandLineFlags();
                    std::cout << "GNSS-SDR Front-end calibration program ended.\n";
                    return 0;
                }
        }
    catch (const boost::exception& e)
        {
            std::cout << "Exception in getting Global ephemeris map\n";
            gflags::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR Front-end calibration program ended.\n";
            return 0;
        }

    // Get user position from config file (or from SUPL using GSM Cell ID)
    double lat_deg = configuration->property("GNSS-SDR.init_latitude_deg", 41.0);
    double lon_deg = configuration->property("GNSS-SDR.init_longitude_deg", 2.0);
    double altitude_m = configuration->property("GNSS-SDR.init_altitude_m", 100);

    std::cout << "Reference location (defined in config file):\n";

    std::cout << "Latitude=" << lat_deg << " [º]\n";
    std::cout << "Longitude=" << lon_deg << " [º]\n";
    std::cout << "Altitude=" << altitude_m << " [m]\n";

    if (doppler_measurements_map.empty())
        {
            std::cout << "Sorry, no GPS satellites detected in the front-end capture, please check the antenna setup...\n";
            gflags::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR Front-end calibration program ended.\n";
            return 0;
        }

    std::map<int, double> f_if_estimation_Hz_map;
    std::map<int, double> f_fs_estimation_Hz_map;
    std::map<int, double> f_ppm_estimation_Hz_map;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << "Doppler analysis results:\n";

    std::cout << "SV ID  Measured [Hz]   Predicted [Hz]\n";

    for (auto& it : doppler_measurements_map)
        {
            try
                {
                    double doppler_estimated_hz;
                    doppler_estimated_hz = front_end_cal.estimate_doppler_from_eph(it.first, current_TOW, lat_deg, lon_deg, altitude_m);
                    std::cout << "  " << it.first << "   " << it.second << "   " << doppler_estimated_hz << '\n';
                    // 7. Compute front-end IF and sampling frequency estimation
                    // Compare with the measurements and compute clock drift using FE model
                    double estimated_fs_Hz;
                    double estimated_f_if_Hz;
                    double f_osc_err_ppm;
                    front_end_cal.GPS_L1_front_end_model_E4000(doppler_estimated_hz, it.second, fs_in_, &estimated_fs_Hz, &estimated_f_if_Hz, &f_osc_err_ppm);

                    f_if_estimation_Hz_map.insert(std::pair<int, double>(it.first, estimated_f_if_Hz));
                    f_fs_estimation_Hz_map.insert(std::pair<int, double>(it.first, estimated_fs_Hz));
                    f_ppm_estimation_Hz_map.insert(std::pair<int, double>(it.first, f_osc_err_ppm));
                }
            catch (const std::logic_error& e)
                {
                    std::cout << "Logic error caught: " << e.what() << '\n';
                }
            catch (const boost::lock_error& e)
                {
                    std::cout << "Exception caught while reading ephemeris\n";
                }
            catch (const std::exception& ex)
                {
                    std::cout << "  " << it.first << "   " << it.second << "  (Eph not found)\n";
                }
        }

    // FINAL FE estimations
    double mean_f_if_Hz = 0;
    double mean_fs_Hz = 0;
    double mean_osc_err_ppm = 0;
    int n_elements = f_if_estimation_Hz_map.size();

    for (auto& it : f_if_estimation_Hz_map)
        {
            mean_f_if_Hz += it.second;
            mean_fs_Hz += f_fs_estimation_Hz_map.find(it.first)->second;
            mean_osc_err_ppm += f_ppm_estimation_Hz_map.find(it.first)->second;
        }

    mean_f_if_Hz /= n_elements;
    mean_fs_Hz /= n_elements;
    mean_osc_err_ppm /= n_elements;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << "Parameters estimation for Elonics E4000 Front-End:\n";

    std::cout << "Sampling frequency =" << mean_fs_Hz << " [Hz]\n";
    std::cout << "IF bias present in baseband=" << mean_f_if_Hz << " [Hz]\n";
    std::cout << "Reference oscillator error =" << mean_osc_err_ppm << " [ppm]\n";

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2)
              << "Corrected Doppler vs. Predicted\n";
    std::cout << "SV ID  Corrected [Hz]   Predicted [Hz]\n";

    for (auto& it : doppler_measurements_map)
        {
            try
                {
                    double doppler_estimated_hz;
                    doppler_estimated_hz = front_end_cal.estimate_doppler_from_eph(it.first, current_TOW, lat_deg, lon_deg, altitude_m);
                    std::cout << "  " << it.first << "   " << it.second - mean_f_if_Hz << "   " << doppler_estimated_hz << '\n';
                }
            catch (const std::logic_error& e)
                {
                    std::cout << "Logic error caught: " << e.what() << '\n';
                }
            catch (const boost::lock_error& e)
                {
                    std::cout << "Exception caught while reading ephemeris\n";
                }
            catch (const std::exception& ex)
                {
                    std::cout << "  " << it.first << "   " << it.second - mean_f_if_Hz << "  (Eph not found)\n";
                }
        }

    gflags::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR Front-end calibration program ended.\n";
}

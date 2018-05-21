/*!
 * \file main.cc
 * \brief Main file of the Front-end calibration program.
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#ifndef FRONT_END_CAL_VERSION
#define FRONT_END_CAL_VERSION "0.0.1"
#endif

#include "front_end_cal.h"
#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "file_configuration.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gnss_signal.h"
#include "gnss_synchro.h"
#include "gnss_block_factory.h"
#include "gps_navigation_message.h"
#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_cnav_iono.h"
#include "gps_utc_model.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "sbas_ephemeris.h"
#include "gnss_sdr_supl_client.h"
#include "gnss_sdr_flags.h"
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/exception/detail/exception_ptr.hpp>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/file_sink.h>
#include <stdlib.h>
#include <chrono>
#include <ctime>  // for ctime
#include <exception>
#include <memory>
#include <queue>
#include <vector>


using google::LogMessage;

DECLARE_string(log_dir);

concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
concurrent_map<Gps_Iono> global_gps_iono_map;
concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
concurrent_map<Gps_Almanac> global_gps_almanac_map;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

bool stop;
concurrent_queue<int> channel_internal_queue;
GpsL1CaPcpsAcquisitionFineDoppler* acquisition;
Gnss_Synchro* gnss_synchro;
std::vector<Gnss_Synchro> gnss_sync_vector;


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class FrontEndCal_msg_rx;

typedef boost::shared_ptr<FrontEndCal_msg_rx> FrontEndCal_msg_rx_sptr;

FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make();


class FrontEndCal_msg_rx : public gr::block
{
private:
    friend FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    FrontEndCal_msg_rx();

public:
    int rx_message;
    ~FrontEndCal_msg_rx();  //!< Default destructor
};


FrontEndCal_msg_rx_sptr FrontEndCal_msg_rx_make()
{
    return FrontEndCal_msg_rx_sptr(new FrontEndCal_msg_rx());
}


void FrontEndCal_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!\n";
            rx_message = 0;
        }
}


FrontEndCal_msg_rx::FrontEndCal_msg_rx() : gr::block("FrontEndCal_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&FrontEndCal_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


FrontEndCal_msg_rx::~FrontEndCal_msg_rx() {}


// ###########################################################

void wait_message()
{
    while (!stop)
        {
            int message;
            channel_internal_queue.wait_and_pop(message);
            //std::cout<<"Acq message rx="<<message<<std::endl;
            switch (message)
                {
                case 1:  // Positive acq
                    gnss_sync_vector.push_back(*gnss_synchro);
                    //acquisition->reset();
                    break;
                case 2:  // negative acq
                    //acquisition->reset();
                    break;
                case 3:
                    stop = true;
                    break;
                default:
                    break;
                }
        }
}


bool front_end_capture(std::shared_ptr<ConfigurationInterface> configuration)
{
    gr::top_block_sptr top_block;
    GNSSBlockFactory block_factory;
    boost::shared_ptr<gr::msg_queue> queue;

    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Acquisition test");

    std::shared_ptr<GNSSBlockInterface> source;
    try
        {
            source = block_factory.GetSignalSource(configuration, queue);
        }
    catch (const boost::exception_ptr& e)
        {
            std::cout << "Exception caught in creating source " << e << std::endl;
            return 0;
        }

    std::shared_ptr<GNSSBlockInterface> conditioner;
    try
        {
            conditioner = block_factory.GetSignalConditioner(configuration);
        }
    catch (const boost::exception_ptr& e)
        {
            std::cout << "Exception caught in creating signal conditioner " << e << std::endl;
            return 0;
        }
    gr::block_sptr sink;
    sink = gr::blocks::file_sink::make(sizeof(gr_complex), "tmp_capture.dat");

    //--- Find number of samples per spreading code ---
    long fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", 2048000);
    int samples_per_code = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
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
            std::cout << "Failure connecting the GNU Radio blocks " << e.what() << std::endl;
            return false;
        }

    //delete conditioner;
    //delete source;
    return true;
}


static time_t utc_time(int week, long tow)
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
        "Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)\n" +
        "This program comes with ABSOLUTELY NO WARRANTY;\n" +
        "See COPYING file to see a copy of the General Public License\n \n");

    google::SetUsageMessage(intro_help);
    google::SetVersionString(FRONT_END_CAL_VERSION);
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "Initializing... Please wait." << std::endl;

    google::InitGoogleLogging(argv[0]);
    if (FLAGS_log_dir.empty())
        {
            std::cout << "Logging will be done at "

                      << "/tmp"
                      << std::endl
                      << "Use front-end-cal --log_dir=/path/to/log to change that."
                      << std::endl;
        }
    else
        {
            const boost::filesystem::path p(FLAGS_log_dir);
            if (!boost::filesystem::exists(p))
                {
                    std::cout << "The path "
                              << FLAGS_log_dir
                              << " does not exist, attempting to create it"
                              << std::endl;
                    boost::filesystem::create_directory(p);
                }
            std::cout << "Logging with be done at "
                      << FLAGS_log_dir << std::endl;
        }


    // 0. Instantiate the FrontEnd Calibration class
    FrontEndCal front_end_cal;

    // 1. Load configuration parameters from config file

    std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<FileConfiguration>(FLAGS_config_file);

    front_end_cal.set_configuration(configuration);


    // 2. Get SUPL information from server: Ephemeris record, assistance info and TOW
    if (front_end_cal.get_ephemeris() == true)
        {
            std::cout << "SUPL data received OK!" << std::endl;
        }
    else
        {
            std::cout << "Failure connecting to SUPL server" << std::endl;
        }

    // 3. Capture some front-end samples to hard disk
    try
        {
            if (front_end_capture(configuration))
                {
                    std::cout << "Front-end RAW samples captured" << std::endl;
                }
            else
                {
                    std::cout << "Failure capturing front-end samples" << std::endl;
                }
        }
    catch (const boost::bad_lexical_cast& e)
        {
            std::cout << "Exception caught while capturing samples (bad lexical cast)" << std::endl;
        }
    catch (const boost::io::too_few_args& e)
        {
            std::cout << "Exception caught while capturing samples (too few args)" << std::endl;
        }
    catch (...)
        {
            std::cout << "Unexpected exception" << std::endl;
        }

    // 4. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    gnss_synchro = new Gnss_Synchro();
    gnss_synchro->Channel_ID = 0;
    gnss_synchro->System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro->Signal, 2, 0);
    gnss_synchro->PRN = 1;

    long fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", 2048000);

    GNSSBlockFactory block_factory;
    acquisition = new GpsL1CaPcpsAcquisitionFineDoppler(configuration.get(), "Acquisition", 1, 1);

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(gnss_synchro);
    acquisition->set_threshold(configuration->property("Acquisition.threshold", 0.0));
    acquisition->set_doppler_max(configuration->property("Acquisition.doppler_max", 10000));
    acquisition->set_doppler_step(configuration->property("Acquisition.doppler_step", 250));

    gr::block_sptr source;
    source = gr::blocks::file_source::make(sizeof(gr_complex), "tmp_capture.dat");
    boost::shared_ptr<FrontEndCal_msg_rx> msg_rx;
    try
        {
            msg_rx = FrontEndCal_msg_rx_make();
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the message port system: " << e.what() << std::endl;
            exit(0);
        }

    //gr_basic_block_sptr head = gr_make_head(sizeof(gr_complex), nsamples);
    //gr_head_sptr head_sptr = boost::dynamic_pointer_cast<gr_head>(head);
    //head_sptr->set_length(nsamples);
    //head_sptr->reset();

    try
        {
            acquisition->connect(top_block);
            top_block->connect(source, 0, acquisition->get_left_block(), 0);
            top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the GNU Radio blocks: " << e.what() << std::endl;
        }

    // 5. Run the flowgraph
    // Get visible GPS satellites (positive acquisitions with Doppler measurements)
    // Compute Doppler estimations

    //todo: Fix the front-end cal to support new channel internal message system (no more external queues)
    std::map<int, double> doppler_measurements_map;
    std::map<int, double> cn0_measurements_map;

    boost::thread ch_thread;

    // record startup time
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;
    start = std::chrono::system_clock::now();

    bool start_msg = true;

    for (unsigned int PRN = 1; PRN < 33; PRN++)
        {
            gnss_synchro->PRN = PRN;
            acquisition->set_gnss_synchro(gnss_synchro);
            acquisition->init();
            acquisition->set_local_code();
            acquisition->reset();
            stop = false;
            try
                {
                    ch_thread = boost::thread(wait_message);
                }
            catch (const boost::thread_resource_error& e)
                {
                    LOG(INFO) << "Exception caught (thread resource error)";
                }
            top_block->run();
            if (start_msg == true)
                {
                    std::cout << "Searching for GPS Satellites in L1 band..." << std::endl;
                    std::cout << "[";
                    start_msg = false;
                }
            if (gnss_sync_vector.size() > 0)
                {
                    std::cout << " " << PRN << " ";
                    double doppler_measurement_hz = 0;
                    for (std::vector<Gnss_Synchro>::iterator it = gnss_sync_vector.begin(); it != gnss_sync_vector.end(); ++it)
                        {
                            doppler_measurement_hz += (*it).Acq_doppler_hz;
                        }
                    doppler_measurement_hz = doppler_measurement_hz / gnss_sync_vector.size();
                    doppler_measurements_map.insert(std::pair<int, double>(PRN, doppler_measurement_hz));
                }
            else
                {
                    std::cout << " . ";
                }
            channel_internal_queue.push(3);
            try
                {
                    ch_thread.join();
                }
            catch (const boost::thread_resource_error& e)
                {
                    LOG(INFO) << "Exception caught while joining threads.";
                }
            gnss_sync_vector.clear();
            boost::dynamic_pointer_cast<gr::blocks::file_source>(source)->seek(0, 0);
            std::cout.flush();
        }
    std::cout << "]" << std::endl;

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]" << std::endl;

    //6. find TOW from SUPL assistance

    double current_TOW = 0;
    if (global_gps_ephemeris_map.size() > 0)
        {
            std::map<int, Gps_Ephemeris> Eph_map;
            try
                {
                    Eph_map = global_gps_ephemeris_map.get_map_copy();
                }
            catch (const boost::exception& e)
                {
                    std::cout << "Exception in getting Global ephemeris map" << std::endl;
                    delete acquisition;
                    delete gnss_synchro;
                    google::ShutDownCommandLineFlags();
                    std::cout << "GNSS-SDR Front-end calibration program ended." << std::endl;
                    return 0;
                }
            current_TOW = Eph_map.begin()->second.d_TOW;

            time_t t = utc_time(Eph_map.begin()->second.i_GPS_week, (long int)current_TOW);

            fprintf(stdout, "Reference Time:\n");
            fprintf(stdout, "  GPS Week: %d\n", Eph_map.begin()->second.i_GPS_week);
            fprintf(stdout, "  GPS TOW:  %ld %lf\n", (long int)current_TOW, (long int)current_TOW * 0.08);
            fprintf(stdout, "  ~ UTC:    %s", ctime(&t));
            std::cout << "Current TOW obtained from SUPL assistance = " << current_TOW << std::endl;
        }
    else
        {
            std::cout << "Unable to get Ephemeris SUPL assistance. TOW is unknown!" << std::endl;
            delete acquisition;
            delete gnss_synchro;
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR Front-end calibration program ended." << std::endl;
            return 0;
        }

    //Get user position from config file (or from SUPL using GSM Cell ID)
    double lat_deg = configuration->property("GNSS-SDR.init_latitude_deg", 41.0);
    double lon_deg = configuration->property("GNSS-SDR.init_longitude_deg", 2.0);
    double altitude_m = configuration->property("GNSS-SDR.init_altitude_m", 100);

    std::cout << "Reference location (defined in config file):" << std::endl;

    std::cout << "Latitude=" << lat_deg << " [�]" << std::endl;
    std::cout << "Longitude=" << lon_deg << " [�]" << std::endl;
    std::cout << "Altitude=" << altitude_m << " [m]" << std::endl;

    if (doppler_measurements_map.size() == 0)
        {
            std::cout << "Sorry, no GPS satellites detected in the front-end capture, please check the antenna setup..." << std::endl;
            delete acquisition;
            delete gnss_synchro;
            google::ShutDownCommandLineFlags();
            std::cout << "GNSS-SDR Front-end calibration program ended." << std::endl;
            return 0;
        }

    std::map<int, double> f_if_estimation_Hz_map;
    std::map<int, double> f_fs_estimation_Hz_map;
    std::map<int, double> f_ppm_estimation_Hz_map;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << "Doppler analysis results:" << std::endl;

    std::cout << "SV ID  Measured [Hz]   Predicted [Hz]" << std::endl;

    for (std::map<int, double>::iterator it = doppler_measurements_map.begin(); it != doppler_measurements_map.end(); ++it)
        {
            try
                {
                    double doppler_estimated_hz;
                    doppler_estimated_hz = front_end_cal.estimate_doppler_from_eph(it->first, current_TOW, lat_deg, lon_deg, altitude_m);
                    std::cout << "  " << it->first << "   " << it->second << "   " << doppler_estimated_hz << std::endl;
                    // 7. Compute front-end IF and sampling frequency estimation
                    // Compare with the measurements and compute clock drift using FE model
                    double estimated_fs_Hz, estimated_f_if_Hz, f_osc_err_ppm;
                    front_end_cal.GPS_L1_front_end_model_E4000(doppler_estimated_hz, it->second, fs_in_, &estimated_fs_Hz, &estimated_f_if_Hz, &f_osc_err_ppm);

                    f_if_estimation_Hz_map.insert(std::pair<int, double>(it->first, estimated_f_if_Hz));
                    f_fs_estimation_Hz_map.insert(std::pair<int, double>(it->first, estimated_fs_Hz));
                    f_ppm_estimation_Hz_map.insert(std::pair<int, double>(it->first, f_osc_err_ppm));
                }
            catch (const std::logic_error& e)
                {
                    std::cout << "Logic error caught: " << e.what() << std::endl;
                }
            catch (const boost::lock_error& e)
                {
                    std::cout << "Exception caught while reading ephemeris" << std::endl;
                }
            catch (int ex)
                {
                    std::cout << "  " << it->first << "   " << it->second << "  (Eph not found)" << std::endl;
                }
        }

    // FINAL FE estimations
    double mean_f_if_Hz = 0;
    double mean_fs_Hz = 0;
    double mean_osc_err_ppm = 0;
    int n_elements = f_if_estimation_Hz_map.size();

    for (std::map<int, double>::iterator it = f_if_estimation_Hz_map.begin(); it != f_if_estimation_Hz_map.end(); ++it)
        {
            mean_f_if_Hz += (*it).second;
            mean_fs_Hz += f_fs_estimation_Hz_map.find((*it).first)->second;
            mean_osc_err_ppm += f_ppm_estimation_Hz_map.find((*it).first)->second;
        }

    mean_f_if_Hz /= n_elements;
    mean_fs_Hz /= n_elements;
    mean_osc_err_ppm /= n_elements;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << "Parameters estimation for Elonics E4000 Front-End:" << std::endl;

    std::cout << "Sampling frequency =" << mean_fs_Hz << " [Hz]" << std::endl;
    std::cout << "IF bias present in baseband=" << mean_f_if_Hz << " [Hz]" << std::endl;
    std::cout << "Reference oscillator error =" << mean_osc_err_ppm << " [ppm]" << std::endl;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2)
              << "Corrected Doppler vs. Predicted" << std::endl;
    std::cout << "SV ID  Corrected [Hz]   Predicted [Hz]" << std::endl;

    for (std::map<int, double>::iterator it = doppler_measurements_map.begin(); it != doppler_measurements_map.end(); ++it)
        {
            try
                {
                    double doppler_estimated_hz;
                    doppler_estimated_hz = front_end_cal.estimate_doppler_from_eph(it->first, current_TOW, lat_deg, lon_deg, altitude_m);
                    std::cout << "  " << it->first << "   " << it->second - mean_f_if_Hz << "   " << doppler_estimated_hz << std::endl;
                }
            catch (const std::logic_error& e)
                {
                    std::cout << "Logic error caught: " << e.what() << std::endl;
                }
            catch (const boost::lock_error& e)
                {
                    std::cout << "Exception caught while reading ephemeris" << std::endl;
                }
            catch (int ex)
                {
                    std::cout << "  " << it->first << "   " << it->second - mean_f_if_Hz << "  (Eph not found)" << std::endl;
                }
        }

    delete acquisition;
    delete gnss_synchro;

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR Front-end calibration program ended." << std::endl;
}

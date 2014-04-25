concurrent_queue<Gps_Utc_Model> global_gps_utc_model_queue;
concurrent_queue<Gps_Almanac> global_gps_almanac_queue;
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_queue<Gps_Ref_Location> global_gps_ref_location_queue;
concurrent_queue<Gps_Ref_Time> global_gps_ref_time_queue;

concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
concurrent_map<Gps_Iono> global_gps_iono_map;
concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
concurrent_map<Gps_Almanac> global_gps_almanac_map;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;
concurrent_map<Gps_Ref_Time> global_gps_ref_time_map;
concurrent_map<Gps_Ref_Location> global_gps_ref_location_map;

// For GALILEO NAVIGATION
concurrent_queue<Galileo_Ephemeris> global_galileo_ephemeris_queue;
concurrent_queue<Galileo_Iono> global_galileo_iono_queue;
concurrent_queue<Galileo_Utc_Model> global_galileo_utc_model_queue;
concurrent_queue<Galileo_Almanac> global_galileo_almanac_queue;

concurrent_map<Galileo_Ephemeris> global_galileo_ephemeris_map;
concurrent_map<Galileo_Iono> global_galileo_iono_map;
concurrent_map<Galileo_Utc_Model> global_galileo_utc_model_map;
concurrent_map<Galileo_Almanac> global_galileo_almanac_map;

// For SBAS CORRECTIONS
concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
concurrent_queue<Sbas_Ionosphere_Correction> global_sbas_iono_queue;
concurrent_queue<Sbas_Satellite_Correction> global_sbas_sat_corr_queue;
concurrent_queue<Sbas_Ephemeris> global_sbas_ephemeris_queue;

concurrent_map<Sbas_Ionosphere_Correction> global_sbas_iono_map;
concurrent_map<Sbas_Satellite_Correction> global_sbas_sat_corr_map;
concurrent_map<Sbas_Ephemeris> global_sbas_ephemeris_map;

int main(int argc, char** argv)
{
    const std::string intro_help(
            std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n")
    +
    "Copyright (C) 2010-2014 (see AUTHORS file for a list of contributors)\n"
    +
    "This program comes with ABSOLUTELY NO WARRANTY;\n"
    +
    "See COPYING file to see a copy of the General Public License\n \n");

    const std::string gnss_sdr_version(GNSS_SDR_VERSION);
    google::SetUsageMessage(intro_help);
    google::SetVersionString(gnss_sdr_version);
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "Initializing GNSS-SDR v" << gnss_sdr_version << " ... Please wait." << std::endl;

    google::InitGoogleLogging(argv[0]);
    if (FLAGS_log_dir.empty())
        {
             // temp_directory_path() is only available from
             // Boost 1.45. Ubuntu 10.10 and Debian 6.0.6 ships with 1.42
             std::cout << "Logging will be done at "
                 //<< boost::filesystem::temp_directory_path()
                 << "/tmp"
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
                        << " does not exist, attempting to create it"
                        << std::endl;
                    boost::filesystem::create_directory(p);
                }
            std::cout << "Logging with be done at "
                      << FLAGS_log_dir << std::endl;
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
    // report the elapsed time
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Total GNSS-SDR run time "
              << ((double)(end - begin))/1000000.0
              << " [seconds]" << std::endl;

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended." << std::endl;
}

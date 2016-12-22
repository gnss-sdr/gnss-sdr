
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <exception>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include "RinexUtilities.hpp"
#include "control_thread.h"
#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "in_memory_configuration.h"



// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

class Trk_System_Test: public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2_static;
    std::string p2_dynamic;
    std::string p3;
    std::string p4;
    std::string p5;

    const int baseband_sampling_freq = 2.6e6;

    std::string filename_rinex_obs = "sim.16o";
    std::string filename_raw_data = "signal_out.bin";

    int configure_generator();
    int generate_signal();
    int configure_receiver();
    int run_receiver();
    int check_results();
    bool check_valid_rinex_nav(std::string filename);  // return true if the file is a valid Rinex navigation file.
    bool check_valid_rinex_obs(std::string filename);  // return true if the file is a valid Rinex observation file.

    std::shared_ptr<InMemoryConfiguration> config;
};


bool Trk_System_Test::check_valid_rinex_nav(std::string filename)
{
    bool res = false;
    res = gpstk::isRinexNavFile(filename);
    return res;
}


bool Trk_System_Test::check_valid_rinex_obs(std::string filename)
{
    bool res = false;
    res = gpstk::isRinexObsFile(filename);
    return res;
}


int Trk_System_Test::configure_generator()
{
    // Configure signal
    generator_binary = std::string(SW_GENERATOR_BIN);

    EXPECT_EQ(true, check_valid_rinex_nav(std::string(DEFAULT_RINEX_NAV)));

    p1 = std::string("-rinex_nav_file=") + std::string(DEFAULT_RINEX_NAV);
    p2_static = std::string("-static_position=30.286502,120.032669,100,1000");
    p2_dynamic = std::string("-obs_pos_file=") + std::string(DEFAULT_POSITION_FILE); // Observer positions file, in .csv or .nmea format"
    p3 = std::string("-rinex_obs_file=") + filename_rinex_obs; // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + filename_raw_data; // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq); //Baseband sampling frequency [MSps]
    return 0;
}


int Trk_System_Test::generate_signal()
{
    pid_t wait_result;
    int child_status;

    char *const parmList[] = { &generator_binary[0], &generator_binary[0], &p1[0], &p2_static[0], &p3[0], &p4[0], &p5[0], NULL };

    int pid;
    if ((pid = fork()) == -1)
        perror("fork error");
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv error." << std::endl;
            std::terminate();
        }

    wait_result = waitpid(pid, &child_status, 0);

    EXPECT_EQ(true, check_valid_rinex_obs(filename_rinex_obs));
    std::cout << "Signal and Observables RINEX files created."  << std::endl;
    return 0;
}


int Trk_System_Test::configure_receiver()
{
    config = std::make_shared<InMemoryConfiguration>();

    const double central_freq = 1575420000.0;
    const int sampling_rate_internal = baseband_sampling_freq;
    const double gain_dB = 40.0;

    const int number_of_taps = 11;
    const int number_of_bands = 2;
    const float band1_begin = 0.0;
    const float band1_end = 0.48;
    const float band2_begin = 0.52;
    const float band2_end = 1.0;
    const float ampl1_begin = 1.0;
    const float ampl1_end = 1.0;
    const float ampl2_begin = 0.0;
    const float ampl2_end = 0.0;
    const float band1_error = 1.0;
    const float band2_error = 1.0;
    const int grid_density = 16;
    const int decimation_factor = 1;

    const float zero = 0.0;
    const int number_of_channels = 8;
    const int in_acquisition = 1;

    const float threshold = 0.01;
    const float doppler_max = 8000.0;
    const float doppler_step = 500.0;
    const int max_dwells = 1;
    const int tong_init_val = 2;
    const int tong_max_val = 10;
    const int tong_max_dwells = 30;
    const int coherent_integration_time_ms = 1;

    const float pll_bw_hz = 30.0;
    const float dll_bw_hz = 4.0;
    const float early_late_space_chips = 0.5;

    const int display_rate_ms = 500;
    const int output_rate_ms = 1000;
    const int averaging_depth = 10;

    bool false_bool = false;

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(sampling_rate_internal));

    // Set the assistance system parameters
    config->set_property("GNSS-SDR.SUPL_read_gps_assistance_xml", "false");
    config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
    config->set_property("GNSS-SDR.SUPL_gps_ephemeris_server", "supl.google.com");
    config->set_property("GNSS-SDR.SUPL_gps_ephemeris_port", std::to_string(7275));
    config->set_property("GNSS-SDR.SUPL_gps_acquisition_server", "supl.google.com");
    config->set_property("GNSS-SDR.SUPL_gps_acquisition_port", std::to_string(7275));
    config->set_property("GNSS-SDR.SUPL_MCC", std::to_string(244));
    config->set_property("GNSS-SDR.SUPL_MNS", std::to_string(5));
    config->set_property("GNSS-SDR.SUPL_LAC", "0x59e2");
    config->set_property("GNSS-SDR.SUPL_CI", "0x31b0");

    // Set the Signal Source
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.filename", "./" + filename_raw_data);
    config->set_property("SignalSource.sampling_frequency", std::to_string(sampling_rate_internal));
    config->set_property("SignalSource.item_type", "ibyte");
    config->set_property("SignalSource.samples", std::to_string(zero));

    // Set the Signal Conditioner
    config->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    config->set_property("DataTypeAdapter.implementation", "Ibyte_To_Complex");
    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.dump", "false");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", std::to_string(number_of_taps));
    config->set_property("InputFilter.number_of_bands", std::to_string(number_of_bands));
    config->set_property("InputFilter.band1_begin", std::to_string(band1_begin));
    config->set_property("InputFilter.band1_end", std::to_string(band1_end));
    config->set_property("InputFilter.band2_begin", std::to_string(band2_begin));
    config->set_property("InputFilter.band2_end", std::to_string(band2_end));
    config->set_property("InputFilter.ampl1_begin", std::to_string(ampl1_begin));
    config->set_property("InputFilter.ampl1_end", std::to_string(ampl1_end));
    config->set_property("InputFilter.ampl2_begin", std::to_string(ampl2_begin));
    config->set_property("InputFilter.ampl2_end", std::to_string(ampl2_end));
    config->set_property("InputFilter.band1_error", std::to_string(band1_error));
    config->set_property("InputFilter.band2_error", std::to_string(band2_error));
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", std::to_string(grid_density));
    config->set_property("InputFilter.sampling_frequency", std::to_string(sampling_rate_internal));
    config->set_property("InputFilter.IF", std::to_string(zero));
    config->set_property("Resampler.implementation", "Pass_Through");
    config->set_property("Resampler.dump", "false");
    config->set_property("Resampler.item_type", "gr_complex");
    config->set_property("Resampler.sample_freq_in", std::to_string(sampling_rate_internal));
    config->set_property("Resampler.sample_freq_out", std::to_string(sampling_rate_internal));

    // Set the number of Channels
    config->set_property("Channels_1C.count", std::to_string(number_of_channels));
    config->set_property("Channels.in_acquisition", std::to_string(in_acquisition));
    config->set_property("Channel.signal", "1C");

    // Set Acquisition
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Tong_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.if", std::to_string(zero));
    config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
    config->set_property("Acquisition_1C.threshold", std::to_string(threshold));
    config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition_1C.bit_transition_flag", "false");
    config->set_property("Acquisition_1C.max_dwells", std::to_string(max_dwells));
    config->set_property("Acquisition_1C.tong_init_val", std::to_string(tong_init_val));
    config->set_property("Acquisition_1C.tong_max_val", std::to_string(tong_max_val));
    config->set_property("Acquisition_1C.tong_max_dwells", std::to_string(tong_max_dwells));

    // Set Tracking
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", std::to_string(zero));
    config->set_property("Tracking_1C.dump", "false");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", std::to_string(pll_bw_hz));
    config->set_property("Tracking_1C.dll_bw_hz", std::to_string(dll_bw_hz));
    config->set_property("Tracking_1C.early_late_space_chips", std::to_string(early_late_space_chips));

    // Set Telemetry
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.dump", "false");
    config->set_property("TelemetryDecoder_1C.decimation_factor", std::to_string(decimation_factor));

    // Set Observables
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("Observables.dump", "false");
    config->set_property("Observables.dump_filename", "./observables.dat");

    // Set PVT
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("PVT.averaging_depth", std::to_string(averaging_depth));
    config->set_property("PVT.flag_averaging", "true");
    config->set_property("PVT.output_rate_ms", std::to_string(output_rate_ms));
    config->set_property("PVT.display_rate_ms", std::to_string(display_rate_ms));
    config->set_property("PVT.dump_filename", "./PVT");
    config->set_property("PVT.nmea_dump_filename", "./gnss_sdr_pvt.nmea");
    config->set_property("PVT.flag_nmea_tty_port", "false");
    config->set_property("PVT.nmea_dump_devname", "/dev/pts/4");
    config->set_property("PVT.flag_rtcm_server", "false");
    config->set_property("PVT.flag_rtcm_tty_port", "false");
    config->set_property("PVT.rtcm_dump_devname", "/dev/pts/1");
    config->set_property("PVT.dump", "false");
    config->set_property("PVT.rinex_version", std::to_string(2));

    return 0;
}


int Trk_System_Test::run_receiver()
{
    std::shared_ptr<ControlThread> control_thread;
    control_thread = std::make_shared<ControlThread>(config);
    // start receiver
    try
    {
            control_thread->run();
    }
    catch( boost::exception & e )
    {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
    }
    catch(std::exception const&  ex)
    {
            std::cout  << "STD exception: " << ex.what();
    }
    return 0;
}


int Trk_System_Test::check_results()
{
    // Open reference RINEX observables file

    // Open generated RINEX observables file

    // Time alignment!

    // Read reference pseudoranges from a given satellite

    // Read obtained pseudoranges from a given satellite

    // Compute pseudorange error

    // Read reference carrier phase from a given satellite

    // Read obtained carrier phase from a given satellite

    // Compute carrier phase error
    return 0;
}


TEST_F(Trk_System_Test, Tracking_system_test)
{
    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    generate_signal();

    // Configure receiver
    configure_receiver();

    // Run the receiver
    run_receiver();

    // Check results
    check_results();
}


int main(int argc, char **argv)
{
    std::cout << "Running Tracking validation test..." << std::endl;
    int res = 0;
    try
    {
            testing::InitGoogleTest(&argc, argv);
    }
    catch(...) {} // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Run the Tests
    try
    {
            res = RUN_ALL_TESTS();
    }
    catch(...)
    {
            LOG(WARNING) << "Unexpected catch";
    }
    google::ShutDownCommandLineFlags();
    return res;
}

/*!
 * \file position_test.cc
 * \brief  This class implements a test for the validation of computed position.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
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

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "in_memory_configuration.h"
#include "file_configuration.h"
#include "MATH_CONSTANTS.h"
#include "gnuplot_i.h"
#include "test_flags.h"
#include "signal_generator_flags.h"
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>
#include <thread>


DEFINE_string(config_file_ptest, std::string(""), "File containing the configuration parameters for the position test.");
DEFINE_bool(plot_position_test, false, "Plots results of FFTLengthTest with gnuplot");

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

class StaticPositionSystemTest : public ::testing::Test
{
public:
    int configure_generator();
    int generate_signal();
    int configure_receiver();
    int run_receiver();
    void check_results();

private:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    const double baseband_sampling_freq = static_cast<double>(FLAGS_fs_gen_sps);

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    void print_results(const std::vector<double>& east,
        const std::vector<double>& north,
        const std::vector<double>& up);

    double compute_stdev_precision(const std::vector<double>& vec);
    double compute_stdev_accuracy(const std::vector<double>& vec, double ref);

    void geodetic2Enu(const double latitude, const double longitude, const double altitude,
        double* east, double* north, double* up);

    void geodetic2Ecef(const double latitude, const double longitude, const double altitude,
        double* x, double* y, double* z);

    std::shared_ptr<InMemoryConfiguration> config;
    std::shared_ptr<FileConfiguration> config_f;
    std::string generated_kml_file;
};


void StaticPositionSystemTest::geodetic2Ecef(const double latitude, const double longitude, const double altitude,
    double* x, double* y, double* z)
{
    const double a = 6378137.0;       // WGS84
    const double b = 6356752.314245;  // WGS84

    double aux_x, aux_y, aux_z;

    // Convert to ECEF (See https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates )
    const double cLat = cos(latitude);
    const double cLon = cos(longitude);
    const double sLon = sin(longitude);
    const double sLat = sin(latitude);
    double N = std::pow(a, 2.0) / sqrt(std::pow(a, 2.0) * std::pow(cLat, 2.0) + std::pow(b, 2.0) * std::pow(sLat, 2.0));

    aux_x = (N + altitude) * cLat * cLon;
    aux_y = (N + altitude) * cLat * sLon;
    aux_z = ((std::pow(b, 2.0) / std::pow(a, 2.0)) * N + altitude) * sLat;

    *x = aux_x;
    *y = aux_y;
    *z = aux_z;
}


void StaticPositionSystemTest::geodetic2Enu(double latitude, double longitude, double altitude,
    double* east, double* north, double* up)
{
    double x, y, z;
    const double d2r = PI / 180.0;

    geodetic2Ecef(latitude * d2r, longitude * d2r, altitude, &x, &y, &z);

    double aux_north, aux_east, aux_down;

    std::istringstream iss2(FLAGS_static_position);
    std::string str_aux;
    std::getline(iss2, str_aux, ',');
    double ref_long = std::stod(str_aux);
    std::getline(iss2, str_aux, ',');
    double ref_lat = std::stod(str_aux);
    std::getline(iss2, str_aux, '\n');
    double ref_h = std::stod(str_aux);
    double ref_x, ref_y, ref_z;

    geodetic2Ecef(ref_lat * d2r, ref_long * d2r, ref_h, &ref_x, &ref_y, &ref_z);

    double aux_x = x - ref_x;
    double aux_y = y - ref_y;
    double aux_z = z - ref_z;

    // ECEF to NED matrix
    double phiP = atan2(ref_z, sqrt(std::pow(ref_x, 2.0) + std::pow(ref_y, 2.0)));
    const double sLat = sin(phiP);
    const double sLon = sin(ref_long * d2r);
    const double cLat = cos(phiP);
    const double cLon = cos(ref_long * d2r);

    aux_north = -aux_x * sLat * cLon - aux_y * sLon + aux_z * cLat * cLon;
    aux_east = -aux_x * sLat * sLon + aux_y * cLon + aux_z * cLat * sLon;
    aux_down = aux_x * cLat + aux_z * sLat;

    *east = aux_east;
    *north = aux_north;
    *up = -aux_down;
}


double StaticPositionSystemTest::compute_stdev_precision(const std::vector<double>& vec)
{
    double sum__ = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean__ = sum__ / vec.size();
    double accum__ = 0.0;
    std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
        accum__ += (d - mean__) * (d - mean__);
    });
    double stdev__ = std::sqrt(accum__ / (vec.size() - 1));
    return stdev__;
}


double StaticPositionSystemTest::compute_stdev_accuracy(const std::vector<double>& vec, const double ref)
{
    const double mean__ = ref;
    double accum__ = 0.0;
    std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
        accum__ += (d - mean__) * (d - mean__);
    });
    double stdev__ = std::sqrt(accum__ / (vec.size() - 1));
    return stdev__;
}


int StaticPositionSystemTest::configure_generator()
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(std::min(FLAGS_duration * 10, 3000));
            if (FLAGS_duration > 300) std::cout << "WARNING: Duration has been set to its maximum value of 300 s" << std::endl;
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  //Baseband sampling frequency [MSps]
    return 0;
}


int StaticPositionSystemTest::generate_signal()
{
    pid_t wait_result;
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], NULL};

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
    if (wait_result == -1) perror("waitpid error");
    return 0;
}


int StaticPositionSystemTest::configure_receiver()
{
    if (FLAGS_config_file_ptest.empty())
        {
            config = std::make_shared<InMemoryConfiguration>();
            const int sampling_rate_internal = baseband_sampling_freq;

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

            const float zero = 0.0;
            const int number_of_channels = 12;
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
            const float pll_bw_narrow_hz = 20.0;
            const float dll_bw_narrow_hz = 2.0;
            const int extend_correlation_ms = 1;

            const int display_rate_ms = 500;
            const int output_rate_ms = 100;

            config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(sampling_rate_internal));

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
            config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
            config->set_property("Acquisition_1C.item_type", "gr_complex");
            config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
            config->set_property("Acquisition_1C.threshold", std::to_string(threshold));
            config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
            config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));
            config->set_property("Acquisition_1C.bit_transition_flag", "false");
            config->set_property("Acquisition_1C.max_dwells", std::to_string(max_dwells));
            config->set_property("Acquisition_1C.tong_init_val", std::to_string(tong_init_val));
            config->set_property("Acquisition_1C.tong_max_val", std::to_string(tong_max_val));
            config->set_property("Acquisition_1C.tong_max_dwells", std::to_string(tong_max_dwells));
            config->set_property("Acquisition_1C.dump", "false");
            config->set_property("Acquisition_1C.dump_filename", "./acquisition");
            config->set_property("Acquisition_1C.dump_channel", "1");

            // Set Tracking
            config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
            //config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_C_Aid_Tracking");
            config->set_property("Tracking_1C.item_type", "gr_complex");
            config->set_property("Tracking_1C.dump", "false");
            config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
            config->set_property("Tracking_1C.pll_bw_hz", std::to_string(pll_bw_hz));
            config->set_property("Tracking_1C.dll_bw_hz", std::to_string(dll_bw_hz));
            config->set_property("Tracking_1C.early_late_space_chips", std::to_string(early_late_space_chips));

            config->set_property("Tracking_1C.pll_bw_narrow_hz", std::to_string(pll_bw_narrow_hz));
            config->set_property("Tracking_1C.dll_bw_narrow_hz", std::to_string(dll_bw_narrow_hz));
            config->set_property("Tracking_1C.extend_correlation_symbols", std::to_string(extend_correlation_ms));

            // Set Telemetry
            config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
            config->set_property("TelemetryDecoder_1C.dump", "false");

            // Set Observables
            config->set_property("Observables.implementation", "Hybrid_Observables");
            config->set_property("Observables.dump", "false");
            config->set_property("Observables.dump_filename", "./observables.dat");

            // Set PVT
            config->set_property("PVT.implementation", "RTKLIB_PVT");
            config->set_property("PVT.positioning_mode", "Single");
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
            config->set_property("PVT.iono_model", "OFF");
            config->set_property("PVT.trop_model", "OFF");
            config->set_property("PVT.AR_GPS", "PPP-AR");
            //config->set_property("PVT.elevation_mask", std::to_string(25));

            config_f = 0;
        }
    else
        {
            config_f = std::make_shared<FileConfiguration>(FLAGS_config_file_ptest);
            config = 0;
        }
    return 0;
}


int StaticPositionSystemTest::run_receiver()
{
    std::shared_ptr<ControlThread> control_thread;
    if (FLAGS_config_file_ptest.empty())
        {
            control_thread = std::make_shared<ControlThread>(config);
        }
    else
        {
            control_thread = std::make_shared<ControlThread>(config_f);
        }

    // start receiver
    try
        {
            control_thread->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }

    // Get the name of the KML file generated by the receiver
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    FILE* fp;
    std::string argum2 = std::string("/bin/ls *kml | tail -1");
    char buffer[1035];
    fp = popen(&argum2[0], "r");
    if (fp == NULL)
        {
            std::cout << "Failed to run command: " << argum2 << std::endl;
            return -1;
        }
    while (fgets(buffer, sizeof(buffer), fp) != NULL)
        {
            std::string aux = std::string(buffer);
            EXPECT_EQ(aux.empty(), false);
            StaticPositionSystemTest::generated_kml_file = aux.erase(aux.length() - 1, 1);
        }
    pclose(fp);
    EXPECT_EQ(StaticPositionSystemTest::generated_kml_file.empty(), false);
    return 0;
}


void StaticPositionSystemTest::check_results()
{
    std::fstream myfile(StaticPositionSystemTest::generated_kml_file, std::ios_base::in);
    ASSERT_TRUE(myfile.is_open()) << "No valid kml file could be opened";
    std::string line;

    std::vector<double> pos_e;
    std::vector<double> pos_n;
    std::vector<double> pos_u;

    // Skip header
    std::getline(myfile, line);
    bool is_header = true;
    while (is_header)
        {
            std::getline(myfile, line);
            ASSERT_FALSE(myfile.eof()) << "No valid kml file found.";
            std::size_t found = line.find("<coordinates>");
            if (found != std::string::npos) is_header = false;
        }
    bool is_data = true;

    //read data
    while (is_data)
        {
            if (!std::getline(myfile, line))
                {
                    is_data = false;
                    break;
                }
            std::size_t found = line.find("</coordinates>");
            if (found != std::string::npos)
                is_data = false;
            else
                {
                    std::string str2;
                    std::istringstream iss(line);
                    double value;
                    double lat = 0.0;
                    double longitude = 0.0;
                    double h = 0.0;
                    for (int i = 0; i < 3; i++)
                        {
                            std::getline(iss, str2, ',');
                            value = std::stod(str2);
                            if (i == 0) lat = value;
                            if (i == 1) longitude = value;
                            if (i == 2) h = value;
                        }

                    double north, east, up;
                    geodetic2Enu(lat, longitude, h, &east, &north, &up);
                    //std::cout << "E = " << east << ", N = " << north << " U = " << up << std::endl;
                    pos_e.push_back(east);
                    pos_n.push_back(north);
                    pos_u.push_back(up);
                }
        }
    myfile.close();
    ASSERT_FALSE(pos_e.size() == 0) << "KML file is empty";

    double sigma_E_2_precision = std::pow(compute_stdev_precision(pos_e), 2.0);
    double sigma_N_2_precision = std::pow(compute_stdev_precision(pos_n), 2.0);
    double sigma_U_2_precision = std::pow(compute_stdev_precision(pos_u), 2.0);

    double sigma_E_2_accuracy = std::pow(compute_stdev_accuracy(pos_e, 0.0), 2.0);
    double sigma_N_2_accuracy = std::pow(compute_stdev_accuracy(pos_n, 0.0), 2.0);
    double sigma_U_2_accuracy = std::pow(compute_stdev_accuracy(pos_u, 0.0), 2.0);

    double sum__e = std::accumulate(pos_e.begin(), pos_e.end(), 0.0);
    double mean__e = sum__e / pos_e.size();
    double sum__n = std::accumulate(pos_n.begin(), pos_n.end(), 0.0);
    double mean__n = sum__n / pos_n.size();
    double sum__u = std::accumulate(pos_u.begin(), pos_u.end(), 0.0);
    double mean__u = sum__u / pos_u.size();

    std::stringstream stm;
    std::ofstream position_test_file;

    if (FLAGS_config_file_ptest.empty())
        {
            stm << "---- ACCURACY ----" << std::endl;
            stm << "2DRMS = " << 2 * sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy) << " [m]" << std::endl;
            stm << "DRMS = " << sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy) << " [m]" << std::endl;
            stm << "CEP = " << 0.62 * compute_stdev_accuracy(pos_n, 0.0) + 0.56 * compute_stdev_accuracy(pos_e, 0.0) << " [m]" << std::endl;
            stm << "99% SAS = " << 1.122 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
            stm << "90% SAS = " << 0.833 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
            stm << "MRSE = " << sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
            stm << "SEP = " << 0.51 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
            stm << "Bias 2D = " << sqrt(std::pow(mean__e, 2.0) + std::pow(mean__n, 2.0)) << " [m]" << std::endl;
            stm << "Bias 3D = " << sqrt(std::pow(mean__e, 2.0) + std::pow(mean__n, 2.0) + std::pow(mean__u, 2.0)) << " [m]" << std::endl;
            stm << std::endl;
        }

    stm << "---- PRECISION ----" << std::endl;
    stm << "2DRMS = " << 2 * sqrt(sigma_E_2_precision + sigma_N_2_precision) << " [m]" << std::endl;
    stm << "DRMS = " << sqrt(sigma_E_2_precision + sigma_N_2_precision) << " [m]" << std::endl;
    stm << "CEP = " << 0.62 * compute_stdev_precision(pos_n) + 0.56 * compute_stdev_precision(pos_e) << " [m]" << std::endl;
    stm << "99% SAS = " << 1.122 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
    stm << "90% SAS = " << 0.833 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
    stm << "MRSE = " << sqrt(sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
    stm << "SEP = " << 0.51 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;

    std::cout << stm.rdbuf();
    std::string output_filename = "position_test_output_" + StaticPositionSystemTest::generated_kml_file.erase(StaticPositionSystemTest::generated_kml_file.length() - 3, 3) + "txt";
    position_test_file.open(output_filename.c_str());
    if (position_test_file.is_open())
        {
            position_test_file << stm.str();
            position_test_file.close();
        }

    // Sanity Check
    double precision_SEP = 0.51 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision);
    ASSERT_LT(precision_SEP, 20.0);

    if (FLAGS_plot_position_test == true)
        {
            print_results(pos_e, pos_n, pos_u);
        }
}


void StaticPositionSystemTest::print_results(const std::vector<double>& east,
    const std::vector<double>& north,
    const std::vector<double>& up)
{
    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
    if (gnuplot_executable.empty())
        {
            std::cout << "WARNING: Although the flag plot_position_test has been set to TRUE," << std::endl;
            std::cout << "gnuplot has not been found in your system." << std::endl;
            std::cout << "Test results will not be plotted." << std::endl;
        }
    else
        {
            double sigma_E_2_precision = std::pow(compute_stdev_precision(east), 2.0);
            double sigma_N_2_precision = std::pow(compute_stdev_precision(north), 2.0);
            double sigma_U_2_precision = std::pow(compute_stdev_precision(up), 2.0);

            double mean_east = std::accumulate(east.begin(), east.end(), 0.0) / east.size();
            double mean_north = std::accumulate(north.begin(), north.end(), 0.0) / north.size();

            auto it_max_east = std::max_element(std::begin(east), std::end(east));
            auto it_min_east = std::min_element(std::begin(east), std::end(east));
            auto it_max_north = std::max_element(std::begin(north), std::end(north));
            auto it_min_north = std::min_element(std::begin(north), std::end(north));
            auto it_max_up = std::max_element(std::begin(up), std::end(up));
            auto it_min_up = std::min_element(std::begin(up), std::end(up));

            auto east_range = std::max(*it_max_east, std::abs(*it_min_east));
            auto north_range = std::max(*it_max_north, std::abs(*it_min_north));
            auto up_range = std::max(*it_max_up, std::abs(*it_min_up));

            double range = std::max(east_range, north_range) * 1.1;
            double range_3d = std::max(std::max(east_range, north_range), up_range) * 1.1;

            double two_drms = 2 * sqrt(sigma_E_2_precision + sigma_N_2_precision);
            double ninty_sas = 0.833 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision);
            try
                {
                    boost::filesystem::path p(gnuplot_executable);
                    boost::filesystem::path dir = p.parent_path();
                    std::string gnuplot_path = dir.native();
                    Gnuplot::set_GNUPlotPath(gnuplot_path);

                    Gnuplot g1("points");
                    if (FLAGS_show_plots)
                        {
                            g1.showonscreen();  // window output
                        }
                    else
                        {
                            g1.disablescreen();
                        }
                    g1.set_title("2D precision");
                    g1.set_xlabel("East [m]");
                    g1.set_ylabel("North [m]");
                    g1.cmd("set size ratio -1");
                    g1.cmd("set xrange [-" + std::to_string(range) + ":" + std::to_string(range) + "]");
                    g1.cmd("set yrange [-" + std::to_string(range) + ":" + std::to_string(range) + "]");

                    g1.plot_xy(east, north, "2D Position Fixes");
                    g1.set_style("lines").plot_circle(mean_east, mean_north, two_drms, "2DRMS");
                    g1.set_style("lines").plot_circle(mean_east, mean_north, two_drms / 2.0, "DRMS");

                    g1.cmd("set grid front");
                    g1.cmd("replot");

                    g1.savetops("Position_test_2D");
                    g1.savetopdf("Position_test_2D", 18);

                    Gnuplot g2("points");
                    if (FLAGS_show_plots)
                        {
                            g2.showonscreen();  // window output
                        }
                    else
                        {
                            g2.disablescreen();
                        }
                    g2.set_title("3D precision");
                    g2.set_xlabel("East [m]");
                    g2.set_ylabel("North [m]");
                    g2.set_zlabel("Up [m]");
                    g2.cmd("set size ratio -1");
                    g2.cmd("set xrange [-" + std::to_string(range_3d) + ":" + std::to_string(range_3d) + "]");
                    g2.cmd("set yrange [-" + std::to_string(range_3d) + ":" + std::to_string(range_3d) + "]");
                    g2.cmd("set zrange [-" + std::to_string(range_3d) + ":" + std::to_string(range_3d) + "]");
                    g2.cmd("set view equal xyz");
                    g2.cmd("set ticslevel 0");

                    g2.cmd("set style fill transparent solid 0.30 border\n set parametric\n set urange [0:2.0*pi]\n set vrange [-pi/2:pi/2]\n r = " +
                           std::to_string(ninty_sas) +
                           "\n fx(v,u) = r*cos(v)*cos(u)\n fy(v,u) = r*cos(v)*sin(u)\n fz(v) = r*sin(v) \n splot fx(v,u),fy(v,u),fz(v) title \"90\%-SAS\" lt rgb \"gray\"\n");
                    g2.plot_xyz(east, north, up, "3D Position Fixes");

                    g2.savetops("Position_test_3D");
                    g2.savetopdf("Position_test_3D");
                }
            catch (const GnuplotException& ge)
                {
                    std::cout << ge.what() << std::endl;
                }
        }
}

TEST_F(StaticPositionSystemTest, Position_system_test)
{
    if (FLAGS_config_file_ptest.empty())
        {
            // Configure the signal generator
            configure_generator();

            // Generate signal raw signal samples and observations RINEX file
            if (!FLAGS_disable_generator)
                {
                    generate_signal();
                }
        }

    // Configure receiver
    configure_receiver();

    // Run the receiver
    EXPECT_EQ(run_receiver(), 0) << "Problem executing the software-defined signal generator";

    // Check results
    check_results();
}


int main(int argc, char** argv)
{
    std::cout << "Running Position precision test..." << std::endl;
    int res = 0;
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Run the Tests
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
    google::ShutDownCommandLineFlags();
    return res;
}

/*!
 * \file position_test.cc
 * \brief  This class implements a test for the validation of computed position.
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *          <li> Javier Arribas, 2018. jarribas(at)cttc.es
 *          </ul>
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

#include "geofunctions.h"
#include "position_test_flags.h"
#include "rtklib_solver_dump_reader.h"
#include "spirent_motion_csv_dump_reader.h"
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
#include <armadillo>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>
#include <thread>
#include <armadillo>

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

class PositionSystemTest : public ::testing::Test
{
public:
    int configure_generator();
    int generate_signal();
    int configure_receiver();
    int run_receiver();
    void check_results();
    std::string config_filename_no_extension;

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

    void print_results(arma::mat R_eb_enu);
    std::shared_ptr<InMemoryConfiguration> config;
    std::shared_ptr<FileConfiguration> config_f;
    std::string generated_kml_file;
};


int PositionSystemTest::configure_generator()
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


int PositionSystemTest::generate_signal()
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


int PositionSystemTest::configure_receiver()
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
            const int number_of_channels = 11;
            const int in_acquisition = 1;

            const float threshold = 2.5;
            const float doppler_max = 5000.0;
            const float doppler_step = 250.0;
            const int max_dwells = 10;
            const int tong_init_val = 2;
            const int tong_max_val = 10;
            const int tong_max_dwells = 30;
            const int coherent_integration_time_ms = 1;

            const float pll_bw_hz = 35.0;
            const float dll_bw_hz = 1.5;
            const float early_late_space_chips = 0.5;
            const float pll_bw_narrow_hz = 1.0;
            const float dll_bw_narrow_hz = 0.1;
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
            config->set_property("InputFilter.implementation", "Freq_Xlating_Fir_Filter");
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
            config->set_property("InputFilter.filter_type", "lowpass");
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
            config->set_property("Acquisition_1C.blocking", "true");

            // Set Tracking
            config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
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
            config->set_property("PVT.dump", "true");
            config->set_property("PVT.rinex_version", std::to_string(2));
            config->set_property("PVT.iono_model", "OFF");
            config->set_property("PVT.trop_model", "OFF");
            config->set_property("PVT.AR_GPS", "PPP-AR");
            config->set_property("PVT.elevation_mask", std::to_string(15));

            config_f = 0;
        }
    else
        {
            config_f = std::make_shared<FileConfiguration>(FLAGS_config_file_ptest);
            config = 0;
        }
    return 0;
}


int PositionSystemTest::run_receiver()
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
            PositionSystemTest::generated_kml_file = aux.erase(aux.length() - 1, 1);
        }
    pclose(fp);
    EXPECT_EQ(PositionSystemTest::generated_kml_file.empty(), false);
    return 0;
}


void PositionSystemTest::check_results()
{
    arma::mat R_eb_e;    //ECEF position (x,y,z) estimation in the Earth frame (Nx3)
    arma::mat R_eb_enu;  //ENU position (N,E,U) estimation in UTM (Nx3)
    arma::mat V_eb_e;    //ECEF velocity (x,y,z) estimation in the Earth frame (Nx3)
    arma::mat LLH;       //Geodetic coordinates (latitude, longitude, height) estimation in WGS84 datum
    arma::vec receiver_time_s;

    arma::mat ref_R_eb_e;  //ECEF position (x,y,z) reference in the Earth frame (Nx3)
    arma::mat ref_V_eb_e;  //ECEF velocity (x,y,z) reference in the Earth frame (Nx3)
    arma::mat ref_LLH;     //Geodetic coordinates (latitude, longitude, height) reference in WGS84 datum
    arma::vec ref_time_s;

    std::istringstream iss2(FLAGS_static_position);
    std::string str_aux;
    std::getline(iss2, str_aux, ',');
    double ref_lat = std::stod(str_aux);
    std::getline(iss2, str_aux, ',');
    double ref_long = std::stod(str_aux);
    std::getline(iss2, str_aux, '\n');
    double ref_h = std::stod(str_aux);
    int utm_zone = findUtmZone(ref_lat, ref_long);

    arma::vec v_eb_n = {0.0, 0.0, 0.0};
    arma::vec true_r_eb_e = {0.0, 0.0, 0.0};
    arma::vec true_v_eb_e = {0.0, 0.0, 0.0};
    pv_Geo_to_ECEF(degtorad(ref_lat), degtorad(ref_long), ref_h, v_eb_n, true_r_eb_e, true_v_eb_e);
    ref_R_eb_e.insert_cols(0, true_r_eb_e);
    arma::vec ref_r_enu = {0, 0, 0};
    cart2utm(true_r_eb_e, utm_zone, ref_r_enu);
    if (!FLAGS_use_pvt_solver_dump)
        {
            //fall back to read receiver KML output (position only)
            std::fstream myfile(PositionSystemTest::generated_kml_file, std::ios_base::in);
            ASSERT_TRUE(myfile.is_open()) << "No valid kml file could be opened";
            std::string line;
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
            int64_t current_epoch = 0;
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
                                    if (i == 0) longitude = value;
                                    if (i == 1) lat = value;
                                    if (i == 2) h = value;
                                }

                            arma::vec tmp_v_ecef;
                            arma::vec tmp_r_ecef;
                            pv_Geo_to_ECEF(degtorad(lat), degtorad(longitude), h, arma::vec{0, 0, 0}, tmp_r_ecef, tmp_v_ecef);
                            R_eb_e.insert_cols(current_epoch, tmp_r_ecef);
                            arma::vec tmp_r_enu = {0, 0, 0};
                            cart2utm(tmp_r_ecef, utm_zone, tmp_r_enu);
                            R_eb_enu.insert_cols(current_epoch, tmp_r_enu);
                            //                            std::cout << "lat = " << lat << ", longitude = " << longitude << "  h = " << h << std::endl;
                            //                            std::cout << "E = " << east << ", N = " << north << " U = " << up << std::endl;
                            //                            getchar();
                        }
                }
            myfile.close();
            ASSERT_FALSE(R_eb_e.n_cols == 0) << "KML file is empty";
        }
    else
        {
            //use complete binary dump from pvt solver
            rtklib_solver_dump_reader pvt_reader;
            pvt_reader.open_obs_file(FLAGS_pvt_solver_dump_filename);
            int64_t n_epochs = pvt_reader.num_epochs();
            R_eb_e = arma::zeros(n_epochs, 3);
            V_eb_e = arma::zeros(n_epochs, 3);
            LLH = arma::zeros(n_epochs, 3);
            receiver_time_s = arma::zeros(n_epochs, 1);
            int64_t current_epoch = 0;
            while (pvt_reader.read_binary_obs())
                {
                    receiver_time_s(current_epoch) = pvt_reader.RX_time - pvt_reader.clk_offset_s;
                    R_eb_e(current_epoch, 0) = pvt_reader.rr[0];
                    R_eb_e(current_epoch, 1) = pvt_reader.rr[1];
                    R_eb_e(current_epoch, 2) = pvt_reader.rr[2];
                    V_eb_e(current_epoch, 0) = pvt_reader.rr[3];
                    V_eb_e(current_epoch, 1) = pvt_reader.rr[4];
                    V_eb_e(current_epoch, 2) = pvt_reader.rr[5];
                    LLH(current_epoch, 0) = pvt_reader.latitude;
                    LLH(current_epoch, 1) = pvt_reader.longitude;
                    LLH(current_epoch, 2) = pvt_reader.height;
                    arma::vec tmp_r_enu;
                    cart2utm(R_eb_e.col(current_epoch), utm_zone, tmp_r_enu);
                    R_eb_enu.insert_cols(current_epoch, tmp_r_enu);

                    //debug check
                    //                    std::cout << "t1: " << pvt_reader.RX_time << std::endl;
                    //                    std::cout << "t2: " << pvt_reader.TOW_at_current_symbol_ms << std::endl;
                    //                    std::cout << "offset: " << pvt_reader.clk_offset_s << std::endl;
                    //                    getchar();
                    current_epoch++;
                }
            ASSERT_FALSE(current_epoch == 0) << "PVT dump is empty";
        }

    // compute results

    if (FLAGS_static_scenario)
        {
            double sigma_E_2_precision = arma::var(R_eb_enu.row(0));
            double sigma_N_2_precision = arma::var(R_eb_enu.row(1));
            double sigma_U_2_precision = arma::var(R_eb_enu.row(2));

            arma::rowvec tmp_vec;
            tmp_vec = R_eb_enu.row(0) - ref_r_enu(0);
            double sigma_E_2_accuracy = sqrt(arma::sum(arma::square(tmp_vec)) / tmp_vec.n_cols);
            tmp_vec = R_eb_enu.row(1) - ref_r_enu(1);
            double sigma_N_2_accuracy = sqrt(arma::sum(arma::square(tmp_vec)) / tmp_vec.n_cols);
            tmp_vec = R_eb_enu.row(2) - ref_r_enu(2);
            double sigma_U_2_accuracy = sqrt(arma::sum(arma::square(tmp_vec)) / tmp_vec.n_cols);

            double mean__e = arma::mean(R_eb_enu.row(0));
            double mean__n = arma::mean(R_eb_enu.row(1));
            double mean__u = arma::mean(R_eb_enu.row(2));

            std::stringstream stm;
            std::ofstream position_test_file;
            if (!FLAGS_config_file_ptest.empty())
                {
                    stm << "Configuration file: " << FLAGS_config_file_ptest << std::endl;
                }
            if (FLAGS_static_scenario)
                {
                    stm << "---- ACCURACY ----" << std::endl;
                    stm << "2DRMS = " << 2 * sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy) << " [m]" << std::endl;
                    stm << "DRMS = " << sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy) << " [m]" << std::endl;
                    stm << "CEP = " << 0.62 * sqrt(sigma_N_2_accuracy) + 0.56 * sqrt(sigma_E_2_accuracy) << " [m]" << std::endl;
                    stm << "99% SAS = " << 1.122 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
                    stm << "90% SAS = " << 0.833 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
                    stm << "MRSE = " << sqrt(sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
                    stm << "SEP = " << 0.51 * (sigma_E_2_accuracy + sigma_N_2_accuracy + sigma_U_2_accuracy) << " [m]" << std::endl;
                    stm << "Bias 2D = " << sqrt(std::pow(fabs(mean__e - ref_r_enu(0)), 2.0) + std::pow(fabs(mean__n - ref_r_enu(1)), 2.0)) << " [m]" << std::endl;
                    stm << "Bias 3D = " << sqrt(std::pow(fabs(mean__e - ref_r_enu(0)), 2.0) + std::pow(fabs(mean__n - ref_r_enu(1)), 2.0) + std::pow(fabs(mean__u - ref_r_enu(2)), 2.0)) << " [m]" << std::endl;
                    stm << std::endl;
                }

            stm << "---- PRECISION ----" << std::endl;
            stm << "2DRMS = " << 2 * sqrt(sigma_E_2_precision + sigma_N_2_precision) << " [m]" << std::endl;
            stm << "DRMS = " << sqrt(sigma_E_2_precision + sigma_N_2_precision) << " [m]" << std::endl;
            stm << "CEP = " << 0.62 * sqrt(sigma_N_2_precision) + 0.56 * sqrt(sigma_E_2_precision) << " [m]" << std::endl;
            stm << "99% SAS = " << 1.122 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
            stm << "90% SAS = " << 0.833 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
            stm << "MRSE = " << sqrt(sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;
            stm << "SEP = " << 0.51 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision) << " [m]" << std::endl;

            std::cout << stm.rdbuf();
            std::string output_filename = "position_test_output_" + PositionSystemTest::generated_kml_file.erase(PositionSystemTest::generated_kml_file.length() - 3, 3) + "txt";
            position_test_file.open(output_filename.c_str());
            if (position_test_file.is_open())
                {
                    position_test_file << stm.str();
                    position_test_file.close();
                }

            // Sanity Check
            double precision_SEP = 0.51 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision);
            ASSERT_LT(precision_SEP, 1.0);

            if (FLAGS_plot_position_test == true)
                {
                    print_results(R_eb_enu);
                }
        }
    else
        {
            //dynamic position
            spirent_motion_csv_dump_reader ref_reader;
            ref_reader.open_obs_file(FLAGS_ref_motion_filename);
            int64_t n_epochs = ref_reader.num_epochs();
            ref_R_eb_e = arma::zeros(n_epochs, 3);
            ref_V_eb_e = arma::zeros(n_epochs, 3);
            ref_LLH = arma::zeros(n_epochs, 3);
            ref_time_s = arma::zeros(n_epochs, 1);
            int64_t current_epoch = 0;
            while (ref_reader.read_csv_obs())
                {
                    ref_time_s(current_epoch) = ref_reader.TOW_ms / 1000.0;
                    ref_R_eb_e(current_epoch, 0) = ref_reader.Pos_X;
                    ref_R_eb_e(current_epoch, 1) = ref_reader.Pos_Y;
                    ref_R_eb_e(current_epoch, 2) = ref_reader.Pos_Z;
                    ref_V_eb_e(current_epoch, 0) = ref_reader.Vel_X;
                    ref_V_eb_e(current_epoch, 1) = ref_reader.Vel_Y;
                    ref_V_eb_e(current_epoch, 2) = ref_reader.Vel_Z;
                    ref_LLH(current_epoch, 0) = ref_reader.Lat;
                    ref_LLH(current_epoch, 1) = ref_reader.Long;
                    ref_LLH(current_epoch, 2) = ref_reader.Height;
                    current_epoch++;
                }

            //interpolation of reference data to receiver epochs timestamps
            arma::mat ref_interp_R_eb_e = arma::zeros(R_eb_e.n_rows, 3);
            arma::mat ref_interp_V_eb_e = arma::zeros(V_eb_e.n_rows, 3);
            arma::mat ref_interp_LLH = arma::zeros(LLH.n_rows, 3);
            arma::vec tmp_vector;
            for (int n = 0; n < 3; n++)
                {
                    arma::interp1(ref_time_s, ref_R_eb_e.col(n), receiver_time_s, tmp_vector);
                    ref_interp_R_eb_e.col(n) = tmp_vector;
                    arma::interp1(ref_time_s, ref_V_eb_e.col(n), receiver_time_s, tmp_vector);
                    ref_interp_V_eb_e.col(n) = tmp_vector;
                    arma::interp1(ref_time_s, ref_LLH.col(n), receiver_time_s, tmp_vector);
                    ref_interp_LLH.col(n) = tmp_vector;
                }

            //compute error vectors

            arma::mat error_R_eb_e = arma::zeros(R_eb_e.n_rows, 3);
            arma::mat error_V_eb_e = arma::zeros(V_eb_e.n_rows, 3);
            arma::mat error_LLH = arma::zeros(LLH.n_rows, 3);
            error_R_eb_e = R_eb_e - ref_interp_R_eb_e;
            error_V_eb_e = V_eb_e - ref_interp_V_eb_e;
            error_LLH = LLH - ref_interp_LLH;
            arma::vec error_module_R_eb_e = arma::zeros(R_eb_e.n_rows, 1);
            arma::vec error_module_V_eb_e = arma::zeros(V_eb_e.n_rows, 1);
            for (uint64_t n = 0; n < R_eb_e.n_rows; n++)
                {
                    error_module_R_eb_e(n) = arma::norm(error_R_eb_e.row(n));
                    error_module_V_eb_e(n) = arma::norm(error_V_eb_e.row(n));
                }
            //Error statistics
            arma::vec tmp_vec;
            //RMSE, Mean, Variance and peaks
            tmp_vec = arma::square(error_module_R_eb_e);
            double rmse_R_eb_e = sqrt(arma::mean(tmp_vec));
            double error_mean_R_eb_e = arma::mean(error_module_R_eb_e);
            double error_var_R_eb_e = arma::var(error_module_R_eb_e);
            double max_error_R_eb_e = arma::max(error_module_R_eb_e);
            double min_error_R_eb_e = arma::min(error_module_R_eb_e);

            tmp_vec = arma::square(error_module_V_eb_e);
            double rmse_V_eb_e = sqrt(arma::mean(tmp_vec));
            double error_mean_V_eb_e = arma::mean(error_module_V_eb_e);
            double error_var_V_eb_e = arma::var(error_module_V_eb_e);
            double max_error_V_eb_e = arma::max(error_module_V_eb_e);
            double min_error_V_eb_e = arma::min(error_module_V_eb_e);

            //report
            std::cout << "----- Position and Velocity 3D ECEF error statistics -----" << std::endl;
            if (!FLAGS_config_file_ptest.empty())
                {
                    std::cout << "---- Configuration file: " << FLAGS_config_file_ptest << std::endl;
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << "---- 3D ECEF Position RMSE = "
                      << rmse_R_eb_e << ", mean = " << error_mean_R_eb_e
                      << ", stdev = " << sqrt(error_var_R_eb_e)
                      << " (max,min) = " << max_error_R_eb_e
                      << "," << min_error_R_eb_e
                      << " [m]" << std::endl;
            std::cout << "---- 3D ECEF Velocity RMSE = "
                      << rmse_V_eb_e << ", mean = " << error_mean_V_eb_e
                      << ", stdev = " << sqrt(error_var_V_eb_e)
                      << " (max,min) = " << max_error_V_eb_e
                      << "," << min_error_V_eb_e
                      << " [m/s]" << std::endl;
            std::cout.precision(ss);

            //plots
            Gnuplot g1("points");
            if (FLAGS_show_plots)
                {
                    g1.showonscreen();  // window output
                }
            else
                {
                    g1.disablescreen();
                }
            g1.set_title("3D ECEF error coordinates");
            g1.set_grid();
            //conversion between arma::vec and std:vector
            std::vector<double> X(error_R_eb_e.colptr(0), error_R_eb_e.colptr(0) + error_R_eb_e.n_rows);
            std::vector<double> Y(error_R_eb_e.colptr(1), error_R_eb_e.colptr(1) + error_R_eb_e.n_rows);
            std::vector<double> Z(error_R_eb_e.colptr(2), error_R_eb_e.colptr(2) + error_R_eb_e.n_rows);

            g1.cmd("set key box opaque");
            g1.plot_xyz(X, Y, Z, "ECEF 3D error");
            g1.set_legend();
            if (FLAGS_config_file_ptest.empty())
                {
                    g1.savetops("ECEF_3d_error");
                }
            else
                {
                    g1.savetops("ECEF_3d_error_" + config_filename_no_extension);
                }
            arma::vec time_vector_from_start_s = receiver_time_s - receiver_time_s(0);
            Gnuplot g3("linespoints");
            if (FLAGS_show_plots)
                {
                    g3.showonscreen();  // window output
                }
            else
                {
                    g3.disablescreen();
                }
            g3.set_title("3D Position estimation error module [m]");
            g3.set_grid();
            g3.set_xlabel("Receiver epoch time from first valid PVT [s]");
            g3.set_ylabel("3D Position error [m]");
            //conversion between arma::vec and std:vector
            std::vector<double> error_vec(error_module_R_eb_e.colptr(0), error_module_R_eb_e.colptr(0) + error_module_R_eb_e.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector_from_start_s, error_vec, "Position 3D error");
            double mean3d = std::accumulate(error_vec.begin(), error_vec.end(), 0.0) / error_vec.size();
            std::vector<double> error_mean(error_module_R_eb_e.n_rows, mean3d);
            g3.set_style("lines");
            g3.plot_xy(time_vector_from_start_s, error_mean, "Mean");
            g3.set_legend();
            if (FLAGS_config_file_ptest.empty())
                {
                    g3.savetops("Position_3d_error");
                }
            else
                {
                    g3.savetops("Position_3d_error_" + config_filename_no_extension);
                }

            Gnuplot g4("linespoints");
            if (FLAGS_show_plots)
                {
                    g4.showonscreen();  // window output
                }
            else
                {
                    g4.disablescreen();
                }
            g4.set_title("3D Velocity estimation error module [m/s]");
            g4.set_grid();
            g4.set_xlabel("Receiver epoch time from first valid PVT [s]");
            g4.set_ylabel("3D Velocity error [m/s]");
            //conversion between arma::vec and std:vector
            std::vector<double> error_vec2(error_module_V_eb_e.colptr(0), error_module_V_eb_e.colptr(0) + error_module_V_eb_e.n_rows);
            g4.cmd("set key box opaque");
            g4.plot_xy(time_vector_from_start_s, error_vec2, "Velocity 3D error");
            double mean3dv = std::accumulate(error_vec2.begin(), error_vec2.end(), 0.0) / error_vec2.size();
            std::vector<double> error_mean_v(error_module_V_eb_e.n_rows, mean3dv);
            g4.set_style("lines");
            g4.plot_xy(time_vector_from_start_s, error_mean_v, "Mean");
            g4.set_legend();
            if (FLAGS_config_file_ptest.empty())
                {
                    g4.savetops("Velocity_3d_error");
                }
            else
                {
                    g4.savetops("Velocity_3d_error_" + config_filename_no_extension);
                }
        }
}


void PositionSystemTest::print_results(arma::mat R_eb_enu)
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
            double sigma_E_2_precision = arma::var(R_eb_enu.row(0));
            double sigma_N_2_precision = arma::var(R_eb_enu.row(1));
            double sigma_U_2_precision = arma::var(R_eb_enu.row(2));

            double mean_east = arma::mean(R_eb_enu.row(0));
            double mean_north = arma::mean(R_eb_enu.row(1));
            double mean_up = arma::mean(R_eb_enu.row(2));

            double it_max_east = arma::max(R_eb_enu.row(0) - mean_east);
            double it_min_east = arma::min(R_eb_enu.row(0) - mean_east);

            double it_max_north = arma::max(R_eb_enu.row(1) - mean_north);
            double it_min_north = arma::min(R_eb_enu.row(1) - mean_north);

            double it_max_up = arma::max(R_eb_enu.row(2) - mean_up);
            double it_min_up = arma::min(R_eb_enu.row(2) - mean_up);

            double east_range = std::max(it_max_east, std::abs(it_min_east));
            double north_range = std::max(it_max_north, std::abs(it_min_north));
            double up_range = std::max(it_max_up, std::abs(it_min_up));

            double range = std::max(east_range, north_range) * 1.1;
            double range_3d = std::max(std::max(east_range, north_range), up_range) * 1.1;

            double two_drms = 2 * sqrt(sigma_E_2_precision + sigma_N_2_precision);
            double ninty_sas = 0.833 * (sigma_E_2_precision + sigma_N_2_precision + sigma_U_2_precision);
            arma::rowvec arma_east = R_eb_enu.row(0) - mean_east;
            arma::rowvec arma_north = R_eb_enu.row(1) - mean_north;
            arma::rowvec arma_up = R_eb_enu.row(2) - mean_up;

            std::vector<double> east(arma_east.colptr(0), arma_east.row(0).colptr(0) + arma_east.row(0).n_cols);
            std::vector<double> north(arma_north.colptr(0), arma_north.colptr(0) + arma_north.n_cols);
            std::vector<double> up(arma_up.colptr(0), arma_up.colptr(0) + arma_up.n_cols);

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
                    if (FLAGS_config_file_ptest.empty())
                        {
                            g1.savetops("Position_test_2D");
                            g1.savetopdf("Position_test_2D", 18);
                        }
                    else
                        {
                            g1.savetops("Position_test_2D_" + config_filename_no_extension);
                            g1.savetopdf("Position_test_2D_" + config_filename_no_extension, 18);
                        }

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
                    if (FLAGS_config_file_ptest.empty())
                        {
                            g2.savetops("Position_test_3D");
                            g2.savetopdf("Position_test_3D");
                        }
                    else
                        {
                            g2.savetops("Position_test_3D_" + config_filename_no_extension);
                            g2.savetopdf("Position_test_3D_" + config_filename_no_extension);
                        }
                }
            catch (const GnuplotException& ge)
                {
                    std::cout << ge.what() << std::endl;
                }
        }
}

TEST_F(PositionSystemTest, Position_system_test)
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
    else
        {
            config_filename_no_extension = FLAGS_config_file_ptest.substr(FLAGS_config_file_ptest.find_last_of("/\\") + 1);
            config_filename_no_extension = config_filename_no_extension.erase(config_filename_no_extension.length() - 5);
        }

    // Configure receiver
    configure_receiver();

    // Run the receiver
    EXPECT_EQ(run_receiver(), 0) << "Problem executing GNSS-SDR";

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

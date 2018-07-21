/*!
 * \file gps_l1_ca_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for GPS_L1_CA_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include <chrono>
#include <unistd.h>
#include <vector>
#include <armadillo>
#include <matio.h>
#include <boost/filesystem.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "GPS_L1_CA.h"
#include "gnss_block_factory.h"
#include "tracking_interface.h"
#include "in_memory_configuration.h"
#include "tracking_true_obs_reader.h"
#include "tracking_dump_reader.h"
#include "signal_generator_flags.h"
#include "gnuplot_i.h"
#include "test_flags.h"
#include "tracking_tests_flags.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CADllPllTrackingTest_msg_rx;

typedef boost::shared_ptr<GpsL1CADllPllTrackingTest_msg_rx> GpsL1CADllPllTrackingTest_msg_rx_sptr;

GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make();

class GpsL1CADllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTrackingTest_msg_rx();  //!< Default destructor
};


GpsL1CADllPllTrackingTest_msg_rx_sptr GpsL1CADllPllTrackingTest_msg_rx_make()
{
    return GpsL1CADllPllTrackingTest_msg_rx_sptr(new GpsL1CADllPllTrackingTest_msg_rx());
}


void GpsL1CADllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;  //3 -> loss of lock
            //std::cout << "Received trk message: " << rx_message << std::endl;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


GpsL1CADllPllTrackingTest_msg_rx::GpsL1CADllPllTrackingTest_msg_rx() : gr::block("GpsL1CADllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CADllPllTrackingTest_msg_rx::~GpsL1CADllPllTrackingTest_msg_rx()
{
}


// ###########################################################

class GpsL1CADllPllTrackingTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;
    std::string implementation = "GPS_L1_CA_DLL_PLL_Tracking";  //"GPS_L1_CA_DLL_PLL_C_Aid_Tracking";

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    int configure_generator(double CN0_dBHz, int file_idx);
    int generate_signal();
    std::vector<double> check_results_doppler(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error,
        double& rmse);
    std::vector<double> check_results_acc_carrier_phase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error,
        double& rmse);
    std::vector<double> check_results_codephase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error,
        double& rmse);

    bool save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename);
    GpsL1CADllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CADllPllTrackingTest()
    {
    }

    void configure_receiver(double PLL_wide_bw_hz,
        double DLL_wide_bw_hz,
        double PLL_narrow_bw_hz,
        double DLL_narrow_bw_hz,
        int extend_correlation_symbols);

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


int GpsL1CADllPllTrackingTest::configure_generator(double CN0_dBHz, int file_idx)
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(FLAGS_duration * 10);
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;                          // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data + std::to_string(file_idx);  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);             //Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(CN0_dBHz);                                // Signal generator CN0
    return 0;
}


int GpsL1CADllPllTrackingTest::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], NULL};

    int pid;
    if ((pid = fork()) == -1)
        perror("fork err");
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv err." << std::endl;
            std::terminate();
        }

    waitpid(pid, &child_status, 0);

    std::cout << "Signal and Observables RINEX and RAW files created." << std::endl;
    return 0;
}


void GpsL1CADllPllTrackingTest::configure_receiver(
    double PLL_wide_bw_hz,
    double DLL_wide_bw_hz,
    double PLL_narrow_bw_hz,
    double DLL_narrow_bw_hz,
    int extend_correlation_symbols)
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;

    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));
    // Set Tracking
    config->set_property("Tracking_1C.implementation", implementation);
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking_1C.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
    config->set_property("Tracking_1C.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking_1C.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking_1C.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    config->set_property("Tracking_1C.early_late_space_narrow_chips", "0.5");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");

    std::cout << "*****************************************\n";
    std::cout << "*** Tracking configuration parameters ***\n";
    std::cout << "*****************************************\n";
    std::cout << "pll_bw_hz: " << config->property("Tracking_1C.pll_bw_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_hz: " << config->property("Tracking_1C.dll_bw_hz", 0.0) << " Hz\n";
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking_1C.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking_1C.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking_1C.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}


std::vector<double> GpsL1CADllPllTrackingTest::check_results_doppler(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value,
    double& mean_error,
    double& std_dev_error,
    double& rmse)
{
    // 1. True value interpolation to match the measurement times
    arma::vec true_value_interp;
    arma::uvec true_time_s_valid = find(true_time_s > 0);
    true_time_s = true_time_s(true_time_s_valid);
    true_value = true_value(true_time_s_valid);
    arma::uvec meas_time_s_valid = find(meas_time_s > 0);
    meas_time_s = meas_time_s(meas_time_s_valid);
    meas_value = meas_value(meas_time_s_valid);

    arma::interp1(true_time_s, true_value, meas_time_s, true_value_interp);

    // 2. RMSE
    arma::vec err;

    err = meas_value - true_value_interp;

    //conversion between arma::vec and std:vector
    std::vector<double> err_std_vector(err.colptr(0), err.colptr(0) + err.n_rows);

    arma::vec err2 = arma::square(err);
    rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    mean_error = error_mean;
    std_dev_error = sqrt(error_var);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK Doppler RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Hz]" << std::endl;
    std::cout.precision(ss);
    return err_std_vector;
}


std::vector<double> GpsL1CADllPllTrackingTest::check_results_acc_carrier_phase(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value,
    double& mean_error,
    double& std_dev_error,
    double& rmse)
{
    // 1. True value interpolation to match the measurement times
    arma::vec true_value_interp;
    arma::uvec true_time_s_valid = find(true_time_s > 0);
    true_time_s = true_time_s(true_time_s_valid);
    true_value = true_value(true_time_s_valid);
    arma::uvec meas_time_s_valid = find(meas_time_s > 0);
    meas_time_s = meas_time_s(meas_time_s_valid);
    meas_value = meas_value(meas_time_s_valid);

    arma::interp1(true_time_s, true_value, meas_time_s, true_value_interp);

    // 2. RMSE
    arma::vec err;
    //it is required to remove the initial offset in the accumulated carrier phase error
    err = (meas_value - meas_value(0)) - (true_value_interp - true_value_interp(0));
    arma::vec err2 = arma::square(err);
    //conversion between arma::vec and std:vector
    std::vector<double> err_std_vector(err.colptr(0), err.colptr(0) + err.n_rows);

    rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    mean_error = error_mean;
    std_dev_error = sqrt(error_var);
    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK acc carrier phase RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Hz]" << std::endl;
    std::cout.precision(ss);
    return err_std_vector;
}


std::vector<double> GpsL1CADllPllTrackingTest::check_results_codephase(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value,
    double& mean_error,
    double& std_dev_error,
    double& rmse)
{
    // 1. True value interpolation to match the measurement times
    arma::vec true_value_interp;
    arma::uvec true_time_s_valid = find(true_time_s > 0);
    true_time_s = true_time_s(true_time_s_valid);
    true_value = true_value(true_time_s_valid);
    arma::uvec meas_time_s_valid = find(meas_time_s > 0);
    meas_time_s = meas_time_s(meas_time_s_valid);
    meas_value = meas_value(meas_time_s_valid);

    arma::interp1(true_time_s, true_value, meas_time_s, true_value_interp);

    // 2. RMSE
    arma::vec err;

    err = meas_value - true_value_interp;
    //conversion between arma::vec and std:vector
    std::vector<double> err_std_vector(err.colptr(0), err.colptr(0) + err.n_rows);

    arma::vec err2 = arma::square(err);
    rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    mean_error = error_mean;
    std_dev_error = sqrt(error_var);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK code phase RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Chips]" << std::endl;
    std::cout.precision(ss);
    return err_std_vector;
}


TEST_F(GpsL1CADllPllTrackingTest, ValidationOfResults)
{
    //*************************************************
    //***** STEP 2: Prepare the parameters sweep ******
    //*************************************************

    std::vector<double> generator_CN0_values;


    //data containers for config param sweep
    std::vector<std::vector<double>> mean_doppler_error_sweep;     //swep config param and cn0 sweep
    std::vector<std::vector<double>> std_dev_doppler_error_sweep;  //swep config param and cn0 sweep
    std::vector<std::vector<double>> rmse_doppler_sweep;           //swep config param and cn0 sweep

    std::vector<std::vector<double>> mean_code_phase_error_sweep;     //swep config param and cn0 sweep
    std::vector<std::vector<double>> std_dev_code_phase_error_sweep;  //swep config param and cn0 sweep
    std::vector<std::vector<double>> rmse_code_phase_sweep;           //swep config param and cn0 sweep

    std::vector<std::vector<double>> mean_carrier_phase_error_sweep;     //swep config param and cn0 sweep
    std::vector<std::vector<double>> std_dev_carrier_phase_error_sweep;  //swep config param and cn0 sweep
    std::vector<std::vector<double>> rmse_carrier_phase_sweep;           //swep config param and cn0 sweep

    std::vector<std::vector<double>> trk_valid_timestamp_s_sweep;
    std::vector<std::vector<double>> generator_CN0_values_sweep_copy;

    int test_satellite_PRN = 0;
    double acq_delay_samples = 0.0;
    double acq_doppler_hz = 0.0;
    tracking_true_obs_reader true_obs_data;


    // CONFIG PARAM SWEEP LOOP
    std::vector<double> PLL_wide_bw_values;
    std::vector<double> DLL_wide_bw_values;


    //***********************************************************
    //***** STEP 2: Tracking configuration parameters sweep *****
    //***********************************************************

    if (FLAGS_PLL_bw_hz_start == FLAGS_PLL_bw_hz_stop)
        {
            if (FLAGS_DLL_bw_hz_start == FLAGS_DLL_bw_hz_stop)
                {
                    //NO PLL/DLL BW sweep
                    PLL_wide_bw_values.push_back(FLAGS_PLL_bw_hz_start);
                    DLL_wide_bw_values.push_back(FLAGS_DLL_bw_hz_start);
                }
            else
                {
                    //DLL BW Sweep
                    for (double dll_bw = FLAGS_DLL_bw_hz_start; dll_bw >= FLAGS_DLL_bw_hz_stop; dll_bw = dll_bw - FLAGS_DLL_bw_hz_step)
                        {
                            PLL_wide_bw_values.push_back(FLAGS_PLL_bw_hz_start);
                            DLL_wide_bw_values.push_back(dll_bw);
                        }
                }
        }
    else
        {
            //PLL BW Sweep
            for (double pll_bw = FLAGS_PLL_bw_hz_start; pll_bw >= FLAGS_PLL_bw_hz_stop; pll_bw = pll_bw - FLAGS_PLL_bw_hz_step)
                {
                    PLL_wide_bw_values.push_back(pll_bw);
                    DLL_wide_bw_values.push_back(FLAGS_DLL_bw_hz_start);
                }
        }

    //*********************************************
    //***** STEP 3: Generate the input signal *****
    //*********************************************


    std::vector<double> cno_vector;
    if (FLAGS_CN0_dBHz_start == FLAGS_CN0_dBHz_stop)
        {
            generator_CN0_values.push_back(FLAGS_CN0_dBHz_start);
        }
    else
        {
            for (double cn0 = FLAGS_CN0_dBHz_start; cn0 > FLAGS_CN0_dBHz_stop; cn0 = cn0 - FLAGS_CN0_dB_step)
                {
                    generator_CN0_values.push_back(cn0);
                }
        }

    // use generator or use an external capture file
    if (FLAGS_enable_external_signal_file)
        {
            //todo: create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
        }
    else
        {
            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                {
                    // Configure the signal generator
                    configure_generator(generator_CN0_values.at(current_cn0_idx), current_cn0_idx);
                    // Generate signal raw signal samples and observations RINEX file
                    if (FLAGS_disable_generator == false)
                        {
                            generate_signal();
                        }
                    // open true observables log file written by the simulator
                }
        }

    //************************************************************
    //***** STEP 4: Configure the signal tracking parameters *****
    //************************************************************
    for (unsigned int config_idx = 0; config_idx < PLL_wide_bw_values.size(); config_idx++)
        {
            //CN0 LOOP
            // data containers for CN0 sweep
            std::vector<std::vector<double>> prompt_sweep;
            std::vector<std::vector<double>> early_sweep;
            std::vector<std::vector<double>> late_sweep;
            std::vector<std::vector<double>> promptI_sweep;
            std::vector<std::vector<double>> promptQ_sweep;
            std::vector<std::vector<double>> CN0_dBHz_sweep;
            std::vector<std::vector<double>> trk_timestamp_s_sweep;

            std::vector<std::vector<double>> doppler_error_sweep;
            std::vector<std::vector<double>> code_phase_error_sweep;
            std::vector<std::vector<double>> code_phase_error_meters_sweep;
            std::vector<std::vector<double>> acc_carrier_phase_error_sweep;

            std::vector<double> mean_doppler_error;
            std::vector<double> std_dev_doppler_error;
            std::vector<double> rmse_doppler;
            std::vector<double> mean_code_phase_error;
            std::vector<double> std_dev_code_phase_error;
            std::vector<double> rmse_code_phase;
            std::vector<double> mean_carrier_phase_error;
            std::vector<double> std_dev_carrier_phase_error;
            std::vector<double> rmse_carrier_phase;
            std::vector<double> valid_CN0_values;

            configure_receiver(PLL_wide_bw_values.at(config_idx),
                DLL_wide_bw_values.at(config_idx),
                FLAGS_PLL_narrow_bw_hz,
                FLAGS_DLL_narrow_bw_hz,
                FLAGS_extend_correlation_symbols);
            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                {
                    //******************************************************************************************
                    //***** Obtain the initial signal sinchronization parameters (emulating an acquisition) ****
                    //******************************************************************************************
                    if (!FLAGS_enable_external_signal_file)
                        {
                            test_satellite_PRN = FLAGS_test_satellite_PRN;
                            std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
                            true_obs_file.append(std::to_string(test_satellite_PRN));
                            true_obs_file.append(".dat");
                            true_obs_data.close_obs_file();
                            ASSERT_EQ(true_obs_data.open_obs_file(true_obs_file), true) << "Failure opening true observables file";
                            // load acquisition data based on the first epoch of the true observations
                            ASSERT_EQ(true_obs_data.read_binary_obs(), true)
                                << "Failure reading true tracking dump file." << std::endl
                                << "Maybe sat PRN #" + std::to_string(FLAGS_test_satellite_PRN) +
                                       " is not available?";
                            std::cout << "Testing satellite PRN=" << test_satellite_PRN << std::endl;
                            std::cout << "Initial Doppler [Hz]=" << true_obs_data.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << std::endl;
                            acq_doppler_hz = true_obs_data.doppler_l1_hz;
                            acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * static_cast<double>(baseband_sampling_freq) * GPS_L1_CA_CODE_PERIOD;
                            // restart the epoch counter
                            true_obs_data.restart();
                        }


                    std::chrono::time_point<std::chrono::system_clock> start, end;

                    top_block = gr::make_top_block("Tracking test");

                    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking_1C", implementation, 1, 1);
                    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);

                    boost::shared_ptr<GpsL1CADllPllTrackingTest_msg_rx> msg_rx = GpsL1CADllPllTrackingTest_msg_rx_make();

                    gnss_synchro.Acq_delay_samples = acq_delay_samples;
                    gnss_synchro.Acq_doppler_hz = acq_doppler_hz;
                    gnss_synchro.Acq_samplestamp_samples = 0;

                    ASSERT_NO_THROW({
                        tracking->set_channel(gnss_synchro.Channel_ID);
                    }) << "Failure setting channel.";

                    ASSERT_NO_THROW({
                        tracking->set_gnss_synchro(&gnss_synchro);
                    }) << "Failure setting gnss_synchro.";

                    ASSERT_NO_THROW({
                        tracking->connect(top_block);
                    }) << "Failure connecting tracking to the top_block.";

                    ASSERT_NO_THROW({
                        std::string file = "./" + filename_raw_data + std::to_string(current_cn0_idx);
                        const char* file_name = file.c_str();
                        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
                        gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
                        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
                        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
                        top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
                        top_block->connect(tracking->get_right_block(), 0, sink, 0);
                        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
                    }) << "Failure connecting the blocks of tracking test.";


                    //********************************************************************
                    //***** STEP 5: Perform the signal tracking and read the results *****
                    //********************************************************************
                    std::cout << "------------ START TRACKING -------------" << std::endl;
                    tracking->start_tracking();

                    EXPECT_NO_THROW({
                        start = std::chrono::system_clock::now();
                        top_block->run();  // Start threads and wait
                        end = std::chrono::system_clock::now();
                    }) << "Failure running the top_block.";

                    std::chrono::duration<double> elapsed_seconds = end - start;
                    std::cout << "Signal tracking completed in " << elapsed_seconds.count() << " seconds" << std::endl;

                    int tracking_last_msg = msg_rx->rx_message;  //save last aasynchronous tracking message in order to detect a loss of lock

                    //check results
                    //load the measured values
                    tracking_dump_reader trk_dump;
                    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
                        << "Failure opening tracking dump file";

                    long int n_measured_epochs = trk_dump.num_epochs();
                    //std::cout << "Measured observation epochs=" << n_measured_epochs << std::endl;

                    arma::vec trk_timestamp_s = arma::zeros(n_measured_epochs, 1);
                    arma::vec trk_acc_carrier_phase_cycles = arma::zeros(n_measured_epochs, 1);
                    arma::vec trk_Doppler_Hz = arma::zeros(n_measured_epochs, 1);
                    arma::vec trk_prn_delay_chips = arma::zeros(n_measured_epochs, 1);

                    long int epoch_counter = 0;

                    std::vector<double> timestamp_s;
                    std::vector<double> prompt;
                    std::vector<double> early;
                    std::vector<double> late;
                    std::vector<double> promptI;
                    std::vector<double> promptQ;
                    std::vector<double> CN0_dBHz;
                    while (trk_dump.read_binary_obs())
                        {
                            trk_timestamp_s(epoch_counter) = static_cast<double>(trk_dump.PRN_start_sample_count) / static_cast<double>(baseband_sampling_freq);
                            trk_acc_carrier_phase_cycles(epoch_counter) = trk_dump.acc_carrier_phase_rad / GPS_TWO_PI;
                            trk_Doppler_Hz(epoch_counter) = trk_dump.carrier_doppler_hz;
                            double delay_chips = GPS_L1_CA_CODE_LENGTH_CHIPS - GPS_L1_CA_CODE_LENGTH_CHIPS * (fmod((static_cast<double>(trk_dump.PRN_start_sample_count) + trk_dump.aux1) / static_cast<double>(baseband_sampling_freq), 1.0e-3) / 1.0e-3);

                            trk_prn_delay_chips(epoch_counter) = delay_chips;

                            timestamp_s.push_back(trk_timestamp_s(epoch_counter));
                            prompt.push_back(trk_dump.abs_P);
                            early.push_back(trk_dump.abs_E);
                            late.push_back(trk_dump.abs_L);
                            promptI.push_back(trk_dump.prompt_I);
                            promptQ.push_back(trk_dump.prompt_Q);
                            CN0_dBHz.push_back(trk_dump.CN0_SNV_dB_Hz);

                            epoch_counter++;
                        }
                    trk_timestamp_s_sweep.push_back(timestamp_s);
                    prompt_sweep.push_back(prompt);
                    early_sweep.push_back(early);
                    late_sweep.push_back(late);
                    promptI_sweep.push_back(promptI);
                    promptQ_sweep.push_back(promptQ);
                    CN0_dBHz_sweep.push_back(CN0_dBHz);

                    //***********************************************************
                    //***** STEP 6: Compare with true values (if available) *****
                    //***********************************************************
                    if (!FLAGS_enable_external_signal_file)
                        {
                            std::vector<double> doppler_error_hz;
                            std::vector<double> code_phase_error_chips;
                            std::vector<double> code_phase_error_meters;
                            std::vector<double> acc_carrier_phase_hz;

                            try
                                {
                                    // load the true values
                                    long int n_true_epochs = true_obs_data.num_epochs();
                                    //std::cout << "True observation epochs=" << n_true_epochs << std::endl;

                                    arma::vec true_timestamp_s = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_acc_carrier_phase_cycles = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_Doppler_Hz = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_prn_delay_chips = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_tow_s = arma::zeros(n_true_epochs, 1);

                                    long int epoch_counter = 0;
                                    while (true_obs_data.read_binary_obs())
                                        {
                                            true_timestamp_s(epoch_counter) = true_obs_data.signal_timestamp_s;
                                            true_acc_carrier_phase_cycles(epoch_counter) = true_obs_data.acc_carrier_phase_cycles;
                                            true_Doppler_Hz(epoch_counter) = true_obs_data.doppler_l1_hz;
                                            true_prn_delay_chips(epoch_counter) = true_obs_data.prn_delay_chips;
                                            true_tow_s(epoch_counter) = true_obs_data.tow;
                                            epoch_counter++;
                                        }
                                    // Align initial measurements and cut the tracking pull-in transitory
                                    double pull_in_offset_s = FLAGS_skip_trk_transitory_s;

                                    arma::uvec initial_meas_point = arma::find(trk_timestamp_s >= (true_timestamp_s(0) + pull_in_offset_s), 1, "first");
                                    if (initial_meas_point.size() > 0 and tracking_last_msg != 3)
                                        {
                                            trk_timestamp_s = trk_timestamp_s.subvec(initial_meas_point(0), trk_timestamp_s.size() - 1);
                                            trk_acc_carrier_phase_cycles = trk_acc_carrier_phase_cycles.subvec(initial_meas_point(0), trk_acc_carrier_phase_cycles.size() - 1);
                                            trk_Doppler_Hz = trk_Doppler_Hz.subvec(initial_meas_point(0), trk_Doppler_Hz.size() - 1);
                                            trk_prn_delay_chips = trk_prn_delay_chips.subvec(initial_meas_point(0), trk_prn_delay_chips.size() - 1);


                                            double mean_error;
                                            double std_dev_error;
                                            double rmse;

                                            valid_CN0_values.push_back(generator_CN0_values.at(current_cn0_idx));  //save the current cn0 value (valid tracking)

                                            doppler_error_hz = check_results_doppler(true_timestamp_s, true_Doppler_Hz, trk_timestamp_s, trk_Doppler_Hz, mean_error, std_dev_error, rmse);
                                            mean_doppler_error.push_back(mean_error);
                                            std_dev_doppler_error.push_back(std_dev_error);
                                            rmse_doppler.push_back(rmse);

                                            code_phase_error_chips = check_results_codephase(true_timestamp_s, true_prn_delay_chips, trk_timestamp_s, trk_prn_delay_chips, mean_error, std_dev_error, rmse);
                                            for (unsigned int ii = 0; ii < code_phase_error_chips.size(); ii++)
                                                {
                                                    code_phase_error_meters.push_back(GPS_L1_CA_CHIP_PERIOD * code_phase_error_chips.at(ii) * GPS_C_m_s);
                                                }
                                            mean_code_phase_error.push_back(mean_error);
                                            std_dev_code_phase_error.push_back(std_dev_error);
                                            rmse_code_phase.push_back(rmse);

                                            acc_carrier_phase_hz = check_results_acc_carrier_phase(true_timestamp_s, true_acc_carrier_phase_cycles, trk_timestamp_s, trk_acc_carrier_phase_cycles, mean_error, std_dev_error, rmse);
                                            mean_carrier_phase_error.push_back(mean_error);
                                            std_dev_carrier_phase_error.push_back(std_dev_error);
                                            rmse_carrier_phase.push_back(rmse);

                                            //save tracking measurement timestamps to std::vector
                                            std::vector<double> vector_trk_timestamp_s(trk_timestamp_s.colptr(0), trk_timestamp_s.colptr(0) + trk_timestamp_s.n_rows);
                                            trk_valid_timestamp_s_sweep.push_back(vector_trk_timestamp_s);

                                            doppler_error_sweep.push_back(doppler_error_hz);
                                            code_phase_error_sweep.push_back(code_phase_error_chips);
                                            code_phase_error_meters_sweep.push_back(code_phase_error_meters);
                                            acc_carrier_phase_error_sweep.push_back(acc_carrier_phase_hz);
                                        }
                                    else
                                        {
                                            std::cout << "Tracking output could not be used, possible loss of lock " << std::endl;
                                        }
                                }
                            catch (const std::exception& ex)
                                {
                                    std::cout << "Tracking output could not be used, possible loss of lock " << ex.what() << std::endl;
                                }
                        }

                }  //CN0 LOOP

            if (!FLAGS_enable_external_signal_file)
                {
                    mean_doppler_error_sweep.push_back(mean_doppler_error);
                    std_dev_doppler_error_sweep.push_back(std_dev_doppler_error);
                    rmse_doppler_sweep.push_back(rmse_doppler);

                    mean_code_phase_error_sweep.push_back(mean_code_phase_error);
                    std_dev_code_phase_error_sweep.push_back(std_dev_code_phase_error);
                    rmse_code_phase_sweep.push_back(rmse_code_phase);

                    mean_carrier_phase_error_sweep.push_back(mean_carrier_phase_error);
                    std_dev_carrier_phase_error_sweep.push_back(std_dev_carrier_phase_error);
                    rmse_carrier_phase_sweep.push_back(rmse_carrier_phase);

                    //make a copy of the CN0 vector for each configuration parameter in order to filter the loss of lock events
                    generator_CN0_values_sweep_copy.push_back(valid_CN0_values);
                }

            //********************************
            //***** STEP 7: Plot results *****
            //********************************
            if (FLAGS_plot_gps_l1_tracking_test == true)
                {
                    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
                    if (gnuplot_executable.empty())
                        {
                            std::cout << "WARNING: Although the flag plot_gps_l1_tracking_test has been set to TRUE," << std::endl;
                            std::cout << "gnuplot has not been found in your system." << std::endl;
                            std::cout << "Test results will not be plotted." << std::endl;
                        }
                    else
                        {
                            try
                                {
                                    boost::filesystem::path p(gnuplot_executable);
                                    boost::filesystem::path dir = p.parent_path();
                                    std::string gnuplot_path = dir.native();
                                    Gnuplot::set_GNUPlotPath(gnuplot_path);
                                    unsigned int decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

                                    if (FLAGS_plot_detail_level >= 2)
                                        {
                                            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                                                {
                                                    Gnuplot g1("linespoints");
                                                    if (FLAGS_show_plots)
                                                        {
                                                            g1.showonscreen();  // window output
                                                        }
                                                    else
                                                        {
                                                            g1.disablescreen();
                                                        }
                                                    g1.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, " + "PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz" + "GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                    g1.set_grid();
                                                    g1.set_xlabel("Time [s]");
                                                    g1.set_ylabel("Correlators' output");
                                                    //g1.cmd("set key box opaque");
                                                    g1.plot_xy(trk_timestamp_s_sweep.at(current_cn0_idx), prompt_sweep.at(current_cn0_idx), "Prompt", decimate);
                                                    g1.plot_xy(trk_timestamp_s_sweep.at(current_cn0_idx), early_sweep.at(current_cn0_idx), "Early", decimate);
                                                    g1.plot_xy(trk_timestamp_s_sweep.at(current_cn0_idx), late_sweep.at(current_cn0_idx), "Late", decimate);
                                                    g1.set_legend();
                                                    g1.savetops("Correlators_outputs" + std::to_string(generator_CN0_values.at(current_cn0_idx)));
                                                    g1.savetopdf("Correlators_outputs" + std::to_string(generator_CN0_values.at(current_cn0_idx)), 18);
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
                                            g2.set_multiplot(ceil(static_cast<float>(generator_CN0_values.size()) / 2.0),
                                                ceil(static_cast<float>(generator_CN0_values.size()) / 2));
                                            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                                                {
                                                    g2.reset_plot();
                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz" + "GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                    g2.set_grid();
                                                    g2.set_xlabel("Inphase");
                                                    g2.set_ylabel("Quadrature");
                                                    //g2.cmd("set size ratio -1");
                                                    g2.plot_xy(promptI_sweep.at(current_cn0_idx), promptQ_sweep.at(current_cn0_idx));
                                                }
                                            g2.unset_multiplot();
                                            g2.savetops("Constellation");
                                            g2.savetopdf("Constellation", 18);

                                            Gnuplot g3("linespoints");
                                            g3.set_title("GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                            g3.set_grid();
                                            g3.set_xlabel("Time [s]");
                                            g3.set_ylabel("Reported CN0 [dB-Hz]");
                                            g3.cmd("set key box opaque");
                                            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                                                {
                                                    g3.plot_xy(trk_timestamp_s_sweep.at(current_cn0_idx), CN0_dBHz_sweep.at(current_cn0_idx),
                                                        std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);
                                                }
                                            g3.set_legend();
                                            g3.savetops("CN0_output");
                                            g3.savetopdf("CN0_output", 18);
                                            if (FLAGS_show_plots)
                                                {
                                                    g3.showonscreen();  // window output
                                                }
                                            else
                                                {
                                                    g3.disablescreen();
                                                }
                                        }

                                    //PLOT ERROR FIGURES (only if it is used the signal generator)
                                    if (!FLAGS_enable_external_signal_file)
                                        {
                                            if (FLAGS_plot_detail_level >= 1)
                                                {
                                                    Gnuplot g5("points");
                                                    if (FLAGS_show_plots)
                                                        {
                                                            g5.showonscreen();  // window output
                                                        }
                                                    else
                                                        {
                                                            g5.disablescreen();
                                                        }
                                                    g5.set_title("Code delay error, PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                    g5.set_grid();
                                                    g5.set_xlabel("Time [s]");
                                                    g5.set_ylabel("Code delay error [Chips]");


                                                    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values_sweep_copy.at(config_idx).size(); current_cn0_idx++)
                                                        {
                                                            try
                                                                {
                                                                    g5.plot_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx), code_phase_error_sweep.at(current_cn0_idx),
                                                                        std::to_string(static_cast<int>(round(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)))) + "[dB-Hz]", decimate);
                                                                }
                                                            catch (const GnuplotException& ge)
                                                                {
                                                                }
                                                            save_mat_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx),
                                                                code_phase_error_sweep.at(current_cn0_idx),
                                                                "Code_error_chips" + std::to_string(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)) +
                                                                    std::to_string(PLL_wide_bw_values.at(config_idx)) + "_" + std::to_string(DLL_wide_bw_values.at(config_idx)));
                                                        }
                                                    g5.set_legend();
                                                    g5.set_legend();
                                                    g5.savetops("Code_error_chips");
                                                    g5.savetopdf("Code_error_chips", 18);

                                                    Gnuplot g5b("points");
                                                    if (FLAGS_show_plots)
                                                        {
                                                            g5b.showonscreen();  // window output
                                                        }
                                                    else
                                                        {
                                                            g5b.disablescreen();
                                                        }
                                                    g5b.set_title("Code delay error, PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                    g5b.set_grid();
                                                    g5b.set_xlabel("Time [s]");
                                                    g5b.set_ylabel("Code delay error [meters]");


                                                    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values_sweep_copy.at(config_idx).size(); current_cn0_idx++)
                                                        {
                                                            try
                                                                {
                                                                    g5b.plot_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx), code_phase_error_meters_sweep.at(current_cn0_idx),
                                                                        std::to_string(static_cast<int>(round(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)))) + "[dB-Hz]", decimate);
                                                                }
                                                            catch (const GnuplotException& ge)
                                                                {
                                                                }
                                                            save_mat_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx),
                                                                code_phase_error_sweep.at(current_cn0_idx),
                                                                "Code_error_meters" + std::to_string(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)) +
                                                                    std::to_string(PLL_wide_bw_values.at(config_idx)) + "_" + std::to_string(DLL_wide_bw_values.at(config_idx)));
                                                        }
                                                    g5b.set_legend();
                                                    g5b.set_legend();
                                                    g5b.savetops("Code_error_meters");
                                                    g5b.savetopdf("Code_error_meters", 18);


                                                    Gnuplot g6("points");
                                                    if (FLAGS_show_plots)
                                                        {
                                                            g6.showonscreen();  // window output
                                                        }
                                                    else
                                                        {
                                                            g6.disablescreen();
                                                        }
                                                    g6.set_title("Accumulated carrier phase error, PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                    g6.set_grid();
                                                    g6.set_xlabel("Time [s]");
                                                    g6.set_ylabel("Accumulated carrier phase error [Cycles]");


                                                    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values_sweep_copy.at(config_idx).size(); current_cn0_idx++)
                                                        {
                                                            try
                                                                {
                                                                    g6.plot_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx), acc_carrier_phase_error_sweep.at(current_cn0_idx),
                                                                        std::to_string(static_cast<int>(round(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)))) + "[dB-Hz]", decimate);
                                                                }
                                                            catch (const GnuplotException& ge)
                                                                {
                                                                }
                                                            save_mat_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx),
                                                                acc_carrier_phase_error_sweep.at(current_cn0_idx),
                                                                "Carrier_phase_error" + std::to_string(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)) +
                                                                    std::to_string(PLL_wide_bw_values.at(config_idx)) + "_" + std::to_string(DLL_wide_bw_values.at(config_idx)));
                                                        }
                                                    g6.set_legend();
                                                    g6.set_legend();
                                                    g6.savetops("Acc_carrier_phase_error_cycles");
                                                    g6.savetopdf("Acc_carrier_phase_error_cycles", 18);

                                                    Gnuplot g4("points");
                                                    if (FLAGS_show_plots)
                                                        {
                                                            g4.showonscreen();  // window output
                                                        }
                                                    else
                                                        {
                                                            g4.disablescreen();
                                                        }
                                                    g4.set_multiplot(ceil(static_cast<float>(generator_CN0_values.size()) / 2.0),
                                                        ceil(static_cast<float>(generator_CN0_values.size()) / 2));
                                                    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values_sweep_copy.at(config_idx).size(); current_cn0_idx++)
                                                        {
                                                            g4.reset_plot();
                                                            g4.set_title("Dopper error" + std::to_string(static_cast<int>(round(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)))) + "[dB-Hz], PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_idx)) + "," + std::to_string(DLL_wide_bw_values.at(config_idx)) + " Hz (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                            g4.set_grid();
                                                            //g4.cmd("set key box opaque");
                                                            g4.set_xlabel("Time [s]");
                                                            g4.set_ylabel("Dopper error [Hz]");
                                                            try
                                                                {
                                                                    g4.plot_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx), doppler_error_sweep.at(current_cn0_idx),
                                                                        std::to_string(static_cast<int>(round(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)))) + "[dB-Hz]", decimate);
                                                                }
                                                            catch (const GnuplotException& ge)
                                                                {
                                                                }

                                                            save_mat_xy(trk_valid_timestamp_s_sweep.at(current_cn0_idx),
                                                                doppler_error_sweep.at(current_cn0_idx),
                                                                "Doppler_error" + std::to_string(generator_CN0_values_sweep_copy.at(config_idx).at(current_cn0_idx)) +
                                                                    std::to_string(PLL_wide_bw_values.at(config_idx)) + "_" + std::to_string(DLL_wide_bw_values.at(config_idx)));
                                                        }
                                                    g4.unset_multiplot();
                                                    g4.savetops("Doppler_error_hz");
                                                    g4.savetopdf("Doppler_error_hz", 18);
                                                }
                                        }
                                }
                            catch (const GnuplotException& ge)
                                {
                                    std::cout << ge.what() << std::endl;
                                }
                        }
                }
        }

    if (FLAGS_plot_gps_l1_tracking_test == true)
        {
            std::cout << "Ploting performance metrics..." << std::endl;
            try
                {
                    if (generator_CN0_values.size() > 1)
                        {
                            //plot metrics

                            Gnuplot g7("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g7.showonscreen();  // window output
                                }
                            else
                                {
                                    g7.disablescreen();
                                }
                            g7.set_title("Doppler error metrics (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                            g7.set_grid();
                            g7.set_xlabel("CN0 [dB-Hz]");
                            g7.set_ylabel("Doppler error [Hz]");
                            g7.set_pointsize(2);
                            g7.cmd("set termoption lw 2");
                            g7.cmd("set key box opaque");
                            for (unsigned int config_sweep_idx = 0; config_sweep_idx < mean_doppler_error_sweep.size(); config_sweep_idx++)
                                {
                                    g7.plot_xy_err(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        mean_doppler_error_sweep.at(config_sweep_idx),
                                        std_dev_doppler_error_sweep.at(config_sweep_idx),
                                        "PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"," + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)) + " Hz");

                                    //matlab save
                                    save_mat_xy(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        rmse_doppler_sweep.at(config_sweep_idx),
                                        "RMSE_Doppler_CN0_Sweep_PLL_DLL" + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"_" + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)));
                                }
                            g7.savetops("Doppler_error_metrics");
                            g7.savetopdf("Doppler_error_metrics", 18);


                            Gnuplot g8("linespoints");
                            g8.set_title("Accumulated carrier phase error metrics (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                            g8.set_grid();
                            g8.set_xlabel("CN0 [dB-Hz]");
                            g8.set_ylabel("Accumulated Carrier Phase error [Cycles]");
                            g8.cmd("set key box opaque");
                            g8.cmd("set termoption lw 2");
                            g8.set_pointsize(2);
                            for (unsigned int config_sweep_idx = 0; config_sweep_idx < mean_doppler_error_sweep.size(); config_sweep_idx++)
                                {
                                    g8.plot_xy_err(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        mean_carrier_phase_error_sweep.at(config_sweep_idx),
                                        std_dev_carrier_phase_error_sweep.at(config_sweep_idx),
                                        "PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"," + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)) + " Hz");
                                    //matlab save
                                    save_mat_xy(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        rmse_carrier_phase_sweep.at(config_sweep_idx),
                                        "RMSE_Carrier_Phase_CN0_Sweep_PLL_DLL" + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"_" + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)));
                                }
                            g8.savetops("Carrier_error_metrics");
                            g8.savetopdf("Carrier_error_metrics", 18);

                            Gnuplot g9("linespoints");
                            g9.set_title("Code Phase error metrics (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                            g9.set_grid();
                            g9.set_xlabel("CN0 [dB-Hz]");
                            g9.set_ylabel("Code Phase error [Chips]");
                            g9.cmd("set key box opaque");
                            g9.cmd("set termoption lw 2");
                            g9.set_pointsize(2);
                            for (unsigned int config_sweep_idx = 0; config_sweep_idx < mean_doppler_error_sweep.size(); config_sweep_idx++)
                                {
                                    g9.plot_xy_err(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        mean_code_phase_error_sweep.at(config_sweep_idx),
                                        std_dev_code_phase_error_sweep.at(config_sweep_idx),
                                        "PLL/DLL BW: " + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"," + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)) + " Hz");
                                    //matlab save
                                    save_mat_xy(generator_CN0_values_sweep_copy.at(config_sweep_idx),
                                        rmse_code_phase_sweep.at(config_sweep_idx),
                                        "RMSE_Code_Phase_CN0_Sweep_PLL_DLL" + std::to_string(PLL_wide_bw_values.at(config_sweep_idx)) +
                                            +"_" + std::to_string(DLL_wide_bw_values.at(config_sweep_idx)));
                                }
                            g9.savetops("Code_error_metrics");
                            g9.savetopdf("Code_error_metrics", 18);
                        }
                }
            catch (const GnuplotException& ge)
                {
                    std::cout << ge.what() << std::endl;
                }
        }
}

bool GpsL1CADllPllTrackingTest::save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.erase(filename.length() - 4, 4);
            filename.append(".mat");
            matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
            if (reinterpret_cast<long*>(matfp) != NULL)
                {
                    size_t dims[2] = {1, x.size()};
                    matvar = Mat_VarCreate("x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, &x[0], 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    matvar = Mat_VarCreate("y", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, &y[0], 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            return false;
        }
}

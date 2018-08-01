/*!
 * \file hybrid_observables_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include <unistd.h>
#include <chrono>
#include <exception>
#include <armadillo>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "GPS_L1_CA.h"
#include "gnss_satellite.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"
#include "in_memory_configuration.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "tracking_true_obs_reader.h"
#include "true_observables_reader.h"
#include "tracking_dump_reader.h"
#include "observables_dump_reader.h"
#include "tlm_dump_reader.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "hybrid_observables.h"
#include "signal_generator_flags.h"
#include "gnss_sdr_sample_counter.h"
#include <matio.h>
#include "test_flags.h"
#include "observable_tests_flags.h"
#include "gnuplot_i.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TRACKING MESSAGES #########
class HybridObservablesTest_msg_rx;

typedef boost::shared_ptr<HybridObservablesTest_msg_rx> HybridObservablesTest_msg_rx_sptr;

HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make();

class HybridObservablesTest_msg_rx : public gr::block
{
private:
    friend HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_msg_rx();

public:
    int rx_message;
    ~HybridObservablesTest_msg_rx();  //!< Default destructor
};

HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make()
{
    return HybridObservablesTest_msg_rx_sptr(new HybridObservablesTest_msg_rx());
}

void HybridObservablesTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

HybridObservablesTest_msg_rx::HybridObservablesTest_msg_rx() : gr::block("HybridObservablesTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

HybridObservablesTest_msg_rx::~HybridObservablesTest_msg_rx()
{
}


// ###########################################################


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TLM MESSAGES #########
class HybridObservablesTest_tlm_msg_rx;

typedef boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> HybridObservablesTest_tlm_msg_rx_sptr;

HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make();

class HybridObservablesTest_tlm_msg_rx : public gr::block
{
private:
    friend HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_tlm_msg_rx();

public:
    int rx_message;
    ~HybridObservablesTest_tlm_msg_rx();  //!< Default destructor
};

HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make()
{
    return HybridObservablesTest_tlm_msg_rx_sptr(new HybridObservablesTest_tlm_msg_rx());
}

void HybridObservablesTest_tlm_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

HybridObservablesTest_tlm_msg_rx::HybridObservablesTest_tlm_msg_rx() : gr::block("HybridObservablesTest_tlm_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_tlm_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

HybridObservablesTest_tlm_msg_rx::~HybridObservablesTest_tlm_msg_rx()
{
}


// ###########################################################


class HybridObservablesTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    int configure_generator();
    int generate_signal();
    bool save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename);
    void check_results_carrier_phase(
        arma::mat& true_ch0,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        std::string data_title);
    void check_results_carrier_doppler(
        arma::mat& true_ch0,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        std::string data_title);
    void check_results_code_pseudorange(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1,
        std::string data_title);

    HybridObservablesTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
    }

    ~HybridObservablesTest()
    {
    }

    void configure_receiver();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    std::vector<Gnss_Synchro> gnss_synchro_vec;
    size_t item_size;
};

int HybridObservablesTest::configure_generator()
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
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  //Baseband sampling frequency [MSps]
    return 0;
}


int HybridObservablesTest::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], NULL};

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


void HybridObservablesTest::configure_receiver()
{
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    // Set Tracking
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "5.0");
    config->set_property("Tracking_1C.dll_bw_hz", "0.20");
    config->set_property("Tracking_1C.pll_bw_narrow_hz", "1.0");
    config->set_property("Tracking_1C.dll_bw_narrow_hz", "0.1");
    config->set_property("Tracking_1C.extend_correlation_symbols", "1");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");

    config->set_property("TelemetryDecoder_1C.dump", "true");
    config->set_property("Observables.dump", "true");
}

void HybridObservablesTest::check_results_carrier_phase(
    arma::mat& true_ch0,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    std::string data_title)
{
    //1. True value interpolation to match the measurement times

    double t0 = measured_ch0(0, 0);
    int size1 = measured_ch0.col(0).n_rows;
    double t1 = measured_ch0(size1 - 1, 0);
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    //conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_phase_interp;
    arma::interp1(true_tow_s, true_ch0.col(3), t, true_ch0_phase_interp);

    arma::vec meas_ch0_phase_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(3), t, meas_ch0_phase_interp);

    //2. RMSE
    arma::vec err_ch0_cycles;

    //compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    err_ch0_cycles = meas_ch0_phase_interp - true_ch0_phase_interp - meas_ch0_phase_interp(0) + true_ch0_phase_interp(0);

    arma::vec err2_ch0 = arma::square(err_ch0_cycles);
    double rmse_ch0 = sqrt(arma::mean(err2_ch0));

    //3. Mean err and variance
    double error_mean_ch0 = arma::mean(err_ch0_cycles);
    double error_var_ch0 = arma::var(err_ch0_cycles);

    // 4. Peaks
    double max_error_ch0 = arma::max(err_ch0_cycles);
    double min_error_ch0 = arma::min(err_ch0_cycles);

    //5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << data_title << " Accumulated Carrier phase RMSE = "
              << rmse_ch0 << ", mean = " << error_mean_ch0
              << ", stdev = " << sqrt(error_var_ch0)
              << " (max,min) = " << max_error_ch0
              << "," << min_error_ch0
              << " [cycles]" << std::endl;
    std::cout.precision(ss);

    //plots
    Gnuplot g3("linespoints");
    g3.set_title(data_title + "Accumulated Carrier phase error [cycles]");
    g3.set_grid();
    g3.set_xlabel("Time [s]");
    g3.set_ylabel("Carrier Phase error [cycles]");
    //conversion between arma::vec and std:vector
    std::vector<double> error_vec(err_ch0_cycles.colptr(0), err_ch0_cycles.colptr(0) + err_ch0_cycles.n_rows);
    g3.cmd("set key box opaque");
    g3.plot_xy(time_vector, error_vec,
        "Delta pseudorrange error");
    g3.set_legend();
    g3.savetops(data_title + "Carrier_phase_error");
    if (FLAGS_show_plots)
        {
            g3.showonscreen();  // window output
        }
    else
        {
            g3.disablescreen();
        }


    ASSERT_LT(rmse_ch0, 5e-2);
    ASSERT_LT(error_mean_ch0, 5e-2);
    ASSERT_GT(error_mean_ch0, -5e-2);
    ASSERT_LT(error_var_ch0, 5e-2);
    ASSERT_LT(max_error_ch0, 5e-2);
    ASSERT_GT(min_error_ch0, -5e-2);
}

void HybridObservablesTest::check_results_carrier_doppler(
    arma::mat& true_ch0,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    std::string data_title)
{
    //1. True value interpolation to match the measurement times

    double t0 = measured_ch0(0, 0);
    int size1 = measured_ch0.col(0).n_rows;
    double t1 = measured_ch0(size1 - 1, 0);
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    //conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_doppler_interp;
    arma::interp1(true_tow_s, true_ch0.col(2), t, true_ch0_doppler_interp);

    arma::vec meas_ch0_doppler_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(2), t, meas_ch0_doppler_interp);

    //2. RMSE
    arma::vec err_ch0_hz;

    //compute error
    err_ch0_hz = meas_ch0_doppler_interp - true_ch0_doppler_interp;

    arma::vec err2_ch0 = arma::square(err_ch0_hz);
    double rmse_ch0 = sqrt(arma::mean(err2_ch0));

    //3. Mean err and variance
    double error_mean_ch0 = arma::mean(err_ch0_hz);
    double error_var_ch0 = arma::var(err_ch0_hz);

    // 4. Peaks
    double max_error_ch0 = arma::max(err_ch0_hz);
    double min_error_ch0 = arma::min(err_ch0_hz);

    //5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << data_title << "Carrier Doppler RMSE = "
              << rmse_ch0 << ", mean = " << error_mean_ch0
              << ", stdev = " << sqrt(error_var_ch0)
              << " (max,min) = " << max_error_ch0
              << "," << min_error_ch0
              << " [Hz]" << std::endl;
    std::cout.precision(ss);

    //plots
    Gnuplot g3("linespoints");
    g3.set_title(data_title + "Carrier Doppler error [Hz]");
    g3.set_grid();
    g3.set_xlabel("Time [s]");
    g3.set_ylabel("Carrier Doppler error [Hz]");
    //conversion between arma::vec and std:vector
    std::vector<double> error_vec(err_ch0_hz.colptr(0), err_ch0_hz.colptr(0) + err_ch0_hz.n_rows);
    g3.cmd("set key box opaque");
    g3.plot_xy(time_vector, error_vec,
        "Delta pseudorrange error");
    g3.set_legend();
    g3.savetops(data_title + "Carrier_doppler_error");
    if (FLAGS_show_plots)
        {
            g3.showonscreen();  // window output
        }
    else
        {
            g3.disablescreen();
        }

    ASSERT_LT(rmse_ch0, 5);
    ASSERT_LT(error_mean_ch0, 5);
    ASSERT_GT(error_mean_ch0, -5);
    ASSERT_LT(error_var_ch0, 10);
    ASSERT_LT(max_error_ch0, 10);
    ASSERT_GT(min_error_ch0, -5);
}

bool HybridObservablesTest::save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.append(".mat");
            std::cout << "save_mat_xy write " << filename << std::endl;
            matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT5);
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
            else
                {
                    std::cout << "save_mat_xy: error creating file" << std::endl;
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_xy: " << ex.what() << std::endl;
            return false;
        }
}

void HybridObservablesTest::check_results_code_pseudorange(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    std::string data_title)
{
    //1. True value interpolation to match the measurement times

    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    //conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);


    arma::vec true_ch0_dist_interp;
    arma::vec true_ch1_dist_interp;
    arma::interp1(true_tow_s, true_ch0.col(1), t, true_ch0_dist_interp);
    arma::interp1(true_tow_s, true_ch1.col(1), t, true_ch1_dist_interp);

    arma::vec meas_ch0_dist_interp;
    arma::vec meas_ch1_dist_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(4), t, meas_ch0_dist_interp);
    arma::interp1(measured_ch1.col(0), measured_ch1.col(4), t, meas_ch1_dist_interp);

    // generate delta pseudoranges
    arma::vec delta_true_dist_m = true_ch0_dist_interp - true_ch1_dist_interp;
    arma::vec delta_measured_dist_m = meas_ch0_dist_interp - meas_ch1_dist_interp;

    //2. RMSE
    arma::vec err;

    err = delta_measured_dist_m - delta_true_dist_m;
    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    //3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    //5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << data_title << "Delta Observables RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [meters]" << std::endl;
    std::cout.precision(ss);

    //plots
    Gnuplot g3("linespoints");
    g3.set_title(data_title + "Delta Pseudorange error [m]");
    g3.set_grid();
    g3.set_xlabel("Time [s]");
    g3.set_ylabel("Pseudorange error [m]");
    //conversion between arma::vec and std:vector
    std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
    g3.cmd("set key box opaque");
    g3.plot_xy(time_vector, range_error_m,
        "Delta pseudorrange error");
    g3.set_legend();
    g3.savetops(data_title + "Delta_pseudorrange_error");
    if (FLAGS_show_plots)
        {
            g3.showonscreen();  // window output
        }
    else
        {
            g3.disablescreen();
        }

    //check results against the test tolerance
    ASSERT_LT(rmse, 0.5);
    ASSERT_LT(error_mean, 0.5);
    ASSERT_GT(error_mean, -0.5);
    ASSERT_LT(error_var, 0.5);
    ASSERT_LT(max_error, 2.0);
    ASSERT_GT(min_error, -2.0);
}


TEST_F(HybridObservablesTest, ValidationOfResults)
{
    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    if (FLAGS_disable_generator == false)
        {
            generate_signal();
        }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(tmp_gnss_synchro.Signal, 2, 0);

    std::istringstream ss(FLAGS_test_satellite_PRN_list);
    std::string token;

    while (std::getline(ss, token, ','))
        {
            tmp_gnss_synchro.PRN = boost::lexical_cast<int>(token);
            gnss_synchro_vec.push_back(tmp_gnss_synchro);
        }

    configure_receiver();

    //open true observables log file written by the simulator
    std::vector<std::shared_ptr<tracking_true_obs_reader>> true_reader_vec;
    std::vector<std::shared_ptr<TrackingInterface>> tracking_ch_vec;
    std::vector<std::shared_ptr<TelemetryDecoderInterface>> tlm_ch_vec;

    std::vector<gr::blocks::null_sink::sptr> null_sink_vec;
    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
        {
            //set channels ids
            gnss_synchro_vec.at(n).Channel_ID = n;
            //read true data from the generator logs
            true_reader_vec.push_back(std::make_shared<tracking_true_obs_reader>());
            std::cout << "Loading true observable data for PRN " << gnss_synchro_vec.at(n).PRN << std::endl;
            std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
            true_obs_file.append(std::to_string(gnss_synchro_vec.at(n).PRN));
            true_obs_file.append(".dat");
            ASSERT_NO_THROW({
                if (true_reader_vec.back()->open_obs_file(true_obs_file) == false)
                    {
                        throw std::exception();
                    };
            }) << "Failure opening true observables file";

            // load acquisition data based on the first epoch of the true observations
            ASSERT_NO_THROW({
                if (true_reader_vec.back()->read_binary_obs() == false)
                    {
                        throw std::exception();
                    };
            }) << "Failure reading true observables file";

            //restart the epoch counter
            true_reader_vec.back()->restart();

            std::cout << "Initial Doppler [Hz]=" << true_reader_vec.back()->doppler_l1_hz << " Initial code delay [Chips]="
                      << true_reader_vec.back()->prn_delay_chips << std::endl;

            gnss_synchro_vec.at(n).Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_reader_vec.back()->prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
            gnss_synchro_vec.at(n).Acq_doppler_hz = true_reader_vec.back()->doppler_l1_hz;
            gnss_synchro_vec.at(n).Acq_samplestamp_samples = 0;


            //create the tracking channels
            tracking_ch_vec.push_back(std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1));
            //create the telemetry decoders
            tlm_ch_vec.push_back(std::make_shared<GpsL1CaTelemetryDecoder>(config.get(), "TelemetryDecoder_1C", 1, 1));
            //create null sinks for observables output
            null_sink_vec.push_back(gr::blocks::null_sink::make(sizeof(Gnss_Synchro)));

            ASSERT_NO_THROW({
                tlm_ch_vec.back()->set_channel(gnss_synchro_vec.at(n).Channel_ID);
                tlm_ch_vec.back()->set_satellite(Gnss_Satellite(std::string("GPS"), gnss_synchro_vec.at(n).PRN));
            }) << "Failure setting gnss_synchro.";

            ASSERT_NO_THROW({
                tracking_ch_vec.back()->set_channel(gnss_synchro_vec.at(n).Channel_ID);
            }) << "Failure setting channel.";

            ASSERT_NO_THROW({
                tracking_ch_vec.back()->set_gnss_synchro(&gnss_synchro_vec.at(n));
            }) << "Failure setting gnss_synchro.";
        }

    top_block = gr::make_top_block("Telemetry_Decoder test");
    boost::shared_ptr<HybridObservablesTest_msg_rx> dummy_msg_rx_trk = HybridObservablesTest_msg_rx_make();
    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> dummy_tlm_msg_rx = HybridObservablesTest_tlm_msg_rx_make();
    //Observables
    std::shared_ptr<ObservablesInterface> observables(new HybridObservables(config.get(), "Observables", tracking_ch_vec.size() + 1, tracking_ch_vec.size()));

    for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
        {
            ASSERT_NO_THROW({
                tracking_ch_vec.at(n)->connect(top_block);
            }) << "Failure connecting tracking to the top_block.";
        }


    ASSERT_NO_THROW({
        std::string file = "./" + filename_raw_data;
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gnss_sdr_sample_counter_sptr samp_counter = gnss_sdr_make_sample_counter(static_cast<double>(baseband_sampling_freq), sizeof(gr_complex));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, samp_counter, 0);

        for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
            {
                top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tracking_ch_vec.at(n)->get_right_block(), 0, tlm_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tlm_ch_vec.at(n)->get_right_block(), 0, observables->get_left_block(), n);
                top_block->msg_connect(tracking_ch_vec.at(n)->get_right_block(), pmt::mp("events"), dummy_msg_rx_trk, pmt::mp("events"));
                top_block->connect(observables->get_right_block(), n, null_sink_vec.at(n), 0);
            }
        //connect sample counter and timmer to the last channel in observables block (extra channel)
        top_block->connect(samp_counter, 0, observables->get_left_block(), tracking_ch_vec.size());

    }) << "Failure connecting the blocks.";

    for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
        {
            tracking_ch_vec.at(n)->start_tracking();
        }

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    //check results
    //load the true values

    true_observables_reader true_observables;

    ASSERT_NO_THROW({
        if (true_observables.open_obs_file(std::string("./obs_out.bin")) == false)
            {
                throw std::exception();
            }
    }) << "Failure opening true observables file";

    unsigned int nepoch = static_cast<unsigned int>(true_observables.num_epochs());

    std::cout << "True observation epochs = " << nepoch << std::endl;
    // Matrices for storing columnwise true GPS time, Range, Doppler and Carrier phase
    std::vector<arma::mat> true_obs_vec;
    true_observables.restart();
    long int epoch_counter = 0;
    for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
        {
            true_obs_vec.push_back(arma::zeros<arma::mat>(nepoch, 4));
        }

    ASSERT_NO_THROW({
        while (true_observables.read_binary_obs())
            {
                for (unsigned int n = 0; n < true_obs_vec.size(); n++)
                    {
                        if (round(true_observables.prn[n]) != gnss_synchro_vec.at(n).PRN)
                            {
                                std::cout << "True observables SV PRN does not match measured ones: "
                                          << round(true_observables.prn[n]) << " vs. " << gnss_synchro_vec.at(n).PRN << std::endl;
                                throw std::exception();
                            }
                        true_obs_vec.at(n)(epoch_counter, 0) = true_observables.gps_time_sec[n];
                        true_obs_vec.at(n)(epoch_counter, 1) = true_observables.dist_m[n];
                        true_obs_vec.at(n)(epoch_counter, 2) = true_observables.doppler_l1_hz[n];
                        true_obs_vec.at(n)(epoch_counter, 3) = true_observables.acc_carrier_phase_l1_cycles[n];
                    }
                epoch_counter++;
            }
    });

    //read measured values
    observables_dump_reader estimated_observables(tracking_ch_vec.size());
    ASSERT_NO_THROW({
        if (estimated_observables.open_obs_file(std::string("./observables.dat")) == false)
            {
                throw std::exception();
            }
    }) << "Failure opening dump observables file";

    nepoch = static_cast<unsigned int>(estimated_observables.num_epochs());
    std::cout << "Measured observations epochs = " << nepoch << std::endl;

    // Matrices for storing columnwise measured RX_time, TOW, Doppler, Carrier phase and Pseudorange
    std::vector<arma::mat> measured_obs_vec;
    std::vector<long int> epoch_counters_vec;
    for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
        {
            measured_obs_vec.push_back(arma::zeros<arma::mat>(nepoch, 5));
            epoch_counters_vec.push_back(0);
        }

    estimated_observables.restart();
    while (estimated_observables.read_binary_obs())
        {
            for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
                {
                    if (static_cast<bool>(estimated_observables.valid[n]))
                        {
                            measured_obs_vec.at(n)(epoch_counters_vec.at(n), 0) = estimated_observables.RX_time[n];
                            measured_obs_vec.at(n)(epoch_counters_vec.at(n), 1) = estimated_observables.TOW_at_current_symbol_s[n];
                            measured_obs_vec.at(n)(epoch_counters_vec.at(n), 2) = estimated_observables.Carrier_Doppler_hz[n];
                            measured_obs_vec.at(n)(epoch_counters_vec.at(n), 3) = estimated_observables.Acc_carrier_phase_hz[n];
                            measured_obs_vec.at(n)(epoch_counters_vec.at(n), 4) = estimated_observables.Pseudorange_m[n];
                            epoch_counters_vec.at(n)++;
                        }
                }
        }


    //Cut measurement tail zeros
    arma::uvec index;
    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            index = arma::find(measured_obs_vec.at(n).col(0) > 0.0, 1, "last");
            if ((index.size() > 0) and index(0) < (nepoch - 1))
                {
                    measured_obs_vec.at(n).shed_rows(index(0) + 1, nepoch - 1);
                }
        }

    //Cut measurement initial transitory of the measurements

    double initial_transitory_s = FLAGS_skip_obs_transitory_s;

    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            index = arma::find(measured_obs_vec.at(n).col(0) >= (measured_obs_vec.at(n)(0, 0) + initial_transitory_s), 1, "first");
            if ((index.size() > 0) and (index(0) > 0))
                {
                    measured_obs_vec.at(n).shed_rows(0, index(0));
                }

            index = arma::find(measured_obs_vec.at(n).col(0) >= true_obs_vec.at(n)(0, 0), 1, "first");
            if ((index.size() > 0) and (index(0) > 0))
                {
                    measured_obs_vec.at(n).shed_rows(0, index(0));
                }
        }


    //Correct the clock error using true values (it is not possible for a receiver to correct
    //the receiver clock offset error at the observables level because it is required the
    //decoding of the ephemeris data and solve the PVT equations)

    //Find the reference satellite (the nearest) and compute the receiver time offset at observable level

    double min_pr = std::numeric_limits<double>::max();
    unsigned int min_pr_ch_id = 0;
    arma::vec receiver_time_offset_s;
    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            if (measured_obs_vec.at(n)(0, 4) < min_pr)
                {
                    min_pr = measured_obs_vec.at(n)(0, 4);
                    min_pr_ch_id = n;
                }
        }

    receiver_time_offset_s = true_obs_vec.at(min_pr_ch_id).col(1) / GPS_C_m_s - GPS_STARTOFFSET_ms / 1000.0;

    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            arma::vec corrected_reference_TOW_s = true_obs_vec.at(min_pr_ch_id).col(0) - receiver_time_offset_s;
            std::cout << "[CH " << n << "] Receiver time offset " << receiver_time_offset_s(0) * 1e3 << " [ms]" << std::endl;

            //Compare measured observables
            if (min_pr_ch_id != n)
                {
                    check_results_code_pseudorange(true_obs_vec.at(n),
                        true_obs_vec.at(min_pr_ch_id),
                        corrected_reference_TOW_s,
                        measured_obs_vec.at(n),
                        measured_obs_vec.at(min_pr_ch_id),
                        "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
                }
            else
                {
                    std::cout << "[CH " << std::to_string(n) << "] PRN " << std::to_string(gnss_synchro_vec.at(n).PRN) << " is the reference satellite" << std::endl;
                }
            std::cout << "true_obs_vec.at(n): " << true_obs_vec.at(n)(0, 2) << std::endl;
            check_results_carrier_phase(true_obs_vec.at(n),
                corrected_reference_TOW_s,
                measured_obs_vec.at(n),
                "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
            check_results_carrier_doppler(true_obs_vec.at(n),
                corrected_reference_TOW_s,
                measured_obs_vec.at(n),
                "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
        }
    std::cout << "Test completed in " << elapsed_seconds.count() << " [s]" << std::endl;
}

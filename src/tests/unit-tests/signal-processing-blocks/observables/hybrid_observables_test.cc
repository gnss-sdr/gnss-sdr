/*!
 * \file hybrid_observables_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
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
    void check_results_carrier_phase(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1);
    void check_results_code_psudorange(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1);

    HybridObservablesTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro_ch0 = Gnss_Synchro();
        gnss_synchro_ch1 = Gnss_Synchro();
    }

    ~HybridObservablesTest()
    {
    }

    void configure_receiver();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro_ch0;
    Gnss_Synchro gnss_synchro_ch1;
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
    gnss_synchro_ch0.Channel_ID = 0;
    gnss_synchro_ch0.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro_ch0.Signal, 2, 0);
    gnss_synchro_ch0.PRN = FLAGS_test_satellite_PRN;

    gnss_synchro_ch1.Channel_ID = 1;
    gnss_synchro_ch1.System = 'G';
    signal.copy(gnss_synchro_ch1.Signal, 2, 0);
    gnss_synchro_ch1.PRN = FLAGS_test_satellite_PRN2;


    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    // Set Tracking
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "35.0");
    config->set_property("Tracking_1C.dll_bw_hz", "0.5");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
    config->set_property("Tracking_1C.unified", "true");

    config->set_property("TelemetryDecoder_1C.dump", "true");
    config->set_property("Observables.dump", "true");
}

void HybridObservablesTest::check_results_carrier_phase(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1)
{
    //1. True value interpolation to match the measurement times

    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));

    arma::vec true_ch0_phase_interp;
    arma::vec true_ch1_phase_interp;
    arma::interp1(true_tow_s, true_ch0.col(3), t, true_ch0_phase_interp);
    arma::interp1(true_tow_s, true_ch1.col(3), t, true_ch1_phase_interp);

    arma::vec meas_ch0_phase_interp;
    arma::vec meas_ch1_phase_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(3), t, meas_ch0_phase_interp);
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), t, meas_ch1_phase_interp);

    //2. RMSE
    arma::vec err_ch0_cycles;
    arma::vec err_ch1_cycles;

    //compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    err_ch0_cycles = meas_ch0_phase_interp - true_ch0_phase_interp - meas_ch0_phase_interp(0) + true_ch0_phase_interp(0);
    err_ch1_cycles = meas_ch1_phase_interp - true_ch1_phase_interp - meas_ch1_phase_interp(0) + true_ch1_phase_interp(0);

    arma::vec err2_ch0 = arma::square(err_ch0_cycles);
    double rmse_ch0 = sqrt(arma::mean(err2_ch0));

    //3. Mean err and variance
    double error_mean_ch0 = arma::mean(err_ch0_cycles);
    double error_var_ch0 = arma::var(err_ch0_cycles);

    // 4. Peaks
    double max_error_ch0 = arma::max(err_ch0_cycles);
    double min_error_ch0 = arma::min(err_ch0_cycles);

    arma::vec err2_ch1 = arma::square(err_ch1_cycles);
    double rmse_ch1 = sqrt(arma::mean(err2_ch1));

    //3. Mean err and variance
    double error_mean_ch1 = arma::mean(err_ch1_cycles);
    double error_var_ch1 = arma::var(err_ch1_cycles);

    // 4. Peaks
    double max_error_ch1 = arma::max(err_ch1_cycles);
    double min_error_ch1 = arma::min(err_ch1_cycles);


    //5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "Channel 0 Carrier phase RMSE = "
              << rmse_ch0 << ", mean = " << error_mean_ch0
              << ", stdev = " << sqrt(error_var_ch0)
              << " (max,min) = " << max_error_ch0
              << "," << min_error_ch0
              << " [cycles]" << std::endl;
    std::cout.precision(ss);

    ASSERT_LT(rmse_ch0, 5e-2);
    ASSERT_LT(error_mean_ch0, 5e-2);
    ASSERT_GT(error_mean_ch0, -5e-2);
    ASSERT_LT(error_var_ch0, 5e-2);
    ASSERT_LT(max_error_ch0, 5e-2);
    ASSERT_GT(min_error_ch0, -5e-2);

    //5. report
    ss = std::cout.precision();
    std::cout << std::setprecision(10) << "Channel 1 Carrier phase RMSE = "
              << rmse_ch1 << ", mean = " << error_mean_ch1
              << ", stdev = " << sqrt(error_var_ch1)
              << " (max,min) = " << max_error_ch1
              << "," << min_error_ch1
              << " [cycles]" << std::endl;
    std::cout.precision(ss);

    ASSERT_LT(rmse_ch1, 5e-2);
    ASSERT_LT(error_mean_ch1, 5e-2);
    ASSERT_GT(error_mean_ch1, -5e-2);
    ASSERT_LT(error_var_ch1, 5e-2);
    ASSERT_LT(max_error_ch1, 5e-2);
    ASSERT_GT(min_error_ch1, -5e-2);
}


void HybridObservablesTest::check_results_code_psudorange(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1)
{
    //1. True value interpolation to match the measurement times

    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));

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
    std::cout << std::setprecision(10) << "Delta Observables RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [meters]" << std::endl;
    std::cout.precision(ss);

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

    configure_receiver();

    //open true observables log file written by the simulator
    tracking_true_obs_reader true_obs_data_ch0;
    tracking_true_obs_reader true_obs_data_ch1;
    int test_satellite_PRN = FLAGS_test_satellite_PRN;
    int test_satellite_PRN2 = FLAGS_test_satellite_PRN2;
    std::cout << "Testing satellite PRNs " << test_satellite_PRN << "," << test_satellite_PRN << std::endl;
    std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN));
    true_obs_file.append(".dat");
    ASSERT_NO_THROW({
        if (true_obs_data_ch0.open_obs_file(true_obs_file) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file";

    true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN2));
    true_obs_file.append(".dat");
    ASSERT_NO_THROW({
        if (true_obs_data_ch1.open_obs_file(true_obs_file) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file";

    top_block = gr::make_top_block("Telemetry_Decoder test");
    std::shared_ptr<TrackingInterface> tracking_ch0 = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    std::shared_ptr<TrackingInterface> tracking_ch1 = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);

    boost::shared_ptr<HybridObservablesTest_msg_rx> msg_rx_ch0 = HybridObservablesTest_msg_rx_make();
    boost::shared_ptr<HybridObservablesTest_msg_rx> msg_rx_ch1 = HybridObservablesTest_msg_rx_make();

    // load acquisition data based on the first epoch of the true observations
    ASSERT_NO_THROW({
        if (true_obs_data_ch0.read_binary_obs() == false)
            {
                throw std::exception();
            };
    }) << "Failure reading true observables file";

    ASSERT_NO_THROW({
        if (true_obs_data_ch1.read_binary_obs() == false)
            {
                throw std::exception();
            };
    }) << "Failure reading true observables file";

    //restart the epoch counter
    true_obs_data_ch0.restart();
    true_obs_data_ch1.restart();

    std::cout << "Initial Doppler [Hz]=" << true_obs_data_ch0.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data_ch0.prn_delay_chips << std::endl;

    gnss_synchro_ch0.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data_ch0.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
    gnss_synchro_ch0.Acq_doppler_hz = true_obs_data_ch0.doppler_l1_hz;
    gnss_synchro_ch0.Acq_samplestamp_samples = 0;

    std::cout << "Initial Doppler [Hz]=" << true_obs_data_ch1.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data_ch1.prn_delay_chips << std::endl;

    gnss_synchro_ch1.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data_ch1.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
    gnss_synchro_ch1.Acq_doppler_hz = true_obs_data_ch1.doppler_l1_hz;
    gnss_synchro_ch1.Acq_samplestamp_samples = 0;

    //telemetry decoders
    std::shared_ptr<TelemetryDecoderInterface> tlm_ch0(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C", 1, 1));
    std::shared_ptr<TelemetryDecoderInterface> tlm_ch1(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C", 1, 1));

    ASSERT_NO_THROW({
        tlm_ch0->set_channel(0);
        tlm_ch1->set_channel(1);

        tlm_ch0->set_satellite(Gnss_Satellite(std::string("GPS"), gnss_synchro_ch0.PRN));
        tlm_ch1->set_satellite(Gnss_Satellite(std::string("GPS"), gnss_synchro_ch1.PRN));
    }) << "Failure setting gnss_synchro.";

    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> tlm_msg_rx_ch1 = HybridObservablesTest_tlm_msg_rx_make();
    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> tlm_msg_rx_ch2 = HybridObservablesTest_tlm_msg_rx_make();

    //Observables
    std::shared_ptr<ObservablesInterface> observables(new HybridObservables(config.get(), "Observables", 3, 2));

    ASSERT_NO_THROW({
        tracking_ch0->set_channel(gnss_synchro_ch0.Channel_ID);
        tracking_ch1->set_channel(gnss_synchro_ch1.Channel_ID);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        tracking_ch0->set_gnss_synchro(&gnss_synchro_ch0);
        tracking_ch1->set_gnss_synchro(&gnss_synchro_ch1);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        tracking_ch0->connect(top_block);
        tracking_ch1->connect(top_block);
    }) << "Failure connecting tracking to the top_block.";

    ASSERT_NO_THROW({
        std::string file = "./" + filename_raw_data;
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink_ch0 = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        gr::blocks::null_sink::sptr sink_ch1 = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        gnss_sdr_sample_counter_sptr samp_counter = gnss_sdr_make_sample_counter(static_cast<double>(baseband_sampling_freq), sizeof(int8_t));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, samp_counter, 0);

        //ch0
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch0->get_left_block(), 0);
        top_block->connect(tracking_ch0->get_right_block(), 0, tlm_ch0->get_left_block(), 0);
        top_block->connect(tlm_ch0->get_right_block(), 0, observables->get_left_block(), 0);
        top_block->msg_connect(tracking_ch0->get_right_block(), pmt::mp("events"), msg_rx_ch0, pmt::mp("events"));
        //ch1
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch1->get_left_block(), 0);
        top_block->connect(tracking_ch1->get_right_block(), 0, tlm_ch1->get_left_block(), 0);
        top_block->connect(tlm_ch1->get_right_block(), 0, observables->get_left_block(), 1);
        top_block->msg_connect(tracking_ch1->get_right_block(), pmt::mp("events"), msg_rx_ch1, pmt::mp("events"));

        top_block->connect(observables->get_right_block(), 0, sink_ch0, 0);
        top_block->connect(observables->get_right_block(), 1, sink_ch1, 0);
        top_block->connect(samp_counter, 0, observables->get_left_block(), 2);

    }) << "Failure connecting the blocks.";

    tracking_ch0->start_tracking();
    tracking_ch1->start_tracking();

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
    arma::mat true_ch0 = arma::zeros<arma::mat>(nepoch, 4);
    arma::mat true_ch1 = arma::zeros<arma::mat>(nepoch, 4);

    true_observables.restart();
    long int epoch_counter = 0;
    ASSERT_NO_THROW({
        while (true_observables.read_binary_obs())
            {
                if (round(true_observables.prn[0]) != gnss_synchro_ch0.PRN)
                    {
                        std::cout << "True observables SV PRN does not match " << round(true_observables.prn[1]) << std::endl;
                        throw std::exception();
                    }
                if (round(true_observables.prn[1]) != gnss_synchro_ch1.PRN)
                    {
                        std::cout << "True observables SV PRN does not match " << round(true_observables.prn[1]) << std::endl;
                        throw std::exception();
                    }
                true_ch0(epoch_counter, 0) = true_observables.gps_time_sec[0];
                true_ch0(epoch_counter, 1) = true_observables.dist_m[0];
                true_ch0(epoch_counter, 2) = true_observables.doppler_l1_hz[0];
                true_ch0(epoch_counter, 3) = true_observables.acc_carrier_phase_l1_cycles[0];

                true_ch1(epoch_counter, 0) = true_observables.gps_time_sec[1];
                true_ch1(epoch_counter, 1) = true_observables.dist_m[1];
                true_ch1(epoch_counter, 2) = true_observables.doppler_l1_hz[1];
                true_ch1(epoch_counter, 3) = true_observables.acc_carrier_phase_l1_cycles[1];

                epoch_counter++;
            }
    });

    //read measured values
    observables_dump_reader estimated_observables(2);  //two channels
    ASSERT_NO_THROW({
        if (estimated_observables.open_obs_file(std::string("./observables.dat")) == false)
            {
                throw std::exception();
            }
    }) << "Failure opening dump observables file";

    nepoch = static_cast<unsigned int>(estimated_observables.num_epochs());
    std::cout << "Measured observation epochs = " << nepoch << std::endl;

    // Matrices for storing columnwise measured RX_time, TOW, Doppler, Carrier phase and Pseudorange
    arma::mat measured_ch0 = arma::zeros<arma::mat>(nepoch, 5);
    arma::mat measured_ch1 = arma::zeros<arma::mat>(nepoch, 5);

    estimated_observables.restart();
    epoch_counter = 0;
    long int epoch_counter2 = 0;
    while (estimated_observables.read_binary_obs())
        {
            if (static_cast<bool>(estimated_observables.valid[0]))
                {
                    measured_ch0(epoch_counter, 0) = estimated_observables.RX_time[0];
                    measured_ch0(epoch_counter, 1) = estimated_observables.TOW_at_current_symbol_s[0];
                    measured_ch0(epoch_counter, 2) = estimated_observables.Carrier_Doppler_hz[0];
                    measured_ch0(epoch_counter, 3) = estimated_observables.Acc_carrier_phase_hz[0];
                    measured_ch0(epoch_counter, 4) = estimated_observables.Pseudorange_m[0];
                    epoch_counter++;
                }
            if (static_cast<bool>(estimated_observables.valid[1]))
                {
                    measured_ch1(epoch_counter2, 0) = estimated_observables.RX_time[1];
                    measured_ch1(epoch_counter2, 1) = estimated_observables.TOW_at_current_symbol_s[1];
                    measured_ch1(epoch_counter2, 2) = estimated_observables.Carrier_Doppler_hz[1];
                    measured_ch1(epoch_counter2, 3) = estimated_observables.Acc_carrier_phase_hz[1];
                    measured_ch1(epoch_counter2, 4) = estimated_observables.Pseudorange_m[1];
                    epoch_counter2++;
                }
        }

    //Cut measurement tail zeros
    arma::uvec index = arma::find(measured_ch0.col(0) > 0.0, 1, "last");
    if ((index.size() > 0) and index(0) < (nepoch - 1))
        {
            measured_ch0.shed_rows(index(0) + 1, nepoch - 1);
        }
    index = arma::find(measured_ch1.col(0) > 0.0, 1, "last");
    if ((index.size() > 0) and index(0) < (nepoch - 1))
        {
            measured_ch1.shed_rows(index(0) + 1, nepoch - 1);
        }

    //Cut measurement initial transitory of the measurements
    index = arma::find(measured_ch0.col(0) >= true_ch0(0, 0), 1, "first");
    if ((index.size() > 0) and (index(0) > 0))
        {
            measured_ch0.shed_rows(0, index(0));
        }
    index = arma::find(measured_ch1.col(0) >= true_ch1(0, 0), 1, "first");
    if ((index.size() > 0) and (index(0) > 0))
        {
            measured_ch1.shed_rows(0, index(0));
        }

    //Correct the clock error using true values (it is not possible for a receiver to correct
    //the receiver clock offset error at the observables level because it is required the
    //decoding of the ephemeris data and solve the PVT equations)

    //Find the reference satellite (the nearest) and compute the receiver time offset at observable level
    arma::vec receiver_time_offset_s;
    if (measured_ch0(0, 4) < measured_ch1(0, 4))
        {
            receiver_time_offset_s = true_ch0.col(1) / GPS_C_m_s - GPS_STARTOFFSET_ms / 1000.0;
        }
    else
        {
            receiver_time_offset_s = true_ch1.col(1) / GPS_C_m_s - GPS_STARTOFFSET_ms / 1000.0;
        }
    arma::vec corrected_reference_TOW_s = true_ch0.col(0) - receiver_time_offset_s;
    std::cout << "Receiver time offset: " << receiver_time_offset_s(0) * 1e3 << " [ms]" << std::endl;

    //Compare measured observables
    check_results_code_psudorange(true_ch0, true_ch1, corrected_reference_TOW_s, measured_ch0, measured_ch1);
    check_results_carrier_phase(true_ch0, true_ch1, corrected_reference_TOW_s, measured_ch0, measured_ch1);

    std::cout << "Test completed in " << elapsed_seconds.count() << " [s]" << std::endl;
}

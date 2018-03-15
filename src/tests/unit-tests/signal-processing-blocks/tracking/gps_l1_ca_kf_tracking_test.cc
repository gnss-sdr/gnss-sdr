/*!
 * \file gps_l1_ca_kf_tracking_test.cc
 * \brief  This class implements a tracking test for GPS_L1_CA_KF_Tracking
 *  implementation based on some input parameters.
 * \author Carles Fernandez, 2018
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2017  (see AUTHORS file for a list of contributors)
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

#include <chrono>
#include <unistd.h>
#include <vector>
#include <armadillo>
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
#include "gnss_sdr_flags.h"

DEFINE_bool(plot_gps_l1_kf_tracking_test, false, "Plots results of GpsL1CAKfTrackingTest with gnuplot");


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CAKfTrackingTest_msg_rx;

typedef boost::shared_ptr<GpsL1CAKfTrackingTest_msg_rx> GpsL1CAKfTrackingTest_msg_rx_sptr;

GpsL1CAKfTrackingTest_msg_rx_sptr GpsL1CAKfTrackingTest_msg_rx_make();

class GpsL1CAKfTrackingTest_msg_rx : public gr::block
{
private:
    friend GpsL1CAKfTrackingTest_msg_rx_sptr GpsL1CAKfTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CAKfTrackingTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CAKfTrackingTest_msg_rx();  //!< Default destructor
};


GpsL1CAKfTrackingTest_msg_rx_sptr GpsL1CAKfTrackingTest_msg_rx_make()
{
    return GpsL1CAKfTrackingTest_msg_rx_sptr(new GpsL1CAKfTrackingTest_msg_rx());
}


void GpsL1CAKfTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
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


GpsL1CAKfTrackingTest_msg_rx::GpsL1CAKfTrackingTest_msg_rx() : gr::block("GpsL1CAKfTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CAKfTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CAKfTrackingTest_msg_rx::~GpsL1CAKfTrackingTest_msg_rx()
{
}


// ###########################################################

class GpsL1CAKfTrackingTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    std::string implementation = "GPS_L1_CA_KF_Tracking";  // "GPS_L1_CA_KF_Tracking";

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    int configure_generator();
    int generate_signal();
    void check_results_doppler(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value);
    void check_results_acc_carrier_phase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value);
    void check_results_codephase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value);

    GpsL1CAKfTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CAKfTrackingTest()
    {
    }

    void configure_receiver();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


int GpsL1CAKfTrackingTest::configure_generator()
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
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  // Baseband sampling frequency [MSps]
    return 0;
}


int GpsL1CAKfTrackingTest::generate_signal()
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


void GpsL1CAKfTrackingTest::configure_receiver()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));
    // Set Tracking
    config->set_property("Tracking_1C.implementation", implementation);
    config->set_property("Tracking_1C.item_type", "gr_complex");
    if (FLAGS_dll_bw_hz != 0.0)
        {
            config->set_property("Tracking_1C.dll_bw_hz", std::to_string(FLAGS_dll_bw_hz));
        }
    else
        {
            config->set_property("Tracking_1C.dll_bw_hz", "2.0");
        }
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
    config->set_property("Tracking_1C.extend_correlation_ms", "1");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
}


void GpsL1CAKfTrackingTest::check_results_doppler(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value)
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
    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK Doppler RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Hz]" << std::endl;
    std::cout.precision(ss);
}


void GpsL1CAKfTrackingTest::check_results_acc_carrier_phase(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value)
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
    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK acc carrier phase RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Hz]" << std::endl;
    std::cout.precision(ss);
}


void GpsL1CAKfTrackingTest::check_results_codephase(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value)
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
    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TRK code phase RMSE=" << rmse
              << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var) << " (max,min)=" << max_error << "," << min_error << " [Chips]" << std::endl;
    std::cout.precision(ss);
}


TEST_F(GpsL1CAKfTrackingTest, ValidationOfResults)
{
    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    if (FLAGS_disable_generator == false)
        {
            generate_signal();
        }

    std::chrono::time_point<std::chrono::system_clock> start, end;

    configure_receiver();

    // open true observables log file written by the simulator
    tracking_true_obs_reader true_obs_data;
    int test_satellite_PRN = FLAGS_test_satellite_PRN;
    std::cout << "Testing satellite PRN=" << test_satellite_PRN << std::endl;
    std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN));
    true_obs_file.append(".dat");
    ASSERT_EQ(true_obs_data.open_obs_file(true_obs_file), true) << "Failure opening true observables file";

    top_block = gr::make_top_block("Tracking test");

    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking_1C", implementation, 1, 1);
    std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);  //std::make_shared<GpsL1CaDllPllCAidTracking>(config.get(), "Tracking_1C", 1, 1);

    boost::shared_ptr<GpsL1CAKfTrackingTest_msg_rx> msg_rx = GpsL1CAKfTrackingTest_msg_rx_make();

    // load acquisition data based on the first epoch of the true observations
    ASSERT_EQ(true_obs_data.read_binary_obs(), true)
        << "Failure reading true tracking dump file." << std::endl
        << "Maybe sat PRN #" + std::to_string(FLAGS_test_satellite_PRN) +
               " is not available?";

    // restart the epoch counter
    true_obs_data.restart();

    std::cout << "Initial Doppler [Hz]=" << true_obs_data.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << std::endl;
    gnss_synchro.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
    gnss_synchro.Acq_doppler_hz = true_obs_data.doppler_l1_hz;
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
        std::string file = "./" + filename_raw_data;
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of tracking test.";

    tracking->start_tracking();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
    }) << "Failure running the top_block.";

    // check results
    // load the true values
    long int nepoch = true_obs_data.num_epochs();
    std::cout << "True observation epochs=" << nepoch << std::endl;

    arma::vec true_timestamp_s = arma::zeros(nepoch, 1);
    arma::vec true_acc_carrier_phase_cycles = arma::zeros(nepoch, 1);
    arma::vec true_Doppler_Hz = arma::zeros(nepoch, 1);
    arma::vec true_prn_delay_chips = arma::zeros(nepoch, 1);
    arma::vec true_tow_s = arma::zeros(nepoch, 1);

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

    //load the measured values
    tracking_dump_reader trk_dump;

    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
        << "Failure opening tracking dump file";

    nepoch = trk_dump.num_epochs();
    std::cout << "Measured observation epochs=" << nepoch << std::endl;

    arma::vec trk_timestamp_s = arma::zeros(nepoch, 1);
    arma::vec trk_acc_carrier_phase_cycles = arma::zeros(nepoch, 1);
    arma::vec trk_Doppler_Hz = arma::zeros(nepoch, 1);
    arma::vec trk_prn_delay_chips = arma::zeros(nepoch, 1);

    std::vector<double> prompt;
    std::vector<double> early;
    std::vector<double> late;
    std::vector<double> promptI;
    std::vector<double> promptQ;

    epoch_counter = 0;
    while (trk_dump.read_binary_obs())
        {
            trk_timestamp_s(epoch_counter) = static_cast<double>(trk_dump.PRN_start_sample_count) / static_cast<double>(baseband_sampling_freq);
            trk_acc_carrier_phase_cycles(epoch_counter) = trk_dump.acc_carrier_phase_rad / GPS_TWO_PI;
            trk_Doppler_Hz(epoch_counter) = trk_dump.carrier_doppler_hz;

            double delay_chips = GPS_L1_CA_CODE_LENGTH_CHIPS - GPS_L1_CA_CODE_LENGTH_CHIPS * (fmod((static_cast<double>(trk_dump.PRN_start_sample_count) + trk_dump.aux1) / static_cast<double>(baseband_sampling_freq), 1.0e-3) / 1.0e-3);

            trk_prn_delay_chips(epoch_counter) = delay_chips;
            epoch_counter++;
            prompt.push_back(trk_dump.abs_P);
            early.push_back(trk_dump.abs_E);
            late.push_back(trk_dump.abs_L);
            promptI.push_back(trk_dump.prompt_I);
            promptQ.push_back(trk_dump.prompt_Q);
        }

    // Align initial measurements and cut the tracking pull-in transitory
    double pull_in_offset_s = 1.0;
    arma::uvec initial_meas_point = arma::find(trk_timestamp_s >= (true_timestamp_s(0) + pull_in_offset_s), 1, "first");

    trk_timestamp_s = trk_timestamp_s.subvec(initial_meas_point(0), trk_timestamp_s.size() - 1);
    trk_acc_carrier_phase_cycles = trk_acc_carrier_phase_cycles.subvec(initial_meas_point(0), trk_acc_carrier_phase_cycles.size() - 1);
    trk_Doppler_Hz = trk_Doppler_Hz.subvec(initial_meas_point(0), trk_Doppler_Hz.size() - 1);
    trk_prn_delay_chips = trk_prn_delay_chips.subvec(initial_meas_point(0), trk_prn_delay_chips.size() - 1);

    check_results_doppler(true_timestamp_s, true_Doppler_Hz, trk_timestamp_s, trk_Doppler_Hz);
    check_results_codephase(true_timestamp_s, true_prn_delay_chips, trk_timestamp_s, trk_prn_delay_chips);
    check_results_acc_carrier_phase(true_timestamp_s, true_acc_carrier_phase_cycles, trk_timestamp_s, trk_acc_carrier_phase_cycles);

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Signal tracking completed in " << elapsed_seconds.count() << " seconds." << std::endl;

    if (FLAGS_plot_gps_l1_kf_tracking_test == true)
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

                            std::vector<double> timevec;
                            double t = 0.0;
                            for (auto it = prompt.begin(); it != prompt.end(); it++)
                                {
                                    timevec.push_back(t);
                                    t = t + GPS_L1_CA_CODE_PERIOD;
                                }
                            Gnuplot g1("linespoints");
                            g1.set_title("GPS L1 C/A signal tracking correlators' output (satellite PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                            g1.set_grid();
                            g1.set_xlabel("Time [s]");
                            g1.set_ylabel("Correlators' output");
                            g1.cmd("set key box opaque");
                            unsigned int decimate = static_cast<unsigned int>(FLAGS_plot_decimate);
                            g1.plot_xy(timevec, prompt, "Prompt", decimate);
                            g1.plot_xy(timevec, early, "Early", decimate);
                            g1.plot_xy(timevec, late, "Late", decimate);
                            g1.savetops("Correlators_outputs");
                            g1.savetopdf("Correlators_outputs", 18);
                            g1.showonscreen();  // window output

                            Gnuplot g2("points");
                            g2.set_title("Constellation diagram (satellite PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                            g2.set_grid();
                            g2.set_xlabel("Inphase");
                            g2.set_ylabel("Quadrature");
                            g2.cmd("set size ratio -1");
                            g2.plot_xy(promptI, promptQ);
                            g2.savetops("Constellation");
                            g2.savetopdf("Constellation", 18);
                            g2.showonscreen();  // window output
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << std::endl;
                        }
                }
        }
}

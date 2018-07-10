/*!
 * \file gps_l1_acq_performance_test.cc
 * \brief This class implements an acquisition performance test
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
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

#include "test_flags.h"
#include "signal_generator_flags.h"
#include "tracking_true_obs_reader.h"
#include "true_observables_reader.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "display.h"
#include "gnuplot_i.h"
#include <boost/filesystem.hpp>
#include <gnuradio/top_block.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DEFINE_string(config_file_ptest, std::string(""), "File containing alternative configuration parameters for the acquisition performance test.");
DEFINE_string(acq_test_input_file, std::string(""), "File containing raw signal data, must be in int8_t format. The signal generator will not be used.");
DEFINE_string(acq_test_implementation, std::string("GPS_L1_CA_PCPS_Acquisition"), "Acquisition block implementation under test. Alternative: GPS_L1_CA_PCPS_Acquisition_Fine_Doppler");

DEFINE_int32(acq_test_doppler_max, 5000, "Maximum Doppler, in Hz");
DEFINE_int32(acq_test_doppler_step, 125, "Doppler step, in Hz.");
DEFINE_int32(acq_test_coherent_time_ms, 1, "Acquisition coherent time, in ms");
DEFINE_int32(acq_test_max_dwells, 1, "Number of non-coherent integrations");
DEFINE_bool(acq_test_use_CFAR_algorithm, true, "Use CFAR algorithm");
DEFINE_bool(acq_test_bit_transition_flag, false, "Bit transition flag");

DEFINE_int32(acq_test_signal_duration_s, 2, "Generated signal duration, in s");
DEFINE_int32(acq_test_num_meas, 0, "Number of measurements per run. 0 means the complete file.");
DEFINE_double(acq_test_cn0_init, 33.0, "Initial CN0, in dBHz.");
DEFINE_double(acq_test_cn0_final, 45.0, "Final CN0, in dBHz.");
DEFINE_double(acq_test_cn0_step, 3.0, "CN0 step, in dB.");

DEFINE_double(acq_test_threshold_init, 11.0, "Initial acquisition threshold");
DEFINE_double(acq_test_threshold_final, 16.0, "Final acquisition threshold");
DEFINE_double(acq_test_threshold_step, 1.0, "Acquisition threshold step");

DEFINE_double(acq_test_pfa_init, 1e-5, "Set initial threshold via probability of false alarm. Disable with -1.0");

DEFINE_int32(acq_test_PRN, 1, "PRN number of a present satellite");
DEFINE_int32(acq_test_fake_PRN, 33, "PRN number of a non-present satellite");

DEFINE_int32(acq_test_iterations, 1, "Number of iterations (same signal, different noise realization)");
DEFINE_bool(plot_acq_test, false, "Plots results with gnuplot, if available");

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class AcqPerfTest_msg_rx;

typedef boost::shared_ptr<AcqPerfTest_msg_rx> AcqPerfTest_msg_rx_sptr;

AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(concurrent_queue<int>& queue);

class AcqPerfTest_msg_rx : public gr::block
{
private:
    friend AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(concurrent_queue<int>& queue);
    void msg_handler_events(pmt::pmt_t msg);
    AcqPerfTest_msg_rx(concurrent_queue<int>& queue);
    concurrent_queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~AcqPerfTest_msg_rx();
};


AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(concurrent_queue<int>& queue)
{
    return AcqPerfTest_msg_rx_sptr(new AcqPerfTest_msg_rx(queue));
}


void AcqPerfTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


AcqPerfTest_msg_rx::AcqPerfTest_msg_rx(concurrent_queue<int>& queue) : gr::block("AcqPerfTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&AcqPerfTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


AcqPerfTest_msg_rx::~AcqPerfTest_msg_rx()
{
}

// -----------------------------------------


class AcquisitionPerformanceTest : public ::testing::Test
{
protected:
    AcquisitionPerformanceTest()
    {
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
        doppler_max = static_cast<unsigned int>(FLAGS_acq_test_doppler_max);
        doppler_step = static_cast<unsigned int>(FLAGS_acq_test_doppler_step);
        stop = false;
        if (FLAGS_acq_test_input_file.empty())
            {
                cn0_vector.push_back(FLAGS_acq_test_cn0_init);
                double aux = FLAGS_acq_test_cn0_init + FLAGS_acq_test_cn0_step;
                while (aux <= FLAGS_acq_test_cn0_final)
                    {
                        cn0_vector.push_back(aux);
                        aux = aux + FLAGS_acq_test_cn0_step;
                    }
            }
        else
            {
                cn0_vector = {0.0};
            }
        init();

        if (FLAGS_acq_test_pfa_init > 0.0)
            {
                pfa_vector.push_back(FLAGS_acq_test_pfa_init);
                float aux = 1.0;
                while ((FLAGS_acq_test_pfa_init * std::pow(10, aux)) < 1)
                    {
                        pfa_vector.push_back(FLAGS_acq_test_pfa_init * std::pow(10, aux));
                        aux = aux + 1.0;
                    }
                pfa_vector.push_back(1.0);
            }
        else
            {
                float aux = static_cast<float>(FLAGS_acq_test_threshold_init);
                pfa_vector.push_back(aux);
                aux = aux + static_cast<float>(FLAGS_acq_test_threshold_step);
                while (aux <= static_cast<float>(FLAGS_acq_test_threshold_final))
                    {
                        pfa_vector.push_back(aux);
                        aux = aux + static_cast<float>(FLAGS_acq_test_threshold_step);
                    }
            }

        num_thresholds = pfa_vector.size();

        int aux2 = ((generated_signal_duration_s * 1000 - (FLAGS_acq_test_coherent_time_ms * FLAGS_acq_test_max_dwells)) / (FLAGS_acq_test_coherent_time_ms * FLAGS_acq_test_max_dwells));
        if ((FLAGS_acq_test_num_meas > 0) and (FLAGS_acq_test_num_meas < aux2))
            {
                num_of_measurements = static_cast<unsigned int>(FLAGS_acq_test_num_meas);
            }
        else
            {
                num_of_measurements = static_cast<unsigned int>(aux2);
            }

        Pd.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++) Pd[i].reserve(num_thresholds);
        Pfa.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++) Pfa[i].reserve(num_thresholds);
        Pd_correct.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++) Pd_correct[i].reserve(num_thresholds);
    }

    ~AcquisitionPerformanceTest()
    {
    }

    std::vector<double> cn0_vector;
    std::vector<float> pfa_vector;

    int N_iterations = FLAGS_acq_test_iterations;
    void init();

    int configure_generator(double cn0);
    int generate_signal();
    int configure_receiver(double cn0, float pfa, unsigned int iter);
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();
    int run_receiver();
    int count_executions(const std::string& basename, unsigned int sat);
    void check_results();
    void plot_results();

    concurrent_queue<int> channel_internal_queue;

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<AcquisitionInterface> acquisition;
    std::shared_ptr<InMemoryConfiguration> config;
    std::shared_ptr<FileConfiguration> config_f;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    unsigned int doppler_max;
    unsigned int doppler_step;
    bool stop;

    int message;
    boost::thread ch_thread;

    std::string implementation = FLAGS_acq_test_implementation;

    const double baseband_sampling_freq = static_cast<double>(FLAGS_fs_gen_sps);
    const int coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
    const int in_acquisition = 1;
    const int dump_channel = 0;

    int generated_signal_duration_s = FLAGS_acq_test_signal_duration_s;
    unsigned int num_of_measurements;
    unsigned int measurement_counter = 0;

    unsigned int observed_satellite = FLAGS_acq_test_PRN;
    std::string path_str = "./acq-perf-test";

    int num_thresholds;

    std::vector<std::vector<float>> Pd;
    std::vector<std::vector<float>> Pfa;
    std::vector<std::vector<float>> Pd_correct;

private:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    double compute_stdev_precision(const std::vector<double>& vec);
    double compute_stdev_accuracy(const std::vector<double>& vec, double ref);
};


void AcquisitionPerformanceTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = observed_satellite;
    message = 0;
    measurement_counter = 0;
}


void AcquisitionPerformanceTest::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&AcquisitionPerformanceTest::wait_message, this);
}


void AcquisitionPerformanceTest::wait_message()
{
    while (!stop)
        {
            channel_internal_queue.wait_and_pop(message);
            process_message();
        }
}


void AcquisitionPerformanceTest::process_message()
{
    measurement_counter++;
    acquisition->reset();
    acquisition->set_state(1);
    std::cout << "Progress: " << round(static_cast<float>(measurement_counter) / static_cast<float>(num_of_measurements) * 100.0) << "% \r" << std::flush;
    if (measurement_counter == num_of_measurements)
        {
            stop_queue();
            top_block->stop();
        }
}


void AcquisitionPerformanceTest::stop_queue()
{
    stop = true;
}


int AcquisitionPerformanceTest::configure_generator(double cn0)
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(std::min(generated_signal_duration_s * 10, 3000));
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  // Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(cn0);
    return 0;
}


int AcquisitionPerformanceTest::generate_signal()
{
    pid_t wait_result;
    int child_status;
    std::cout << "Generating signal for " << p6 << "..." << std::endl;
    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], NULL};

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


int AcquisitionPerformanceTest::configure_receiver(double cn0, float pfa, unsigned int iter)
{
    if (FLAGS_config_file_ptest.empty())
        {
            config = std::make_shared<InMemoryConfiguration>();
            const int sampling_rate_internal = baseband_sampling_freq;

            config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(sampling_rate_internal));

            // Set Acquisition
            config->set_property("Acquisition_1C.implementation", implementation);
            config->set_property("Acquisition_1C.item_type", "gr_complex");
            config->set_property("Acquisition_1C.doppler_max", std::to_string(doppler_max));
            config->set_property("Acquisition_1C.doppler_min", std::to_string(-doppler_max));
            config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));

            config->set_property("Acquisition_1C.threshold", std::to_string(pfa));
            //if (FLAGS_acq_test_pfa_init > 0.0) config->supersede_property("Acquisition_1C.pfa", std::to_string(pfa));
            if (FLAGS_acq_test_pfa_init > 0.0)
                {
                    config->supersede_property("Acquisition_1C.pfa", std::to_string(pfa));
                }
            if (FLAGS_acq_test_use_CFAR_algorithm)
                {
                    config->set_property("Acquisition_1C.use_CFAR_algorithm", "true");
                }
            else
                {
                    config->set_property("Acquisition_1C.use_CFAR_algorithm", "false");
                }

            config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
            if (FLAGS_acq_test_bit_transition_flag)
                {
                    config->set_property("Acquisition_1C.bit_transition_flag", "true");
                }
            else
                {
                    config->set_property("Acquisition_1C.bit_transition_flag", "false");
                }

            config->set_property("Acquisition_1C.max_dwells", std::to_string(FLAGS_acq_test_max_dwells));

            config->set_property("Acquisition_1C.repeat_satellite", "true");

            config->set_property("Acquisition_1C.blocking", "true");
            config->set_property("Acquisition_1C.make_two_steps", "false");
            config->set_property("Acquisition_1C.second_nbins", std::to_string(4));
            config->set_property("Acquisition_1C.second_doppler_step", std::to_string(125));

            config->set_property("Acquisition_1C.dump", "true");
            std::string dump_file = path_str + std::string("/acquisition_") + std::to_string(cn0) + "_" + std::to_string(iter) + "_" + std::to_string(pfa);
            config->set_property("Acquisition_1C.dump_filename", dump_file);
            config->set_property("Acquisition_1C.dump_channel", std::to_string(dump_channel));
            config->set_property("Acquisition_1C.blocking_on_standby", "true");

            config_f = 0;
        }
    else
        {
            config_f = std::make_shared<FileConfiguration>(FLAGS_config_file_ptest);
            config = 0;
        }
    return 0;
}


int AcquisitionPerformanceTest::run_receiver()
{
    std::string file;
    if (FLAGS_acq_test_input_file.empty())
        {
            file = "./" + filename_raw_data;
        }
    else
        {
            file = FLAGS_acq_test_input_file;
        }
    const char* file_name = file.c_str();
    gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);

    gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();

    top_block = gr::make_top_block("Acquisition test");
    boost::shared_ptr<AcqPerfTest_msg_rx> msg_rx = AcqPerfTest_msg_rx_make(channel_internal_queue);

    queue = gr::msg_queue::make(0);
    gnss_synchro = Gnss_Synchro();
    init();

    int nsamples = floor(config->property("GNSS-SDR.internal_fs_sps", 2000000) * generated_signal_duration_s);
    boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
    if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
        }
    else
        {
            if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fine_Doppler") == 0)
                {
                    acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFineDoppler>(config.get(), "Acquisition_1C", 1, 0);
                }
            else
                {
                    bool aux = false;
                    EXPECT_EQ(true, aux);
                }
        }

    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->set_channel(0);
    acquisition->set_doppler_max(config->property("Acquisition_1C.doppler_max", 10000));
    acquisition->set_doppler_step(config->property("Acquisition_1C.doppler_step", 500));
    acquisition->set_threshold(config->property("Acquisition_1C.threshold", 0.0));
    acquisition->init();
    acquisition->set_local_code();

    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->connect(top_block);

    acquisition->reset();
    top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
    top_block->connect(gr_interleaved_char_to_complex, 0, valve, 0);
    top_block->connect(valve, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    start_queue();

    top_block->run();  // Start threads and wait

#ifdef OLD_BOOST
    ch_thread.timed_join(boost::posix_time::seconds(1));
#endif
#ifndef OLD_BOOST
    ch_thread.try_join_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(50));
#endif

    return 0;
}


int AcquisitionPerformanceTest::count_executions(const std::string& basename, unsigned int sat)
{
    FILE* fp;
    std::string argum2 = std::string("/usr/bin/find ") + path_str + std::string(" -maxdepth 1 -name ") + basename.substr(path_str.length() + 1, basename.length() - path_str.length()) + std::string("* | grep sat_") + std::to_string(sat) + std::string(" | wc -l");
    char buffer[1024];
    fp = popen(&argum2[0], "r");
    int num_executions = 1;
    if (fp == NULL)
        {
            std::cout << "Failed to run command: " << argum2 << std::endl;
            return 0;
        }
    while (fgets(buffer, sizeof(buffer), fp) != NULL)
        {
            std::string aux = std::string(buffer);
            EXPECT_EQ(aux.empty(), false);
            num_executions = std::stoi(aux);
        }
    pclose(fp);
    return num_executions;
}


void AcquisitionPerformanceTest::plot_results()
{
    if (FLAGS_plot_acq_test == true)
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

                            Gnuplot g1("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g1.showonscreen();  // window output
                                }
                            else
                                {
                                    g1.disablescreen();
                                }
                            g1.cmd("set font \"Times,18\"");
                            g1.set_title("Receiver Operating Characteristic for GPS L1 C/A acquisition");
                            g1.cmd("set label 1 \"" + std::string("Coherent integration time: ") + std::to_string(config->property("Acquisition_1C.coherent_integration_time_ms", 1)) + " ms, Non-coherent integrations: " + std::to_string(config->property("Acquisition_1C.max_dwells", 1)) + " \" at screen 0.12, 0.83 font \"Times,16\"");
                            g1.cmd("set logscale x");
                            g1.cmd("set yrange [0:1]");
                            g1.cmd("set xrange[0.0001:1]");
                            g1.cmd("set grid mxtics");
                            g1.cmd("set grid ytics");
                            g1.set_xlabel("Pfa");
                            g1.set_ylabel("Pd");
                            g1.set_grid();
                            g1.cmd("show grid");
                            for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++)
                                {
                                    std::vector<float> Pd_i;
                                    std::vector<float> Pfa_i;
                                    for (int k = 0; k < num_thresholds; k++)
                                        {
                                            Pd_i.push_back(Pd[i][k]);
                                            Pfa_i.push_back(Pfa[i][k]);
                                        }
                                    g1.plot_xy(Pfa_i, Pd_i, "CN0 = " + std::to_string(static_cast<int>(cn0_vector[i])) + " dBHz");
                                }
                            g1.set_legend();
                            g1.savetops("ROC");
                            g1.savetopdf("ROC", 18);

                            Gnuplot g2("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g2.showonscreen();  // window output
                                }
                            else
                                {
                                    g2.disablescreen();
                                }
                            g2.cmd("set font \"Times,18\"");
                            g2.set_title("Receiver Operating Characteristic for GPS L1 C/A valid acquisition");
                            g2.cmd("set label 1 \"" + std::string("Coherent integration time: ") + std::to_string(config->property("Acquisition_1C.coherent_integration_time_ms", 1)) + " ms, Non-coherent integrations: " + std::to_string(config->property("Acquisition_1C.max_dwells", 1)) + " \" at screen  0.12, 0.83 font \"Times,16\"");
                            g2.cmd("set logscale x");
                            g2.cmd("set yrange [0:1]");
                            g2.cmd("set xrange[0.0001:1]");
                            g2.cmd("set grid mxtics");
                            g2.cmd("set grid ytics");
                            g2.set_xlabel("Pfa");
                            g2.set_ylabel("Valid Pd");
                            g2.set_grid();
                            g2.cmd("show grid");
                            for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++)
                                {
                                    std::vector<float> Pd_i_correct;
                                    std::vector<float> Pfa_i;
                                    for (int k = 0; k < num_thresholds; k++)
                                        {
                                            Pd_i_correct.push_back(Pd_correct[i][k]);
                                            Pfa_i.push_back(Pfa[i][k]);
                                        }
                                    g2.plot_xy(Pfa_i, Pd_i_correct, "CN0 = " + std::to_string(static_cast<int>(cn0_vector[i])) + " dBHz");
                                }
                            g2.set_legend();
                            g2.savetops("ROC-valid-detection");
                            g2.savetopdf("ROC-valid-detection", 18);
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << std::endl;
                        }
                }
        }
}


TEST_F(AcquisitionPerformanceTest, ROC)
{
    tracking_true_obs_reader true_trk_data;

    if (boost::filesystem::exists(path_str))
        {
            boost::filesystem::remove_all(path_str);
        }
    boost::system::error_code ec;
    ASSERT_TRUE(boost::filesystem::create_directory(path_str, ec)) << "Could not create the " << path_str << " folder.";

    unsigned int cn0_index = 0;
    for (std::vector<double>::const_iterator it = cn0_vector.cbegin(); it != cn0_vector.cend(); ++it)
        {
            std::vector<double> meas_Pd_;
            std::vector<double> meas_Pd_correct_;
            std::vector<double> meas_Pfa_;

            if (FLAGS_acq_test_input_file.empty()) std::cout << "Execution for CN0 = " << *it << " dB-Hz" << std::endl;

            // Do N_iterations of the experiment
            for (int pfa_iter = 0; pfa_iter < static_cast<int>(pfa_vector.size()); pfa_iter++)
                {
                    if (FLAGS_acq_test_pfa_init > 0.0)
                        {
                            std::cout << "Setting threshold for Pfa = " << pfa_vector[pfa_iter] << std::endl;
                        }
                    else
                        {
                            std::cout << "Setting threshold to " << pfa_vector[pfa_iter] << std::endl;
                        }

                    // Configure the signal generator
                    if (FLAGS_acq_test_input_file.empty()) configure_generator(*it);

                    for (int iter = 0; iter < N_iterations; iter++)
                        {
                            // Generate signal raw signal samples and observations RINEX file
                            if (FLAGS_acq_test_input_file.empty()) generate_signal();

                            for (unsigned k = 0; k < 2; k++)
                                {
                                    if (k == 0)
                                        {
                                            observed_satellite = FLAGS_acq_test_PRN;
                                        }
                                    else
                                        {
                                            observed_satellite = FLAGS_acq_test_fake_PRN;
                                        }
                                    init();

                                    // Configure the receiver
                                    configure_receiver(*it, pfa_vector[pfa_iter], iter);

                                    // Run it
                                    run_receiver();

                                    // count executions
                                    std::string basename = path_str + std::string("/acquisition_") + std::to_string(*it) + "_" + std::to_string(iter) + "_" + std::to_string(pfa_vector[pfa_iter]) + "_" + gnss_synchro.System + "_1C";
                                    int num_executions = count_executions(basename, observed_satellite);

                                    // Read measured data
                                    int ch = config->property("Acquisition_1C.dump_channel", 0);
                                    arma::vec meas_timestamp_s = arma::zeros(num_executions, 1);
                                    arma::vec meas_doppler = arma::zeros(num_executions, 1);
                                    arma::vec positive_acq = arma::zeros(num_executions, 1);
                                    arma::vec meas_acq_delay_chips = arma::zeros(num_executions, 1);

                                    double coh_time_ms = config->property("Acquisition_1C.coherent_integration_time_ms", 1);

                                    std::cout << "Num executions: " << num_executions << std::endl;
                                    for (int execution = 1; execution <= num_executions; execution++)
                                        {
                                            acquisition_dump_reader acq_dump(basename, observed_satellite, config->property("Acquisition_1C.doppler_max", 0), config->property("Acquisition_1C.doppler_step", 0), config->property("GNSS-SDR.internal_fs_sps", 0) * GPS_L1_CA_CODE_PERIOD * static_cast<double>(coh_time_ms), ch, execution);
                                            acq_dump.read_binary_acq();
                                            if (acq_dump.positive_acq)
                                                {
                                                    //std::cout << "Meas acq_delay_samples: " << acq_dump.acq_delay_samples << " chips: " << acq_dump.acq_delay_samples / (baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD / GPS_L1_CA_CODE_LENGTH_CHIPS) << std::endl;
                                                    meas_timestamp_s(execution - 1) = acq_dump.sample_counter / baseband_sampling_freq;
                                                    meas_doppler(execution - 1) = acq_dump.acq_doppler_hz;
                                                    meas_acq_delay_chips(execution - 1) = acq_dump.acq_delay_samples / (baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD / GPS_L1_CA_CODE_LENGTH_CHIPS);
                                                    positive_acq(execution - 1) = acq_dump.positive_acq;
                                                }
                                            else
                                                {
                                                    //std::cout << "Failed acquisition." << std::endl;
                                                    meas_timestamp_s(execution - 1) = arma::datum::inf;
                                                    meas_doppler(execution - 1) = arma::datum::inf;
                                                    meas_acq_delay_chips(execution - 1) = arma::datum::inf;
                                                    positive_acq(execution - 1) = acq_dump.positive_acq;
                                                }
                                        }

                                    // Read reference data
                                    std::string true_trk_file = std::string("./gps_l1_ca_obs_prn");
                                    true_trk_file.append(std::to_string(observed_satellite));
                                    true_trk_file.append(".dat");
                                    true_trk_data.close_obs_file();
                                    true_trk_data.open_obs_file(true_trk_file);

                                    // load the true values
                                    long int n_true_epochs = true_trk_data.num_epochs();
                                    arma::vec true_timestamp_s = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_acc_carrier_phase_cycles = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_Doppler_Hz = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_prn_delay_chips = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_tow_s = arma::zeros(n_true_epochs, 1);

                                    long int epoch_counter = 0;
                                    int num_clean_executions = 0;
                                    while (true_trk_data.read_binary_obs())
                                        {
                                            true_timestamp_s(epoch_counter) = true_trk_data.signal_timestamp_s;
                                            true_acc_carrier_phase_cycles(epoch_counter) = true_trk_data.acc_carrier_phase_cycles;
                                            true_Doppler_Hz(epoch_counter) = true_trk_data.doppler_l1_hz;
                                            true_prn_delay_chips(epoch_counter) = GPS_L1_CA_CODE_LENGTH_CHIPS - true_trk_data.prn_delay_chips;
                                            true_tow_s(epoch_counter) = true_trk_data.tow;
                                            epoch_counter++;
                                            //std::cout << "True PRN_Delay chips = " << GPS_L1_CA_CODE_LENGTH_CHIPS - true_trk_data.prn_delay_chips << " at " << true_trk_data.signal_timestamp_s << std::endl;
                                        }

                                    // Process results
                                    arma::vec clean_doppler_estimation_error;
                                    arma::vec clean_delay_estimation_error;

                                    if (epoch_counter > 2)
                                        {
                                            arma::vec true_interpolated_doppler = arma::zeros(num_executions, 1);
                                            arma::vec true_interpolated_prn_delay_chips = arma::zeros(num_executions, 1);
                                            interp1(true_timestamp_s, true_Doppler_Hz, meas_timestamp_s, true_interpolated_doppler);
                                            interp1(true_timestamp_s, true_prn_delay_chips, meas_timestamp_s, true_interpolated_prn_delay_chips);

                                            arma::vec doppler_estimation_error = true_interpolated_doppler - meas_doppler;
                                            arma::vec delay_estimation_error = true_interpolated_prn_delay_chips - (meas_acq_delay_chips - ((1.0 / baseband_sampling_freq) / GPS_L1_CA_CHIP_PERIOD));  // compensate 1 sample delay

                                            // Cut measurements without reference
                                            for (int i = 0; i < num_executions; i++)
                                                {
                                                    if (!std::isnan(doppler_estimation_error(i)) and !std::isnan(delay_estimation_error(i)))
                                                        {
                                                            num_clean_executions++;
                                                        }
                                                }
                                            clean_doppler_estimation_error = arma::zeros(num_clean_executions, 1);
                                            clean_delay_estimation_error = arma::zeros(num_clean_executions, 1);
                                            num_clean_executions = 0;
                                            for (int i = 0; i < num_executions; i++)
                                                {
                                                    if (!std::isnan(doppler_estimation_error(i)) and !std::isnan(delay_estimation_error(i)))
                                                        {
                                                            clean_doppler_estimation_error(num_clean_executions) = doppler_estimation_error(i);
                                                            clean_delay_estimation_error(num_clean_executions) = delay_estimation_error(i);
                                                            num_clean_executions++;
                                                        }
                                                }

                                            /* std::cout << "Doppler estimation error [Hz]: ";
                                            for (int i = 0; i < num_executions - 1; i++)
                                                {
                                                    std::cout << doppler_estimation_error(i) << " ";
                                                }
                                            std::cout << std::endl;

                                            std::cout << "Delay estimation error [chips]: ";
                                            for (int i = 0; i < num_executions - 1; i++)
                                                {
                                                    std::cout << delay_estimation_error(i) << " ";

                                                }
                                            std::cout << std::endl; */
                                        }
                                    if (k == 0)
                                        {
                                            double detected = arma::accu(positive_acq);
                                            double computed_Pd = detected / static_cast<double>(num_executions);
                                            if (num_executions > 0)
                                                {
                                                    meas_Pd_.push_back(computed_Pd);
                                                }
                                            else
                                                {
                                                    meas_Pd_.push_back(0.0);
                                                }
                                            std::cout << TEXT_BOLD_BLACK << "Probability of detection for channel=" << ch << ", CN0=" << *it << " dBHz"
                                                      << ": " << (num_executions > 0 ? computed_Pd : 0.0) << TEXT_RESET << std::endl;
                                        }
                                    if (num_clean_executions > 0)
                                        {
                                            arma::vec correct_acq = arma::zeros(num_executions, 1);
                                            double correctly_detected = 0.0;
                                            for (int i = 0; i < num_clean_executions - 1; i++)

                                                {
                                                    if (abs(clean_delay_estimation_error(i)) < 0.5 and abs(clean_doppler_estimation_error(i)) < static_cast<float>(config->property("Acquisition_1C.doppler_step", 1)) / 2.0)
                                                        {
                                                            correctly_detected = correctly_detected + 1.0;
                                                        }
                                                }
                                            double computed_Pd_correct = correctly_detected / static_cast<double>(num_clean_executions);
                                            meas_Pd_correct_.push_back(computed_Pd_correct);
                                            std::cout << TEXT_BOLD_BLACK << "Probability of correct detection for channel=" << ch << ", CN0=" << *it << " dBHz"
                                                      << ": " << computed_Pd_correct << TEXT_RESET << std::endl;
                                        }
                                    else
                                        {
                                            //std::cout << "No reference data has been found. Maybe a non-present satellite?" << num_executions << std::endl;
                                            if (k == 1)
                                                {
                                                    double wrongly_detected = arma::accu(positive_acq);
                                                    double computed_Pfa = wrongly_detected / static_cast<double>(num_executions);
                                                    if (num_executions > 0)
                                                        {
                                                            meas_Pfa_.push_back(computed_Pfa);
                                                        }
                                                    else
                                                        {
                                                            meas_Pfa_.push_back(0.0);
                                                        }
                                                    std::cout << TEXT_BOLD_BLACK << "Probability of false alarm for channel=" << ch << ", CN0=" << *it << " dBHz"
                                                              << ": " << (num_executions > 0 ? computed_Pfa : 0.0) << TEXT_RESET << std::endl;
                                                }
                                        }
                                    true_trk_data.restart();
                                }
                        }
                    true_trk_data.close_obs_file();
                    float sum_pd = static_cast<float>(std::accumulate(meas_Pd_.begin(), meas_Pd_.end(), 0.0));
                    float sum_pd_correct = static_cast<float>(std::accumulate(meas_Pd_correct_.begin(), meas_Pd_correct_.end(), 0.0));
                    float sum_pfa = static_cast<float>(std::accumulate(meas_Pfa_.begin(), meas_Pfa_.end(), 0.0));
                    if (meas_Pd_.size() > 0 and meas_Pfa_.size() > 0)
                        {
                            Pd[cn0_index][pfa_iter] = sum_pd / static_cast<float>(meas_Pd_.size());
                            Pfa[cn0_index][pfa_iter] = sum_pfa / static_cast<float>(meas_Pfa_.size());
                        }
                    else
                        {
                            if (meas_Pd_.size() > 0)
                                {
                                    Pd[cn0_index][pfa_iter] = sum_pd / static_cast<float>(meas_Pd_.size());
                                }
                            else
                                {
                                    Pd[cn0_index][pfa_iter] = 0.0;
                                }
                            if (meas_Pfa_.size() > 0)
                                {
                                    Pfa[cn0_index][pfa_iter] = sum_pfa / static_cast<float>(meas_Pfa_.size());
                                }
                            else
                                {
                                    Pfa[cn0_index][pfa_iter] = 0.0;
                                }
                        }
                    if (meas_Pd_correct_.size() > 0)
                        {
                            Pd_correct[cn0_index][pfa_iter] = sum_pd_correct / static_cast<float>(meas_Pd_correct_.size());
                        }
                    else
                        {
                            Pd_correct[cn0_index][pfa_iter] = 0.0;
                        }
                    meas_Pd_.clear();
                    meas_Pfa_.clear();
                    meas_Pd_correct_.clear();
                }
            cn0_index++;
        }

    // Compute results
    unsigned int aux_index = 0;
    for (std::vector<double>::const_iterator it = cn0_vector.cbegin(); it != cn0_vector.cend(); ++it)
        {
            std::cout << "Results for CN0 = " << *it << " dBHz:" << std::endl;
            std::cout << "Pd = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pd[aux_index][pfa_iter] << " ";
                }
            std::cout << std::endl;
            std::cout << "Pd_correct = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pd_correct[aux_index][pfa_iter] << " ";
                }
            std::cout << std::endl;
            std::cout << "Pfa = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pfa[aux_index][pfa_iter] << " ";
                }
            std::cout << std::endl;

            aux_index++;
        }

    plot_results();
}

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
#include "display.h"
#include <boost/filesystem/operations.hpp>  // for create_directories, exists
//#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <gnuradio/top_block.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

DEFINE_string(config_file_ptest, std::string(""), "File containing the configuration parameters for the position test.");
DEFINE_double(acq_test_threshold, 0.001, "Acquisition threshold");
DEFINE_double(acq_test_pfa, -1.0, "Set threshold via probability of false alarm");
DEFINE_int32(acq_test_coherent_time_ms, 10, "Acquisition coherent time, in ms");
DEFINE_int32(acq_test_PRN, 1, "PRN number");
DEFINE_int32(acq_test_fake_PRN, 33, "Fake PRN number");
DEFINE_int32(acq_test_signal_duration_s, 2, "Generated signal duration");
DEFINE_bool(acq_test_bit_transition_flag, false, "Bit transition flag");
DEFINE_int32(acq_test_iterations, 1, "Number of iterations");

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
        doppler_max = 5000;
        doppler_step = 125;
        stop = false;
        acquisition = 0;
        init();
        Pd.resize(cn0_.size());
        for (int i = 0; i < static_cast<int>(cn0_.size()); i++) Pd[i].reserve(num_thresholds);
        Pfa.resize(cn0_.size());
        for (int i = 0; i < static_cast<int>(cn0_.size()); i++) Pfa[i].reserve(num_thresholds);
        Pd_correct.resize(cn0_.size());
        for (int i = 0; i < static_cast<int>(cn0_.size()); i++) Pd_correct[i].reserve(num_thresholds);
    }

    ~AcquisitionPerformanceTest()
    {
    }

    std::vector<double> cn0_ = {35.0, 38.0, 43.0};
    std::vector<float> pfa_local = {0.01, 0.1};  //{FLAGS_acq_test_pfa};  //{0.001, 0.01, 0.1, 1};
    int N_iterations = FLAGS_acq_test_iterations;
    void init();
    //void plot_grid();

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

    concurrent_queue<int> channel_internal_queue;

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GpsL1CaPcpsAcquisition> acquisition;
    std::shared_ptr<InMemoryConfiguration> config;
    std::shared_ptr<FileConfiguration> config_f;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    unsigned int doppler_max;
    unsigned int doppler_step;
    bool stop;

    int message;
    boost::thread ch_thread;

    std::string implementation = "GPS_L1_CA_PCPS_Acquisition";

    const double baseband_sampling_freq = static_cast<double>(FLAGS_fs_gen_sps);
    const int coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
    const int number_of_channels = 2;
    const int in_acquisition = 1;
    const float threshold = FLAGS_acq_test_threshold;
    const int max_dwells = 1;
    const int dump_channel = 0;

    int generated_signal_duration_s = FLAGS_acq_test_signal_duration_s;

    unsigned int num_of_realizations = (generated_signal_duration_s * 1000) / FLAGS_acq_test_coherent_time_ms;
    unsigned int realization_counter;

    unsigned int observed_satellite = FLAGS_acq_test_PRN;
    std::string path_str = "./acq-perf-test";

    int num_thresholds = pfa_local.size();

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


    //std::string generated_kml_file;
};


void AcquisitionPerformanceTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = observed_satellite;
    message = 0;
    realization_counter = 0;
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
            acquisition->reset();
            acquisition->set_state(1);

            channel_internal_queue.wait_and_pop(message);

            process_message();
        }
}


void AcquisitionPerformanceTest::process_message()
{
    realization_counter++;
    acquisition->reset();
    acquisition->set_state(1);

    if (realization_counter == num_of_realizations)
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
            config->set_property("Acquisition_1C.doppler_step", std::to_string(doppler_step));

            config->set_property("Acquisition_1C.threshold", std::to_string(threshold));
            if (FLAGS_acq_test_pfa > 0.0) config->supersede_property("Acquisition_1C.pfa", std::to_string(pfa));

            config->set_property("Acquisition_1C.use_CFAR_algorithm", "true");

            config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
            if (FLAGS_acq_test_bit_transition_flag)
                {
                    config->set_property("Acquisition_1C.bit_transition_flag", "true");
                }
            else
                {
                    config->set_property("Acquisition_1C.bit_transition_flag", "false");
                }

            config->set_property("Acquisition_1C.max_dwells", std::to_string(1));

            config->set_property("Acquisition_1C.repeat_satellite", "true");

            config->set_property("Acquisition_1C.blocking", "true");
            config->set_property("Acquisition_1C.make_two_steps", "false");
            config->set_property("Acquisition_1C.second_nbins", std::to_string(4));
            config->set_property("Acquisition_1C.second_doppler_step", std::to_string(125));

            config->set_property("Acquisition_1C.dump", "true");
            std::string dump_file = path_str + std::string("/acquisition_") + std::to_string(cn0) + "_" + std::to_string(iter) + "_" + std::to_string(pfa);
            config->set_property("Acquisition_1C.dump_filename", dump_file);
            config->set_property("Acquisition_1C.dump_channel", std::to_string(dump_channel));

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
    std::string file = "./" + filename_raw_data;
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

    acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition_1C", 1, 0);
    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->set_channel(0);
    acquisition->set_local_code();
    acquisition->set_doppler_max(config->property("Acquisition_1C.doppler_max", 10000));
    acquisition->set_doppler_step(config->property("Acquisition_1C.doppler_step", 500));
    acquisition->set_threshold(config->property("Acquisition_1C.threshold", 0.0));
    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->connect(top_block);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    acquisition->init();

    top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
    top_block->connect(gr_interleaved_char_to_complex, 0, valve, 0);
    top_block->connect(valve, 0, acquisition->get_left_block(), 0);

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
    std::string argum2 = std::string("/bin/ls ") + basename + "* | grep sat_" + std::to_string(sat) + " | wc -l";
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


TEST_F(AcquisitionPerformanceTest, PdvsCn0)
{
    tracking_true_obs_reader true_trk_data;

    if (boost::filesystem::exists(path_str))
        {
            boost::filesystem::remove_all(path_str);
        }
    boost::system::error_code ec;
    ASSERT_TRUE(boost::filesystem::create_directory(path_str, ec)) << "Could not create the " << path_str << " folder.";

    unsigned int cn0_index = 0;
    //for (unsigned iter = 0; iter < N_iterations; iter++)
    //unsigned iter = 0;
    //{
    for (std::vector<double>::const_iterator it = cn0_.cbegin(); it != cn0_.cend(); ++it)
        {
            // Do N_iterations of the experiment

            std::vector<double> meas_Pd_;
            std::vector<double> meas_Pd_correct_;
            std::vector<double> meas_Pfa_;

            // Set parameter to sweep
            std::cout << "Execution for CN0 = " << *it << " dB-Hz" << std::endl;
            for (int pfa_iter = 0; pfa_iter < static_cast<int>(pfa_local.size()); pfa_iter++)
                {
                    for (int iter = 0; iter < N_iterations; iter++)
                        {
                            std::string basename = path_str + std::string("/acquisition_") + std::to_string(*it) + "_" + std::to_string(iter) + "_" + std::to_string(pfa_local[pfa_iter]) + "_" + gnss_synchro.System + "_1C";
                            // Configure the signal generator
                            configure_generator(*it);

                            // Generate signal raw signal samples and observations RINEX file
                            generate_signal();

                            //std::cout << "Execution for CN0 = " << *it << " dB-Hz" << std::endl;
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
                                    configure_receiver(*it, pfa_local[pfa_iter], iter);

                                    // Run it
                                    run_receiver();

                                    // count executions
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

                                            std::cout << "Doppler estimation error [Hz]: ";
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
                                            std::cout << std::endl;
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
                                            // std::cout << "No reference data has been found. Maybe a non-present satellite?" << std::endl;
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
                }
            cn0_index++;
        }

    // Compute results
    unsigned int aux_index = 0;
    for (std::vector<double>::const_iterator it = cn0_.cbegin(); it != cn0_.cend(); ++it)
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
}

/*!
 * \file acq_performance_test.cc
 * \brief This class implements an acquisition performance test
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"
#include "acquisition_dump_reader.h"
#include "display.h"
#include "file_configuration.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "glonass_l1_ca_pcps_acquisition.h"
#include "glonass_l2_ca_pcps_acquisition.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_valve.h"
#include "gnuplot_i.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "gps_l5i_pcps_acquisition.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include "test_flags.h"
#include "tracking_true_obs_reader.h"
#include "true_observables_reader.h"
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>
#include <thread>
#include <utility>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

DEFINE_string(config_file_ptest, std::string(""), "File containing alternative configuration parameters for the acquisition performance test.");
DEFINE_string(acq_test_input_file, std::string(""), "File containing raw signal data, must be in int8_t format. The signal generator will not be used.");
DEFINE_string(acq_test_implementation, std::string("GPS_L1_CA_PCPS_Acquisition"), "Acquisition block implementation under test. Alternatives: GPS_L1_CA_PCPS_Acquisition, GPS_L1_CA_PCPS_Acquisition_Fine_Doppler, Galileo_E1_PCPS_Ambiguous_Acquisition, GLONASS_L1_CA_PCPS_Acquisition, GLONASS_L2_CA_PCPS_Acquisition, GPS_L2_M_PCPS_Acquisition, Galileo_E5a_Pcps_Acquisition, GPS_L5i_PCPS_Acquisition");

DEFINE_int32(acq_test_doppler_max, 5000, "Maximum Doppler, in Hz");
DEFINE_int32(acq_test_doppler_step, 125, "Doppler step, in Hz.");
DEFINE_int32(acq_test_coherent_time_ms, 1, "Acquisition coherent time, in ms");
DEFINE_int32(acq_test_max_dwells, 1, "Number of non-coherent integrations.");
DEFINE_bool(acq_test_bit_transition_flag, false, "Bit transition flag.");
DEFINE_bool(acq_test_make_two_steps, false, "Perform second step in a thinner grid.");
DEFINE_int32(acq_test_second_nbins, 4, "If --acq_test_make_two_steps is set to true, this parameter sets the number of bins done in the acquisition refinement stage.");
DEFINE_int32(acq_test_second_doppler_step, 10, "If --acq_test_make_two_steps is set to true, this parameter sets the Doppler step applied in the acquisition refinement stage, in Hz.");

DEFINE_int32(acq_test_signal_duration_s, 2, "Generated signal duration, in s");
DEFINE_int32(acq_test_num_meas, 0, "Number of measurements per run. 0 means the complete file.");
DEFINE_double(acq_test_cn0_init, 30.0, "Initial CN0, in dBHz.");
DEFINE_double(acq_test_cn0_final, 45.0, "Final CN0, in dBHz.");
DEFINE_double(acq_test_cn0_step, 3.0, "CN0 step, in dB.");

DEFINE_double(acq_test_threshold_init, 3.0, "Initial acquisition threshold");
DEFINE_double(acq_test_threshold_final, 4.0, "Final acquisition threshold");
DEFINE_double(acq_test_threshold_step, 0.5, "Acquisition threshold step");

DEFINE_double(acq_test_pfa_init, 1e-5, "Set initial threshold via probability of false alarm. Disable with -1.0");

DEFINE_int32(acq_test_PRN, 1, "PRN number of a present satellite");
DEFINE_int32(acq_test_fake_PRN, 33, "PRN number of a non-present satellite");

DEFINE_int32(acq_test_iterations, 1, "Number of iterations (same signal, different noise realization)");
DEFINE_bool(plot_acq_test, false, "Plots results with gnuplot, if available");
DEFINE_int32(acq_test_skiphead, 0, "Number of samples to skip in the input file");

// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class AcqPerfTest_msg_rx;

using AcqPerfTest_msg_rx_sptr = gnss_shared_ptr<AcqPerfTest_msg_rx>;

AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(Concurrent_Queue<int>& queue);

class AcqPerfTest_msg_rx : public gr::block
{
private:
    friend AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(Concurrent_Queue<int>& queue);
    void msg_handler_channel_events(const pmt::pmt_t msg);
    explicit AcqPerfTest_msg_rx(Concurrent_Queue<int>& queue);
    Concurrent_Queue<int>& channel_internal_queue;

public:
    int rx_message;
    ~AcqPerfTest_msg_rx();
};


AcqPerfTest_msg_rx_sptr AcqPerfTest_msg_rx_make(Concurrent_Queue<int>& queue)
{
    return AcqPerfTest_msg_rx_sptr(new AcqPerfTest_msg_rx(queue));
}


void AcqPerfTest_msg_rx::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
            channel_internal_queue.push(rx_message);
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


AcqPerfTest_msg_rx::AcqPerfTest_msg_rx(Concurrent_Queue<int>& queue) : gr::block("AcqPerfTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)), channel_internal_queue(queue)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&AcqPerfTest_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&AcqPerfTest_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


AcqPerfTest_msg_rx::~AcqPerfTest_msg_rx() = default;

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

        if (implementation == "GPS_L1_CA_PCPS_Acquisition")
            {
                signal_id = "1C";
                system_id = 'G';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
            }
        else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
            {
                signal_id = "1C";
                system_id = 'G';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
            }
        else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
            {
                signal_id = "1B";
                system_id = 'E';
                min_integration_ms = 4;
                if (FLAGS_acq_test_coherent_time_ms == 1)
                    {
                        coherent_integration_time_ms = 4;
                    }
                else
                    {
                        coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                    }
            }
        else if (implementation == "GLONASS_L1_CA_PCPS_Acquisition")
            {
                signal_id = "1G";
                system_id = 'R';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
            }
        else if (implementation == "GLONASS_L2_CA_PCPS_Acquisition")
            {
                signal_id = "2G";
                system_id = 'R';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
            }
        else if (implementation == "GPS_L2_M_PCPS_Acquisition")
            {
                signal_id = "2S";
                system_id = 'G';
                if (FLAGS_acq_test_coherent_time_ms == 1)
                    {
                        coherent_integration_time_ms = 20;
                    }
                else
                    {
                        coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                    }
                min_integration_ms = 20;
            }
        else if (implementation == "Galileo_E5a_Pcps_Acquisition")
            {
                signal_id = "5X";
                system_id = 'E';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
            }
        else if (implementation == "GPS_L5i_PCPS_Acquisition")
            {
                signal_id = "L5";
                system_id = 'G';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
            }
        else
            {
                signal_id = "1C";
                system_id = 'G';
                coherent_integration_time_ms = FLAGS_acq_test_coherent_time_ms;
                min_integration_ms = 1;
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
                pfa_vector.push_back(0.999);
            }
        else
            {
                auto aux = static_cast<float>(FLAGS_acq_test_threshold_init);
                pfa_vector.push_back(aux);
                aux = aux + static_cast<float>(FLAGS_acq_test_threshold_step);
                while (aux <= static_cast<float>(FLAGS_acq_test_threshold_final))
                    {
                        pfa_vector.push_back(aux);
                        aux = aux + static_cast<float>(FLAGS_acq_test_threshold_step);
                    }
            }

        num_thresholds = pfa_vector.size();

        // the gnss simulator does not dump the trk observables for the last 100 ms of generated signal
        int aux2;
        if (FLAGS_acq_test_bit_transition_flag)
            {
                aux2 = floor((generated_signal_duration_s * ms_per_s - 100) / (FLAGS_acq_test_coherent_time_ms * 2.0) - 1);
            }
        else
            {
                aux2 = floor((generated_signal_duration_s * ms_per_s - 100) / (FLAGS_acq_test_coherent_time_ms * FLAGS_acq_test_max_dwells) - 1);
            }
        if ((FLAGS_acq_test_num_meas > 0) && (FLAGS_acq_test_num_meas < aux2))
            {
                num_of_measurements = static_cast<unsigned int>(FLAGS_acq_test_num_meas);
            }
        else
            {
                num_of_measurements = static_cast<unsigned int>(aux2);
            }

        Pd.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++)
            {
                Pd[i].reserve(num_thresholds);
            }
        Pfa.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++)
            {
                Pfa[i].reserve(num_thresholds);
            }
        Pd_correct.resize(cn0_vector.size());
        for (int i = 0; i < static_cast<int>(cn0_vector.size()); i++)
            {
                Pd_correct[i].reserve(num_thresholds);
            }
    }

    ~AcquisitionPerformanceTest() = default;

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

    Concurrent_Queue<int> channel_internal_queue;

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
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
    std::thread ch_thread;

    std::string implementation = FLAGS_acq_test_implementation;

    const double baseband_sampling_freq = static_cast<double>(FLAGS_fs_gen_sps);
    int coherent_integration_time_ms;
    const int in_acquisition = 1;
    const int dump_channel = 0;

    int generated_signal_duration_s = FLAGS_acq_test_signal_duration_s;
    unsigned int num_of_measurements;
    unsigned int measurement_counter = 0;

    unsigned int observed_satellite = FLAGS_acq_test_PRN;
    std::string path_str = "./acq-perf-test";

    int num_thresholds;
    unsigned int min_integration_ms;

    std::vector<std::vector<float>> Pd;
    std::vector<std::vector<float>> Pfa;
    std::vector<std::vector<float>> Pd_correct;

    std::string signal_id;

private:
    static const uint32_t ms_per_s = 1000;

    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;
    char system_id;

    double compute_stdev_precision(const std::vector<double>& vec);
    double compute_stdev_accuracy(const std::vector<double>& vec, double ref);
};


void AcquisitionPerformanceTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = system_id;
    std::string signal = signal_id;
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = observed_satellite;
    message = 0;
    measurement_counter = 0;
}


void AcquisitionPerformanceTest::start_queue()
{
    stop = false;
    ch_thread = std::thread(&AcquisitionPerformanceTest::wait_message, this);
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
    std::cout << "Generating signal for " << p6 << "...\n";
    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], nullptr};

    int pid;
    if ((pid = fork()) == -1)
        {
            perror("fork error");
        }
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv error.\n";
            std::terminate();
        }

    wait_result = waitpid(pid, &child_status, 0);
    if (wait_result == -1)
        {
            perror("waitpid error");
        }
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
            config->set_property("Acquisition.implementation", implementation);
            config->set_property("Acquisition.item_type", "gr_complex");
            config->set_property("Acquisition.doppler_max", std::to_string(doppler_max));
            config->set_property("Acquisition.doppler_min", std::to_string(-doppler_max));
            config->set_property("Acquisition.doppler_step", std::to_string(doppler_step));

            config->set_property("Acquisition.threshold", std::to_string(pfa));
            // if (FLAGS_acq_test_pfa_init > 0.0) config->supersede_property("Acquisition.pfa", std::to_string(pfa));
            if (FLAGS_acq_test_pfa_init > 0.0)
                {
                    config->supersede_property("Acquisition.pfa", std::to_string(pfa));
                }

            config->set_property("Acquisition.coherent_integration_time_ms", std::to_string(coherent_integration_time_ms));
            if (FLAGS_acq_test_bit_transition_flag)
                {
                    config->set_property("Acquisition.bit_transition_flag", "true");
                }
            else
                {
                    config->set_property("Acquisition.bit_transition_flag", "false");
                }

            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_acq_test_max_dwells));

            config->set_property("Acquisition.repeat_satellite", "true");

            config->set_property("Acquisition.blocking", "true");
            if (FLAGS_acq_test_make_two_steps)
                {
                    config->set_property("Acquisition.make_two_steps", "true");
                    config->set_property("Acquisition.second_nbins", std::to_string(FLAGS_acq_test_second_nbins));
                    config->set_property("Acquisition.second_doppler_step", std::to_string(FLAGS_acq_test_second_doppler_step));
                }
            else
                {
                    config->set_property("Acquisition.make_two_steps", "false");
                }

            config->set_property("Acquisition.dump", "true");

            // std::string dump_file = path_str + std::string("/acquisition_") + std::to_string(cn0) + "_" + std::to_string(iter) + "_" + std::to_string(pfa);
            std::string dump_file = path_str + std::string("/acquisition_") + std::to_string(static_cast<int>(cn0)) + "_" + std::to_string(iter) + "_" + std::to_string(static_cast<int>(pfa * 1.0e5));
            config->set_property("Acquisition.dump_filename", dump_file);
            config->set_property("Acquisition.dump_channel", std::to_string(dump_channel));
            config->set_property("Acquisition.blocking_on_standby", "true");

            config_f = nullptr;
        }
    else
        {
            config_f = std::make_shared<FileConfiguration>(FLAGS_config_file_ptest);
            config = nullptr;
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
    auto msg_rx = AcqPerfTest_msg_rx_make(channel_internal_queue);
    gr::blocks::skiphead::sptr skiphead = gr::blocks::skiphead::make(sizeof(gr_complex), FLAGS_acq_test_skiphead);

    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    gnss_synchro = Gnss_Synchro();
    init();

    int nsamples = floor(config->property("GNSS-SDR.internal_fs_sps", 2000000) * generated_signal_duration_s);
    auto valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue.get());
    if (implementation == "GPS_L1_CA_PCPS_Acquisition")
        {
            acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFineDoppler>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
        {
            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GLONASS_L1_CA_PCPS_Acquisition")
        {
            acquisition = std::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GLONASS_L2_CA_PCPS_Acquisition")
        {
            acquisition = std::make_shared<GlonassL2CaPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition")
        {
            acquisition = std::make_shared<GpsL2MPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition")
        {
            acquisition = std::make_shared<GalileoE5aPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GPS_L5i_PCPS_Acquisition")
        {
            acquisition = std::make_shared<GpsL5iPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else
        {
            bool aux = false;
            EXPECT_EQ(true, aux);
        }

    acquisition->set_gnss_synchro(&gnss_synchro);
    acquisition->set_channel(0);
    acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
    acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
    acquisition->init();
    acquisition->set_local_code();

    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->connect(top_block);

    acquisition->reset();
    top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
    top_block->connect(gr_interleaved_char_to_complex, 0, skiphead, 0);
    top_block->connect(skiphead, 0, valve, 0);
    top_block->connect(valve, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    start_queue();

    top_block->run();  // Start threads and wait

    ch_thread.join();

    return 0;
}


int AcquisitionPerformanceTest::count_executions(const std::string& basename, unsigned int sat)
{
    FILE* fp;
    std::string argum2 = std::string("/usr/bin/find ") + path_str + std::string(" -maxdepth 1 -name ") + basename.substr(path_str.length() + 1, basename.length() - path_str.length()) + std::string("* | grep sat_") + std::to_string(sat) + std::string(" | wc -l");
    char buffer[1024];
    fp = popen(&argum2[0], "r");
    int num_executions = 1;
    if (fp == nullptr)
        {
            std::cout << "Failed to run command: " << argum2 << '\n';
            return 0;
        }
    while (fgets(buffer, sizeof(buffer), fp) != nullptr)
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
                    std::cout << "WARNING: Although the flag plot_gps_l1_tracking_test has been set to TRUE,\n";
                    std::cout << "gnuplot has not been found in your system.\n";
                    std::cout << "Test results will not be plotted.\n";
                }
            else
                {
                    try
                        {
                            fs::path p(gnuplot_executable);
                            fs::path dir = p.parent_path();
                            const std::string& gnuplot_path = dir.native();
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
                            g1.cmd("set label 1 \"" + std::string("Coherent integration time: ") + std::to_string(config->property("Acquisition.coherent_integration_time_ms", 1)) + " ms, Non-coherent integrations: " + std::to_string(config->property("Acquisition.max_dwells", 1)) + R"( " at screen 0.12, 0.83 font "Times,16")");
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
                                            Pfa_i.push_back(pfa_vector[k]);
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
                            g2.cmd("set label 1 \"" + std::string("Coherent integration time: ") + std::to_string(config->property("Acquisition.coherent_integration_time_ms", 1)) + " ms, Non-coherent integrations: " + std::to_string(config->property("Acquisition.max_dwells", 1)) + R"( " at screen  0.12, 0.83 font "Times,16")");
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
                                            Pfa_i.push_back(pfa_vector[k]);
                                        }
                                    g2.plot_xy(Pfa_i, Pd_i_correct, "CN0 = " + std::to_string(static_cast<int>(cn0_vector[i])) + " dBHz");
                                }
                            g2.set_legend();
                            g2.savetops("ROC-valid-detection");
                            g2.savetopdf("ROC-valid-detection", 18);
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << '\n';
                        }
                }
        }
}


TEST_F(AcquisitionPerformanceTest, ROC)
{
    Tracking_True_Obs_Reader true_trk_data;

    if (fs::exists(path_str))
        {
            std::cout << "Deleting old files at " << path_str << " ...\n";
            fs::remove_all(path_str);
        }
    errorlib::error_code ec;
    ASSERT_TRUE(fs::create_directory(path_str, ec)) << "Could not create the " << path_str << " folder.";

    unsigned int cn0_index = 0;
    for (double it : cn0_vector)
        {
            std::vector<double> meas_Pd_;
            std::vector<double> meas_Pd_correct_;
            std::vector<double> meas_Pfa_;

            if (FLAGS_acq_test_input_file.empty())
                {
                    std::cout << "Execution for CN0 = " << it << " dB-Hz\n";
                }

            // Do N_iterations of the experiment
            for (int pfa_iter = 0; pfa_iter < static_cast<int>(pfa_vector.size()); pfa_iter++)
                {
                    if (FLAGS_acq_test_pfa_init > 0.0)
                        {
                            std::cout << "Setting threshold for Pfa = " << pfa_vector[pfa_iter] << '\n';
                        }
                    else
                        {
                            std::cout << "Setting threshold to " << pfa_vector[pfa_iter] << '\n';
                        }

                    // Configure the signal generator
                    if (FLAGS_acq_test_input_file.empty())
                        {
                            configure_generator(it);
                        }

                    for (int iter = 0; iter < N_iterations; iter++)
                        {
                            // Generate signal raw signal samples and observations RINEX file
                            if (FLAGS_acq_test_input_file.empty())
                                {
                                    generate_signal();
                                }

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
                                    configure_receiver(it, pfa_vector[pfa_iter], iter);

                                    // Run it
                                    run_receiver();

                                    // count executions
                                    std::string basename = path_str + std::string("/acquisition_") + std::to_string(static_cast<int>(it)) + "_" + std::to_string(iter) + "_" + std::to_string(static_cast<int>(pfa_vector[pfa_iter] * 1.0e5)) + "_" + gnss_synchro.System + "_" + gnss_synchro.Signal;
                                    int num_executions = count_executions(basename, observed_satellite);

                                    // Read measured data
                                    int ch = config->property("Acquisition.dump_channel", 0);
                                    arma::vec meas_timestamp_s = arma::zeros(num_executions, 1);
                                    arma::vec meas_doppler = arma::zeros(num_executions, 1);
                                    arma::vec positive_acq = arma::zeros(num_executions, 1);
                                    arma::vec meas_acq_delay_chips = arma::zeros(num_executions, 1);

                                    double coh_time_ms = config->property("Acquisition.coherent_integration_time_ms", 1);

                                    std::cout << "Num executions: " << num_executions << '\n';

                                    unsigned int fft_size = 0;
                                    unsigned int d_consumed_samples = coh_time_ms * config->property("GNSS-SDR.internal_fs_sps", 0) * 0.001;  // * (config->property("Acquisition.bit_transition_flag", false) ? 2 : 1);
                                    if (coh_time_ms == min_integration_ms)
                                        {
                                            fft_size = d_consumed_samples;
                                        }
                                    else
                                        {
                                            fft_size = d_consumed_samples * 2;
                                        }

                                    for (int execution = 1; execution <= num_executions; execution++)
                                        {
                                            Acquisition_Dump_Reader acq_dump(basename,
                                                observed_satellite,
                                                config->property("Acquisition.doppler_max", 0),
                                                config->property("Acquisition.doppler_step", 0),
                                                fft_size,
                                                ch,
                                                execution);
                                            acq_dump.read_binary_acq();
                                            if (acq_dump.positive_acq)
                                                {
                                                    // std::cout << "Meas acq_delay_samples: " << acq_dump.acq_delay_samples << " chips: " << acq_dump.acq_delay_samples / (baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD_S / GPS_L1_CA_CODE_LENGTH_CHIPS) << '\n';
                                                    meas_timestamp_s(execution - 1) = acq_dump.sample_counter / baseband_sampling_freq;
                                                    meas_doppler(execution - 1) = acq_dump.acq_doppler_hz;
                                                    meas_acq_delay_chips(execution - 1) = acq_dump.acq_delay_samples / (baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD_S / GPS_L1_CA_CODE_LENGTH_CHIPS);
                                                    positive_acq(execution - 1) = acq_dump.positive_acq;
                                                }
                                            else
                                                {
                                                    // std::cout << "Failed acquisition.\n";
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
                                    int64_t n_true_epochs = true_trk_data.num_epochs();
                                    arma::vec true_timestamp_s = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_acc_carrier_phase_cycles = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_Doppler_Hz = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_prn_delay_chips = arma::zeros(n_true_epochs, 1);
                                    arma::vec true_tow_s = arma::zeros(n_true_epochs, 1);

                                    int64_t epoch_counter = 0;
                                    int num_clean_executions = 0;
                                    while (true_trk_data.read_binary_obs())
                                        {
                                            true_timestamp_s(epoch_counter) = true_trk_data.signal_timestamp_s;
                                            true_acc_carrier_phase_cycles(epoch_counter) = true_trk_data.acc_carrier_phase_cycles;
                                            true_Doppler_Hz(epoch_counter) = true_trk_data.doppler_l1_hz;
                                            true_prn_delay_chips(epoch_counter) = GPS_L1_CA_CODE_LENGTH_CHIPS - true_trk_data.prn_delay_chips;
                                            true_tow_s(epoch_counter) = true_trk_data.tow;
                                            epoch_counter++;
                                            // std::cout << "True PRN_Delay chips = " << GPS_L1_CA_CODE_LENGTH_CHIPS - true_trk_data.prn_delay_chips << " at " << true_trk_data.signal_timestamp_s << '\n';
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
                                            arma::vec delay_estimation_error = true_interpolated_prn_delay_chips - (meas_acq_delay_chips - ((1.0 / baseband_sampling_freq) / GPS_L1_CA_CHIP_PERIOD_S));  // compensate 1 sample delay

                                            // Cut measurements without reference
                                            for (int i = 0; i < num_executions; i++)
                                                {
                                                    if (!std::isnan(doppler_estimation_error(i)) && !std::isnan(delay_estimation_error(i)))
                                                        {
                                                            num_clean_executions++;
                                                        }
                                                }
                                            clean_doppler_estimation_error = arma::zeros(num_clean_executions, 1);
                                            clean_delay_estimation_error = arma::zeros(num_clean_executions, 1);
                                            num_clean_executions = 0;
                                            for (int i = 0; i < num_executions; i++)
                                                {
                                                    if (!std::isnan(doppler_estimation_error(i)) && !std::isnan(delay_estimation_error(i)))
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
                                            std::cout << '\n';

                                            std::cout << "Delay estimation error [chips]: ";
                                            for (int i = 0; i < num_executions - 1; i++)
                                                {
                                                    std::cout << delay_estimation_error(i) << " ";

                                                }
                                            std::cout << '\n'; */
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
                                            std::cout << TEXT_BOLD_BLUE << "Probability of detection for channel=" << ch << ", CN0=" << it << " dBHz"
                                                      << ": " << (num_executions > 0 ? computed_Pd : 0.0) << TEXT_RESET << '\n';
                                        }
                                    if (num_clean_executions > 0)
                                        {
                                            arma::vec correct_acq = arma::zeros(num_executions, 1);
                                            double correctly_detected = 0.0;
                                            for (int i = 0; i < num_clean_executions; i++)
                                                {
                                                    if (abs(clean_delay_estimation_error(i)) < 0.5 && abs(clean_doppler_estimation_error(i)) < static_cast<float>(config->property("Acquisition.doppler_step", 1)))
                                                        {
                                                            correctly_detected = correctly_detected + 1.0;
                                                        }
                                                }
                                            double computed_Pd_correct = correctly_detected / static_cast<double>(num_clean_executions);
                                            meas_Pd_correct_.push_back(computed_Pd_correct);
                                            std::cout << TEXT_BOLD_BLUE << "Probability of correct detection for channel=" << ch << ", CN0=" << it << " dBHz"
                                                      << ": " << computed_Pd_correct << TEXT_RESET << '\n';
                                        }
                                    else
                                        {
                                            // std::cout << "No reference data has been found. Maybe a non-present satellite?" << num_executions << '\n';
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
                                                    std::cout << TEXT_BOLD_BLUE << "Probability of false alarm for channel=" << ch << ", CN0=" << it << " dBHz"
                                                              << ": " << (num_executions > 0 ? computed_Pfa : 0.0) << TEXT_RESET << '\n';
                                                }
                                        }
                                    true_trk_data.restart();
                                }
                        }
                    true_trk_data.close_obs_file();
                    float sum_pd = static_cast<float>(std::accumulate(meas_Pd_.begin(), meas_Pd_.end(), 0.0));
                    float sum_pd_correct = static_cast<float>(std::accumulate(meas_Pd_correct_.begin(), meas_Pd_correct_.end(), 0.0));
                    float sum_pfa = static_cast<float>(std::accumulate(meas_Pfa_.begin(), meas_Pfa_.end(), 0.0));
                    if (!meas_Pd_.empty() && !meas_Pfa_.empty())
                        {
                            Pd[cn0_index][pfa_iter] = sum_pd / static_cast<float>(meas_Pd_.size());
                            Pfa[cn0_index][pfa_iter] = sum_pfa / static_cast<float>(meas_Pfa_.size());
                        }
                    else
                        {
                            if (!meas_Pd_.empty())
                                {
                                    Pd[cn0_index][pfa_iter] = sum_pd / static_cast<float>(meas_Pd_.size());
                                }
                            else
                                {
                                    Pd[cn0_index][pfa_iter] = 0.0;
                                }
                            if (!meas_Pfa_.empty())
                                {
                                    Pfa[cn0_index][pfa_iter] = sum_pfa / static_cast<float>(meas_Pfa_.size());
                                }
                            else
                                {
                                    Pfa[cn0_index][pfa_iter] = 0.0;
                                }
                        }
                    if (!meas_Pd_correct_.empty())
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
    for (double it : cn0_vector)
        {
            std::cout << "Results for CN0 = " << it << " dBHz:\n";
            std::cout << "Pd = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pd[aux_index][pfa_iter] << " ";
                }
            std::cout << '\n';
            std::cout << "Pd_correct = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pd_correct[aux_index][pfa_iter] << " ";
                }
            std::cout << '\n';
            std::cout << "Pfa = ";
            for (int pfa_iter = 0; pfa_iter < num_thresholds; pfa_iter++)
                {
                    std::cout << Pfa[aux_index][pfa_iter] << " ";
                }
            std::cout << '\n';

            aux_index++;
        }

    plot_results();
}

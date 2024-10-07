/*!
 * \file tracking_pull-in_test_fpga.cc
 * \brief  This class implements a tracking Pull-In test for FPGA HW accelerator
 *  implementations based on some input parameters.
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2018. jarribas(at)cttc.es
 *          </ul>
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "acquisition_msg_rx.h"
#include "concurrent_queue.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnuplot_i.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "gps_l5i_pcps_acquisition_fpga.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include "test_flags.h"
#include "tracking_dump_reader.h"
#include "tracking_interface.h"
#include "tracking_tests_flags.h"
#include "tracking_true_obs_reader.h"
#include <armadillo>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstdint>
#include <pthread.h>
#include <utility>
#include <vector>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#endif

#if PMT_USES_BOOST_ANY
namespace wht = boost;
#else
namespace wht = std;
#endif

// ######## GNURADIO TRACKING BLOCK MESSAGE RECEVER #########
class TrackingPullInTest_msg_rx_Fpga;

using TrackingPullInTest_msg_rx_Fpga_sptr = gnss_shared_ptr<TrackingPullInTest_msg_rx_Fpga>;

TrackingPullInTest_msg_rx_Fpga_sptr TrackingPullInTest_msg_rx_Fpga_make();

class TrackingPullInTest_msg_rx_Fpga : public gr::block
{
private:
    friend TrackingPullInTest_msg_rx_Fpga_sptr TrackingPullInTest_msg_rx_Fpga_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    TrackingPullInTest_msg_rx_Fpga();

public:
    int rx_message;
    ~TrackingPullInTest_msg_rx_Fpga();  //!< Default destructor
};


TrackingPullInTest_msg_rx_Fpga_sptr TrackingPullInTest_msg_rx_Fpga_make()
{
    return TrackingPullInTest_msg_rx_Fpga_sptr(new TrackingPullInTest_msg_rx_Fpga());
}


void TrackingPullInTest_msg_rx_Fpga::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;  // 3 -> loss of lock
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_tracking Bad cast!";
            rx_message = 0;
        }
}


TrackingPullInTest_msg_rx_Fpga::TrackingPullInTest_msg_rx_Fpga() : gr::block("TrackingPullInTest_msg_rx_Fpga", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&TrackingPullInTest_msg_rx_Fpga::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&TrackingPullInTest_msg_rx_Fpga::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


TrackingPullInTest_msg_rx_Fpga::~TrackingPullInTest_msg_rx_Fpga() = default;


struct DMA_handler_args_trk_pull_in_test
{
    std::string file;
    int32_t nsamples_tx;
    int32_t skip_used_samples;
    unsigned int freq_band;  // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
    float scaling_factor;
};

struct acquisition_handler_args_trk_pull_in_test
{
    std::shared_ptr<AcquisitionInterface> acquisition;
};


void* handler_acquisition_trk_pull_in_test(void* arguments)
{
    // the acquisition is a blocking function so we have to
    // create a thread
    auto* args = (struct acquisition_handler_args_trk_pull_in_test*)arguments;
    args->acquisition->reset();
    return nullptr;
}


void* handler_DMA_trk_pull_in_test(void* arguments)
{
    const int MAX_INPUT_SAMPLES_TOTAL = 16384;

    auto* args = (struct DMA_handler_args_trk_pull_in_test*)arguments;

    std::string Filename = args->file;  // input filename
    int32_t skip_used_samples = args->skip_used_samples;
    int32_t nsamples_tx = args->nsamples_tx;

    std::vector<int8_t> input_samples(MAX_INPUT_SAMPLES_TOTAL * 2);
    std::vector<int8_t> input_samples_dma(MAX_INPUT_SAMPLES_TOTAL * 2 * 2);
    bool file_completed = false;
    int32_t nsamples_remaining;
    int32_t nsamples_block_size;
    unsigned int dma_index;

    int tx_fd;  // DMA descriptor
    std::ifstream infile;

    infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
        {
            infile.open(Filename, std::ios::binary);
        }
    catch (const std::ifstream::failure& e)
        {
            std::cerr << "Exception opening file " << Filename << '\n';
            return nullptr;
        }

    // *************************************************************************
    // Open DMA device
    // *************************************************************************
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 0)
        {
            std::cout << "Cannot open loop device\n";
            return nullptr;
        }

        // *************************************************************************
        // Open input file
        // *************************************************************************

#if USE_GLOG_AND_GFLAGS
    uint32_t skip_samples = static_cast<uint32_t>(FLAGS_skip_samples);
#else
    uint32_t skip_samples = static_cast<uint32_t>(absl::GetFlag(FLAGS_skip_samples));
#endif

    if (skip_samples + skip_used_samples > 0)
        {
            try
                {
                    infile.ignore((skip_samples + skip_used_samples) * 2);
                }
            catch (const std::ifstream::failure& e)
                {
                    std::cerr << "Exception reading file " << Filename << '\n';
                }
        }

    nsamples_remaining = nsamples_tx;
    nsamples_block_size = 0;

    while (file_completed == false)
        {
            dma_index = 0;

            if (nsamples_remaining > MAX_INPUT_SAMPLES_TOTAL)
                {
                    nsamples_block_size = MAX_INPUT_SAMPLES_TOTAL;
                }
            else
                {
                    nsamples_block_size = nsamples_remaining;
                }

            try
                {
                    // 2 bytes per complex sample
                    infile.read(reinterpret_cast<char*>(input_samples.data()), nsamples_block_size * 2);
                }
            catch (const std::ifstream::failure& e)
                {
                    std::cerr << "Exception reading file " << Filename << '\n';
                }

            for (int index0 = 0; index0 < (nsamples_block_size * 2); index0 += 2)
                {
                    if (args->freq_band == 0)
                        {
                            // channel 1 (queue 1) -> E5/L5
                            input_samples_dma[dma_index] = 0;
                            input_samples_dma[dma_index + 1] = 0;
                            // channel 0 (queue 0) -> E1/L1
                            input_samples_dma[dma_index + 2] = static_cast<int8_t>(input_samples[index0] * args->scaling_factor);
                            input_samples_dma[dma_index + 3] = static_cast<int8_t>(input_samples[index0 + 1] * args->scaling_factor);
                        }
                    else
                        {
                            // channel 1 (queue 1) -> E5/L5
                            input_samples_dma[dma_index] = static_cast<int8_t>(input_samples[index0] * args->scaling_factor);
                            input_samples_dma[dma_index + 1] = static_cast<int8_t>(input_samples[index0 + 1] * args->scaling_factor);
                            // channel 0 (queue 0) -> E1/L1
                            input_samples_dma[dma_index + 2] = 0;
                            input_samples_dma[dma_index + 3] = 0;
                        }

                    dma_index += 4;
                }

            if (write(tx_fd, input_samples_dma.data(), nsamples_block_size * 2 * 2) != nsamples_block_size * 2 * 2)
                {
                    std::cerr << "Error: DMA could not send all the required samples \n";
                }

            // Throttle the DMA
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            nsamples_remaining -= nsamples_block_size;

            if (nsamples_remaining == 0)
                {
                    file_completed = true;
                }
        }

    try
        {
            infile.close();
        }
    catch (const std::ifstream::failure& e)
        {
            std::cerr << "Exception closing files " << Filename << '\n';
        }

    try
        {
            close(tx_fd);
        }
    catch (const std::ifstream::failure& e)
        {
            std::cerr << "Exception closing loop device \n";
        }

    return nullptr;
}


class TrackingPullInTestFpga : public ::testing::Test
{
public:
    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGLO_1G,
        evGLO_2G
    };

    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;
#if USE_GLOG_AND_GFLAGS
    std::string implementation = FLAGS_trk_test_implementation;
    const int baseband_sampling_freq = FLAGS_fs_gen_sps;
    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_signal_file;
#else
    std::string implementation = absl::GetFlag(FLAGS_trk_test_implementation);
    const int baseband_sampling_freq = absl::GetFlag(FLAGS_fs_gen_sps);
    std::string filename_rinex_obs = absl::GetFlag(FLAGS_filename_rinex_obs);
    std::string filename_raw_data = absl::GetFlag(FLAGS_signal_file);
#endif
    std::map<int, double> doppler_measurements_map;
    std::map<int, double> code_delay_measurements_map;
    std::map<int, uint64_t> acq_samplestamp_map;

    int configure_generator(double CN0_dBHz, int file_idx);
    int generate_signal();
    std::vector<double> check_results_doppler(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);
    std::vector<double> check_results_acc_carrier_phase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);
    std::vector<double> check_results_codephase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);

    TrackingPullInTestFpga()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~TrackingPullInTestFpga() = default;

    void configure_receiver(double PLL_wide_bw_hz,
        double DLL_wide_bw_hz,
        double PLL_narrow_bw_hz,
        double DLL_narrow_bw_hz,
        int extend_correlation_symbols);

    bool acquire_signal(int SV_ID);
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;

    static const int32_t TEST_TRK_PULL_IN_TEST_SKIP_SAMPLES = 1024;  // 48
    static constexpr float DMA_SIGNAL_SCALING_FACTOR = 8.0;
};


int TrackingPullInTestFpga::configure_generator(double CN0_dBHz, int file_idx)
{
#if USE_GLOG_AND_GFLAGS
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
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;                    // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_signal_file + std::to_string(file_idx);  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);       // Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(CN0_dBHz);                          // Signal generator CN0
#else
    // Configure signal generator
    generator_binary = absl::GetFlag(FLAGS_generator_binary);

    p1 = std::string("-rinex_nav_file=") + absl::GetFlag(FLAGS_rinex_nav_file);
    if (absl::GetFlag(FLAGS_dynamic_position).empty())
        {
            p2 = std::string("-static_position=") + absl::GetFlag(FLAGS_static_position) + std::string(",") + std::to_string(absl::GetFlag(FLAGS_duration) * 10);
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(absl::GetFlag(FLAGS_dynamic_position));
        }
    p3 = std::string("-rinex_obs_file=") + absl::GetFlag(FLAGS_filename_rinex_obs);                    // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + absl::GetFlag(FLAGS_signal_file) + std::to_string(file_idx);  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);                      // Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(CN0_dBHz);                                         // Signal generator CN0
#endif

    return 0;
}


int TrackingPullInTestFpga::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], nullptr};

    int pid;
    if ((pid = fork()) == -1)
        {
            perror("fork err");
        }
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv err.\n";
            std::terminate();
        }

    waitpid(pid, &child_status, 0);

    std::cout << "Signal and Observables RINEX and RAW files created.\n";
    return 0;
}


// When using the FPGA the acquisition class calls the states
// of the channel finite state machine directly. This is done
// in order to reduce the latency of the receiver when going
// from acquisition to tracking. In order to execute the
// acquisition in the unit tests we need to create a derived
// class of the channel finite state machine. Some of the states
// of the channel state machine are modified here, in order to
// simplify the instantiation of the acquisition class in the
// unit test.
class ChannelFsm_trk_pull_in_test : public ChannelFsm
{
public:
    bool Event_valid_acquisition() override
    {
        acquisition_successful = true;
        return true;
    }

    bool Event_failed_acquisition_repeat() override
    {
        acquisition_successful = false;
        return true;
    }

    bool Event_failed_acquisition_no_repeat() override
    {
        acquisition_successful = false;
        return true;
    }

    bool Event_check_test_result()
    {
        return acquisition_successful;
    }

    void Event_clear_test_result()
    {
        acquisition_successful = false;
    }

private:
    bool acquisition_successful;
};


void TrackingPullInTestFpga::configure_receiver(
    double PLL_wide_bw_hz,
    double DLL_wide_bw_hz,
    double PLL_narrow_bw_hz,
    double DLL_narrow_bw_hz,
    int extend_correlation_symbols)
{
    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "./tracking_ch_");
    config->set_property("Tracking.implementation", implementation);
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
#if USE_GLOG_AND_GFLAGS
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;
#else
    gnss_synchro.PRN = absl::GetFlag(FLAGS_test_satellite_PRN);
#endif
    gnss_synchro.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            gnss_synchro.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.5");
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            gnss_synchro.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.6");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_narrow_chips", "0.6");
            config->set_property("Tracking.track_pilot", "true");
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA" or implementation == "Galileo_E5a_DLL_PLL_Tracking_b_Fpga")
        {
            gnss_synchro.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            signal.copy(gnss_synchro.Signal, 2, 0);
            if (implementation == "Galileo_E5a_DLL_PLL_Tracking_b")
                {
                    config->supersede_property("Tracking.implementation", std::string("Galileo_E5a_DLL_PLL_Tracking_FPGA"));
                }
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA")
        {
            gnss_synchro.System = 'G';
            std::string signal = "L5";
            System_and_Signal = "GPS L5I";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }

    std::cout << "*****************************************\n";
    std::cout << "*** Tracking configuration parameters ***\n";
    std::cout << "*****************************************\n";
    std::cout << "Signal: " << System_and_Signal << "\n";
    std::cout << "implementation: " << config->property("Tracking.implementation", std::string("undefined")) << " \n";
    std::cout << "pll_bw_hz: " << config->property("Tracking.pll_bw_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_hz: " << config->property("Tracking.dll_bw_hz", 0.0) << " Hz\n";
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}


bool TrackingPullInTestFpga::acquire_signal(int SV_ID)
{
    pthread_t thread_DMA, thread_acquisition;

    // fsm
    std::shared_ptr<ChannelFsm_trk_pull_in_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_trk_pull_in_test>();
    bool acquisition_successful;

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    // config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string System_and_Signal;
    std::string signal;
    struct DMA_handler_args_trk_pull_in_test args;
    struct acquisition_handler_args_trk_pull_in_test args_acq;

#if USE_GLOG_AND_GFLAGS
    std::string file = FLAGS_signal_file;
#else
    std::string file = absl::GetFlag(FLAGS_signal_file);
#endif
    args.file = file;  // DMA file configuration

    // instantiate the FPGA switch and set the
    // switch position to DMA.
    std::shared_ptr<Fpga_Switch> switch_fpga;
    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(0);  // set switch position to DMA

    // create the correspondign acquisition block according to the desired tracking signal
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            tmp_gnss_synchro.System = 'G';
            signal = "1C";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L1 CA";
            acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            args.freq_band = 0;  // frequency band on which the DMA has to transfer the samples
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            tmp_gnss_synchro.System = 'E';
            signal = "1B";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E1B";
            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            args.freq_band = 0;  // frequency band on which the DMA has to transfer the samples
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
        {
            tmp_gnss_synchro.System = 'E';
            signal = "5X";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";
            acquisition = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            args.freq_band = 1;  // frequency band on which the DMA has to transfer the samples
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA")
        {
            tmp_gnss_synchro.System = 'G';
            signal = "L5";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L5I";
            acquisition = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            args.freq_band = 1;  // frequency band on which the DMA has to transfer the samples
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }

    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_channel_fsm(channel_fsm_);
    acquisition->set_channel(0);
#if USE_GLOG_AND_GFLAGS
    acquisition->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
    acquisition->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
    acquisition->set_doppler_center(0);
    acquisition->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
#else
    acquisition->set_doppler_max(config->property("Acquisition.doppler_max", absl::GetFlag(FLAGS_external_signal_acquisition_doppler_max_hz)));
    acquisition->set_doppler_step(config->property("Acquisition.doppler_step", absl::GetFlag(FLAGS_external_signal_acquisition_doppler_step_hz)));
    acquisition->set_doppler_center(0);
    acquisition->set_threshold(config->property("Acquisition.threshold", absl::GetFlag(FLAGS_external_signal_acquisition_threshold)));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;
    start = std::chrono::system_clock::now();

    bool start_msg = true;

    doppler_measurements_map.clear();
    code_delay_measurements_map.clear();
    acq_samplestamp_map.clear();

    unsigned int MAX_PRN_IDX = 0;

    switch (tmp_gnss_synchro.System)
        {
        case 'G':
            MAX_PRN_IDX = 33;
            break;
        case 'E':
            MAX_PRN_IDX = 37;
            break;
        default:
            MAX_PRN_IDX = 33;
        }

    // number of samples that the DMA has to transfer
    unsigned int nsamples_to_transfer;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS)));
        }
    else  // (if (implementation.compare("GPS_L5_DLL_PLL_Tracking_FPGA") == 0))
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L5I_CODE_RATE_CPS / GPS_L5I_CODE_LENGTH_CHIPS)));
        }

    // set the scaling factor
    args.scaling_factor = DMA_SIGNAL_SCALING_FACTOR;

    for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
        {
            tmp_gnss_synchro.PRN = PRN;

            channel_fsm_->Event_clear_test_result();

            acquisition->stop_acquisition();  // reset the whole system including the sample counters
            acquisition->init();
            acquisition->set_local_code();

            if ((implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA") or (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA"))
                {
                    // Configure the DMA to send TEST_TRK_PULL_IN_TEST_SKIP_SAMPLES in order to initialize the internal
                    // states of the downsampling filter in the FPGA
                    args.skip_used_samples = 0;
                    args.nsamples_tx = TEST_TRK_PULL_IN_TEST_SKIP_SAMPLES;

                    // create DMA child process
                    if (pthread_create(&thread_DMA, nullptr, handler_DMA_trk_pull_in_test, reinterpret_cast<void*>(&args)) < 0)
                        {
                            std::cout << "ERROR cannot create DMA Process\n";
                        }

                    pthread_join(thread_DMA, nullptr);

                    // Configure the DMA to skip the samples that were used to initialize the internal states of the
                    // downsampling filter in the FPGA
                    args.skip_used_samples = TEST_TRK_PULL_IN_TEST_SKIP_SAMPLES;
                }
            else
                {
                    args.skip_used_samples = 0;
                }

            // Configure the DMA to send the required samples to perform an acquisition
            args.nsamples_tx = nsamples_to_transfer;

            // run the acquisition. The acquisition must run in a separate thread because it is a blocking function
            args_acq.acquisition = acquisition;

            if (pthread_create(&thread_acquisition, nullptr, handler_acquisition_trk_pull_in_test, reinterpret_cast<void*>(&args_acq)) < 0)
                {
                    std::cout << "ERROR cannot create acquisition Process\n";
                }

            if (start_msg == true)
                {
#if USE_GLOG_AND_GFLAGS
                    std::cout << "Reading external signal file: " << FLAGS_signal_file << '\n';
#else
                    std::cout << "Reading external signal file: " << absl::GetFlag(FLAGS_signal_file) << '\n';
#endif
                    std::cout << "Searching for " << System_and_Signal << " Satellites...\n";
                    std::cout << "[";
                    start_msg = false;
                }

            // wait to give time for the acquisition thread to set up the acquisition HW accelerator in the FPGA
            usleep(1000000);

            // create DMA child process
            if (pthread_create(&thread_DMA, nullptr, handler_DMA_trk_pull_in_test, reinterpret_cast<void*>(&args)) < 0)
                {
                    std::cout << "ERROR cannot create DMA Process\n";
                }

            // wait until the acquisition is finished
            pthread_join(thread_acquisition, nullptr);

            // wait for the child DMA process to finish
            pthread_join(thread_DMA, nullptr);

            acquisition_successful = channel_fsm_->Event_check_test_result();

            if (acquisition_successful)
                {
                    std::cout << " " << PRN << " ";
                    doppler_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_doppler_hz));
                    code_delay_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_delay_samples));
                    acq_samplestamp_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_samplestamp_samples));
                }
            else
                {
                    std::cout << " . ";
                }

            std::cout.flush();
        }

    std::cout << "]\n";
    std::cout << "-------------------------------------------\n";

    for (auto& x : doppler_measurements_map)
        {
            std::cout << "DETECTED SATELLITE " << System_and_Signal << " PRN: " << x.first << " with Doppler: " << x.second << " [Hz], code phase: " << code_delay_measurements_map.at(x.first) << " [samples] at signal SampleStamp " << acq_samplestamp_map.at(x.first) << "\n";
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]\n";
    return true;
}


TEST_F(TrackingPullInTestFpga, ValidationOfResults)
{
    // pointer to the DMA thread that sends the samples to the acquisition engine
    pthread_t thread_DMA;

    struct DMA_handler_args_trk_pull_in_test args;

    // *************************************************
    // ***** STEP 1: Prepare the parameters sweep ******
    // *************************************************
    std::vector<double>
        acq_doppler_error_hz_values;
    std::vector<std::vector<double>> acq_delay_error_chips_values;  // vector of vector
#if USE_GLOG_AND_GFLAGS
    for (double doppler_hz = FLAGS_acq_Doppler_error_hz_start; doppler_hz >= FLAGS_acq_Doppler_error_hz_stop; doppler_hz = doppler_hz + FLAGS_acq_Doppler_error_hz_step)
#else
    for (double doppler_hz = absl::GetFlag(FLAGS_acq_Doppler_error_hz_start); doppler_hz >= absl::GetFlag(FLAGS_acq_Doppler_error_hz_stop); doppler_hz = doppler_hz + absl::GetFlag(FLAGS_acq_Doppler_error_hz_step))
#endif
        {
            acq_doppler_error_hz_values.push_back(doppler_hz);
            std::vector<double> tmp_vector;
            // Code Delay Sweep
#if USE_GLOG_AND_GFLAGS
            for (double code_delay_chips = FLAGS_acq_Delay_error_chips_start; code_delay_chips >= FLAGS_acq_Delay_error_chips_stop; code_delay_chips = code_delay_chips + FLAGS_acq_Delay_error_chips_step)
#else
            for (double code_delay_chips = absl::GetFlag(FLAGS_acq_Delay_error_chips_start); code_delay_chips >= absl::GetFlag(FLAGS_acq_Delay_error_chips_stop); code_delay_chips = code_delay_chips + absl::GetFlag(FLAGS_acq_Delay_error_chips_step))
#endif
                {
                    tmp_vector.push_back(code_delay_chips);
                }
            acq_delay_error_chips_values.push_back(tmp_vector);
        }

    // ***********************************************************
    // ***** STEP 2: Generate the input signal (if required) *****
    // ***********************************************************
    std::vector<double> generator_CN0_values;
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_enable_external_signal_file)
        {
            generator_CN0_values.push_back(999);  // an external input signal capture is selected, no CN0 information available
        }
    else
        {
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
        }
#else
    if (absl::GetFlag(FLAGS_enable_external_signal_file))
        {
            generator_CN0_values.push_back(999);  // an external input signal capture is selected, no CN0 information available
        }
    else
        {
            if (absl::GetFlag(FLAGS_CN0_dBHz_start) == absl::GetFlag(FLAGS_CN0_dBHz_stop))
                {
                    generator_CN0_values.push_back(absl::GetFlag(FLAGS_CN0_dBHz_start));
                }
            else
                {
                    for (double cn0 = absl::GetFlag(FLAGS_CN0_dBHz_start); cn0 > absl::GetFlag(FLAGS_CN0_dBHz_stop); cn0 = cn0 - absl::GetFlag(FLAGS_CN0_dB_step))
                        {
                            generator_CN0_values.push_back(cn0);
                        }
                }
        }
#endif

        // use generator or use an external capture file

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_enable_external_signal_file)
        {
            // create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(FLAGS_test_satellite_PRN), true);
            bool found_satellite = doppler_measurements_map.find(FLAGS_test_satellite_PRN) != doppler_measurements_map.end();
            EXPECT_TRUE(found_satellite) << "Error: satellite SV: " << FLAGS_test_satellite_PRN << " is not acquired";
#else
    if (absl::GetFlag(FLAGS_enable_external_signal_file))
        {
            // create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(absl::GetFlag(FLAGS_test_satellite_PRN)), true);
            bool found_satellite = doppler_measurements_map.find(absl::GetFlag(FLAGS_test_satellite_PRN)) != doppler_measurements_map.end();
            EXPECT_TRUE(found_satellite) << "Error: satellite SV: " << absl::GetFlag(FLAGS_test_satellite_PRN) << " is not acquired";
#endif
            if (!found_satellite)
                {
                    return;
                }
        }
    else
        {
            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                {
                    // Configure the signal generator
                    configure_generator(generator_CN0_values.at(current_cn0_idx), current_cn0_idx);
                    // Generate signal raw signal samples and observations RINEX file

#if USE_GLOG_AND_GFLAGS
                    if (FLAGS_disable_generator == false)
#else
                    if (absl::GetFlag(FLAGS_disable_generator) == false)
#endif
                        {
                            generate_signal();
                        }
                }
        }
#if USE_GLOG_AND_GFLAGS
    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols);
#else
    configure_receiver(absl::GetFlag(FLAGS_PLL_bw_hz_start),
        absl::GetFlag(FLAGS_DLL_bw_hz_start),
        absl::GetFlag(FLAGS_PLL_narrow_bw_hz),
        absl::GetFlag(FLAGS_DLL_narrow_bw_hz),
        absl::GetFlag(FLAGS_extend_correlation_symbols));
#endif

    // ******************************************************************************************
    // ***** Obtain the initial signal sinchronization parameters (emulating an acquisition) ****
    // ******************************************************************************************
    int test_satellite_PRN = 0;
    double true_acq_doppler_hz = 0.0;
    double true_acq_delay_samples = 0.0;
    uint64_t acq_samplestamp_samples = 0;

    Tracking_True_Obs_Reader true_obs_data;
#if USE_GLOG_AND_GFLAGS
    if (!FLAGS_enable_external_signal_file)
        {
            test_satellite_PRN = FLAGS_test_satellite_PRN;
#else
    if (!absl::GetFlag(FLAGS_enable_external_signal_file))
        {
            test_satellite_PRN = absl::GetFlag(FLAGS_test_satellite_PRN);
#endif
            std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
            true_obs_file.append(std::to_string(test_satellite_PRN));
            true_obs_file.append(".dat");
            true_obs_data.close_obs_file();
            ASSERT_EQ(true_obs_data.open_obs_file(true_obs_file), true) << "Failure opening true observables file";
            // load acquisition data based on the first epoch of the true observations
            ASSERT_EQ(true_obs_data.read_binary_obs(), true)
                << "Failure reading true tracking dump file.\n"
#if USE_GLOG_AND_GFLAGS
                << "Maybe sat PRN #" + std::to_string(FLAGS_test_satellite_PRN) +
#else
                << "Maybe sat PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) +
#endif
                       " is not available?";
            std::cout << "Testing satellite PRN=" << test_satellite_PRN << '\n';
            std::cout << "True Initial Doppler " << true_obs_data.doppler_l1_hz << " [Hz], true Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << "[Chips]\n";
            true_acq_doppler_hz = true_obs_data.doppler_l1_hz;
            true_acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * static_cast<double>(baseband_sampling_freq) * GPS_L1_CA_CODE_PERIOD_S;
            acq_samplestamp_samples = 0;
        }
    else
        {
#if USE_GLOG_AND_GFLAGS
            true_acq_doppler_hz = doppler_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            true_acq_delay_samples = code_delay_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            acq_samplestamp_samples = acq_samplestamp_map.find(FLAGS_test_satellite_PRN)->second;
#else
            true_acq_doppler_hz = doppler_measurements_map.find(absl::GetFlag(FLAGS_test_satellite_PRN))->second;
            true_acq_delay_samples = code_delay_measurements_map.find(absl::GetFlag(FLAGS_test_satellite_PRN))->second;
            acq_samplestamp_samples = acq_samplestamp_map.find(absl::GetFlag(FLAGS_test_satellite_PRN))->second;
#endif

            std::cout << "Estimated Initial Doppler " << true_acq_doppler_hz
                      << " [Hz], estimated Initial code delay " << true_acq_delay_samples << " [Samples]"
#if USE_GLOG_AND_GFLAGS
                      << " Acquisition SampleStamp is " << acq_samplestamp_map.find(FLAGS_test_satellite_PRN)->second << '\n';
#else
                      << " Acquisition SampleStamp is " << acq_samplestamp_map.find(absl::GetFlag(FLAGS_test_satellite_PRN))->second << '\n';
#endif
        }
#if USE_GLOG_AND_GFLAGS
    int64_t acq_to_trk_delay_samples = ceil(static_cast<double>(FLAGS_fs_gen_sps) * FLAGS_acq_to_trk_delay_s);
#else
    int64_t acq_to_trk_delay_samples = ceil(static_cast<double>(absl::GetFlag(FLAGS_fs_gen_sps)) * absl::GetFlag(FLAGS_acq_to_trk_delay_s));
#endif

    // set the scaling factor
    args.scaling_factor = DMA_SIGNAL_SCALING_FACTOR;

    // CN0 LOOP
    std::vector<std::vector<double>> pull_in_results_v_v;

    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
        {
            std::vector<double> pull_in_results_v;
            for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
                {
                    for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                        {
                            gnss_synchro.Acq_samplestamp_samples = acq_samplestamp_samples;
                            // simulate a Doppler error in acquisition
                            gnss_synchro.Acq_doppler_hz = true_acq_doppler_hz + acq_doppler_error_hz_values.at(current_acq_doppler_error_idx);
                            // simulate Code Delay error in acquisition
                            gnss_synchro.Acq_delay_samples = true_acq_delay_samples + (acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) / GPS_L1_CA_CODE_RATE_CPS) * static_cast<double>(baseband_sampling_freq);

                            // We need to reset the HW again in order to reset the sample counter.
                            // The HW is reset by sending a command to the acquisition HW accelerator
                            // In order to send the reset command to the HW we instantiate the acquisition module.
                            std::shared_ptr<AcquisitionInterface> acquisition;

                            // reset the HW to clear the sample counters: the acquisition constructor generates a reset
                            if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
                                {
                                    acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                                    args.freq_band = 0;
                                }
                            else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
                                {
                                    acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                                    args.freq_band = 0;
                                }
                            else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
                                {
                                    acquisition = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                                    args.freq_band = 1;
                                }
                            else if (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA")
                                {
                                    acquisition = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                                    args.freq_band = 1;
                                }
                            else
                                {
                                    std::cout << "The test can not run with the selected tracking implementation\n ";
                                    throw(std::exception());
                                }

                            acquisition->stop_acquisition();  // reset the whole system including the sample counters

                            // create flowgraph
                            top_block = gr::make_top_block("Tracking test");
                            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config.get(), "Tracking", 1, 1);
                            std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);
                            auto msg_rx = TrackingPullInTest_msg_rx_Fpga_make();

                            ASSERT_NO_THROW({
                                tracking->set_channel(gnss_synchro.Channel_ID);
                            }) << "Failure setting channel.";

                            ASSERT_NO_THROW({
                                tracking->set_gnss_synchro(&gnss_synchro);
                            }) << "Failure setting gnss_synchro.";

                            ASSERT_NO_THROW({
                                tracking->connect(top_block);
                            }) << "Failure connecting tracking to the top_block.";

                            std::string file;
                            ASSERT_NO_THROW({
#if USE_GLOG_AND_GFLAGS
                                if (!FLAGS_enable_external_signal_file)
#else
                                if (!absl::GetFlag(FLAGS_enable_external_signal_file))
#endif
                                    {
                                        file = "./" + filename_raw_data + std::to_string(current_cn0_idx);
                                    }
                                else
                                    {
#if USE_GLOG_AND_GFLAGS
                                        file = FLAGS_signal_file;
#else
                                        file = absl::GetFlag(FLAGS_signal_file);
#endif
                                    }

                                gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
                                top_block->connect(tracking->get_right_block(), 0, sink, 0);
                                top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
                            }) << "Failure connecting the blocks of tracking test.";

                            // initialize the internal status of the LPF in the FPGA in the L1/E1 frequency band

                            // ********************************************************************
                            // ***** STEP 5: Perform the signal tracking and read the results *****
                            // ********************************************************************
                            std::cout << "--- START TRACKING WITH PULL-IN ERROR: " << acq_doppler_error_hz_values.at(current_acq_doppler_error_idx) << " [Hz] and " << acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) << " [Chips] ---\n";
                            std::chrono::time_point<std::chrono::system_clock> start, end;

                            top_block->start();

                            usleep(1000000);  // give time for the system to start before receiving the start tracking command.

                            if (acq_to_trk_delay_samples > 0)
                                {
#if USE_GLOG_AND_GFLAGS
                                    std::cout << "--- SIMULATING A PULL-IN DELAY OF " << FLAGS_acq_to_trk_delay_s << " SECONDS ---\n";
#else
                                    std::cout << "--- SIMULATING A PULL-IN DELAY OF " << absl::GetFlag(FLAGS_acq_to_trk_delay_s) << " SECONDS ---\n";
#endif
                                    args.file = file;
                                    args.nsamples_tx = acq_to_trk_delay_samples;  // 150 s for now but will be all file

                                    args.skip_used_samples = 0;

                                    if (pthread_create(&thread_DMA, nullptr, handler_DMA_trk_pull_in_test, reinterpret_cast<void*>(&args)) < 0)
                                        {
                                            std::cout << "ERROR cannot create DMA Process\n";
                                        }
                                }

                            std::cout << " Starting tracking...\n";

                            tracking->start_tracking();
                            std::cout << " Waiting flowgraph..\n";

                            args.file = file;
#if USE_GLOG_AND_GFLAGS
                            args.nsamples_tx = baseband_sampling_freq * FLAGS_duration;
#else
                            args.nsamples_tx = baseband_sampling_freq * absl::GetFlag(FLAGS_duration);
#endif
                            args.skip_used_samples = acq_to_trk_delay_samples;

                            if (pthread_create(&thread_DMA, nullptr, handler_DMA_trk_pull_in_test, reinterpret_cast<void*>(&args)) < 0)
                                {
                                    std::cout << "ERROR cannot create DMA Process\n";
                                }

                            // wait for the child DMA process to finish
                            pthread_join(thread_DMA, nullptr);

                            // stop the top block
                            top_block->stop();

                            tracking->stop_tracking();

                            // reset the HW in order to produce an interrupt to the tracking
                            // modules that are in a waiting state
                            acquisition->stop_acquisition();

                            std::chrono::duration<double> elapsed_seconds = end - start;
                            std::cout << "Signal tracking completed in " << elapsed_seconds.count() << " seconds\n";

                            pull_in_results_v.push_back(msg_rx->rx_message != 3);  // save last asynchronous tracking message in order to detect a loss of lock

                            // ********************************
                            // ***** STEP 7: Plot results *****
                            // ********************************

#if USE_GLOG_AND_GFLAGS
                            if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
#else
                            if (absl::GetFlag(FLAGS_plot_detail_level) >= 2 and absl::GetFlag(FLAGS_show_plots))
#endif
                                {
                                    // load the measured values
                                    Tracking_Dump_Reader trk_dump;
                                    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
                                        << "Failure opening tracking dump file";

                                    int64_t n_measured_epochs = trk_dump.num_epochs();
                                    // todo: use vectors instead
                                    arma::vec trk_timestamp_s = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_acc_carrier_phase_cycles = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_Doppler_Hz = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_prn_delay_chips = arma::zeros(n_measured_epochs, 1);
                                    std::vector<double> timestamp_s;
                                    std::vector<double> prompt;
                                    std::vector<double> early;
                                    std::vector<double> late;
                                    std::vector<double> v_early;
                                    std::vector<double> v_late;
                                    std::vector<double> promptI;
                                    std::vector<double> promptQ;
                                    std::vector<double> CN0_dBHz;
                                    std::vector<double> Doppler;
                                    int64_t epoch_counter = 0;
                                    while (trk_dump.read_binary_obs())
                                        {
                                            trk_timestamp_s(epoch_counter) = static_cast<double>(trk_dump.PRN_start_sample_count) / static_cast<double>(baseband_sampling_freq);
                                            trk_acc_carrier_phase_cycles(epoch_counter) = trk_dump.acc_carrier_phase_rad / TWO_PI;
                                            trk_Doppler_Hz(epoch_counter) = trk_dump.carrier_doppler_hz;
                                            double delay_chips = GPS_L1_CA_CODE_LENGTH_CHIPS - GPS_L1_CA_CODE_LENGTH_CHIPS * (fmod((static_cast<double>(trk_dump.PRN_start_sample_count) + trk_dump.aux1) / static_cast<double>(baseband_sampling_freq), 1.0e-3) / 1.0e-3);

                                            trk_prn_delay_chips(epoch_counter) = delay_chips;

                                            timestamp_s.push_back(trk_timestamp_s(epoch_counter));
                                            prompt.push_back(trk_dump.abs_P);
                                            early.push_back(trk_dump.abs_E);
                                            late.push_back(trk_dump.abs_L);
                                            v_early.push_back(trk_dump.abs_VE);
                                            v_late.push_back(trk_dump.abs_VL);
                                            promptI.push_back(trk_dump.prompt_I);
                                            promptQ.push_back(trk_dump.prompt_Q);
                                            CN0_dBHz.push_back(trk_dump.CN0_SNV_dB_Hz);
                                            Doppler.push_back(trk_dump.carrier_doppler_hz);
                                            epoch_counter++;
                                        }
#if USE_GLOG_AND_GFLAGS
                                    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
#else
                                    const std::string gnuplot_executable(absl::GetFlag(FLAGS_gnuplot_executable));
#endif
                                    if (gnuplot_executable.empty())
                                        {
                                            std::cout << "WARNING: Although the flag show_plots has been set to TRUE,\n";
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
#if USE_GLOG_AND_GFLAGS
                                                    auto decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

                                                    if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                                        {
                                                            Gnuplot g1("linespoints");
                                                            g1.showonscreen();  // window output
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g1.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g1.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
#else
                                                    auto decimate = static_cast<unsigned int>(absl::GetFlag(FLAGS_plot_decimate));

                                                    if (absl::GetFlag(FLAGS_plot_detail_level) >= 2 and absl::GetFlag(FLAGS_show_plots))
                                                        {
                                                            Gnuplot g1("linespoints");
                                                            g1.showonscreen();  // window output
                                                            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
                                                                {
                                                                    g1.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, " + "PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g1.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
#endif
                                                            g1.set_grid();
                                                            g1.set_xlabel("Time [s]");
                                                            g1.set_ylabel("Correlators' output");
                                                            // g1.cmd("set key box opaque");
                                                            g1.plot_xy(trk_timestamp_s, prompt, "Prompt", decimate);
                                                            g1.plot_xy(trk_timestamp_s, early, "Early", decimate);
                                                            g1.plot_xy(trk_timestamp_s, late, "Late", decimate);
                                                            if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
                                                                {
                                                                    g1.plot_xy(trk_timestamp_s, v_early, "Very Early", decimate);
                                                                    g1.plot_xy(trk_timestamp_s, v_late, "Very Late", decimate);
                                                                }
                                                            g1.set_legend();
                                                            g1.savetops("Correlators_outputs");

                                                            Gnuplot g2("points");
                                                            g2.showonscreen();  // window output
#if USE_GLOG_AND_GFLAGS
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g2.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
#else
                                                            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
                                                                {
                                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g2.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
#endif
                                                            g2.set_grid();
                                                            g2.set_xlabel("Inphase");
                                                            g2.set_ylabel("Quadrature");
                                                            // g2.cmd("set size ratio -1");
                                                            g2.plot_xy(promptI, promptQ);
                                                            g2.savetops("Constellation");

                                                            Gnuplot g3("linespoints");
#if USE_GLOG_AND_GFLAGS
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g3.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g3.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
#else
                                                            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
                                                                {
                                                                    g3.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g3.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
#endif
                                                            g3.set_grid();
                                                            g3.set_xlabel("Time [s]");
                                                            g3.set_ylabel("Reported CN0 [dB-Hz]");
                                                            g3.cmd("set key box opaque");

                                                            g3.plot_xy(trk_timestamp_s, CN0_dBHz,
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g3.set_legend();
                                                            g3.savetops("CN0_output");

                                                            g3.showonscreen();  // window output

                                                            Gnuplot g4("linespoints");
#if USE_GLOG_AND_GFLAGS
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g4.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g4.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
#else
                                                            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
                                                                {
                                                                    g4.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g4.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                                                                }
#endif
                                                            g4.set_grid();
                                                            g4.set_xlabel("Time [s]");
                                                            g4.set_ylabel("Estimated Doppler [Hz]");
                                                            g4.cmd("set key box opaque");

                                                            g4.plot_xy(trk_timestamp_s, Doppler,
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g4.set_legend();
                                                            g4.savetops("Doppler");

                                                            g4.showonscreen();  // window output
                                                        }
                                                }
                                            catch (const GnuplotException& ge)
                                                {
                                                    std::cout << ge.what() << '\n';
                                                }
                                        }
                                }  // end plot
                        }          // end acquisition Delay errors loop
                }                  // end acquisition Doppler errors loop
            pull_in_results_v_v.push_back(pull_in_results_v);
        }  // end CN0 LOOP

    // build the mesh grid
    std::vector<double> doppler_error_mesh;
    std::vector<double> code_delay_error_mesh;
    for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
        {
            for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                {
                    doppler_error_mesh.push_back(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx));
                    code_delay_error_mesh.push_back(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx));
                }
        }

    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
        {
            std::vector<double> pull_in_result_mesh;
            pull_in_result_mesh = pull_in_results_v_v.at(current_cn0_idx);
            // plot grid
            Gnuplot g4("points palette pointsize 2 pointtype 7");
#if USE_GLOG_AND_GFLAGS
            if (FLAGS_show_plots)
#else
            if (absl::GetFlag(FLAGS_show_plots))
#endif
                {
                    g4.showonscreen();  // window output
                }
            else
                {
                    g4.disablescreen();
                }
            g4.cmd(R"(set palette defined ( 0 "black", 1 "green" ))");
            g4.cmd("set key off");
            g4.cmd("set view map");
            std::string title;
#if USE_GLOG_AND_GFLAGS
            if (!FLAGS_enable_external_signal_file)
                {
                    title = std::string("Tracking Pull-in result grid at CN0:" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + " [dB-Hz], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz].");
                }
            else
                {
                    title = std::string("Tracking Pull-in result grid, PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                }
#else
            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
                {
                    title = std::string("Tracking Pull-in result grid at CN0:" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + " [dB-Hz], PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz].");
                }
            else
                {
                    title = std::string("Tracking Pull-in result grid, PLL/DLL BW: " + std::to_string(absl::GetFlag(FLAGS_PLL_bw_hz_start)) + "," + std::to_string(absl::GetFlag(FLAGS_DLL_bw_hz_start)) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(absl::GetFlag(FLAGS_test_satellite_PRN)) + ")");
                }
#endif
            g4.set_title(title);
            g4.set_grid();
            g4.set_xlabel("Acquisition Doppler error [Hz]");
            g4.set_ylabel("Acquisition Code Delay error [Chips]");
            g4.cmd("set cbrange[0:1]");
            g4.plot_xyz(doppler_error_mesh,
                code_delay_error_mesh,
                pull_in_result_mesh);
            g4.set_legend();
#if USE_GLOG_AND_GFLAGS
            if (!FLAGS_enable_external_signal_file)
#else
            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
#endif
                {
                    g4.savetops("trk_pull_in_grid_" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))));
                    g4.savetopdf("trk_pull_in_grid_" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))), 12);
                }
            else
                {
                    g4.savetops("trk_pull_in_grid_external_file");
                    g4.savetopdf("trk_pull_in_grid_external_file", 12);
                }
        }
}

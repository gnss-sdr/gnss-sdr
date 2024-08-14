/*!
 * \file hybrid_observables_test_fpga.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
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
#include "fpga_switch.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_sdr_fpga_sample_counter.h"
#include "gnss_synchro.h"
#include "gnuplot_i.h"
#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "gps_l5i_pcps_acquisition_fpga.h"
#include "hybrid_observables.h"
#include "in_memory_configuration.h"
#include "observable_tests_flags.h"
#include "observables_dump_reader.h"
#include "signal_generator_flags.h"
#include "telemetry_decoder_interface.h"
#include "test_flags.h"
#include "tlm_dump_reader.h"
#include "tracking_dump_reader.h"
#include "tracking_interface.h"
#include "tracking_tests_flags.h"
#include "tracking_true_obs_reader.h"
#include "true_observables_reader.h"
#include <armadillo>
#include <boost/lexical_cast.hpp>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <matio.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <pthread.h>
#include <unistd.h>
#include <utility>

#if GNSSTK_USES_GPSTK_NAMESPACE
#include <gpstk/GPSWeekSecond.hpp>
#include <gpstk/Rinex3ObsBase.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <gpstk/RinexUtilities.hpp>
namespace gnsstk = gpstk;
#else
#include <gnsstk/GPSWeekSecond.hpp>
#include <gnsstk/Rinex3ObsBase.hpp>
#include <gnsstk/Rinex3ObsData.hpp>
#include <gnsstk/Rinex3ObsHeader.hpp>
#include <gnsstk/Rinex3ObsStream.hpp>
#include <gnsstk/RinexUtilities.hpp>
#endif

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

class HybridObservablesTest_msg_rx_Fpga;

using HybridObservablesTest_msg_rx_Fpga_sptr = gnss_shared_ptr<HybridObservablesTest_msg_rx_Fpga>;

HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make();

class HybridObservablesTest_msg_rx_Fpga : public gr::block
{
private:
    friend HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    HybridObservablesTest_msg_rx_Fpga();

public:
    int rx_message;
    ~HybridObservablesTest_msg_rx_Fpga();  //!< Default destructor
};


HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make()
{
    return HybridObservablesTest_msg_rx_Fpga_sptr(new HybridObservablesTest_msg_rx_Fpga());
}


void HybridObservablesTest_msg_rx_Fpga::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


HybridObservablesTest_msg_rx_Fpga::HybridObservablesTest_msg_rx_Fpga() : gr::block("HybridObservablesTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&HybridObservablesTest_msg_rx_Fpga::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&HybridObservablesTest_msg_rx_Fpga::msg_handler_channel_events, this, _1));
#endif
#endif
    rx_message = 0;
}


HybridObservablesTest_msg_rx_Fpga::~HybridObservablesTest_msg_rx_Fpga() = default;


class HybridObservablesTest_tlm_msg_rx_Fpga;


using HybridObservablesTest_tlm_msg_rx_Fpga_sptr = std::shared_ptr<HybridObservablesTest_tlm_msg_rx_Fpga>;


HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make();


class HybridObservablesTest_tlm_msg_rx_Fpga : public gr::block
{
private:
    friend HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make();
    void msg_handler_channel_events(const pmt::pmt_t msg);
    HybridObservablesTest_tlm_msg_rx_Fpga();

public:
    int rx_message;
    ~HybridObservablesTest_tlm_msg_rx_Fpga();  //!< Default destructor
};


HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make()
{
    return HybridObservablesTest_tlm_msg_rx_Fpga_sptr(new HybridObservablesTest_tlm_msg_rx_Fpga());
}


void HybridObservablesTest_tlm_msg_rx_Fpga::msg_handler_channel_events(const pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_channel_events Bad any_cast: " << e.what();
            rx_message = 0;
        }
}


HybridObservablesTest_tlm_msg_rx_Fpga::HybridObservablesTest_tlm_msg_rx_Fpga() : gr::block("HybridObservablesTest_tlm_msg_rx_Fpga", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_tlm_msg_rx_Fpga::msg_handler_channel_events, this, boost::placeholders::_1));
    rx_message = 0;
}


HybridObservablesTest_tlm_msg_rx_Fpga::~HybridObservablesTest_tlm_msg_rx_Fpga() = default;


class HybridObservablesTestFpga : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
#if USE_GLOG_AND_GFLAGS
    std::string implementation = FLAGS_trk_test_implementation;
    const int baseband_sampling_freq = FLAGS_fs_gen_sps;
    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;
#else
    std::string implementation = absl::GetFlag(FLAGS_trk_test_implementation);
    const int baseband_sampling_freq = absl::GetFlag(FLAGS_fs_gen_sps);
    std::string filename_rinex_obs = absl::GetFlag(FLAGS_filename_rinex_obs);
    std::string filename_raw_data = absl::GetFlag(FLAGS_filename_raw_data);
#endif

    int configure_generator();
    int generate_signal();
    bool save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename);
    void check_results_carrier_phase(
        arma::mat& true_ch0,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        const std::string& data_title);
    void check_results_carrier_phase_double_diff(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_ch0_s,
        arma::vec& true_tow_ch1_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1,
        const std::string& data_title);
    void check_results_carrier_doppler(arma::mat& true_ch0,
        arma::vec& true_tow_s,
        arma::mat& measured_ch0,
        const std::string& data_title);
    void check_results_carrier_doppler_double_diff(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_ch0_s,
        arma::vec& true_tow_ch1_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1,
        const std::string& data_title);
    void check_results_code_pseudorange(
        arma::mat& true_ch0,
        arma::mat& true_ch1,
        arma::vec& true_tow_ch0_s,
        arma::vec& true_tow_ch1_s,
        arma::mat& measured_ch0,
        arma::mat& measured_ch1,
        const std::string& data_title);

    void check_results_duplicated_satellite(
        arma::mat& measured_sat1,
        arma::mat& measured_sat2,
        int ch_id,
        const std::string& data_title);

    HybridObservablesTestFpga()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
    }

    ~HybridObservablesTestFpga() = default;

    bool ReadRinexObs(std::vector<arma::mat>* obs_vec, Gnss_Synchro gnss);

    bool acquire_signal();
    void configure_receiver(
        double PLL_wide_bw_hz,
        double DLL_wide_bw_hz,
        double PLL_narrow_bw_hz,
        double DLL_narrow_bw_hz,
        int extend_correlation_symbols,
        uint32_t smoother_length,
        bool high_dyn);

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro_master;
    std::vector<Gnss_Synchro> gnss_synchro_vec;
    size_t item_size;
    pthread_mutex_t mutex_obs_test = PTHREAD_MUTEX_INITIALIZER;

    static const int32_t TEST_OBS_SKIP_SAMPLES = 1024;
    static constexpr float DMA_SIGNAL_SCALING_FACTOR = 8.0;
};


int HybridObservablesTestFpga::configure_generator()
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
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  // Baseband sampling frequency [MSps]
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
    p3 = std::string("-rinex_obs_file=") + absl::GetFlag(FLAGS_filename_rinex_obs);  // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + absl::GetFlag(FLAGS_filename_raw_data);     // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);    // Baseband sampling frequency [MSps]
#endif

    return 0;
}


int HybridObservablesTestFpga::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], nullptr};

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


struct DMA_handler_args_obs_test
{
    std::string file;
    int32_t nsamples_tx;
    int32_t skip_used_samples;
    unsigned int freq_band;  // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
    float scaling_factor;
};


struct acquisition_handler_args_obs_test
{
    std::shared_ptr<AcquisitionInterface> acquisition;
};


void* handler_acquisition_obs_test(void* arguments)
{
    // the acquisition is a blocking function so we have to
    // create a thread
    auto* args = (struct acquisition_handler_args_obs_test*)arguments;
    args->acquisition->reset();
    return nullptr;
}


void* handler_DMA_obs_test(void* arguments)
{
    const int MAX_INPUT_SAMPLES_TOTAL = 16384;

    auto* args = (struct DMA_handler_args_obs_test*)arguments;

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

            // std::cout << "DMA: sending nsamples_block_size = " << nsamples_block_size << " samples\n";
            if (write(tx_fd, input_samples_dma.data(), (int)(nsamples_block_size * 4)) != (int)(nsamples_block_size * 4))
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


// When using the FPGA the acquisition class calls the states
// of the channel finite state machine directly. This is done
// in order to reduce the latency of the receiver when going
// from acquisition to tracking. In order to execute the
// acquisition in the unit tests we need to create a derived
// class of the channel finite state machine. Some of the states
// of the channel state machine are modified here, in order to
// simplify the instantiation of the acquisition class in the
// unit test.
class ChannelFsm_obs_test : public ChannelFsm
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


bool HybridObservablesTestFpga::acquire_signal()
{
    pthread_t thread_DMA, thread_acquisition;

    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    int SV_ID = 1;  // initial sv id

    // fsm
    std::shared_ptr<ChannelFsm_obs_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_obs_test>();
    bool acquisition_successful;

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string System_and_Signal;
    std::string signal;
    struct DMA_handler_args_obs_test args;
    struct acquisition_handler_args_obs_test args_acq;
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
                    // Skip the first TEST_OBS_SKIP_SAMPLES samples
                    args.skip_used_samples = 0;
                    args.nsamples_tx = TEST_OBS_SKIP_SAMPLES;  // limit is between 645 and 650 samples

                    // create DMA child process
                    if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, reinterpret_cast<void*>(&args)) < 0)
                        {
                            std::cout << "ERROR cannot create DMA Process\n";
                        }

                    pthread_join(thread_DMA, nullptr);

                    args.skip_used_samples = TEST_OBS_SKIP_SAMPLES;
                }
            else
                {
                    args.skip_used_samples = 0;
                }

            // Configure the DMA to send the required samples to perform an acquisition
            args.nsamples_tx = nsamples_to_transfer;

            // run the acquisition. The acquisition must run in a separate thread because it is a blocking function
            args_acq.acquisition = acquisition;

            if (pthread_create(&thread_acquisition, nullptr, handler_acquisition_obs_test, reinterpret_cast<void*>(&args_acq)) < 0)
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
            if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, reinterpret_cast<void*>(&args)) < 0)
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

                    gnss_synchro_vec.push_back(tmp_gnss_synchro);
                }
            else
                {
                    std::cout << " . ";
                }

            std::cout.flush();
        }

    std::cout << "]\n";
    std::cout << "-------------------------------------------\n";

    for (auto& x : gnss_synchro_vec)
        {
            std::cout << "DETECTED SATELLITE " << System_and_Signal
                      << " PRN: " << x.PRN
                      << " with Doppler: " << x.Acq_doppler_hz
                      << " [Hz], code phase: " << x.Acq_delay_samples
                      << " [samples] at signal SampleStamp " << x.Acq_samplestamp_samples << "\n";
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]\n";
    if (!gnss_synchro_vec.empty())
        {
            return true;
        }
    else
        {
            return false;
        }
}


void HybridObservablesTestFpga::configure_receiver(
    double PLL_wide_bw_hz,
    double DLL_wide_bw_hz,
    double PLL_narrow_bw_hz,
    double DLL_narrow_bw_hz,
    int extend_correlation_symbols,
    uint32_t smoother_length,
    bool high_dyn)
{
    config = std::make_shared<InMemoryConfiguration>();
    if (high_dyn)
        {
            config->set_property("Tracking.high_dyn", "true");
        }
    else
        {
            config->set_property("Tracking.high_dyn", "false");
        }
    config->set_property("Tracking.implementation", implementation);
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
#if USE_GLOG_AND_GFLAGS
    config->set_property("Tracking.fll_bw_hz", std::to_string(FLAGS_fll_bw_hz));
    config->set_property("Tracking.enable_fll_pull_in", FLAGS_enable_fll_pull_in ? "true" : "false");
    config->set_property("Tracking.enable_fll_steady_state", FLAGS_enable_fll_steady_state ? "true" : "false");
#else
    config->set_property("Tracking.fll_bw_hz", std::to_string(absl::GetFlag(FLAGS_fll_bw_hz)));
    config->set_property("Tracking.enable_fll_pull_in", absl::GetFlag(FLAGS_enable_fll_pull_in) ? "true" : "false");
    config->set_property("Tracking.enable_fll_steady_state", absl::GetFlag(FLAGS_enable_fll_steady_state) ? "true" : "false");
#endif
    config->set_property("Tracking.smoother_length", std::to_string(smoother_length));
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "./tracking_ch_");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.dump", "true");
    config->set_property("TelemetryDecoder.dump", "true");

    gnss_synchro_master.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            gnss_synchro_master.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.1");

            config->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            gnss_synchro_master.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.5");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_narrow_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");

            config->set_property("TelemetryDecoder.implementation", "Galileo_E1B_Telemetry_Decoder");
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")  // or implementation.compare("Galileo_E5a_DLL_PLL_Tracking_b") == 0)
        {
            gnss_synchro_master.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");


            config->set_property("TelemetryDecoder.implementation", "Galileo_E5a_Telemetry_Decoder");
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA")
        {
            gnss_synchro_master.System = 'G';
            std::string signal = "L5";
            System_and_Signal = "GPS L5I";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");

            config->set_property("TelemetryDecoder.implementation", "GPS_L5_Telemetry_Decoder");
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
    std::cout << "fll_bw_hz: " << config->property("Tracking.fll_bw_hz", 0.0) << " Hz\n";
    std::cout << "enable_fll_pull_in: " << config->property("Tracking.enable_fll_pull_in", false) << "\n";
    std::cout << "enable_fll_steady_state: " << config->property("Tracking.enable_fll_steady_state", false) << "\n";
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "high_dyn: " << config->property("Tracking.high_dyn", false) << "\n";
    std::cout << "smoother_length: " << config->property("Tracking.smoother_length", 0) << "\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}


void HybridObservablesTestFpga::check_results_carrier_phase(
    arma::mat& true_ch0,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    double t0 = measured_ch0(0, 0);
    int size1 = measured_ch0.col(0).n_rows;
    double t1 = measured_ch0(size1 - 1, 0);
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    // conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_phase_interp;
    arma::interp1(true_tow_s, true_ch0.col(3), t, true_ch0_phase_interp);

    arma::vec meas_ch0_phase_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(3), t, meas_ch0_phase_interp);

    // 2. RMSE
    arma::vec err_ch0_cycles;

    // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    err_ch0_cycles = meas_ch0_phase_interp - true_ch0_phase_interp - meas_ch0_phase_interp(0) + true_ch0_phase_interp(0);

    arma::vec err2_ch0 = arma::square(err_ch0_cycles);
    double rmse_ch0 = sqrt(arma::mean(err2_ch0));

    // 3. Mean err and variance
    double error_mean_ch0 = arma::mean(err_ch0_cycles);
    double error_var_ch0 = arma::var(err_ch0_cycles);

    // 4. Peaks
    double max_error_ch0 = arma::max(err_ch0_cycles);
    double min_error_ch0 = arma::min(err_ch0_cycles);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << data_title << " Accumulated Carrier phase RMSE = "
              << rmse_ch0 << ", mean = " << error_mean_ch0
              << ", stdev = " << sqrt(error_var_ch0)
              << " (max,min) = " << max_error_ch0
              << "," << min_error_ch0
              << " [cycles]\n";
    std::cout.precision(ss);

    // plots
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_show_plots)
#else
    if (absl::GetFlag(FLAGS_show_plots))
#endif
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Accumulated Carrier phase error [cycles]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Carrier Phase error [cycles]");
            // conversion between arma::vec and std:vector
            std::vector<double> error_vec(err_ch0_cycles.colptr(0), err_ch0_cycles.colptr(0) + err_ch0_cycles.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, error_vec,
                "Carrier Phase error");
            g3.set_legend();
            g3.savetops(data_title + "Carrier_phase_error");

            g3.showonscreen();  // window output
        }

    // check results against the test tolerance
    ASSERT_LT(rmse_ch0, 0.25);
    ASSERT_LT(error_mean_ch0, 0.2);
    ASSERT_GT(error_mean_ch0, -0.2);
    ASSERT_LT(error_var_ch0, 0.5);
    ASSERT_LT(max_error_ch0, 0.5);
    ASSERT_GT(min_error_ch0, -0.5);
}


void HybridObservablesTestFpga::check_results_carrier_phase_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_ch0_s,
    arma::vec& true_tow_ch1_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    // conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_carrier_phase_interp;
    arma::vec true_ch1_carrier_phase_interp;
    arma::interp1(true_tow_ch0_s, true_ch0.col(3), t, true_ch0_carrier_phase_interp);
    arma::interp1(true_tow_ch1_s, true_ch1.col(3), t, true_ch1_carrier_phase_interp);

    arma::vec meas_ch0_carrier_phase_interp;
    arma::vec meas_ch1_carrier_phase_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(3), t, meas_ch0_carrier_phase_interp);
    arma::interp1(measured_ch1.col(0), measured_ch1.col(3), t, meas_ch1_carrier_phase_interp);

    // generate double difference accumulated carrier phases
    // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    arma::vec delta_true_carrier_phase_cycles = (true_ch0_carrier_phase_interp - true_ch0_carrier_phase_interp(0)) - (true_ch1_carrier_phase_interp - true_ch1_carrier_phase_interp(0));
    arma::vec delta_measured_carrier_phase_cycles = (meas_ch0_carrier_phase_interp - meas_ch0_carrier_phase_interp(0)) - (meas_ch1_carrier_phase_interp - meas_ch1_carrier_phase_interp(0));

    // 2. RMSE
    arma::vec err;

    err = delta_measured_carrier_phase_cycles - delta_true_carrier_phase_cycles;
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
    std::cout << std::setprecision(10) << data_title << "Double diff Carrier Phase RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [Cycles]\n";
    std::cout.precision(ss);

    // plots
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_show_plots)
#else
    if (absl::GetFlag(FLAGS_show_plots))
#endif
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Carrier Phase error [Cycles]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Carrier Phase error [Cycles]");
            // conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Carrier Phase error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_carrier_phase_error");

            g3.showonscreen();  // window output
        }

    // check results against the test tolerance
    ASSERT_LT(rmse, 0.25);
    ASSERT_LT(error_mean, 0.2);
    ASSERT_GT(error_mean, -0.2);
    ASSERT_LT(error_var, 0.5);
    ASSERT_LT(max_error, 0.5);
    ASSERT_GT(min_error, -0.5);
}


void HybridObservablesTestFpga::check_results_carrier_doppler_double_diff(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_ch0_s,
    arma::vec& true_tow_ch1_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    // conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_carrier_doppler_interp;
    arma::vec true_ch1_carrier_doppler_interp;
    arma::interp1(true_tow_ch0_s, true_ch0.col(2), t, true_ch0_carrier_doppler_interp);
    arma::interp1(true_tow_ch1_s, true_ch1.col(2), t, true_ch1_carrier_doppler_interp);

    arma::vec meas_ch0_carrier_doppler_interp;
    arma::vec meas_ch1_carrier_doppler_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(2), t, meas_ch0_carrier_doppler_interp);
    arma::interp1(measured_ch1.col(0), measured_ch1.col(2), t, meas_ch1_carrier_doppler_interp);

    // generate double difference carrier Doppler
    arma::vec delta_true_carrier_doppler_cycles = true_ch0_carrier_doppler_interp - true_ch1_carrier_doppler_interp;
    arma::vec delta_measured_carrier_doppler_cycles = meas_ch0_carrier_doppler_interp - meas_ch1_carrier_doppler_interp;

    // 2. RMSE
    arma::vec err;

    err = delta_measured_carrier_doppler_cycles - delta_true_carrier_doppler_cycles;
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
    std::cout << std::setprecision(10) << data_title << "Double diff Carrier Doppler RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [Hz]\n";
    std::cout.precision(ss);

// plots
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_show_plots)
#else
    if (absl::GetFlag(FLAGS_show_plots))
#endif
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Carrier Doppler error [Hz]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Carrier Doppler error [Hz]");
            // conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Carrier Doppler error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_carrier_doppler_error");

            g3.showonscreen();  // window output
        }

    // check results against the test tolerance
    ASSERT_LT(error_mean, 5);
    ASSERT_GT(error_mean, -5);
    // assuming PLL BW=35
    ASSERT_LT(error_var, 200);
    ASSERT_LT(max_error, 70);
    ASSERT_GT(min_error, -70);
    ASSERT_LT(rmse, 30);
}


void HybridObservablesTestFpga::check_results_carrier_doppler(
    arma::mat& true_ch0,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    double t0 = measured_ch0(0, 0);
    int size1 = measured_ch0.col(0).n_rows;
    double t1 = measured_ch0(size1 - 1, 0);
    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    // conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_doppler_interp;
    arma::interp1(true_tow_s, true_ch0.col(2), t, true_ch0_doppler_interp);

    arma::vec meas_ch0_doppler_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(2), t, meas_ch0_doppler_interp);

    // 2. RMSE
    arma::vec err_ch0_hz;

    // compute error
    err_ch0_hz = meas_ch0_doppler_interp - true_ch0_doppler_interp;

    arma::vec err2_ch0 = arma::square(err_ch0_hz);
    double rmse_ch0 = sqrt(arma::mean(err2_ch0));

    // 3. Mean err and variance
    double error_mean_ch0 = arma::mean(err_ch0_hz);
    double error_var_ch0 = arma::var(err_ch0_hz);

    // 4. Peaks
    double max_error_ch0 = arma::max(err_ch0_hz);
    double min_error_ch0 = arma::min(err_ch0_hz);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << data_title << "Carrier Doppler RMSE = "
              << rmse_ch0 << ", mean = " << error_mean_ch0
              << ", stdev = " << sqrt(error_var_ch0)
              << " (max,min) = " << max_error_ch0
              << "," << min_error_ch0
              << " [Hz]\n";
    std::cout.precision(ss);

    // plots
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_show_plots)
#else
    if (absl::GetFlag(FLAGS_show_plots))
#endif
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Carrier Doppler error [Hz]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Carrier Doppler error [Hz]");
            // conversion between arma::vec and std:vector
            std::vector<double> error_vec(err_ch0_hz.colptr(0), err_ch0_hz.colptr(0) + err_ch0_hz.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, error_vec,
                "Carrier Doppler error");
            g3.set_legend();
            g3.savetops(data_title + "Carrier_doppler_error");

            g3.showonscreen();  // window output
        }

    // check results against the test tolerance
    ASSERT_LT(error_mean_ch0, 5);
    ASSERT_GT(error_mean_ch0, -5);
    // assuming PLL BW=35
    ASSERT_LT(error_var_ch0, 200);
    ASSERT_LT(max_error_ch0, 70);
    ASSERT_GT(min_error_ch0, -70);
    ASSERT_LT(rmse_ch0, 30);
}


void HybridObservablesTestFpga::check_results_duplicated_satellite(
    arma::mat& measured_sat1,
    arma::mat& measured_sat2,
    int ch_id,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times

    // define the common measured time interval
    double t0_sat1 = measured_sat1(0, 0);
    int size1 = measured_sat1.col(0).n_rows;
    double t1_sat1 = measured_sat1(size1 - 1, 0);

    double t0_sat2 = measured_sat2(0, 0);
    int size2 = measured_sat2.col(0).n_rows;
    double t1_sat2 = measured_sat2(size2 - 1, 0);

    double t0;
    double t1;
    if (t0_sat1 > t0_sat2)
        {
            t0 = t0_sat1;
        }
    else
        {
            t0 = t0_sat2;
        }

    if (t1_sat1 > t1_sat2)
        {
            t1 = t1_sat2;
        }
    else
        {
            t1 = t1_sat1;
        }

    if ((t1 - t0) > 0)
        {
            arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
            // conversion between arma::vec and std:vector
            arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
            std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);
            // Doppler
            arma::vec meas_sat1_doppler_interp;
            arma::interp1(measured_sat1.col(0), measured_sat1.col(2), t, meas_sat1_doppler_interp);
            arma::vec meas_sat2_doppler_interp;
            arma::interp1(measured_sat2.col(0), measured_sat2.col(2), t, meas_sat2_doppler_interp);

            // Carrier Phase
            arma::vec meas_sat1_carrier_phase_interp;
            arma::vec meas_sat2_carrier_phase_interp;
            arma::interp1(measured_sat1.col(0), measured_sat1.col(3), t, meas_sat1_carrier_phase_interp);
            arma::interp1(measured_sat2.col(0), measured_sat2.col(3), t, meas_sat2_carrier_phase_interp);

            // generate double difference accumulated carrier phases
            // compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
            arma::vec delta_measured_carrier_phase_cycles = (meas_sat1_carrier_phase_interp - meas_sat1_carrier_phase_interp(0)) - (meas_sat2_carrier_phase_interp - meas_sat2_carrier_phase_interp(0));

            // Pseudoranges
            arma::vec meas_sat1_dist_interp;
            arma::vec meas_sat2_dist_interp;
            arma::interp1(measured_sat1.col(0), measured_sat1.col(4), t, meas_sat1_dist_interp);
            arma::interp1(measured_sat2.col(0), measured_sat2.col(4), t, meas_sat2_dist_interp);
            // generate delta pseudoranges
            arma::vec delta_measured_dist_m = meas_sat1_dist_interp - meas_sat2_dist_interp;

            // Carrier Doppler error
            // 2. RMSE
            arma::vec err_ch0_hz;

            // compute error
            err_ch0_hz = meas_sat1_doppler_interp - meas_sat2_doppler_interp;

            // save matlab file for further analysis
            std::vector<double> tmp_vector_common_time_s(t.colptr(0),
                t.colptr(0) + t.n_rows);

            std::vector<double> tmp_vector_err_ch0_hz(err_ch0_hz.colptr(0),
                err_ch0_hz.colptr(0) + err_ch0_hz.n_rows);
            save_mat_xy(tmp_vector_common_time_s, tmp_vector_err_ch0_hz, std::string("measured_doppler_error_ch_" + std::to_string(ch_id)));

            // compute statistics
            arma::vec err2_ch0 = arma::square(err_ch0_hz);
            double rmse_ch0 = sqrt(arma::mean(err2_ch0));

            // 3. Mean err and variance
            double error_mean_ch0 = arma::mean(err_ch0_hz);
            double error_var_ch0 = arma::var(err_ch0_hz);

            // 4. Peaks
            double max_error_ch0 = arma::max(err_ch0_hz);
            double min_error_ch0 = arma::min(err_ch0_hz);

            // 5. report
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Carrier Doppler RMSE = "
                      << rmse_ch0 << ", mean = " << error_mean_ch0
                      << ", stdev = " << sqrt(error_var_ch0)
                      << " (max,min) = " << max_error_ch0
                      << "," << min_error_ch0
                      << " [Hz]\n";
            std::cout.precision(ss);

            // plots
#if USE_GLOG_AND_GFLAGS
            if (FLAGS_show_plots)
#else
            if (absl::GetFlag(FLAGS_show_plots))
#endif
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Carrier Doppler error [Hz]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Carrier Doppler error [Hz]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> error_vec(err_ch0_hz.colptr(0), err_ch0_hz.colptr(0) + err_ch0_hz.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, error_vec,
                        "Carrier Doppler error");
                    g3.set_legend();
                    g3.savetops(data_title + "Carrier_doppler_error");

                    g3.showonscreen();  // window output
                }

            // check results against the test tolerance
            EXPECT_LT(error_mean_ch0, 5);
            EXPECT_GT(error_mean_ch0, -5);
            // assuming PLL BW=35
            EXPECT_LT(error_var_ch0, 250);
            EXPECT_LT(max_error_ch0, 100);
            EXPECT_GT(min_error_ch0, -100);
            EXPECT_LT(rmse_ch0, 30);

            // Carrier Phase error
            // 2. RMSE
            arma::vec err_carrier_phase;

            err_carrier_phase = delta_measured_carrier_phase_cycles;

            // save matlab file for further analysis
            std::vector<double> tmp_vector_err_carrier_phase(err_carrier_phase.colptr(0),
                err_carrier_phase.colptr(0) + err_carrier_phase.n_rows);
            save_mat_xy(tmp_vector_common_time_s, tmp_vector_err_carrier_phase, std::string("measured_carrier_phase_error_ch_" + std::to_string(ch_id)));

            arma::vec err2_carrier_phase = arma::square(err_carrier_phase);
            double rmse_carrier_phase = sqrt(arma::mean(err2_carrier_phase));

            // 3. Mean err and variance
            double error_mean_carrier_phase = arma::mean(err_carrier_phase);
            double error_var_carrier_phase = arma::var(err_carrier_phase);

            // 4. Peaks
            double max_error_carrier_phase = arma::max(err_carrier_phase);
            double min_error_carrier_phase = arma::min(err_carrier_phase);

            // 5. report
            ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Carrier Phase RMSE = "
                      << rmse_carrier_phase << ", mean = " << error_mean_carrier_phase
                      << ", stdev = " << sqrt(error_var_carrier_phase)
                      << " (max,min) = " << max_error_carrier_phase
                      << "," << min_error_carrier_phase
                      << " [Cycles]\n";
            std::cout.precision(ss);

            // plots
#if USE_GLOG_AND_GFLAGS
            if (FLAGS_show_plots)
#else
            if (absl::GetFlag(FLAGS_show_plots))
#endif
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Carrier Phase error [Cycles]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Carrier Phase error [Cycles]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err_carrier_phase.colptr(0), err_carrier_phase.colptr(0) + err_carrier_phase.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Carrier Phase error");
                    g3.set_legend();
                    g3.savetops(data_title + "duplicated_satellite_carrier_phase_error");

                    g3.showonscreen();  // window output
                }

            // check results against the test tolerance
            EXPECT_LT(rmse_carrier_phase, 0.25);
            EXPECT_LT(error_mean_carrier_phase, 0.2);
            EXPECT_GT(error_mean_carrier_phase, -0.2);
            EXPECT_LT(error_var_carrier_phase, 0.5);
            EXPECT_LT(max_error_carrier_phase, 0.5);
            EXPECT_GT(min_error_carrier_phase, -0.5);

            // Pseudorange error
            // 2. RMSE
            arma::vec err_pseudorange;

            err_pseudorange = delta_measured_dist_m;

            // save matlab file for further analysis
            std::vector<double> tmp_vector_err_pseudorange(err_pseudorange.colptr(0),
                err_pseudorange.colptr(0) + err_pseudorange.n_rows);
            save_mat_xy(tmp_vector_common_time_s, tmp_vector_err_pseudorange, std::string("measured_pr_error_ch_" + std::to_string(ch_id)));

            arma::vec err2_pseudorange = arma::square(err_pseudorange);
            double rmse_pseudorange = sqrt(arma::mean(err2_pseudorange));

            // 3. Mean err and variance
            double error_mean_pseudorange = arma::mean(err_pseudorange);
            double error_var_pseudorange = arma::var(err_pseudorange);

            // 4. Peaks
            double max_error_pseudorange = arma::max(err_pseudorange);
            double min_error_pseudorange = arma::min(err_pseudorange);

            // 5. report
            ss = std::cout.precision();
            std::cout << std::setprecision(10) << data_title << "Pseudorange RMSE = "
                      << rmse_pseudorange << ", mean = " << error_mean_pseudorange
                      << ", stdev = " << sqrt(error_var_pseudorange)
                      << " (max,min) = " << max_error_pseudorange
                      << "," << min_error_pseudorange
                      << " [meters]\n";
            std::cout.precision(ss);

            // plots
#if USE_GLOG_AND_GFLAGS
            if (FLAGS_show_plots)
#else
            if (absl::GetFlag(FLAGS_show_plots))
#endif
                {
                    Gnuplot g3("linespoints");
                    g3.set_title(data_title + "Pseudorange error [m]");
                    g3.set_grid();
                    g3.set_xlabel("Time [s]");
                    g3.set_ylabel("Pseudorange error [m]");
                    // conversion between arma::vec and std:vector
                    std::vector<double> range_error_m(err_pseudorange.colptr(0), err_pseudorange.colptr(0) + err_pseudorange.n_rows);
                    g3.cmd("set key box opaque");
                    g3.plot_xy(time_vector, range_error_m,
                        "Pseudorrange error");
                    g3.set_legend();
                    g3.savetops(data_title + "duplicated_satellite_pseudorrange_error");

                    g3.showonscreen();  // window output
                }

            // check results against the test tolerance
            EXPECT_LT(rmse_pseudorange, 3.0);
            EXPECT_LT(error_mean_pseudorange, 1.0);
            EXPECT_GT(error_mean_pseudorange, -1.0);
            EXPECT_LT(error_var_pseudorange, 10.0);
            EXPECT_LT(max_error_pseudorange, 15.0);
            EXPECT_GT(min_error_pseudorange, -15.0);
        }
}


bool HybridObservablesTestFpga::save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.append(".mat");
            std::cout << "save_mat_xy write " << filename << '\n';
            matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT5);
            if (reinterpret_cast<int64_t*>(matfp) != nullptr)
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
                    std::cout << "save_mat_xy: error creating file\n";
                }
            Mat_Close(matfp);
            return true;
        }
    catch (const std::exception& ex)
        {
            std::cout << "save_mat_xy: " << ex.what() << '\n';
            return false;
        }
}


void HybridObservablesTestFpga::check_results_code_pseudorange(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_ch0_s,
    arma::vec& true_tow_ch1_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
{
    // 1. True value interpolation to match the measurement times
    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    // conversion between arma::vec and std:vector
    arma::vec t_from_start = arma::linspace<arma::vec>(0, t1 - t0, floor((t1 - t0) * 1e3));
    std::vector<double> time_vector(t_from_start.colptr(0), t_from_start.colptr(0) + t_from_start.n_rows);

    arma::vec true_ch0_dist_interp;
    arma::vec true_ch1_dist_interp;
    arma::interp1(true_tow_ch0_s, true_ch0.col(1), t, true_ch0_dist_interp);
    arma::interp1(true_tow_ch1_s, true_ch1.col(1), t, true_ch1_dist_interp);

    arma::vec meas_ch0_dist_interp;
    arma::vec meas_ch1_dist_interp;
    arma::interp1(measured_ch0.col(0), measured_ch0.col(4), t, meas_ch0_dist_interp);
    arma::interp1(measured_ch1.col(0), measured_ch1.col(4), t, meas_ch1_dist_interp);

    // generate delta pseudoranges
    arma::vec delta_true_dist_m = true_ch0_dist_interp - true_ch1_dist_interp;
    arma::vec delta_measured_dist_m = meas_ch0_dist_interp - meas_ch1_dist_interp;

    // 2. RMSE
    arma::vec err;

    err = delta_measured_dist_m - delta_true_dist_m;
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
    std::cout << std::setprecision(10) << data_title << "Double diff Pseudorange RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [meters]\n";
    std::cout.precision(ss);

// plots
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_show_plots)
#else
    if (absl::GetFlag(FLAGS_show_plots))
#endif
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Pseudorange error [m]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Pseudorange error [m]");
            // conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Pseudorrange error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_pseudorrange_error");

            g3.showonscreen();  // window output
        }

    // check results against the test tolerance
    ASSERT_LT(rmse, 3.0);
    ASSERT_LT(error_mean, 1.0);
    ASSERT_GT(error_mean, -1.0);
    ASSERT_LT(error_var, 10.0);
    ASSERT_LT(max_error, 10.0);
    ASSERT_GT(min_error, -10.0);
}


bool HybridObservablesTestFpga::ReadRinexObs(std::vector<arma::mat>* obs_vec, Gnss_Synchro gnss)
{
    // Open and read reference RINEX observables file
    try
        {
#if USE_GLOG_AND_GFLAGS
            gnsstk::Rinex3ObsStream r_ref(FLAGS_filename_rinex_obs);
#else
            gnsstk::Rinex3ObsStream r_ref(absl::GetFlag(FLAGS_filename_rinex_obs));
#endif
            r_ref.exceptions(std::ios::failbit);
            gnsstk::Rinex3ObsData r_ref_data;
            gnsstk::Rinex3ObsHeader r_ref_header;

            gnsstk::RinexDatum dataobj;

            r_ref >> r_ref_header;

            std::vector<bool> first_row;
            gnsstk::SatID prn;
            for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
                {
                    first_row.push_back(true);
                    obs_vec->push_back(arma::zeros<arma::mat>(1, 4));
                }
            while (r_ref >> r_ref_data)
                {
                    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
                        {
                            int myprn = gnss_synchro_vec.at(n).PRN;

                            switch (gnss.System)
                                {
                                case 'G':
#if OLD_GPSTK
                                    prn = gnsstk::SatID(myprn, gnsstk::SatID::systemGPS);
#else
                                    prn = gnsstk::SatID(myprn, gnsstk::SatelliteSystem::GPS);
#endif
                                    break;
                                case 'E':
#if OLD_GPSTK
                                    prn = gnsstk::SatID(myprn, gnsstk::SatID::systemGalileo);
#else
                                    prn = gnsstk::SatID(myprn, gnsstk::SatelliteSystem::Galileo);
#endif
                                    break;
                                default:
#if OLD_GPSTK
                                    prn = gnsstk::SatID(myprn, gnsstk::SatID::systemGPS);
#else
                                    prn = gnsstk::SatID(myprn, gnsstk::SatelliteSystem::GPS);
#endif
                                }

                            gnsstk::CommonTime time = r_ref_data.time;
#if GNSSTK_OLDER_THAN_9
                            double sow(static_cast<gnsstk::GPSWeekSecond>(time).sow);
#else
                            gnsstk::GPSWeekSecond gws(time);
                            double sow(gws.getSOW());
#endif

                            auto pointer = r_ref_data.obs.find(prn);
                            if (pointer == r_ref_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    if (first_row.at(n) == false)
                                        {
                                            // insert next column
                                            obs_vec->at(n).insert_rows(obs_vec->at(n).n_rows, 1);
                                        }
                                    else
                                        {
                                            first_row.at(n) = false;
                                        }
                                    if (strcmp("1C\0", gnss.Signal) == 0)
                                        {
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C1C", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;  // C1C P1 (psudorange L1)
                                            dataobj = r_ref_data.getObs(prn, "D1C", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;  // D1C Carrier Doppler
                                            dataobj = r_ref_data.getObs(prn, "L1C", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;  // L1C Carrier Phase
                                        }
                                    else if (strcmp("1B\0", gnss.Signal) == 0)
                                        {
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C1B", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D1B", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L1B", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("2S\0", gnss.Signal) == 0)  // L2M
                                        {
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C2S", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D2S", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L2S", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("L5\0", gnss.Signal) == 0)
                                        {
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C5I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D5I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L5I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;
                                        }
                                    else if (strcmp("5X\0", gnss.Signal) == 0)  // Simulator gives RINEX with E5a+E5b. Doppler and accumulated Carrier phase WILL differ
                                        {
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 0) = sow;
                                            dataobj = r_ref_data.getObs(prn, "C8I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "D8I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;
                                            dataobj = r_ref_data.getObs(prn, "L8I", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;
                                        }
                                    else
                                        {
                                            std::cout << "ReadRinexObs unknown signal requested: " << gnss.Signal << '\n';
                                            return false;
                                        }
                                }
                        }
                }  // end while
        }          // End of 'try' block

    catch (const gnsstk::FFStreamError& e)
        {
            std::cout << e;
            return false;
        }
    catch (const gnsstk::Exception& e)
        {
            std::cout << e;
            return false;
        }
    catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what();
            std::cout << "unknown error.  I don't feel so well...\n";
            return false;
        }
    std::cout << "ReadRinexObs info:\n";

    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
        {
            std::cout << "SAT PRN " << gnss_synchro_vec.at(n).PRN << " RINEX epoch read: " << obs_vec->at(n).n_rows << '\n';
        }
    return true;
}


TEST_F(HybridObservablesTestFpga, ValidationOfResults)
{
    // pointer to the DMA thread that sends the samples to the acquisition engine
    pthread_t thread_DMA;

    struct DMA_handler_args_obs_test args;

    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_disable_generator == false)
#else
    if (absl::GetFlag(FLAGS_disable_generator) == false)
#endif
        {
            generate_signal();
        }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

// use generator or use an external capture file
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_enable_external_signal_file)
#else
    if (absl::GetFlag(FLAGS_enable_external_signal_file))
#endif
        {
            // create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(), true);
        }
    else
        {
            Gnss_Synchro tmp_gnss_synchro;
            tmp_gnss_synchro.System = 'G';
            std::string signal = "1C";
            signal.copy(tmp_gnss_synchro.Signal, 2, 0);

#if USE_GLOG_AND_GFLAGS
            std::istringstream ss(FLAGS_test_satellite_PRN_list);
#else
            std::istringstream ss(absl::GetFlag(FLAGS_test_satellite_PRN_list));
#endif
            std::string token;

            while (std::getline(ss, token, ','))
                {
                    tmp_gnss_synchro.PRN = boost::lexical_cast<int>(token);
                    gnss_synchro_vec.push_back(tmp_gnss_synchro);
                }
        }
#if USE_GLOG_AND_GFLAGS
    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols,
        FLAGS_smoother_length,
        FLAGS_high_dyn);
#else
    configure_receiver(absl::GetFlag(FLAGS_PLL_bw_hz_start),
        absl::GetFlag(FLAGS_DLL_bw_hz_start),
        absl::GetFlag(FLAGS_PLL_narrow_bw_hz),
        absl::GetFlag(FLAGS_DLL_narrow_bw_hz),
        absl::GetFlag(FLAGS_extend_correlation_symbols),
        absl::GetFlag(FLAGS_smoother_length),
        absl::GetFlag(FLAGS_high_dyn));
#endif

    for (auto& n : gnss_synchro_vec)
        {
            // setup the signal synchronization, simulating an acquisition
#if USE_GLOG_AND_GFLAGS
            if (!FLAGS_enable_external_signal_file)
#else
            if (!absl::GetFlag(FLAGS_enable_external_signal_file))
#endif
                {
                    // based on true observables metadata (for custom sdr generator)
                    // open true observables log file written by the simulator or based on provided RINEX obs
                    std::vector<std::shared_ptr<Tracking_True_Obs_Reader>> true_reader_vec;
                    // read true data from the generator logs
                    true_reader_vec.push_back(std::make_shared<Tracking_True_Obs_Reader>());
                    std::cout << "Loading true observable data for PRN " << n.PRN << '\n';
                    std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
                    true_obs_file.append(std::to_string(n.PRN));
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

                    // restart the epoch counter
                    true_reader_vec.back()->restart();

                    std::cout << "Initial Doppler [Hz]=" << true_reader_vec.back()->doppler_l1_hz << " Initial code delay [Chips]="
                              << true_reader_vec.back()->prn_delay_chips << '\n';
                    n.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_reader_vec.back()->prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD_S;
                    n.Acq_doppler_hz = true_reader_vec.back()->doppler_l1_hz;
                    n.Acq_samplestamp_samples = 0;
                }
            else
                {
                    // based on the signal acquisition process
                    std::cout << "Estimated Initial Doppler " << n.Acq_doppler_hz
                              << " [Hz], estimated Initial code delay " << n.Acq_delay_samples << " [Samples]"
                              << " Acquisition SampleStamp is " << n.Acq_samplestamp_samples << '\n';
                    // n.Acq_samplestamp_samples = 0;
                }
        }

    // We need to reset the HW again in order to reset the sample counter.
    // The HW is reset by sending a command to the acquisition HW accelerator
    // In order to send the reset command to the HW we instantiate the acquisition module.
    std::shared_ptr<AcquisitionInterface> acquisition;

    // set the scaling factor
    args.scaling_factor = DMA_SIGNAL_SCALING_FACTOR;

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

    std::vector<std::shared_ptr<TrackingInterface>> tracking_ch_vec;
    std::vector<std::shared_ptr<TelemetryDecoderInterface>> tlm_ch_vec;

    std::vector<gr::blocks::null_sink::sptr> null_sink_vec;
    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
        {
            // set channels ids
            gnss_synchro_vec.at(n).Channel_ID = n;

            // create the tracking channels and create the telemetry decoders
            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config.get(), "Tracking", 1, 1);
            tracking_ch_vec.push_back(std::dynamic_pointer_cast<TrackingInterface>(trk_));
            std::shared_ptr<GNSSBlockInterface> tlm_ = factory->GetBlock(config.get(), "TelemetryDecoder", 1, 1);
            tlm_ch_vec.push_back(std::dynamic_pointer_cast<TelemetryDecoderInterface>(tlm_));

            // create null sinks for observables output
            null_sink_vec.push_back(gr::blocks::null_sink::make(sizeof(Gnss_Synchro)));

            ASSERT_NO_THROW({
                tlm_ch_vec.back()->set_channel(gnss_synchro_vec.at(n).Channel_ID);

                switch (gnss_synchro_master.System)
                    {
                    case 'G':
                        tlm_ch_vec.back()->set_satellite(Gnss_Satellite(std::string("GPS"), gnss_synchro_vec.at(n).PRN));
                        break;
                    case 'E':
                        tlm_ch_vec.back()->set_satellite(Gnss_Satellite(std::string("Galileo"), gnss_synchro_vec.at(n).PRN));
                        break;
                    default:
                        tlm_ch_vec.back()->set_satellite(Gnss_Satellite(std::string("GPS"), gnss_synchro_vec.at(n).PRN));
                    }
            }) << "Failure setting gnss_synchro.";

            ASSERT_NO_THROW({
                tracking_ch_vec.back()->set_channel(gnss_synchro_vec.at(n).Channel_ID);
            }) << "Failure setting channel.";

            ASSERT_NO_THROW({
                tracking_ch_vec.back()->set_gnss_synchro(&gnss_synchro_vec.at(n));
            }) << "Failure setting gnss_synchro.";
        }

    top_block = gr::make_top_block("Telemetry_Decoder test");
    auto dummy_msg_rx_trk = HybridObservablesTest_msg_rx_Fpga_make();
    auto dummy_tlm_msg_rx = HybridObservablesTest_tlm_msg_rx_Fpga_make();
    // Observables
    std::shared_ptr<ObservablesInterface> observables(new HybridObservables(config.get(), "Observables", tracking_ch_vec.size() + 1, tracking_ch_vec.size()));

    for (auto& n : tracking_ch_vec)
        {
            ASSERT_NO_THROW({
                n->connect(top_block);
            }) << "Failure connecting tracking to the top_block.";
        }

    std::string file;

    ASSERT_NO_THROW({
#if USE_GLOG_AND_GFLAGS
        if (!FLAGS_enable_external_signal_file)
            {
                file = "./" + filename_raw_data;
            }
        else
            {
                file = FLAGS_signal_file;
            }
#else
        if (!absl::GetFlag(FLAGS_enable_external_signal_file))
            {
                file = "./" + filename_raw_data;
            }
        else
            {
                file = absl::GetFlag(FLAGS_signal_file);
            }
#endif
        int observable_interval_ms = 20;

        double fs = static_cast<double>(config->property("GNSS-SDR.internal_fs_sps", 0));

        gnss_sdr_fpga_sample_counter_sptr ch_out_fpga_sample_counter;
        ch_out_fpga_sample_counter = gnss_sdr_make_fpga_sample_counter(fs, observable_interval_ms);

        for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
            {
                // top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tracking_ch_vec.at(n)->get_right_block(), 0, tlm_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tlm_ch_vec.at(n)->get_right_block(), 0, observables->get_left_block(), n);
                top_block->msg_connect(tracking_ch_vec.at(n)->get_right_block(), pmt::mp("events"), dummy_msg_rx_trk, pmt::mp("events"));
                top_block->connect(observables->get_right_block(), n, null_sink_vec.at(n), 0);
            }
        // connect sample counter and timmer to the last channel in observables block (extra channel)
        top_block->connect(ch_out_fpga_sample_counter, 0, observables->get_left_block(), tracking_ch_vec.size());  // extra port for the sample counter pulse
    }) << "Failure connecting the blocks.";

    top_block->start();

    usleep(1000000);  // give time for the system to start before receiving the start tracking command.

    for (auto& n : tracking_ch_vec)
        {
            n->start_tracking();
        }

    // wait to give time for the acquisition thread to set up the tracking process
    usleep(1000000);

    args.file = file;
#if USE_GLOG_AND_GFLAGS
    args.nsamples_tx = baseband_sampling_freq * FLAGS_duration;
#else
    args.nsamples_tx = baseband_sampling_freq * absl::GetFlag(FLAGS_duration);
#endif

    args.skip_used_samples = 0;

    if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, reinterpret_cast<void*>(&args)) < 0)
        {
            std::cout << "ERROR cannot create DMA Process\n";
        }

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
    }) << "Failure running the top_block.";

    // wait for the child DMA process to finish
    pthread_join(thread_DMA, nullptr);

    // stop the top block
    top_block->stop();

    // stop the tracking process:
    for (auto& n : tracking_ch_vec)
        {
            ASSERT_NO_THROW({
                n->stop_tracking();
            }) << "Failure connecting tracking to the top_block.";
        }

    acquisition->stop_acquisition();

    EXPECT_NO_THROW({
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    // check results
    // Matrices for storing columnwise true GPS time, Range, Doppler and Carrier phase
    std::vector<arma::mat> true_obs_vec;
#if USE_GLOG_AND_GFLAGS
    if (!FLAGS_enable_external_signal_file)
#else
    if (!absl::GetFlag(FLAGS_enable_external_signal_file))
#endif
        {
            // load the true values
            True_Observables_Reader true_observables;
            ASSERT_NO_THROW({
                if (true_observables.open_obs_file(std::string("./obs_out.bin")) == false)
                    {
                        throw std::exception();
                    }
            }) << "Failure opening true observables file";

            auto nepoch = static_cast<unsigned int>(true_observables.num_epochs());

            std::cout << "True observation epochs = " << nepoch << '\n';

            true_observables.restart();
            int64_t epoch_counter = 0;
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
                                                  << round(true_observables.prn[n]) << " vs. " << gnss_synchro_vec.at(n).PRN << '\n';
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
        }
    else
        {
#if USE_GLOG_AND_GFLAGS
            if (!FLAGS_duplicated_satellites_test)
#else
            if (!absl::GetFlag(FLAGS_duplicated_satellites_test))
#endif
                {
                    ASSERT_EQ(ReadRinexObs(&true_obs_vec, gnss_synchro_master), true)
                        << "Failure reading RINEX file";
                }
        }

    // read measured values
    Observables_Dump_Reader estimated_observables(tracking_ch_vec.size());
    ASSERT_NO_THROW({
        if (estimated_observables.open_obs_file(std::string("./observables.dat")) == false)
            {
                throw std::exception();
            }
    }) << "Failure opening dump observables file";

    auto nepoch = static_cast<unsigned int>(estimated_observables.num_epochs());
    std::cout << "Measured observations epochs = " << nepoch << '\n';

    // Matrices for storing columnwise measured RX_time, TOW, Doppler, Carrier phase and Pseudorange
    std::vector<arma::mat> measured_obs_vec;
    std::vector<int64_t> epoch_counters_vec;
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

    // Cut measurement tail zeros
    arma::uvec index;
    for (auto& n : measured_obs_vec)
        {
            index = arma::find(n.col(0) > 0.0, 1, "last");
            if ((!index.empty()) and index(0) < (nepoch - 1))
                {
                    n.shed_rows(index(0) + 1, nepoch - 1);
                }
        }

// Cut measurement initial transitory of the measurements
#if USE_GLOG_AND_GFLAGS
    double initial_transitory_s = FLAGS_skip_obs_transitory_s;
#else
    double initial_transitory_s = absl::GetFlag(FLAGS_skip_obs_transitory_s);
#endif
    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            index = arma::find(measured_obs_vec.at(n).col(0) >= (measured_obs_vec.at(n)(0, 0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    measured_obs_vec.at(n).shed_rows(0, index(0));
                }
#if USE_GLOG_AND_GFLAGS
            if (!FLAGS_duplicated_satellites_test)
#else
            if (!absl::GetFlag(FLAGS_duplicated_satellites_test))
#endif
                {
                    index = arma::find(measured_obs_vec.at(n).col(0) >= true_obs_vec.at(n)(0, 0), 1, "first");
                    if ((!index.empty()) and (index(0) > 0))
                        {
                            measured_obs_vec.at(n).shed_rows(0, index(0));
                        }
                }
        }
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_duplicated_satellites_test)
#else
    if (absl::GetFlag(FLAGS_duplicated_satellites_test))
#endif
        {
            // special test mode for duplicated satellites
            std::vector<unsigned int> prn_pairs;
#if USE_GLOG_AND_GFLAGS
            std::stringstream ss(FLAGS_duplicated_satellites_prns);
#else
            std::stringstream ss(absl::GetFlag(FLAGS_duplicated_satellites_prns));
#endif
            unsigned int i;
            while (ss >> i)
                {
                    prn_pairs.push_back(i);
                    if (ss.peek() == ',')
                        {
                            ss.ignore();
                        }
                }

            if (prn_pairs.size() % 2 != 0)
                {
                    std::cout << "Test settings error: duplicated_satellites_prns are even\n";
                }
            else
                {
                    for (unsigned int n = 0; n < prn_pairs.size(); n = n + 2)
                        {
                            int sat1_ch_id = -1;
                            int sat2_ch_id = -1;
                            for (unsigned int ch = 0; ch < measured_obs_vec.size(); ch++)
                                {
                                    if (epoch_counters_vec.at(ch) > 100)  // discard non-valid channels
                                        {
                                            if (gnss_synchro_vec.at(ch).PRN == prn_pairs.at(n))
                                                {
                                                    sat1_ch_id = ch;
                                                }
                                            if (gnss_synchro_vec.at(ch).PRN == prn_pairs.at(n + 1))
                                                {
                                                    sat2_ch_id = ch;
                                                }
                                        }
                                }
                            if (sat1_ch_id != -1 and sat2_ch_id != -1)
                                {
                                    // compute single differences for the duplicated satellite

                                    check_results_duplicated_satellite(
                                        measured_obs_vec.at(sat1_ch_id),
                                        measured_obs_vec.at(sat2_ch_id),
                                        sat1_ch_id,
                                        "Duplicated sat [CH " + std::to_string(sat1_ch_id) + "," + std::to_string(sat2_ch_id) + "] PRNs " + std::to_string(gnss_synchro_vec.at(sat1_ch_id).PRN) + "," + std::to_string(gnss_synchro_vec.at(sat2_ch_id).PRN) + " ");
                                }
                            else
                                {
                                    std::cout << "Satellites PRNs " << prn_pairs.at(n) << "and " << prn_pairs.at(n) << " not found\n";
                                }
                        }
                }
        }
    else
        {
            // normal mode

            // Correct the clock error using true values (it is not possible for a receiver to correct
            // the receiver clock offset error at the observables level because it is required the
            // decoding of the ephemeris data and solve the PVT equations)

            // Find the reference satellite (the nearest) and compute the receiver time offset at observable level
            double min_pr = std::numeric_limits<double>::max();
            unsigned int min_pr_ch_id = 0;
            for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
                {
                    if (epoch_counters_vec.at(n) > 100)  // discard non-valid channels
                        {
                            {
                                if (measured_obs_vec.at(n)(0, 4) < min_pr)
                                    {
                                        min_pr = measured_obs_vec.at(n)(0, 4);
                                        min_pr_ch_id = n;
                                    }
                            }
                        }
                    else
                        {
                            std::cout << "PRN " << gnss_synchro_vec.at(n).PRN << " has NO observations!\n";
                        }
                }

            arma::vec receiver_time_offset_ref_channel_s;
            arma::uvec index2;
            index2 = arma::find(true_obs_vec.at(min_pr_ch_id).col(0) >= measured_obs_vec.at(min_pr_ch_id).col(0)(0), 1, "first");
            if ((!index2.empty()) and (index2(0) > 0))
                {
                    receiver_time_offset_ref_channel_s = (true_obs_vec.at(min_pr_ch_id).col(1)(index2(0)) - measured_obs_vec.at(min_pr_ch_id).col(4)(0)) / SPEED_OF_LIGHT_M_S;
                    std::cout << "Ref. channel initial Receiver time offset " << receiver_time_offset_ref_channel_s(0) * 1e3 << " [ms]\n";
                }
            else
                {
                    ASSERT_NO_THROW(
                        throw std::exception();)
                        << "Error finding observation time epoch in the reference data";
                }

            for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
                {
                    // debug save to .mat
                    std::vector<double> tmp_vector_x(true_obs_vec.at(n).col(0).colptr(0),
                        true_obs_vec.at(n).col(0).colptr(0) + true_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y(true_obs_vec.at(n).col(1).colptr(0),
                        true_obs_vec.at(n).col(1).colptr(0) + true_obs_vec.at(n).col(1).n_rows);
                    save_mat_xy(tmp_vector_x, tmp_vector_y, std::string("true_pr_ch_" + std::to_string(n)));

                    std::vector<double> tmp_vector_x2(measured_obs_vec.at(n).col(0).colptr(0),
                        measured_obs_vec.at(n).col(0).colptr(0) + measured_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y2(measured_obs_vec.at(n).col(4).colptr(0),
                        measured_obs_vec.at(n).col(4).colptr(0) + measured_obs_vec.at(n).col(4).n_rows);
                    save_mat_xy(tmp_vector_x2, tmp_vector_y2, std::string("measured_pr_ch_" + std::to_string(n)));

                    std::vector<double> tmp_vector_x3(true_obs_vec.at(n).col(0).colptr(0),
                        true_obs_vec.at(n).col(0).colptr(0) + true_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y3(true_obs_vec.at(n).col(2).colptr(0),
                        true_obs_vec.at(n).col(2).colptr(0) + true_obs_vec.at(n).col(2).n_rows);
                    save_mat_xy(tmp_vector_x3, tmp_vector_y3, std::string("true_doppler_ch_" + std::to_string(n)));

                    std::vector<double> tmp_vector_x4(measured_obs_vec.at(n).col(0).colptr(0),
                        measured_obs_vec.at(n).col(0).colptr(0) + measured_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y4(measured_obs_vec.at(n).col(2).colptr(0),
                        measured_obs_vec.at(n).col(2).colptr(0) + measured_obs_vec.at(n).col(2).n_rows);
                    save_mat_xy(tmp_vector_x4, tmp_vector_y4, std::string("measured_doppler_ch_" + std::to_string(n)));

                    std::vector<double> tmp_vector_x5(true_obs_vec.at(n).col(0).colptr(0),
                        true_obs_vec.at(n).col(0).colptr(0) + true_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y5(true_obs_vec.at(n).col(3).colptr(0),
                        true_obs_vec.at(n).col(3).colptr(0) + true_obs_vec.at(n).col(3).n_rows);
                    save_mat_xy(tmp_vector_x5, tmp_vector_y5, std::string("true_cp_ch_" + std::to_string(n)));

                    std::vector<double> tmp_vector_x6(measured_obs_vec.at(n).col(0).colptr(0),
                        measured_obs_vec.at(n).col(0).colptr(0) + measured_obs_vec.at(n).col(0).n_rows);
                    std::vector<double> tmp_vector_y6(measured_obs_vec.at(n).col(3).colptr(0),
                        measured_obs_vec.at(n).col(3).colptr(0) + measured_obs_vec.at(n).col(3).n_rows);
                    save_mat_xy(tmp_vector_x6, tmp_vector_y6, std::string("measured_cp_ch_" + std::to_string(n)));

                    if (epoch_counters_vec.at(n) > 100)  // discard non-valid channels
                        {
                            arma::vec true_TOW_ref_ch_s = true_obs_vec.at(min_pr_ch_id).col(0) - receiver_time_offset_ref_channel_s(0);
                            arma::vec true_TOW_ch_s = true_obs_vec.at(n).col(0) - receiver_time_offset_ref_channel_s(0);
                            // Compare measured observables
                            if (min_pr_ch_id != n)
                                {
                                    check_results_code_pseudorange(true_obs_vec.at(n),
                                        true_obs_vec.at(min_pr_ch_id),
                                        true_TOW_ch_s,
                                        true_TOW_ref_ch_s,
                                        measured_obs_vec.at(n),
                                        measured_obs_vec.at(min_pr_ch_id),
                                        "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
                                    // Do not compare E5a with E5 RINEX due to the Doppler frequency discrepancy caused by the different center frequencies
                                    // E5a_fc=1176.45e6, E5b_fc=1207.14e6, E5_fc=1191.795e6;
#if USE_GLOG_AND_GFLAGS
                                    if (strcmp("5X\0", gnss_synchro_vec.at(n).Signal) != 0 or FLAGS_compare_with_5X)
#else
                                    if (strcmp("5X\0", gnss_synchro_vec.at(n).Signal) != 0 or absl::GetFlag(FLAGS_compare_with_5X))
#endif
                                        {
                                            check_results_carrier_phase_double_diff(true_obs_vec.at(n),
                                                true_obs_vec.at(min_pr_ch_id),
                                                true_TOW_ch_s,
                                                true_TOW_ref_ch_s,
                                                measured_obs_vec.at(n),
                                                measured_obs_vec.at(min_pr_ch_id),
                                                "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");

                                            check_results_carrier_doppler_double_diff(true_obs_vec.at(n),
                                                true_obs_vec.at(min_pr_ch_id),
                                                true_TOW_ch_s,
                                                true_TOW_ref_ch_s,
                                                measured_obs_vec.at(n),
                                                measured_obs_vec.at(min_pr_ch_id),
                                                "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
                                        }
                                }
                            else
                                {
                                    std::cout << "[CH " << std::to_string(n) << "] PRN " << std::to_string(gnss_synchro_vec.at(n).PRN) << " is the reference satellite\n";
                                }
#if USE_GLOG_AND_GFLAGS
                            if (FLAGS_compute_single_diffs)
#else
                            if (absl::GetFlag(FLAGS_compute_single_diffs))
#endif
                                {
                                    check_results_carrier_phase(true_obs_vec.at(n),
                                        true_TOW_ch_s,
                                        measured_obs_vec.at(n),
                                        "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
                                    check_results_carrier_doppler(true_obs_vec.at(n),
                                        true_TOW_ch_s,
                                        measured_obs_vec.at(n),
                                        "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
                                }
                        }
                    else
                        {
                            std::cout << "PRN " << gnss_synchro_vec.at(n).PRN << " has NO observations!\n";
                        }
                }
        }

    std::cout << "Test completed in " << elapsed_seconds.count() << " [s]\n";
}

/*!
 * \file hybrid_observables_test_fpga.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
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

#include "GPS_L1_CA.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_sdr_fpga_sample_counter.h"
#include "gnss_synchro.h"
#include "gnuplot_i.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "gps_l1_ca_telemetry_decoder.h"
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
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/top_block.h>
#include <gpstk/Rinex3ObsBase.hpp>
#include <gpstk/Rinex3ObsData.hpp>
#include <gpstk/Rinex3ObsHeader.hpp>
#include <gpstk/Rinex3ObsStream.hpp>
#include <gpstk/RinexUtilities.hpp>
#include <gtest/gtest.h>
#include <matio.h>
#include <chrono>
#include <exception>
#include <unistd.h>

// threads
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout, endl
#include <pthread.h>   // for pthread stuff
#include <sys/mman.h>  // for mmap
#include <utility>

#define TEST_OBS_MAX_INPUT_COMPLEX_SAMPLES_TOTAL 8192  // maximum DMA sample block size in complex samples
#define TEST_OBS_COMPLEX_SAMPLE_SIZE 2                 // sample size in bytes
#define TEST_OBS_NUM_QUEUES 2                          // number of queues (1 for GPS L1/Galileo E1, and 1 for GPS L5/Galileo E5)
#define TEST_OBS_DOWNAMPLING_FILTER_INIT_SAMPLES 100   // some samples to initialize the state of the downsampling filter
#define TEST_OBS_DOWNSAMPLING_FILTER_DELAY 48


// HW related options
bool test_observables_show_results_table = false;        // 1 => show matrix of (doppler, (max value, power sum)) results (only if test_observables_doppler_control_in_sw = 1), 0=> do not show it
bool test_observables_skip_samples_already_used = true;  // if test_observables_doppler_control_in_sw = 1 and test_observables_skip_samples_already_used = 1 => for each PRN loop skip the samples used in the previous PRN loops
                                                         // (exactly in the same way as the SW)

class HybridObservablesTest_msg_rx_Fpga;

using HybridObservablesTest_msg_rx_Fpga_sptr = boost::shared_ptr<HybridObservablesTest_msg_rx_Fpga>;

HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make();

class HybridObservablesTest_msg_rx_Fpga : public gr::block
{
private:
    friend HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_msg_rx_Fpga();

public:
    int rx_message;
    ~HybridObservablesTest_msg_rx_Fpga();  //!< Default destructor
};

HybridObservablesTest_msg_rx_Fpga_sptr HybridObservablesTest_msg_rx_Fpga_make()
{
    return HybridObservablesTest_msg_rx_Fpga_sptr(new HybridObservablesTest_msg_rx_Fpga());
}

void HybridObservablesTest_msg_rx_Fpga::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

HybridObservablesTest_msg_rx_Fpga::HybridObservablesTest_msg_rx_Fpga() : gr::block("HybridObservablesTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_msg_rx_Fpga::msg_handler_events, this, _1));
    rx_message = 0;
}

HybridObservablesTest_msg_rx_Fpga::~HybridObservablesTest_msg_rx_Fpga() = default;


class HybridObservablesTest_tlm_msg_rx_Fpga;

using HybridObservablesTest_tlm_msg_rx_Fpga_sptr = boost::shared_ptr<HybridObservablesTest_tlm_msg_rx_Fpga>;

HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make();

class HybridObservablesTest_tlm_msg_rx_Fpga : public gr::block
{
private:
    friend HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_tlm_msg_rx_Fpga();

public:
    int rx_message;
    ~HybridObservablesTest_tlm_msg_rx_Fpga();  //!< Default destructor
};

HybridObservablesTest_tlm_msg_rx_Fpga_sptr HybridObservablesTest_tlm_msg_rx_Fpga_make()
{
    return HybridObservablesTest_tlm_msg_rx_Fpga_sptr(new HybridObservablesTest_tlm_msg_rx_Fpga());
}

void HybridObservablesTest_tlm_msg_rx_Fpga::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

HybridObservablesTest_tlm_msg_rx_Fpga::HybridObservablesTest_tlm_msg_rx_Fpga() : gr::block("HybridObservablesTest_tlm_msg_rx_Fpga", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_tlm_msg_rx_Fpga::msg_handler_events, this, _1));
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
    std::string implementation = FLAGS_trk_test_implementation;

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
        int extend_correlation_symbols);

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro_master;
    std::vector<Gnss_Synchro> gnss_synchro_vec;
    size_t item_size;
    pthread_mutex_t mutex_obs_test = PTHREAD_MUTEX_INITIALIZER;
};

int HybridObservablesTestFpga::configure_generator()
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


int HybridObservablesTestFpga::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], nullptr};

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


void setup_fpga_switch_obs_test(void)
{
    const size_t TEST_OBS_PAGE_SIZE = 0x10000;
    const unsigned int TEST_OBS_TEST_REGISTER_TRACK_WRITEVAL = 0x55AA;
    int switch_device_descriptor;        // driver descriptor
    volatile unsigned* switch_map_base;  // driver memory map

    if ((switch_device_descriptor = open("/dev/uio1", O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio"
                         << "/dev/uio1";
        }
    switch_map_base = reinterpret_cast<volatile unsigned*>(mmap(nullptr, TEST_OBS_PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, switch_device_descriptor, 0));

    if (switch_map_base == reinterpret_cast<void*>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA switch module into tracking memory";
            std::cout << "Could not map switch memory." << std::endl;
        }

    // sanity check : check test register
    unsigned writeval = TEST_OBS_TEST_REGISTER_TRACK_WRITEVAL;
    unsigned readval;
    // write value to test register
    switch_map_base[3] = writeval;
    // read value from test register
    readval = switch_map_base[3];

    if (writeval != readval)
        {
            LOG(WARNING) << "Test register sanity check failed";
        }
    else
        {
            LOG(INFO) << "Test register sanity check success !";
        }

    switch_map_base[0] = 0;  //0 -> DMA to queue 0, 1 -> DMA to queue 1, 2 -> A/Ds to queues
}


//static pthread_mutex_t mutex_obs_test = PTHREAD_MUTEX_INITIALIZER;

volatile unsigned int send_samples_start_obs_test = 0;

int8_t input_samples_obs_test[TEST_OBS_MAX_INPUT_COMPLEX_SAMPLES_TOTAL * TEST_OBS_COMPLEX_SAMPLE_SIZE];  // re - im
int8_t input_samples_dma_obs_test[TEST_OBS_MAX_INPUT_COMPLEX_SAMPLES_TOTAL * TEST_OBS_COMPLEX_SAMPLE_SIZE * TEST_OBS_NUM_QUEUES];

struct DMA_handler_args_obs_test
{
    std::string file;
    unsigned int nsamples_tx;
    unsigned int skip_used_samples;
    unsigned int freq_band;  // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
};

void* handler_DMA_obs_test(void* arguments)
{
    // DMA process that configures the DMA to send the samples to the acquisition engine
    int tx_fd;                           // DMA descriptor
    FILE* rx_signal_file_id;             // Input file descriptor
    bool file_completed = false;         // flag to indicate if the file is completed
    unsigned int nsamples_block;         // number of samples to send in the next DMA block of samples
    unsigned int nread_elements;         // number of elements effectively read from the input file
    unsigned int nsamples = 0;           // number of complex samples effectively transferred
    unsigned int index0, dma_index = 0;  // counters used for putting the samples in the order expected by the DMA

    unsigned int nsamples_transmitted;

    auto* args = (struct DMA_handler_args*)arguments;

    unsigned int nsamples_tx = args->nsamples_tx;
    std::string file = args->file;  // input filename
    unsigned int skip_used_samples = args->skip_used_samples;

    // open DMA device
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 0)
        {
            std::cout << "DMA can't open loop device" << std::endl;
            exit(1);
        }
    else

        // open input file
        rx_signal_file_id = fopen(file.c_str(), "rb");
    if (rx_signal_file_id == nullptr)
        {
            std::cout << "DMA can't open input file" << std::endl;
            exit(1);
        }
    while (send_samples_start_obs_test == 0)
        ;  // wait until acquisition starts
    // skip initial samples
    int skip_samples = (int)FLAGS_skip_samples;

    fseek(rx_signal_file_id, (skip_samples + skip_used_samples) * 2, SEEK_SET);

    usleep(50000);  // wait some time to give time to the main thread to start the acquisition module

    while (file_completed == false)
        {
            if (nsamples_tx - nsamples > TEST_OBS_MAX_INPUT_COMPLEX_SAMPLES_TOTAL)
                {
                    nsamples_block = TEST_OBS_MAX_INPUT_COMPLEX_SAMPLES_TOTAL;
                }
            else
                {
                    nsamples_block = nsamples_tx - nsamples;  // remaining samples to be sent
                    file_completed = true;
                }

            nread_elements = fread(input_samples_obs_test, sizeof(int8_t), nsamples_block * TEST_OBS_COMPLEX_SAMPLE_SIZE, rx_signal_file_id);

            if (nread_elements != nsamples_block * TEST_OBS_COMPLEX_SAMPLE_SIZE)
                {
                    std::cout << "file completed" << std::endl;
                    file_completed = true;
                }

            nsamples += (nread_elements / TEST_OBS_COMPLEX_SAMPLE_SIZE);

            if (nread_elements > 0)
                {
                    // for the 32-BIT DMA
                    dma_index = 0;
                    for (index0 = 0; index0 < (nread_elements); index0 += TEST_OBS_COMPLEX_SAMPLE_SIZE)
                        {
                            if (args->freq_band == 0)
                                {
                                    // channel 1 (queue 1) -> E5/L5
                                    input_samples_dma_obs_test[dma_index] = 0;
                                    input_samples_dma_obs_test[dma_index + 1] = 0;
                                    // channel 0 (queue 0) -> E1/L1
                                    input_samples_dma_obs_test[dma_index + 2] = input_samples_obs_test[index0];
                                    input_samples_dma_obs_test[dma_index + 3] = input_samples_obs_test[index0 + 1];
                                }
                            else
                                {
                                    // channel 1 (queue 1) -> E5/L5
                                    input_samples_dma_obs_test[dma_index] = input_samples_obs_test[index0];
                                    input_samples_dma_obs_test[dma_index + 1] = input_samples_obs_test[index0 + 1];
                                    // channel 0 (queue 0) -> E1/L1
                                    input_samples_dma_obs_test[dma_index + 2] = 0;
                                    input_samples_dma_obs_test[dma_index + 3] = 0;
                                }
                            dma_index += 4;
                        }
                    nsamples_transmitted = write(tx_fd, &input_samples_dma_obs_test[0], nread_elements * TEST_OBS_NUM_QUEUES);
                    if (nsamples_transmitted != nread_elements * TEST_OBS_NUM_QUEUES)
                        {
                            std::cout << "Error : DMA could not send all the requested samples" << std::endl;
                        }
                }
        }


    close(tx_fd);
    fclose(rx_signal_file_id);
    return nullptr;
}


bool HybridObservablesTestFpga::acquire_signal()
{
    pthread_t thread_DMA;

    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");
    int SV_ID = 1;  //initial sv id

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string System_and_Signal;

    struct DMA_handler_args_obs_test args;

    //create the correspondign acquisition block according to the desired tracking signal
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "1C";
            signal.copy(tmp_gnss_synchro.Signal, 2, 0);
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L1 CA";

            args.freq_band = 0;

            acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "1B";
            signal.copy(tmp_gnss_synchro.Signal, 2, 0);
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E1B";

            args.freq_band = 0;

            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }

    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "5X";
            signal.copy(tmp_gnss_synchro.Signal, 2, 0);
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";

            args.freq_band = 1;

            acquisition = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "L5";
            signal.copy(tmp_gnss_synchro.Signal, 2, 0);
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L5I";

            args.freq_band = 1;

            acquisition = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }

    acquisition->set_channel(0);
    acquisition->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
    acquisition->connect(top_block);

    std::string file = FLAGS_signal_file;

    boost::shared_ptr<Acquisition_msg_rx> msg_rx;
    try
        {
            msg_rx = Acquisition_msg_rx_make();
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the message port system: " << e.what() << std::endl;
            exit(0);
        }

    msg_rx->top_block = top_block;

    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));


    // 5. Run the flowgraph
    // Get visible GPS satellites (positive acquisitions with Doppler measurements)
    // record startup time
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

    setup_fpga_switch_obs_test();

    unsigned int nsamples_to_transfer;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GALILEO_E1_CODE_CHIP_RATE_HZ / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GALILEO_E5A_CODE_CHIP_RATE_HZ / GALILEO_E5A_CODE_LENGTH_CHIPS)));
        }
    else  // (if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0))
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L5I_CODE_RATE_HZ / GPS_L5I_CODE_LENGTH_CHIPS)));
        }

    int acq_doppler_max = config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz);
    int acq_doppler_step = config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz);


    for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
        {
            tmp_gnss_synchro.PRN = PRN;

            acquisition->stop_acquisition();  // reset the whole system including the sample counters
            acquisition->set_doppler_max(acq_doppler_max);
            acquisition->set_doppler_step(acq_doppler_step);
            acquisition->set_gnss_synchro(&tmp_gnss_synchro);
            acquisition->init();
            acquisition->set_local_code();

            args.file = file;


            send_samples_start_obs_test = 0;

            if ((implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga") or (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga"))
                {
                    args.skip_used_samples = -TEST_OBS_DOWNAMPLING_FILTER_INIT_SAMPLES;

                    args.nsamples_tx = TEST_OBS_DOWNAMPLING_FILTER_INIT_SAMPLES + TEST_OBS_DOWNSAMPLING_FILTER_DELAY;

                    if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, (void*)&args) < 0)
                        {
                            std::cout << "ERROR cannot create DMA Process" << std::endl;
                        }
                    pthread_mutex_lock(&mutex_obs_test);
                    send_samples_start_obs_test = 1;
                    pthread_mutex_unlock(&mutex_obs_test);
                    pthread_join(thread_DMA, nullptr);
                    send_samples_start_obs_test = 0;

                    args.nsamples_tx = nsamples_to_transfer;

                    args.skip_used_samples = TEST_OBS_DOWNSAMPLING_FILTER_DELAY;
                }
            else
                {
                    args.nsamples_tx = nsamples_to_transfer;

                    args.skip_used_samples = 0;
                }


            // create DMA child process
            if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, (void*)&args) < 0)
                {
                    std::cout << "ERROR cannot create DMA Process" << std::endl;
                }

            msg_rx->rx_message = 0;
            top_block->start();

            pthread_mutex_lock(&mutex_obs_test);
            send_samples_start_obs_test = 1;
            pthread_mutex_unlock(&mutex_obs_test);

            acquisition->reset();  // set active

            if (start_msg == true)
                {
                    std::cout << "Reading external signal file: " << FLAGS_signal_file << std::endl;
                    std::cout << "Searching for " << System_and_Signal << " Satellites..." << std::endl;
                    std::cout << "[";
                    start_msg = false;
                }

            // wait for the child DMA process to finish
            pthread_join(thread_DMA, nullptr);

            pthread_mutex_lock(&mutex_obs_test);
            send_samples_start_obs_test = 0;
            pthread_mutex_unlock(&mutex_obs_test);

            // the DMA sends the exact number of samples needed for the acquisition.
            // however because of the LPF in the GPS L1/Gal E1 acquisition, this calculation is approximate
            // and some extra samples might be sent. Wait at least once to give time the HW to consume any extra
            // sample the DMA might have sent.
            do
                {
                    usleep(100000);
                }
            while (msg_rx->rx_message == 0);

            if (msg_rx->rx_message == 1)
                {
                    std::cout << " " << PRN << " ";

                    tmp_gnss_synchro.Acq_doppler_hz = tmp_gnss_synchro.Acq_doppler_hz;
                    tmp_gnss_synchro.Acq_delay_samples = tmp_gnss_synchro.Acq_delay_samples;
                    tmp_gnss_synchro.Acq_samplestamp_samples = 0;                                         // do not take into account the filter internal state initialisation
                    tmp_gnss_synchro.Acq_samplestamp_samples = tmp_gnss_synchro.Acq_samplestamp_samples;  // delay due to the downsampling filter in the acquisition


                    gnss_synchro_vec.push_back(tmp_gnss_synchro);
                }
            else
                {
                    std::cout << " . ";
                }


            top_block->stop();


            std::cout.flush();
        }
    std::cout << "]" << std::endl;
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
              << " [seconds]" << std::endl;
    if (!gnss_synchro_vec.empty())
        {
            return true;
        }
    else
        {
            return false;
        }


    return true;
}


void HybridObservablesTestFpga::configure_receiver(
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
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.dump", "true");
    config->set_property("TelemetryDecoder.dump", "true");

    gnss_synchro_master.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            gnss_synchro_master.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.5");

            config->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            gnss_synchro_master.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.6");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_narrow_chips", "0.6");
            config->set_property("Tracking.track_pilot", "true");

            config->set_property("TelemetryDecoder.implementation", "Galileo_E1B_Telemetry_Decoder");
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")  // or implementation.compare("Galileo_E5a_DLL_PLL_Tracking_b") == 0)
        {
            gnss_synchro_master.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");

            config->set_property("TelemetryDecoder.implementation", "Galileo_E5a_Telemetry_Decoder");
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga")
        {
            gnss_synchro_master.System = 'G';
            std::string signal = "L5";
            System_and_Signal = "GPS L5I";
            const char* str = signal.c_str();                                     // get a C style null terminated string
            std::memcpy(static_cast<void*>(gnss_synchro_master.Signal), str, 3);  // copy string into synchro char array: 2 char + null

            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");

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
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}

void HybridObservablesTestFpga::check_results_carrier_phase(
    arma::mat& true_ch0,
    arma::vec& true_tow_s,
    arma::mat& measured_ch0,
    const std::string& data_title)
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
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Accumulated Carrier phase error [cycles]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Carrier Phase error [cycles]");
            //conversion between arma::vec and std:vector
            std::vector<double> error_vec(err_ch0_cycles.colptr(0), err_ch0_cycles.colptr(0) + err_ch0_cycles.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, error_vec,
                "Carrier Phase error");
            g3.set_legend();
            g3.savetops(data_title + "Carrier_phase_error");

            g3.showonscreen();  // window output
        }

    //check results against the test tolerance
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
    //1. True value interpolation to match the measurement times

    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    //conversion between arma::vec and std:vector
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
    //compute error without the accumulated carrier phase offsets (which depends on the receiver starting time)
    arma::vec delta_true_carrier_phase_cycles = (true_ch0_carrier_phase_interp - true_ch0_carrier_phase_interp(0)) - (true_ch1_carrier_phase_interp - true_ch1_carrier_phase_interp(0));
    arma::vec delta_measured_carrier_phase_cycles = (meas_ch0_carrier_phase_interp - meas_ch0_carrier_phase_interp(0)) - (meas_ch1_carrier_phase_interp - meas_ch1_carrier_phase_interp(0));

    //2. RMSE
    arma::vec err;

    err = delta_measured_carrier_phase_cycles - delta_true_carrier_phase_cycles;
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
    std::cout << std::setprecision(10) << data_title << "Double diff Carrier Phase RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [Cycles]" << std::endl;
    std::cout.precision(ss);

    //plots
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Carrier Phase error [Cycles]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Carrier Phase error [Cycles]");
            //conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Carrier Phase error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_carrier_phase_error");

            g3.showonscreen();  // window output
        }

    //check results against the test tolerance
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
    //1. True value interpolation to match the measurement times

    double t0 = std::max(measured_ch0(0, 0), measured_ch1(0, 0));
    int size1 = measured_ch0.col(0).n_rows;
    int size2 = measured_ch1.col(0).n_rows;
    double t1 = std::min(measured_ch0(size1 - 1, 0), measured_ch1(size2 - 1, 0));

    arma::vec t = arma::linspace<arma::vec>(t0, t1, floor((t1 - t0) * 1e3));
    //conversion between arma::vec and std:vector
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

    //2. RMSE
    arma::vec err;

    err = delta_measured_carrier_doppler_cycles - delta_true_carrier_doppler_cycles;
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
    std::cout << std::setprecision(10) << data_title << "Double diff Carrier Doppler RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [Hz]" << std::endl;
    std::cout.precision(ss);

    //plots
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Carrier Doppler error [Hz]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Carrier Doppler error [Hz]");
            //conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Carrier Doppler error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_carrier_doppler_error");

            g3.showonscreen();  // window output
        }

    //check results against the test tolerance
    ASSERT_LT(error_mean, 5);
    ASSERT_GT(error_mean, -5);
    //assuming PLL BW=35
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
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Carrier Doppler error [Hz]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Carrier Doppler error [Hz]");
            //conversion between arma::vec and std:vector
            std::vector<double> error_vec(err_ch0_hz.colptr(0), err_ch0_hz.colptr(0) + err_ch0_hz.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, error_vec,
                "Carrier Doppler error");
            g3.set_legend();
            g3.savetops(data_title + "Carrier_doppler_error");

            g3.showonscreen();  // window output
        }

    //check results against the test tolerance
    ASSERT_LT(error_mean_ch0, 5);
    ASSERT_GT(error_mean_ch0, -5);
    //assuming PLL BW=35
    ASSERT_LT(error_var_ch0, 200);
    ASSERT_LT(max_error_ch0, 70);
    ASSERT_GT(min_error_ch0, -70);
    ASSERT_LT(rmse_ch0, 30);
}

bool HybridObservablesTestFpga::save_mat_xy(std::vector<double>& x, std::vector<double>& y, std::string filename)
{
    try
        {
            // WRITE MAT FILE
            mat_t* matfp;
            matvar_t* matvar;
            filename.append(".mat");
            std::cout << "save_mat_xy write " << filename << std::endl;
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

void HybridObservablesTestFpga::check_results_code_pseudorange(
    arma::mat& true_ch0,
    arma::mat& true_ch1,
    arma::vec& true_tow_ch0_s,
    arma::vec& true_tow_ch1_s,
    arma::mat& measured_ch0,
    arma::mat& measured_ch1,
    const std::string& data_title)
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
    arma::interp1(true_tow_ch0_s, true_ch0.col(1), t, true_ch0_dist_interp);
    arma::interp1(true_tow_ch1_s, true_ch1.col(1), t, true_ch1_dist_interp);

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
    std::cout << std::setprecision(10) << data_title << "Double diff Pseudorange RMSE = "
              << rmse << ", mean = " << error_mean
              << ", stdev = " << sqrt(error_var)
              << " (max,min) = " << max_error
              << "," << min_error
              << " [meters]" << std::endl;
    std::cout.precision(ss);

    //plots
    if (FLAGS_show_plots)
        {
            Gnuplot g3("linespoints");
            g3.set_title(data_title + "Double diff Pseudorange error [m]");
            g3.set_grid();
            g3.set_xlabel("Time [s]");
            g3.set_ylabel("Double diff Pseudorange error [m]");
            //conversion between arma::vec and std:vector
            std::vector<double> range_error_m(err.colptr(0), err.colptr(0) + err.n_rows);
            g3.cmd("set key box opaque");
            g3.plot_xy(time_vector, range_error_m,
                "Double diff Pseudorrange error");
            g3.set_legend();
            g3.savetops(data_title + "double_diff_pseudorrange_error");

            g3.showonscreen();  // window output
        }

    //check results against the test tolerance
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
            gpstk::Rinex3ObsStream r_ref(FLAGS_filename_rinex_obs);
            r_ref.exceptions(std::ios::failbit);
            gpstk::Rinex3ObsData r_ref_data;
            gpstk::Rinex3ObsHeader r_ref_header;

            gpstk::RinexDatum dataobj;

            r_ref >> r_ref_header;

            std::vector<bool> first_row;
            gpstk::SatID prn;
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
                                    prn = gpstk::SatID(myprn, gpstk::SatID::systemGPS);
                                    break;
                                case 'E':
                                    prn = gpstk::SatID(myprn, gpstk::SatID::systemGalileo);
                                    break;
                                default:
                                    prn = gpstk::SatID(myprn, gpstk::SatID::systemGPS);
                                }

                            gpstk::CommonTime time = r_ref_data.time;
                            double sow(static_cast<gpstk::GPSWeekSecond>(time).sow);

                            auto pointer = r_ref_data.obs.find(prn);
                            if (pointer == r_ref_data.obs.end())
                                {
                                    // PRN not present; do nothing
                                }
                            else
                                {
                                    if (first_row.at(n) == false)
                                        {
                                            //insert next column
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
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 1) = dataobj.data;  //C1C P1 (psudorange L1)
                                            dataobj = r_ref_data.getObs(prn, "D1C", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 2) = dataobj.data;  //D1C Carrier Doppler
                                            dataobj = r_ref_data.getObs(prn, "L1C", r_ref_header);
                                            obs_vec->at(n)(obs_vec->at(n).n_rows - 1, 3) = dataobj.data;  //L1C Carrier Phase
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
                                    else if (strcmp("2S\0", gnss.Signal) == 0)  //L2M
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
                                    else if (strcmp("5X\0", gnss.Signal) == 0)  //Simulator gives RINEX with E5a+E5b
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
                                            std::cout << "ReadRinexObs unknown signal requested: " << gnss.Signal << std::endl;
                                            return false;
                                        }
                                }
                        }
                }  // end while
        }          // End of 'try' block
    catch (const gpstk::FFStreamError& e)
        {
            std::cout << e;
            return false;
        }
    catch (const gpstk::Exception& e)
        {
            std::cout << e;
            return false;
        }
    catch (const std::exception& e)
        {
            std::cout << "Exception: " << e.what();
            std::cout << "unknown error.  I don't feel so well..." << std::endl;
            return false;
        }
    std::cout << "ReadRinexObs info:" << std::endl;
    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
        {
            std::cout << "SAT PRN " << gnss_synchro_vec.at(n).PRN << " RINEX epoch read: " << obs_vec->at(n).n_rows << std::endl;
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
    if (FLAGS_disable_generator == false)
        {
            generate_signal();
        }


    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    // use generator or use an external capture file
    if (FLAGS_enable_external_signal_file)
        {
            //create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(), true);
        }
    else
        {
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
        }

    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols);

    for (auto& n : gnss_synchro_vec)
        {
            //setup the signal synchronization, simulating an acquisition
            if (!FLAGS_enable_external_signal_file)
                {
                    //based on true observables metadata (for custom sdr generator)
                    //open true observables log file written by the simulator or based on provided RINEX obs
                    //std::vector<std::shared_ptr<tracking_true_obs_reader>> true_reader_vec;
                    std::vector<std::shared_ptr<Tracking_True_Obs_Reader>> true_reader_vec;
                    //read true data from the generator logs
                    true_reader_vec.push_back(std::make_shared<Tracking_True_Obs_Reader>());
                    std::cout << "Loading true observable data for PRN " << n.PRN << std::endl;
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

                    //restart the epoch counter
                    true_reader_vec.back()->restart();

                    std::cout << "Initial Doppler [Hz]=" << true_reader_vec.back()->doppler_l1_hz << " Initial code delay [Chips]="
                              << true_reader_vec.back()->prn_delay_chips << std::endl;
                    n.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_reader_vec.back()->prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
                    n.Acq_doppler_hz = true_reader_vec.back()->doppler_l1_hz;
                    n.Acq_samplestamp_samples = 0;
                }
            else
                {
                    //based on the signal acquisition process
                    std::cout << "Estimated Initial Doppler " << n.Acq_doppler_hz
                              << " [Hz], estimated Initial code delay " << n.Acq_delay_samples << " [Samples]"
                              << " Acquisition SampleStamp is " << n.Acq_samplestamp_samples << std::endl;
                }
        }


    // The HW has been reset after the acquisition phase when the acquisition class was destroyed.
    // No more samples remained in the DMA. Therefore any intermediate state in the LPF of the
    // GPS L1 / Galileo E1 filter has been cleared.
    // During this test all the samples coming from the DMA are consumed so in principle there would be
    // no need to reset the HW. However we need to clear the sample counter in each test. Therefore we have
    // to reset the HW at the beginning of each test.

    // instantiate the acquisition modules in order to use them to reset the HW.
    // (note that the constructor of the acquisition modules resets the HW too)


    std::shared_ptr<AcquisitionInterface> acquisition;

    // reset the HW to clear the sample counters: the acquisition constructor generates a reset
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
            args.freq_band = 0;
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
            args.freq_band = 0;
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            acquisition = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
            args.freq_band = 1;
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga")
        {
            acquisition = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
            args.freq_band = 1;
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }


    std::vector<std::shared_ptr<TrackingInterface>> tracking_ch_vec;
    std::vector<std::shared_ptr<TelemetryDecoderInterface>> tlm_ch_vec;

    std::vector<gr::blocks::null_sink::sptr> null_sink_vec;
    for (unsigned int n = 0; n < gnss_synchro_vec.size(); n++)
        {
            //set channels ids
            gnss_synchro_vec.at(n).Channel_ID = n;

            //create the tracking channels and create the telemetry decoders

            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", config->property("Tracking.implementation", std::string("undefined")), 1, 1);
            tracking_ch_vec.push_back(std::dynamic_pointer_cast<TrackingInterface>(trk_));
            std::shared_ptr<GNSSBlockInterface> tlm_ = factory->GetBlock(config, "TelemetryDecoder", config->property("TelemetryDecoder.implementation", std::string("undefined")), 1, 1);
            tlm_ch_vec.push_back(std::dynamic_pointer_cast<TelemetryDecoderInterface>(tlm_));

            //create null sinks for observables output
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
    boost::shared_ptr<HybridObservablesTest_msg_rx_Fpga> dummy_msg_rx_trk = HybridObservablesTest_msg_rx_Fpga_make();
    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx_Fpga> dummy_tlm_msg_rx = HybridObservablesTest_tlm_msg_rx_Fpga_make();
    //Observables
    std::shared_ptr<ObservablesInterface> observables(new HybridObservables(config.get(), "Observables", tracking_ch_vec.size() + 1, tracking_ch_vec.size()));

    for (auto& n : tracking_ch_vec)
        {
            ASSERT_NO_THROW({
                n->connect(top_block);
            }) << "Failure connecting tracking to the top_block.";
        }

    std::string file;

    ASSERT_NO_THROW({
        if (!FLAGS_enable_external_signal_file)
            {
                file = "./" + filename_raw_data;
            }
        else
            {
                file = FLAGS_signal_file;
            }
        int observable_interval_ms = 20;

        double fs = static_cast<double>(config->property("GNSS-SDR.internal_fs_sps", 0));

        gnss_sdr_fpga_sample_counter_sptr ch_out_fpga_sample_counter;
        ch_out_fpga_sample_counter = gnss_sdr_make_fpga_sample_counter(fs, observable_interval_ms);


        for (unsigned int n = 0; n < tracking_ch_vec.size(); n++)
            {
                //top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tracking_ch_vec.at(n)->get_right_block(), 0, tlm_ch_vec.at(n)->get_left_block(), 0);
                top_block->connect(tlm_ch_vec.at(n)->get_right_block(), 0, observables->get_left_block(), n);
                top_block->msg_connect(tracking_ch_vec.at(n)->get_right_block(), pmt::mp("events"), dummy_msg_rx_trk, pmt::mp("events"));
                top_block->connect(observables->get_right_block(), n, null_sink_vec.at(n), 0);
            }
        //connect sample counter and timmer to the last channel in observables block (extra channel)
        //top_block->connect(samp_counter, 0, observables->get_left_block(), tracking_ch_vec.size());
        top_block->connect(ch_out_fpga_sample_counter, 0, observables->get_left_block(), tracking_ch_vec.size());  //extra port for the sample counter pulse
    }) << "Failure connecting the blocks.";


    args.file = file;
    args.nsamples_tx = baseband_sampling_freq * FLAGS_duration;
    ;

    args.skip_used_samples = 0;

    if (pthread_create(&thread_DMA, nullptr, handler_DMA_obs_test, (void*)&args) < 0)
        {
            std::cout << "ERROR cannot create DMA Process" << std::endl;
        }


    for (auto& n : tracking_ch_vec)
        {
            n->start_tracking();
        }

    pthread_mutex_lock(&mutex_obs_test);
    send_samples_start_obs_test = 1;
    pthread_mutex_unlock(&mutex_obs_test);


    top_block->start();


    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        //top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";


    // wait for the child DMA process to finish
    pthread_join(thread_DMA, nullptr);


    top_block->stop();


    // reset the HW AGAIN
    acquisition->stop_acquisition();


    //	pthread_mutex_lock(&mutex_obs_test);
    //	send_samples_start_obs_test = 0;
    //	pthread_mutex_unlock(&mutex_obs_test);


    //check results
    // Matrices for storing columnwise true GPS time, Range, Doppler and Carrier phase
    std::vector<arma::mat> true_obs_vec;

    if (!FLAGS_enable_external_signal_file)
        {
            //load the true values
            True_Observables_Reader true_observables;
            ASSERT_NO_THROW({
                if (true_observables.open_obs_file(std::string("./obs_out.bin")) == false)
                    {
                        throw std::exception();
                    }
            }) << "Failure opening true observables file";

            auto nepoch = static_cast<unsigned int>(true_observables.num_epochs());

            std::cout << "True observation epochs = " << nepoch << std::endl;

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
        }
    else
        {
            ASSERT_EQ(ReadRinexObs(&true_obs_vec, gnss_synchro_master), true)
                << "Failure reading RINEX file";
        }


    //read measured values
    Observables_Dump_Reader estimated_observables(tracking_ch_vec.size());
    ASSERT_NO_THROW({
        if (estimated_observables.open_obs_file(std::string("./observables.dat")) == false)
            {
                throw std::exception();
            }
    }) << "Failure opening dump observables file";

    auto nepoch = static_cast<unsigned int>(estimated_observables.num_epochs());
    std::cout << "Measured observations epochs = " << nepoch << std::endl;

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


    //Cut measurement tail zeros
    arma::uvec index;
    for (auto& n : measured_obs_vec)
        {
            index = arma::find(n.col(0) > 0.0, 1, "last");
            if ((!index.empty()) and index(0) < (nepoch - 1))
                {
                    n.shed_rows(index(0) + 1, nepoch - 1);
                }
        }

    //Cut measurement initial transitory of the measurements

    double initial_transitory_s = FLAGS_skip_obs_transitory_s;

    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            index = arma::find(measured_obs_vec.at(n).col(0) >= (measured_obs_vec.at(n)(0, 0) + initial_transitory_s), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
                {
                    measured_obs_vec.at(n).shed_rows(0, index(0));
                }

            index = arma::find(measured_obs_vec.at(n).col(0) >= true_obs_vec.at(n)(0, 0), 1, "first");
            if ((!index.empty()) and (index(0) > 0))
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
    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            if (epoch_counters_vec.at(n) > 10)  //discard non-valid channels
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
    //receiver_time_offset_ref_channel_s = true_obs_vec.at(min_pr_ch_id).col(1) / GPS_C_m_s - GPS_STARTOFFSET_ms / 1000.0;
    receiver_time_offset_ref_channel_s = (true_obs_vec.at(min_pr_ch_id).col(1)(0) - measured_obs_vec.at(min_pr_ch_id).col(4)(0)) / GPS_C_M_S;
    std::cout << "Ref channel initial Receiver time offset " << receiver_time_offset_ref_channel_s(0) * 1e3 << " [ms]" << std::endl;

    for (unsigned int n = 0; n < measured_obs_vec.size(); n++)
        {
            //debug save to .mat
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

            if (epoch_counters_vec.at(n) > 10)  //discard non-valid channels
                {
                    arma::vec true_TOW_ref_ch_s = true_obs_vec.at(min_pr_ch_id).col(0) - receiver_time_offset_ref_channel_s(0);
                    arma::vec true_TOW_ch_s = true_obs_vec.at(n).col(0) - receiver_time_offset_ref_channel_s(0);
                    //Compare measured observables
                    if (min_pr_ch_id != n)
                        {
                            check_results_code_pseudorange(true_obs_vec.at(n),
                                true_obs_vec.at(min_pr_ch_id),
                                true_TOW_ch_s,
                                true_TOW_ref_ch_s,
                                measured_obs_vec.at(n),
                                measured_obs_vec.at(min_pr_ch_id),
                                "[CH " + std::to_string(n) + "] PRN " + std::to_string(gnss_synchro_vec.at(n).PRN) + " ");
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
                    else
                        {
                            std::cout << "[CH " << std::to_string(n) << "] PRN " << std::to_string(gnss_synchro_vec.at(n).PRN) << " is the reference satellite" << std::endl;
                        }
                    if (FLAGS_compute_single_diffs)
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


    std::cout << "Test completed in " << elapsed_seconds.count() << " [s]" << std::endl;
}

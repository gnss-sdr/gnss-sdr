/*!
 * \file tracking_pull-in_test_fpga.cc
 * \brief  This class implements a tracking Pull-In test for FPGA HW accelerator
 *  implementations based on some input parameters.
 * \author 	Marc Majoral, 2019. majoralmarc(at)cttc.es
 * 			Javier Arribas, 2018. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
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
#include "acquisition_msg_rx.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "gnss_block_factory.h"
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
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <chrono>
#include <unistd.h>
#include <utility>
#include <vector>

#if HAS_STD_FILESYSTEM
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

// threads
#include <fcntl.h>     // for open, O_RDWR, O_SYNC
#include <iostream>    // for cout, endl
#include <pthread.h>   // for pthread stuff
#include <sys/mman.h>  // for mmap

#define MAX_INPUT_COMPLEX_SAMPLES_TOTAL 8192  // maximum DMA sample block size in complex samples
#define COMPLEX_SAMPLE_SIZE 2                 // sample size in bytes
#define NUM_QUEUES 2                          // number of queues (1 for GPS L1/Galileo E1, and 1 for GPS L5/Galileo E5)
#define DOWNAMPLING_FILTER_INIT_SAMPLES 100   // some samples to initialize the state of the downsampling filter
#define DOWNSAMPLING_FILTER_DELAY 48          // delay of the downsampling filter in samples


// HW related options
bool skip_samples_already_used = false;  // if skip_samples_already_used = 1 => for each PRN loop skip the samples used in the previous PRN loops
                                         // (exactly in the same way as the SW)

class Acquisition_msg_rx_Fpga;

using Acquisition_msg_rx_Fpga_sptr = boost::shared_ptr<Acquisition_msg_rx_Fpga>;

Acquisition_msg_rx_Fpga_sptr Acquisition_msg_rx_Fpga_make();

class Acquisition_msg_rx_Fpga : public gr::block
{
private:
    friend Acquisition_msg_rx_Fpga_sptr Acquisition_msg_rx_Fpga_make();
    void msg_handler_events(pmt::pmt_t msg);
    Acquisition_msg_rx_Fpga();

public:
    int rx_message;
    gr::top_block_sptr top_block;
    ~Acquisition_msg_rx_Fpga();  //!< Default destructor
};

Acquisition_msg_rx_Fpga_sptr Acquisition_msg_rx_Fpga_make()
{
    return Acquisition_msg_rx_Fpga_sptr(new Acquisition_msg_rx_Fpga());
}

void Acquisition_msg_rx_Fpga::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
            top_block->stop();  //stop the flowgraph
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_acquisition Bad cast!\n";
            rx_message = 0;
        }
}

Acquisition_msg_rx_Fpga::Acquisition_msg_rx_Fpga() : gr::block("Acquisition_msg_rx_Fpga", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&Acquisition_msg_rx_Fpga::msg_handler_events, this, _1));
    rx_message = 0;
}

Acquisition_msg_rx_Fpga::~Acquisition_msg_rx_Fpga() = default;

class TrackingPullInTestFpga_msg_rx;

using TrackingPullInTestFpga_msg_rx_sptr = boost::shared_ptr<TrackingPullInTestFpga_msg_rx>;

TrackingPullInTestFpga_msg_rx_sptr TrackingPullInTestFpga_msg_rx_make();

class TrackingPullInTestFpga_msg_rx : public gr::block
{
private:
    friend TrackingPullInTestFpga_msg_rx_sptr TrackingPullInTestFpga_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    TrackingPullInTestFpga_msg_rx();

public:
    int rx_message;
    ~TrackingPullInTestFpga_msg_rx();  //!< Default destructor
};

TrackingPullInTestFpga_msg_rx_sptr TrackingPullInTestFpga_msg_rx_make()
{
    return TrackingPullInTestFpga_msg_rx_sptr(new TrackingPullInTestFpga_msg_rx());
}

void TrackingPullInTestFpga_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;  //3 -> loss of lock
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_tracking Bad cast!";
            rx_message = 0;
        }
}

TrackingPullInTestFpga_msg_rx::TrackingPullInTestFpga_msg_rx() : gr::block("TrackingPullInTestFpga_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&TrackingPullInTestFpga_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


TrackingPullInTestFpga_msg_rx::~TrackingPullInTestFpga_msg_rx() = default;


class TrackingPullInTestFpga : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;
    std::string implementation = FLAGS_trk_test_implementation;

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;
    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_signal_file;

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
    pthread_mutex_t the_mutex = PTHREAD_MUTEX_INITIALIZER;
};


int TrackingPullInTestFpga::configure_generator(double CN0_dBHz, int file_idx)
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
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;                    // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_signal_file + std::to_string(file_idx);  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);       //Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(CN0_dBHz);                          // Signal generator CN0
    return 0;
}


int TrackingPullInTestFpga::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], nullptr};

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
    config->set_property("Tracking.item_type", "cshort");
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;
    gnss_synchro.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;

    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            gnss_synchro.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.if", "0");
            config->set_property("Tracking.order", "3");
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            gnss_synchro.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.6");
            config->set_property("Tracking.track_pilot", "true");

            // added by me
            config->set_property("Tracking.if", "0");
            config->set_property("Tracking.devicename", "/dev/uio");
            config->set_property("Tracking.device_base", "15");
        }

    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")  // or implementation.compare("Galileo_E5a_DLL_PLL_Tracking_b") == 0)
        {
            gnss_synchro.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "false");
            config->set_property("Tracking.order", "2");
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga")
        {
            gnss_synchro.System = 'G';
            std::string signal = "L5";
            System_and_Signal = "GPS L5I";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "false");
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


void setup_fpga_switch(void)
{
    const size_t PAGE_SIZE_ = 0x10000;
    const unsigned int TEST_REGISTER_TRACK_WRITEVAL = 0x55AA;
    int switch_device_descriptor;        // driver descriptor
    volatile unsigned* switch_map_base;  // driver memory map

    if ((switch_device_descriptor = open("/dev/uio1", O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio"
                         << "/dev/uio1";
        }
    switch_map_base = reinterpret_cast<volatile unsigned*>(mmap(nullptr, PAGE_SIZE_,
        PROT_READ | PROT_WRITE, MAP_SHARED, switch_device_descriptor, 0));

    if (switch_map_base == reinterpret_cast<void*>(-1))
        {
            LOG(WARNING) << "Cannot map the FPGA switch module into tracking memory";
            std::cout << "Could not map switch memory." << std::endl;
        }

    // sanity check : check test register
    unsigned writeval = TEST_REGISTER_TRACK_WRITEVAL;
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


//static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

volatile unsigned int send_samples_start = 0;

int8_t input_samples[MAX_INPUT_COMPLEX_SAMPLES_TOTAL * COMPLEX_SAMPLE_SIZE];  // re - im
int8_t input_samples_dma[MAX_INPUT_COMPLEX_SAMPLES_TOTAL * COMPLEX_SAMPLE_SIZE * NUM_QUEUES];


struct DMA_handler_args
{
    std::string file;
    unsigned int nsamples_tx;
    unsigned int skip_used_samples;
    unsigned int freq_band;  // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
};


void* handler_DMA(void* arguments)
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
    while (send_samples_start == 0)
        ;  // wait until main thread tells the DMA to start

    // skip initial samples
    int skip_samples = (int)FLAGS_skip_samples;

    fseek(rx_signal_file_id, (skip_samples + skip_used_samples) * 2, SEEK_SET);

    usleep(50000);  // wait some time to give time to the main thread to start the acquisition module

    while (file_completed == false)
        {
            if (nsamples_tx - nsamples > MAX_INPUT_COMPLEX_SAMPLES_TOTAL)
                {
                    nsamples_block = MAX_INPUT_COMPLEX_SAMPLES_TOTAL;
                }
            else
                {
                    nsamples_block = nsamples_tx - nsamples;  // remaining samples to be sent
                    file_completed = true;
                }

            nread_elements = fread(input_samples, sizeof(int8_t), nsamples_block * COMPLEX_SAMPLE_SIZE, rx_signal_file_id);

            if (nread_elements != nsamples_block * COMPLEX_SAMPLE_SIZE)
                {
                    std::cout << "file completed" << std::endl;
                    file_completed = true;
                }

            nsamples += (nread_elements / COMPLEX_SAMPLE_SIZE);

            if (nread_elements > 0)
                {
                    // for the 32-BIT DMA
                    dma_index = 0;
                    for (index0 = 0; index0 < (nread_elements); index0 += COMPLEX_SAMPLE_SIZE)
                        {
                            if (args->freq_band == 0)
                                {
                                    // channel 1 (queue 1) -> E5/L5
                                    input_samples_dma[dma_index] = 0;
                                    input_samples_dma[dma_index + 1] = 0;
                                    // channel 0 (queue 0) -> E1/L1
                                    input_samples_dma[dma_index + 2] = input_samples[index0];
                                    input_samples_dma[dma_index + 3] = input_samples[index0 + 1];
                                }
                            else
                                {
                                    // channel 1 (queue 1) -> E5/L5
                                    input_samples_dma[dma_index] = input_samples[index0];
                                    input_samples_dma[dma_index + 1] = input_samples[index0 + 1];
                                    // channel 0 (queue 0) -> E1/L1
                                    input_samples_dma[dma_index + 2] = 0;
                                    input_samples_dma[dma_index + 3] = 0;
                                }
                            dma_index += 4;
                        }
                    nsamples_transmitted = write(tx_fd, &input_samples_dma[0], nread_elements * NUM_QUEUES);
                    if (nsamples_transmitted != nread_elements * NUM_QUEUES)
                        {
                            std::cout << "Error : DMA could not send all the requested samples" << std::endl;
                        }
                }
        }

    close(tx_fd);
    fclose(rx_signal_file_id);
    return nullptr;
}


bool TrackingPullInTestFpga::acquire_signal(int SV_ID)
{
    pthread_t thread_DMA;

    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string System_and_Signal;

    struct DMA_handler_args args;

    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "1C";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L1 CA";

            args.freq_band = 0;

            acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "1B";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E1B";

            args.freq_band = 0;

            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "5X";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";

            args.freq_band = 1;

            acquisition = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga")
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "L5";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L5I";

            args.freq_band = 1;
            acquisition = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation \n ";
            throw(std::exception());
        }

    acquisition->set_channel(0);
    acquisition->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
    acquisition->connect(top_block);

    std::string file = FLAGS_signal_file;

    boost::shared_ptr<Acquisition_msg_rx_Fpga> msg_rx;
    try
        {
            msg_rx = Acquisition_msg_rx_Fpga_make();
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

    setup_fpga_switch();

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
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
        }
    else  // (if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0))
        {
            nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
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

            if ((implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga") or (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga"))
                {
                    // send the previous samples to set the downsampling filter in a good condition
                    send_samples_start = 0;

                    args.skip_used_samples = -DOWNAMPLING_FILTER_INIT_SAMPLES;

                    args.nsamples_tx = DOWNAMPLING_FILTER_INIT_SAMPLES + DOWNSAMPLING_FILTER_DELAY;

                    if (pthread_create(&thread_DMA, nullptr, handler_DMA, (void*)&args) < 0)
                        {
                            std::cout << "ERROR cannot create DMA Process" << std::endl;
                        }
                    pthread_mutex_lock(&the_mutex);
                    send_samples_start = 1;
                    pthread_mutex_unlock(&the_mutex);
                    pthread_join(thread_DMA, nullptr);
                    send_samples_start = 0;

                    args.nsamples_tx = nsamples_to_transfer;

                    args.skip_used_samples = DOWNSAMPLING_FILTER_DELAY;
                }
            else
                {
                    args.nsamples_tx = nsamples_to_transfer;

                    args.skip_used_samples = 0;
                }

            if (pthread_create(&thread_DMA, nullptr, handler_DMA, (void*)&args) < 0)
                {
                    std::cout << "ERROR cannot create DMA Process" << std::endl;
                }

            msg_rx->rx_message = 0;
            top_block->start();

            pthread_mutex_lock(&the_mutex);
            send_samples_start = 1;
            pthread_mutex_unlock(&the_mutex);

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

            pthread_mutex_lock(&the_mutex);
            send_samples_start = 0;
            pthread_mutex_unlock(&the_mutex);

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
                    doppler_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_doppler_hz));
                    code_delay_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_delay_samples));
                    tmp_gnss_synchro.Acq_samplestamp_samples = 0;  // do not take into account the filter internal state initialisation
                    acq_samplestamp_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_samplestamp_samples));
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

    for (auto& x : doppler_measurements_map)
        {
            std::cout << "DETECTED SATELLITE " << System_and_Signal << " PRN: " << x.first << " with Doppler: " << x.second << " [Hz], code phase: " << code_delay_measurements_map.at(x.first) << " [samples] at signal SampleStamp " << acq_samplestamp_map.at(x.first) << "\n";
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]" << std::endl;
    return true;
}


TEST_F(TrackingPullInTestFpga, ValidationOfResults)
{
    // pointer to the DMA thread that sends the samples to the acquisition engine
    pthread_t thread_DMA;
    struct DMA_handler_args args;

    //*************************************************
    //***** STEP 1: Prepare the parameters sweep ******
    //*************************************************
    std::vector<double>
        acq_doppler_error_hz_values;
    std::vector<std::vector<double>> acq_delay_error_chips_values;  //vector of vector

    for (double doppler_hz = FLAGS_acq_Doppler_error_hz_start; doppler_hz >= FLAGS_acq_Doppler_error_hz_stop; doppler_hz = doppler_hz + FLAGS_acq_Doppler_error_hz_step)
        {
            acq_doppler_error_hz_values.push_back(doppler_hz);
            std::vector<double> tmp_vector;
            //Code Delay Sweep
            for (double code_delay_chips = FLAGS_acq_Delay_error_chips_start; code_delay_chips >= FLAGS_acq_Delay_error_chips_stop; code_delay_chips = code_delay_chips + FLAGS_acq_Delay_error_chips_step)
                {
                    tmp_vector.push_back(code_delay_chips);
                }
            acq_delay_error_chips_values.push_back(tmp_vector);
        }

    //***********************************************************
    //***** STEP 2: Generate the input signal (if required) *****
    //***********************************************************
    std::vector<double> generator_CN0_values;
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

    // use generator or use an external capture file
    if (FLAGS_enable_external_signal_file)
        {
            //create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(FLAGS_test_satellite_PRN), true);
            bool found_satellite = doppler_measurements_map.find(FLAGS_test_satellite_PRN) != doppler_measurements_map.end();
            EXPECT_TRUE(found_satellite) << "Error: satellite SV: " << FLAGS_test_satellite_PRN << " is not acquired";
            if (!found_satellite) return;
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
                }
        }

    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols);

    //******************************************************************************************
    //***** Obtain the initial signal sinchronization parameters (emulating an acquisition) ****
    //******************************************************************************************
    int test_satellite_PRN = 0;
    double true_acq_doppler_hz = 0.0;
    double true_acq_delay_samples = 0.0;
    uint64_t acq_samplestamp_samples = 0;

    Tracking_True_Obs_Reader true_obs_data;
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
            std::cout << "True Initial Doppler " << true_obs_data.doppler_l1_hz << " [Hz], true Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << "[Chips]" << std::endl;
            true_acq_doppler_hz = true_obs_data.doppler_l1_hz;
            true_acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * static_cast<double>(baseband_sampling_freq) * GPS_L1_CA_CODE_PERIOD;
            acq_samplestamp_samples = 0;
        }
    else
        {
            true_acq_doppler_hz = doppler_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            true_acq_delay_samples = code_delay_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            acq_samplestamp_samples = acq_samplestamp_map.find(FLAGS_test_satellite_PRN)->second;
            std::cout << "Estimated Initial Doppler " << true_acq_doppler_hz
                      << " [Hz], estimated Initial code delay " << true_acq_delay_samples << " [Samples]"
                      << " Acquisition SampleStamp is " << acq_samplestamp_samples << std::endl;
        }

    std::vector<std::vector<double>> pull_in_results_v_v;

    unsigned int code_length;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GALILEO_E1_CODE_CHIP_RATE_HZ / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / GALILEO_E5A_CODE_CHIP_RATE_HZ * static_cast<double>(GALILEO_E5A_CODE_LENGTH_CHIPS)));
        }
    else  // (if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0))
        {
            code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L5I_CODE_RATE_HZ / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS))));
        }

    float nbits = ceilf(log2f((float)code_length));
    unsigned int fft_size = pow(2, nbits);

    // The HW has been reset after the acquisition phase when the acquisition class was destroyed.
    // No more samples remained in the DMA. Therefore any intermediate state in the LPF of the
    // GPS L1 / Galileo E1 filter has been cleared.
    // During this test all the samples coming from the DMA are consumed so in principle there would be
    // no need to reset the HW. However we need to clear the sample counter in each test. Therefore we have
    // to reset the HW at the beginning of each test.

    // instantiate the acquisition modules in order to use them to reset the HW.
    // (note that the constructor of the acquisition modules resets the HW too)

    std::shared_ptr<AcquisitionInterface> acquisition;

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

    for (double generator_CN0_value : generator_CN0_values)
        {
            std::vector<double> pull_in_results_v;
            for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
                {
                    for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                        {
                            // reset the HW to clear the sample counters
                            acquisition->stop_acquisition();

                            gnss_synchro.Acq_samplestamp_samples = acq_samplestamp_samples;
                            //simulate a Doppler error in acquisition
                            gnss_synchro.Acq_doppler_hz = true_acq_doppler_hz + acq_doppler_error_hz_values.at(current_acq_doppler_error_idx);
                            //simulate Code Delay error in acquisition
                            gnss_synchro.Acq_delay_samples = true_acq_delay_samples + (acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) / GPS_L1_CA_CODE_RATE_HZ) * static_cast<double>(baseband_sampling_freq);

                            //create flowgraph
                            top_block = gr::make_top_block("Tracking test");
                            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", config->property("Tracking.implementation", std::string("undefined")), 1, 1);
                            std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);
                            boost::shared_ptr<TrackingPullInTestFpga_msg_rx> msg_rx = TrackingPullInTestFpga_msg_rx_make();

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
                                gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
                                top_block->connect(tracking->get_right_block(), 0, sink, 0);
                                top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
                            }) << "Failure connecting the blocks of tracking test.";

                            std::string file = FLAGS_signal_file;

                            args.file = file;

                            if (skip_samples_already_used == 1)
                                {
                                    args.skip_used_samples = (gnss_synchro.PRN - 1) * fft_size;
                                }
                            else
                                {
                                    args.skip_used_samples = 0;
                                }

                            //********************************************************************
                            //***** STEP 5: Perform the signal tracking and read the results *****
                            //********************************************************************
                            args.nsamples_tx = baseband_sampling_freq * FLAGS_duration;

                            if (pthread_create(&thread_DMA, nullptr, handler_DMA, (void*)&args) < 0)
                                {
                                    std::cout << "ERROR cannot create DMA Process" << std::endl;
                                }

                            std::cout << "--- START TRACKING WITH PULL-IN ERROR: " << acq_doppler_error_hz_values.at(current_acq_doppler_error_idx) << " [Hz] and " << acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) << " [Chips] ---" << std::endl;

                            tracking->start_tracking();

                            pthread_mutex_lock(&the_mutex);
                            send_samples_start = 1;
                            pthread_mutex_unlock(&the_mutex);

                            top_block->start();

                            // wait for the child DMA process to finish
                            pthread_join(thread_DMA, nullptr);

                            top_block->stop();

                            // reset the HW to launch the pending interrupts
                            acquisition->stop_acquisition();

                            pthread_mutex_lock(&the_mutex);
                            send_samples_start = 0;
                            pthread_mutex_unlock(&the_mutex);

                            pull_in_results_v.push_back(msg_rx->rx_message != 3);  //save last asynchronous tracking message in order to detect a loss of lock

                            //********************************
                            //***** STEP 7: Plot results *****
                            //********************************
                            if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                {
                                    //load the measured values
                                    Tracking_Dump_Reader trk_dump;
                                    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
                                        << "Failure opening tracking dump file";

                                    int64_t n_measured_epochs = trk_dump.num_epochs();
                                    //todo: use vectors instead
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
                                            trk_acc_carrier_phase_cycles(epoch_counter) = trk_dump.acc_carrier_phase_rad / GPS_TWO_PI;
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

                                    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
                                    if (gnuplot_executable.empty())
                                        {
                                            std::cout << "WARNING: Although the flag show_plots has been set to TRUE," << std::endl;
                                            std::cout << "gnuplot has not been found in your system." << std::endl;
                                            std::cout << "Test results will not be plotted." << std::endl;
                                        }
                                    else
                                        {
                                            try
                                                {
                                                    fs::path p(gnuplot_executable);
                                                    fs::path dir = p.parent_path();
                                                    const std::string& gnuplot_path = dir.native();
                                                    Gnuplot::set_GNUPlotPath(gnuplot_path);
                                                    auto decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

                                                    if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                                        {
                                                            Gnuplot g1("linespoints");
                                                            g1.showonscreen();  // window output
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g1.set_title(std::to_string(generator_CN0_value) + " dB-Hz, " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g1.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g1.set_grid();
                                                            g1.set_xlabel("Time [s]");
                                                            g1.set_ylabel("Correlators' output");
                                                            //g1.cmd("set key box opaque");
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
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g2.set_title(std::to_string(generator_CN0_value) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g2.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g2.set_grid();
                                                            g2.set_xlabel("Inphase");
                                                            g2.set_ylabel("Quadrature");
                                                            //g2.cmd("set size ratio -1");
                                                            g2.plot_xy(promptI, promptQ);
                                                            g2.savetops("Constellation");

                                                            Gnuplot g3("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g3.set_title(std::to_string(generator_CN0_value) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g3.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            g3.set_grid();
                                                            g3.set_xlabel("Time [s]");
                                                            g3.set_ylabel("Reported CN0 [dB-Hz]");
                                                            g3.cmd("set key box opaque");

                                                            g3.plot_xy(trk_timestamp_s, CN0_dBHz,
                                                                std::to_string(static_cast<int>(round(generator_CN0_value))) + "[dB-Hz]", decimate);

                                                            g3.set_legend();
                                                            g3.savetops("CN0_output");

                                                            g3.showonscreen();  // window output

                                                            Gnuplot g4("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g4.set_title(std::to_string(generator_CN0_value) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g4.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            g4.set_grid();
                                                            g4.set_xlabel("Time [s]");
                                                            g4.set_ylabel("Estimated Doppler [Hz]");
                                                            g4.cmd("set key box opaque");

                                                            g4.plot_xy(trk_timestamp_s, Doppler,
                                                                std::to_string(static_cast<int>(round(generator_CN0_value))) + "[dB-Hz]", decimate);

                                                            g4.set_legend();
                                                            g4.savetops("Doppler");

                                                            g4.showonscreen();  // window output
                                                        }
                                                }
                                            catch (const GnuplotException& ge)
                                                {
                                                    std::cout << ge.what() << std::endl;
                                                }
                                        }
                                }  //end plot

                        }  //end acquisition Delay errors loop

                    usleep(100000);  // give time to the HW to consume all the remaining samples

                }  //end acquisition Doppler errors loop

            pull_in_results_v_v.push_back(pull_in_results_v);

        }  //end CN0 LOOP
           //build the mesh grid

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
            //plot grid
            if (FLAGS_show_plots)
                {
                    Gnuplot g4("points palette pointsize 2 pointtype 7");
                    g4.showonscreen();  // window output
                    g4.cmd(R"(set palette defined ( 0 "black", 1 "green" ))");
                    g4.cmd("set key off");
                    g4.cmd("set view map");
                    std::string title;
                    if (!FLAGS_enable_external_signal_file)
                        {
                            title = std::string("Tracking Pull-in result grid at CN0:" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + " [dB-Hz], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz].");
                        }
                    else
                        {
                            title = std::string("Tracking Pull-in result grid, PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                        }

                    g4.set_title(title);
                    g4.set_grid();
                    g4.set_xlabel("Acquisition Doppler error [Hz]");
                    g4.set_ylabel("Acquisition Code Delay error [Chips]");
                    g4.cmd("set cbrange[0:1]");
                    g4.plot_xyz(doppler_error_mesh,
                        code_delay_error_mesh,
                        pull_in_result_mesh);
                    g4.set_legend();
                    if (!FLAGS_enable_external_signal_file)
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
}

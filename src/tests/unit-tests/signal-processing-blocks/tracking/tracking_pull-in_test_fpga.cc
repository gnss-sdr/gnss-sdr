/*!
 * \file tracking_pull-in_test_fpga.cc
 * \brief  This class implements a tracking Pull-In test for FPGA HW accelerator
 *  implementations based on some input parameters.
 * \author 	Marc Majoral, 2018. majoralmarc(at)cttc.es
 * 			Javier Arribas, 2018. jarribas(at)cttc.es
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

#include <chrono>
#include <unistd.h>
#include <vector>
#include <armadillo>
#include <boost/filesystem.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/blocks/head.h>
#include <gtest/gtest.h>
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "GPS_L5.h"
#include "gnss_block_factory.h"
#include "tracking_interface.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "gps_l5i_pcps_acquisition_fpga.h"
#include "in_memory_configuration.h"
#include "tracking_true_obs_reader.h"
#include "tracking_dump_reader.h"
#include "signal_generator_flags.h"
#include "gnuplot_i.h"
#include "test_flags.h"
#include "tracking_tests_flags.h"

// threads
#include <pthread.h>	// for pthread stuff
#include <fcntl.h>     	// for open, O_RDWR, O_SYNC
#include <iostream>    	// for cout, endl
#include <sys/mman.h>  	// for mmap

#define MAX_INPUT_COMPLEX_SAMPLES_TOTAL 8192 	// maximum DMA sample block size in complex samples
#define COMPLEX_SAMPLE_SIZE 2					// sample size in bytes
#define NUM_QUEUES 2							// number of queues (1 for GPS L1/Galileo E1, and 1 for GPS L5/Galileo E5)
#define NSAMPLES_TRACKING 200000000				// number of samples during which we test the tracking module
#define NSAMPLES_FINAL 50000					// number of samples sent after running tracking to unblock the SW if it is waiting for an interrupt of the tracking module
#define NSAMPLES_ACQ_DOPPLER_SWEEP 50000000		// number of samples sent to the acquisition module when running acquisition when the HW controls the doppler loop
#define DOWNAMPLING_FILTER_INIT_SAMPLES 100		// some samples to initialize the state of the downsampling filter
#define DOWNSAMPLING_FILTER_DELAY 48
#define DOWNSAMPLING_FILTER_OFFSET_SAMPLES 0
// HW related options
bool doppler_control_in_sw = 1;		// 1 => doppler sweep controlled by the SW test code , 0 => doppler sweep controlled by the HW
bool show_results_table = 0;		// 1 => show matrix of (doppler, (max value, power sum)) results (only if doppler_control_in_sw = 1), 0=> do not show it
bool skip_samples_already_used = 0;	// if doppler_control_in_sw = 1 and skip_samples_already_used = 1 => for each PRN loop skip the samples used in the previous PRN loops
									// (exactly in the same way as the SW)
									// if doppler_control_in_sw = 1 and skip_samples_already_used = 0 => the sampe samples are used for each doppler sweep
									// if doppler_control_in_sw = 0 => skip_samples_already_used is not applicable


// ######## GNURADIO ACQUISITION BLOCK MESSAGE RECEVER #########
class Acquisition_msg_rx_Fpga;

typedef boost::shared_ptr<Acquisition_msg_rx_Fpga> Acquisition_msg_rx_Fpga_sptr;

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
            int64_t message = pmt::to_long(msg);
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

Acquisition_msg_rx_Fpga::~Acquisition_msg_rx_Fpga() {}
// ######## GNURADIO TRACKING BLOCK MESSAGE RECEVER #########
class TrackingPullInTestFpga_msg_rx;

typedef boost::shared_ptr<TrackingPullInTestFpga_msg_rx> TrackingPullInTestFpga_msg_rx_sptr;

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
            int64_t message = pmt::to_long(msg);
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


TrackingPullInTestFpga_msg_rx::~TrackingPullInTestFpga_msg_rx()
{
}

// ###########################################################

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

    ~TrackingPullInTestFpga()
    {
    }

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

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], NULL};

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
    //config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.item_type", "cshort");
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    //config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    //config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    //config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;
    gnss_synchro.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;

    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
        {
            gnss_synchro.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            //config->set_property("Tracking.early_late_space_narrow_chips", "0.5");
            // added by me
            config->set_property("Tracking.if", "0");
            config->set_property("Tracking.order", "3");


        }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
        {
            gnss_synchro.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.6");
            //config->set_property("Tracking.early_late_space_narrow_chips", "0.15");
            //config->set_property("Tracking.very_early_late_space_narrow_chips", "0.6");
            config->set_property("Tracking.track_pilot", "true");

            // added by me
            config->set_property("Tracking.if", "0");
            config->set_property("Tracking.devicename", "/dev/uio");
            config->set_property("Tracking.device_base", "15");
        }

    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0) // or implementation.compare("Galileo_E5a_DLL_PLL_Tracking_b") == 0)
        {
            gnss_synchro.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            signal.copy(gnss_synchro.Signal, 2, 0);
            //if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_b") == 0)
            //    {
            //        config->supersede_property("Tracking.implementation", std::string("Galileo_E5a_DLL_PLL_Tracking"));
            //    }
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "false");
            config->set_property("Tracking.order", "2");
        }
    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
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

const size_t PAGE_SIZE = 0x10000;
const unsigned int TEST_REGISTER_TRACK_WRITEVAL = 0x55AA;

void setup_fpga_switch(void)
{
    int switch_device_descriptor;        // driver descriptor
    volatile unsigned *switch_map_base;  // driver memory map

    if ((switch_device_descriptor = open("/dev/uio1", O_RDWR | O_SYNC)) == -1)
        {
            LOG(WARNING) << "Cannot open deviceio" << "/dev/uio1";
        }
    switch_map_base = reinterpret_cast<volatile unsigned *>(mmap(nullptr, PAGE_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, switch_device_descriptor, 0));

    if (switch_map_base == reinterpret_cast<void *>(-1))
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

    switch_map_base[0] = 0; //0 -> DMA to queue 0, 1 -> DMA to queue 1, 2 -> A/Ds to queues
}


static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

volatile unsigned int send_samples_start = 0;

int8_t input_samples[MAX_INPUT_COMPLEX_SAMPLES_TOTAL*COMPLEX_SAMPLE_SIZE]; // re - im
int8_t input_samples_dma[MAX_INPUT_COMPLEX_SAMPLES_TOTAL*COMPLEX_SAMPLE_SIZE*NUM_QUEUES];

struct DMA_handler_args {
    std::string file;
    unsigned int nsamples_tx;
    unsigned int skip_used_samples;
    unsigned int freq_band; // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
};

void *handler_DMA(void *arguments)
{

	// DMA process that configures the DMA to send the samples to the acquisition engine
	int tx_fd; 															// DMA descriptor
	FILE *rx_signal_file_id;											// Input file descriptor
	bool file_completed = false;										// flag to indicate if the file is completed
	unsigned int nsamples_block;										// number of samples to send in the next DMA block of samples
	unsigned int nread_elements;										// number of elements effectively read from the input file
	unsigned int nsamples = 0;											// number of complex samples effectively transferred
	unsigned int index0, dma_index = 0;									// counters used for putting the samples in the order expected by the DMA
	unsigned int num_bytes_to_transfer;									// DMA transfer block size in bytes

	unsigned int nsamples_transmitted;

	struct DMA_handler_args *args = (struct DMA_handler_args *) arguments;

	unsigned int nsamples_tx = args->nsamples_tx;
	//printf("in handler DMA to send %d\n", nsamples_tx);
	std::string file = args->file; // input filename
	unsigned int skip_used_samples = args->skip_used_samples;

	// open DMA device
	tx_fd = open("/dev/loop_tx", O_WRONLY);
	if ( tx_fd < 0 )
	{
		printf("DMA can't open loop device\n");
		exit(1);
	}
	else

	// open input file
	rx_signal_file_id = fopen(file.c_str(), "rb");
	if (rx_signal_file_id < 0)
	{
		printf("DMA can't open input file\n");
		exit(1);
	}
	//printf("in handler DMA waiting for send samples start\n");
	while(send_samples_start == 0); // wait until acquisition starts
	//printf("in handler DMA going to send samples\n");
	// skip initial samples
	int skip_samples = (int) FLAGS_skip_samples;

	fseek( rx_signal_file_id, (skip_samples + skip_used_samples)*2, SEEK_SET );
	//printf("sending %d samples starting at position %d\n", nsamples_tx,skip_samples + skip_used_samples);
	//printf("\n dma skip_samples = %d\n", skip_samples);
	//printf("\n dma skip used samples = %d\n", skip_used_samples);
	//printf("dma skip_samples = %d\n", skip_samples);
	//printf("dma skip used samples = %d\n", skip_used_samples);
	//printf("dma file_completed = %d\n", file_completed);
	//printf("dma nsamples = %d\n", nsamples);
	//printf("dma nsamples_tx = %d\n", nsamples_tx);

	usleep(50000); // wait some time to give time to the main thread to start the acquisition module
	unsigned int kk;
	//printf("enter kk");
	//scanf("%d", &kk);

	while (file_completed == false)
	{
		//printf("samples sent = %d\n", nsamples);
		if (nsamples_tx - nsamples > MAX_INPUT_COMPLEX_SAMPLES_TOTAL)
		{
			nsamples_block = MAX_INPUT_COMPLEX_SAMPLES_TOTAL;
		}
		else
		{
			nsamples_block = nsamples_tx - nsamples; // remaining samples to be sent
			file_completed = true;
		}

		nread_elements = fread(input_samples, sizeof(int8_t), nsamples_block*COMPLEX_SAMPLE_SIZE, rx_signal_file_id);

		if (nread_elements != nsamples_block * COMPLEX_SAMPLE_SIZE)
		{
			printf("could not read the desired number of samples from the input file\n");
			file_completed = true;
		}

		nsamples+=(nread_elements/COMPLEX_SAMPLE_SIZE);

		if (nread_elements > 0)
		{
			// for the 32-BIT DMA
			dma_index = 0;
			for (index0 = 0;index0 < (nread_elements);index0+=COMPLEX_SAMPLE_SIZE)
			{
				if (args->freq_band == 0)
				{
					// channel 1 (queue 1) -> E5/L5
					input_samples_dma[dma_index] = 0;
					input_samples_dma[dma_index+1] = 0;
					// channel 0 (queue 0) -> E1/L1
					input_samples_dma[dma_index+2] = input_samples[index0];
					input_samples_dma[dma_index+3] = input_samples[index0+1];
				}
				else
				{
					// channel 1 (queue 1) -> E5/L5
					input_samples_dma[dma_index] = input_samples[index0];
					input_samples_dma[dma_index+1] = input_samples[index0+1];
					// channel 0 (queue 0) -> E1/L1
					input_samples_dma[dma_index+2] = 0;
					input_samples_dma[dma_index+3] = 0;
				}
				dma_index += 4;
			}
			//printf("writing samples to send\n");
			nsamples_transmitted = write(tx_fd, &input_samples_dma[0], nread_elements*NUM_QUEUES);
			//printf("exited writing samples to send\n");
			if (nsamples_transmitted != nread_elements*NUM_QUEUES)
			{
				std::cout << "Error : DMA could not send all the requested samples" << std::endl;
			}
		}
	}


	close(tx_fd);
	fclose(rx_signal_file_id);
	//printf("DMA finished\n");
	return NULL;
}




void *handler_DMA_tracking(void *arguments)
{
	//printf("in handler DMA NO tracking\n");
	// DMA process that configures the DMA to send the samples to the acquisition engine
	int tx_fd; 															// DMA descriptor
	FILE *rx_signal_file_id;											// Input file descriptor
	bool file_completed = false;										// flag to indicate if the file is completed
	unsigned int nsamples_block;										// number of samples to send in the next DMA block of samples
	unsigned int nread_elements;										// number of elements effectively read from the input file
	unsigned int nsamples = 0;											// number of complex samples effectively transferred
	unsigned int index0, dma_index = 0;									// counters used for putting the samples in the order expected by the DMA
	unsigned int num_bytes_to_transfer;									// DMA transfer block size in bytes

	unsigned int nsamples_transmitted;

	struct DMA_handler_args *args = (struct DMA_handler_args *) arguments;

	unsigned int nsamples_tx = args->nsamples_tx;
	std::string file = args->file; // input filename
	unsigned int skip_used_samples = args->skip_used_samples;

	// open DMA device
	tx_fd = open("/dev/loop_tx", O_WRONLY);
	if ( tx_fd < 0 )
	{
		printf("DMA can't open loop device\n");
		exit(1);
	}
	else

	// open input file
	rx_signal_file_id = fopen(file.c_str(), "rb");
	if (rx_signal_file_id < 0)
	{
		printf("DMA can't open input file\n");
		exit(1);
	}

	while(send_samples_start == 0); // wait until acquisition starts

	// skip initial samples
	int skip_samples = (int) FLAGS_skip_samples;


	fseek( rx_signal_file_id, (skip_samples + skip_used_samples)*2, SEEK_SET );
	//printf("sending %d samples starting at position %d\n", nsamples_tx,skip_samples + skip_used_samples);
	//printf("\nTRK skip 0 samples\n");
	//printf("\n dma skip_samples = %d\n", skip_samples);
	//printf("\n dma skip used samples = %d\n", skip_used_samples);

	//printf("dma file_completed = %d\n", file_completed);
	//printf("dma nsamples = %d\n", nsamples);
	//printf("dma nsamples_tx = %d\n", nsamples_tx);
	usleep(50000); // wait some time to give time to the main thread to start the acquisition module

	while (file_completed == false)
	{
		//printf("dma remaining samples = %d\n", (int) (nsamples_tx - nsamples));
		if (nsamples_tx - nsamples > MAX_INPUT_COMPLEX_SAMPLES_TOTAL)
		{
			nsamples_block = MAX_INPUT_COMPLEX_SAMPLES_TOTAL;
		}
		else
		{
			nsamples_block = nsamples_tx - nsamples; // remaining samples to be sent
			file_completed = true;
		}

		nread_elements = fread(input_samples, sizeof(int8_t), nsamples_block*COMPLEX_SAMPLE_SIZE, rx_signal_file_id);

		if (nread_elements != nsamples_block * COMPLEX_SAMPLE_SIZE)
		{
			printf("could not read the desired number of samples from the input file\n");
			file_completed = true;
		}

		nsamples+=(nread_elements/COMPLEX_SAMPLE_SIZE);

		if (nread_elements > 0)
		{
			// for the 32-BIT DMA
			dma_index = 0;
			for (index0 = 0;index0 < (nread_elements);index0+=COMPLEX_SAMPLE_SIZE)
			{
				if (args->freq_band == 0)
				{
					// channel 1 (queue 1) -> E5/L5
					input_samples_dma[dma_index] = 0;
					input_samples_dma[dma_index+1] = 0;
					// channel 0 (queue 0) -> E1/L1
					input_samples_dma[dma_index+2] = input_samples[index0];
					input_samples_dma[dma_index+3] = input_samples[index0+1];
				}
				else
				{
					// channel 1 (queue 1) -> E5/L5
					input_samples_dma[dma_index] = input_samples[index0];
					input_samples_dma[dma_index+1] = input_samples[index0+1];
					// channel 0 (queue 0) -> E1/L1
					input_samples_dma[dma_index+2] = 0;
					input_samples_dma[dma_index+3] = 0;
				}
				dma_index += 4;
			}

			nsamples_transmitted = write(tx_fd, &input_samples_dma[0], nread_elements*NUM_QUEUES);

			if (nsamples_transmitted != nread_elements*NUM_QUEUES)
			{
				std::cout << "Error : DMA could not send all the requested samples" << std::endl;
			}
		}
	}

	printf("tracking dma process finished\n");
	close(tx_fd);
	fclose(rx_signal_file_id);

	return NULL;
}










bool TrackingPullInTestFpga::acquire_signal(int SV_ID)
{
	pthread_t thread_DMA;

	int baseband_sampling_freq_acquisition;
	// step 0 determine the sampling frequency
	if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
	{
		baseband_sampling_freq_acquisition = baseband_sampling_freq/4;	// downsampling filter in L1/E1
		printf(" aaaaaa baseband_sampling_freq_acquisition = %d\n", baseband_sampling_freq_acquisition);
	}
	else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
	{
		baseband_sampling_freq_acquisition = baseband_sampling_freq/4;  // downsampling filter in L1/E1
		printf(" aaaaaa baseband_sampling_freq_acquisition = %d\n", baseband_sampling_freq_acquisition);
	}

	// 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));
    config->set_property("Acquisition.blocking_on_standby", "true");
    config->set_property("Acquisition.blocking", "true");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.dump_filename", "./data/acquisition.dat");
    config->set_property("Acquisition.use_CFAR_algorithm", "false");

    config->set_property("Acquisition.item_type", "cshort");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.sampled_ms", "4");
    config->set_property("Acquisition.select_queue_Fpga", "0");
    config->set_property("Acquisition.devicename", "/dev/uio0");

    if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
    {
    	config->set_property("Acquisition.acquire_pilot", "false");
    }
    std::shared_ptr<GpsL1CaPcpsAcquisitionFpga> acquisition_GpsL1Ca_Fpga;
    std::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionFpga> acquisition_GpsE1_Fpga;
    std::shared_ptr<GalileoE5aPcpsAcquisitionFpga> acquisition_GpsE5a_Fpga;
    std::shared_ptr<GpsL5iPcpsAcquisitionFpga> acquisition_GpsL5_Fpga;

    std::string System_and_Signal;


    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "1C";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L1 CA";
            acquisition_GpsL1Ca_Fpga = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            acquisition_GpsL1Ca_Fpga->set_channel(0);
            acquisition_GpsL1Ca_Fpga->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
            acquisition_GpsL1Ca_Fpga->connect(top_block);

        }


    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "1B";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E1B";
            //config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            //acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisition>(config.get(), "Acquisition", 1, 0);
            //std::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionFpga> acquisition_Fpga;
            acquisition_GpsE1_Fpga = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            acquisition_GpsE1_Fpga->set_channel(0);
            acquisition_GpsE1_Fpga->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
            acquisition_GpsE1_Fpga->connect(top_block);
            printf(" bbbbbbbb baseband_sampling_freq_acquisition = %d\n", baseband_sampling_freq_acquisition);
        }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
        {
            tmp_gnss_synchro.System = 'E';
            std::string signal = "5X";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";
            //config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            //acquisition = std::make_shared<GalileoE5aPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
            //std::shared_ptr<GalileoE5aPcpsAcquisitionFpga> acquisition_Fpga;
            acquisition_GpsE5a_Fpga = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            acquisition_GpsE5a_Fpga->set_channel(0);
            acquisition_GpsE5a_Fpga->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
            acquisition_GpsE5a_Fpga->connect(top_block);
        }
    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
        {
            tmp_gnss_synchro.System = 'G';
            std::string signal = "L5";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L5I";
            //config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            //acquisition = std::make_shared<GpsL5iPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
            //std::shared_ptr<GpsL5iPcpsAcquisitionFpga> acquisition_Fpga;
            acquisition_GpsL5_Fpga = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

            acquisition_GpsL5_Fpga->set_channel(0);
            acquisition_GpsL5_Fpga->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
            acquisition_GpsL5_Fpga->connect(top_block);
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation \n ";
            throw(std::exception());
        }

    std::string file = FLAGS_signal_file;

    struct DMA_handler_args args;

	const char* file_name = file.c_str();

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

    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
    {
    	top_block->msg_connect(acquisition_GpsL1Ca_Fpga->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
    {
    	top_block->msg_connect(acquisition_GpsE1_Fpga->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    	printf(" cccccc baseband_sampling_freq_acquisition = %d\n", baseband_sampling_freq_acquisition);
    }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
    {
    	top_block->msg_connect(acquisition_GpsE5a_Fpga->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }
    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
    {
    	top_block->msg_connect(acquisition_GpsL5_Fpga->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }

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

    // debug
    //MAX_PRN_IDX = 10;

    setup_fpga_switch();

//    if (doppler_control_in_sw == 0)
//    	{
//
//        args.file = file;
//        args.nsamples_tx = NSAMPLES_ACQ_DOPPLER_SWEEP;		// number of samples to transfer
//        args.skip_used_samples = 0;
//
//
//	    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
//	    {
//	    	acquisition_GpsL1Ca_Fpga->set_single_doppler_flag(0);
//	    	args.freq_band = 0;
//	    }
//	    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
//	    {
//	    	acquisition_GpsE1_Fpga->set_single_doppler_flag(0);
//	    	args.freq_band = 0;
//	    }
//	    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
//	    {
//	    	acquisition_GpsE5a_Fpga->set_single_doppler_flag(0);
//	    	args.freq_band = 1;
//	    }
//	    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
//	    {
//	    	acquisition_GpsL5_Fpga->set_single_doppler_flag(0);
//	    	args.freq_band = 1;
//	    }
//
//        for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
//            {
//
//				tmp_gnss_synchro.PRN = PRN;
//
//			    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsL1Ca_Fpga->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
//			    	acquisition_GpsL1Ca_Fpga->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
//
//			    	acquisition_GpsL1Ca_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
//			    	acquisition_GpsL1Ca_Fpga->init();
//			    	acquisition_GpsL1Ca_Fpga->set_local_code();
//			    }
//			    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsE1_Fpga->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
//			    	acquisition_GpsE1_Fpga->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
//
//			    	acquisition_GpsE1_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
//			    	acquisition_GpsE1_Fpga->init();
//			    	acquisition_GpsE1_Fpga->set_local_code();
//			    }
//			    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsE5a_Fpga->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
//			    	acquisition_GpsE5a_Fpga->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
//
//			    	acquisition_GpsE5a_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
//			    	acquisition_GpsE5a_Fpga->init();
//			    	acquisition_GpsE5a_Fpga->set_local_code();
//			    }
//			    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsL5_Fpga->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
//			    	acquisition_GpsL5_Fpga->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
//
//			    	acquisition_GpsL5_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
//			    	acquisition_GpsL5_Fpga->init();
//			    	acquisition_GpsL5_Fpga->set_local_code();
//			    }
//
//				// create DMA child process
//			    //printf("|||||||| args freq_band = %d\n", args.freq_band);
//				if (pthread_create(&thread_DMA, NULL, handler_DMA, (void *)&args) < 0)
//				{
//					printf("ERROR cannot create DMA Process\n");
//				}
//
//				msg_rx->rx_message = 0;
//				top_block->start();
//
//				pthread_mutex_lock(&mutex);
//				send_samples_start = 1;
//				pthread_mutex_unlock(&mutex);
//
//			    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsL1Ca_Fpga->reset(); // set active
//			    }
//			    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsE1_Fpga->reset(); // set active
//			    }
//			    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsE5a_Fpga->reset(); // set active
//			    }
//			    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
//			    {
//			    	acquisition_GpsL5_Fpga->reset(); // set active
//			    }
//
//				if (start_msg == true)
//					{
//						std::cout << "Reading external signal file: " << FLAGS_signal_file << std::endl;
//						std::cout << "Searching for " << System_and_Signal << " Satellites..." << std::endl;
//						std::cout << "[";
//						start_msg = false;
//					}
//
//				// wait for the child DMA process to finish
//				pthread_join(thread_DMA, NULL);
//
//				pthread_mutex_lock(&mutex);
//				send_samples_start = 0;
//				pthread_mutex_unlock(&mutex);
//
//				while (msg_rx->rx_message == 0)
//					{
//						usleep(100000);
//					}
//
//				if (msg_rx->rx_message == 1)
//					{
//						std::cout << " " << PRN << " ";
//						doppler_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_doppler_hz));
//						code_delay_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_delay_samples));
//						acq_samplestamp_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_samplestamp_samples));
//					}
//				else
//					{
//						std::cout << " . ";
//					}
//				top_block->stop();
//				std::cout.flush();
//            }
//    	}
//    else
//		{

    	unsigned int code_length;
    	unsigned int nsamples_to_transfer;
	    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
	    {
	    	code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq_acquisition) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
	    	nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
	    }
	    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
	    {
	        code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq_acquisition) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)));
	    	nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)));
	    	printf("dddddd code_length = %d nsamples_to_transfer = %d\n", code_length, nsamples_to_transfer);
	    }
	    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
	    {
	        code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / Galileo_E5a_CODE_CHIP_RATE_HZ * static_cast<double>(Galileo_E5a_CODE_LENGTH_CHIPS)));
	    	nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
	    }
	    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
	    {
	        code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L5i_CODE_RATE_HZ / static_cast<double>(GPS_L5i_CODE_LENGTH_CHIPS))));
	    	nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
	    }


        float nbits = ceilf(log2f((float)code_length*2));
        unsigned int fft_size = pow(2, nbits);
        unsigned int nsamples_total = fft_size;

	    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
	    {
	    	acquisition_GpsL1Ca_Fpga->set_single_doppler_flag(1);
	    }
	    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
	    {
	    	acquisition_GpsE1_Fpga->set_single_doppler_flag(1);
	    	printf("eeeeeee just set doppler flag\n");
	    }
	    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
	    {
	    	acquisition_GpsE5a_Fpga->set_single_doppler_flag(1);
	    }
	    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
	    {
	    	acquisition_GpsL5_Fpga->set_single_doppler_flag(1);
	    }

        int acq_doppler_max = config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz);
        int acq_doppler_step = config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz);

        printf("acq_doppler_max = %d\n", acq_doppler_max);
        printf("acq_doppler_step = %d\n", acq_doppler_step);

        int num_doppler_steps = (2*acq_doppler_max)/acq_doppler_step + 1;

        float result_table[MAX_PRN_IDX][num_doppler_steps][3];

        uint32_t index_debug[MAX_PRN_IDX];
        uint32_t samplestamp_debug[MAX_PRN_IDX];

        for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
      	//for (unsigned int PRN = 0; PRN < 17; PRN++)
			{

        	uint32_t max_index = 0;
	        float max_magnitude = 0.0;
	        float second_magnitude = 0.0;
	        uint64_t initial_sample = 0;
	        //float power_sum = 0;
	        uint32_t doppler_index = 0;

	        uint32_t max_index_iteration;
	        uint32_t total_fft_scaling_factor;
	        uint32_t fw_fft_scaling_factor;
	        float max_magnitude_iteration;
	        float second_magnitude_iteration;
	        uint64_t initial_sample_iteration;
	        //float power_sum_iteration;
	        uint32_t doppler_index_iteration;
	        int doppler_shift_selected;
	        int doppler_num = 0;




			for (int doppler_shift = -acq_doppler_max;doppler_shift <= acq_doppler_max;doppler_shift = doppler_shift + acq_doppler_step)
				{
					//printf("doppler_shift = %d\n", doppler_shift);
					tmp_gnss_synchro.PRN = PRN;

					pthread_mutex_lock(&mutex);
					send_samples_start = 0;
					pthread_mutex_unlock(&mutex);

				    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL1Ca_Fpga->reset_acquisition(); // reset the whole system including the sample counters
				    	acquisition_GpsL1Ca_Fpga->set_doppler_max(doppler_shift);
				    	acquisition_GpsL1Ca_Fpga->set_doppler_step(0);

				    	acquisition_GpsL1Ca_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
				    	acquisition_GpsL1Ca_Fpga->init();
				    	acquisition_GpsL1Ca_Fpga->set_local_code();
				    	args.freq_band = 0;
				    }
				    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
				    {
				    	//printf("starting configuring acquisition\n");
				    	acquisition_GpsE1_Fpga->reset_acquisition(); // reset the whole system including the sample counters
				    	acquisition_GpsE1_Fpga->set_doppler_max(doppler_shift);
				    	acquisition_GpsE1_Fpga->set_doppler_step(0);

				    	acquisition_GpsE1_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
				    	acquisition_GpsE1_Fpga->init();
				    	acquisition_GpsE1_Fpga->set_local_code();
				    	args.freq_band = 0;
				    	//printf("ffffffffffff ending configuring acquisition\n");
				    }
				    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsE5a_Fpga->reset_acquisition(); // reset the whole system including the sample counters
				    	acquisition_GpsE5a_Fpga->set_doppler_max(doppler_shift);
				    	acquisition_GpsE5a_Fpga->set_doppler_step(0);

				    	acquisition_GpsE5a_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
				    	acquisition_GpsE5a_Fpga->init();
				    	acquisition_GpsE5a_Fpga->set_local_code();
				    	args.freq_band = 1;
				    }
				    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL5_Fpga->reset_acquisition(); // reset the whole system including the sample counters
				    	acquisition_GpsL5_Fpga->set_doppler_max(doppler_shift);
				    	acquisition_GpsL5_Fpga->set_doppler_step(0);

				    	acquisition_GpsL5_Fpga->set_gnss_synchro(&tmp_gnss_synchro);
				    	acquisition_GpsL5_Fpga->init();
				    	acquisition_GpsL5_Fpga->set_local_code();
				    	args.freq_band = 1;
				    }

			        args.file = file;


			        if ((implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0) or (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0))
			        {
			        	//printf("gggggggg \n");
						//----------------------------------------------------------------------------------
						// send the previous samples to set the downsampling filter in a good condition
						send_samples_start = 0;
						if (skip_samples_already_used == 1)
						{
							args.skip_used_samples = (PRN -1)*fft_size - DOWNAMPLING_FILTER_INIT_SAMPLES; // set the counter 2000 samples before
						}
						else
						{
							args.skip_used_samples = - DOWNAMPLING_FILTER_INIT_SAMPLES; // set the counter 2000 samples before
						}
						args.nsamples_tx = DOWNAMPLING_FILTER_INIT_SAMPLES + DOWNSAMPLING_FILTER_DELAY + DOWNSAMPLING_FILTER_OFFSET_SAMPLES; //50000;		// max size of the FFT
						//printf("sending pre init %d\n", args.nsamples_tx);

						if (pthread_create(&thread_DMA, NULL, handler_DMA, (void *)&args) < 0)
						{
							printf("ERROR cannot create DMA Process\n");
						}
						pthread_mutex_lock(&mutex);
						send_samples_start = 1;
						pthread_mutex_unlock(&mutex);
						pthread_join(thread_DMA, NULL);
						send_samples_start = 0;
						//printf("finished sending samples init filter status\n");
						//-----------------------------------------------------------------------------------

				        // debug
						args.nsamples_tx = nsamples_to_transfer; //fft_size; //50000;		// max size of the FFT

				        if (skip_samples_already_used == 1)
				        {
				        	args.skip_used_samples = (PRN -1)*fft_size + DOWNSAMPLING_FILTER_DELAY + DOWNSAMPLING_FILTER_OFFSET_SAMPLES;
				        }
				        else
				        {
				        	args.skip_used_samples = DOWNSAMPLING_FILTER_DELAY + DOWNSAMPLING_FILTER_OFFSET_SAMPLES;
				        }
			        }
			        else
			        {
				        // debug
						args.nsamples_tx = nsamples_to_transfer; //fft_size; //50000;		// max size of the FFT

				        if (skip_samples_already_used == 1)
				        {
				        	args.skip_used_samples = (PRN -1)*fft_size;
				        }
				        else
				        {
				        	args.skip_used_samples = 0;
				        }
			        }




					// create DMA child process
			        //printf("||||||||1 args freq_band = %d\n", args.freq_band);
			        //printf("sending samples main DMA %d\n", args.nsamples_tx);
					if (pthread_create(&thread_DMA, NULL, handler_DMA, (void *)&args) < 0)
					{
						printf("ERROR cannot create DMA Process\n");
					}

					msg_rx->rx_message = 0;
					top_block->start();

					pthread_mutex_lock(&mutex);
					send_samples_start = 1;
					pthread_mutex_unlock(&mutex);

				    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL1Ca_Fpga->reset(); // set active
				    }
				    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
				    {
				    	//printf("hhhhhhhhhhhh\n");
				    	acquisition_GpsE1_Fpga->reset(); // set active
				    }
				    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsE5a_Fpga->reset(); // set active
				    }
				    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL5_Fpga->reset(); // set active
				    }

//					pthread_mutex_lock(&mutex); // it doesn't work if it is done here
//					send_samples_start = 1;
//					pthread_mutex_unlock(&mutex);

					if (start_msg == true)
						{ 
							std::cout << "Reading external signal file: " << FLAGS_signal_file << std::endl;
							std::cout << "Searching for " << System_and_Signal << " Satellites..." << std::endl;
							std::cout << "[";
							start_msg = false;
						}

					// wait for the child DMA process to finish
					pthread_join(thread_DMA, NULL);

					pthread_mutex_lock(&mutex);
					send_samples_start = 0;
					pthread_mutex_unlock(&mutex);

					while (msg_rx->rx_message == 0)
						{
							usleep(100000);
						}

				    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL1Ca_Fpga->read_acquisition_results(&max_index_iteration, &max_magnitude_iteration, &second_magnitude_iteration, &initial_sample_iteration, &doppler_index_iteration, &total_fft_scaling_factor);
				    	//acquisition_GpsL1Ca_Fpga->read_fpga_total_scale_factor(&total_fft_scaling_factor, &fw_fft_scaling_factor);
				    }
				    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
				    {
				    	//printf("iiiiiiiiiiiiii\n");
				    	acquisition_GpsE1_Fpga->read_acquisition_results(&max_index_iteration, &max_magnitude_iteration, &second_magnitude_iteration, &initial_sample_iteration, &doppler_index_iteration, &total_fft_scaling_factor);
				    	//acquisition_GpsE1_Fpga->read_fpga_total_scale_factor(&total_fft_scaling_factor, &fw_fft_scaling_factor);
				    }
				    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsE5a_Fpga->read_acquisition_results(&max_index_iteration, &max_magnitude_iteration, &second_magnitude_iteration, &initial_sample_iteration, &doppler_index_iteration, &total_fft_scaling_factor);
				    	//acquisition_GpsE5a_Fpga->read_fpga_total_scale_factor(&total_fft_scaling_factor, &fw_fft_scaling_factor);
				    }
				    else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
				    {
				    	acquisition_GpsL5_Fpga->read_acquisition_results(&max_index_iteration, &max_magnitude_iteration, &second_magnitude_iteration, &initial_sample_iteration, &doppler_index_iteration, &total_fft_scaling_factor);
				    	//acquisition_GpsL5_Fpga->read_fpga_total_scale_factor(&total_fft_scaling_factor, &fw_fft_scaling_factor);
				    }

					result_table[PRN][doppler_num][0] = max_magnitude_iteration;
					result_table[PRN][doppler_num][1] = second_magnitude_iteration;
					result_table[PRN][doppler_num][2] = total_fft_scaling_factor;
					doppler_num = doppler_num + 1;

					//printf("max_magnitude_iteration = %f\n", max_magnitude_iteration);
					//printf("second_magnitude_iteration = %f\n", second_magnitude_iteration);
			        if ((implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0) or (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0))
			        {
			        	//printf("jjjjjjjjjjjjjjj\n");
						if (max_magnitude_iteration > max_magnitude)
						{
							int interpolation_factor = 4;
							index_debug[PRN - 1] = max_index_iteration;
							max_index = max_index_iteration; // - interpolation_factor*(DOWNSAMPLING_FILTER_DELAY - 1);
							max_magnitude = max_magnitude_iteration;
							second_magnitude = second_magnitude_iteration;
							samplestamp_debug[PRN - 1] = initial_sample_iteration;
							initial_sample = 0; //initial_sample_iteration;
							doppler_index = doppler_index_iteration;
							doppler_shift_selected = doppler_shift;
						}
			        }
					else
					{

						if (max_magnitude_iteration > max_magnitude)
						{
							max_index = max_index_iteration;
							max_magnitude = max_magnitude_iteration;
							second_magnitude = second_magnitude_iteration;
							initial_sample = initial_sample_iteration;
							doppler_index = doppler_index_iteration;
							doppler_shift_selected = doppler_shift;
						}
					}
					top_block->stop();
				}

		        //power_sum = (power_sum - max_magnitude) / (fft_size - 1);
				//float test_statistics = (max_magnitude / power_sum);
				float test_statistics = max_magnitude/second_magnitude;
				float threshold = config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold);
				if (test_statistics > threshold)
					{
						//printf("XXXXXXXXXXXXXXXXXXXXXXXXXXX max index = %d = %d \n", max_index, max_index % nsamples_total);
						std::cout << " " << PRN << " ";
						doppler_measurements_map.insert(std::pair<int, double>(PRN, static_cast<double>(doppler_shift_selected)));
						code_delay_measurements_map.insert(std::pair<int, double>(PRN, static_cast<double>(max_index % nsamples_total)));
						code_delay_measurements_map.insert(std::pair<int, double>(PRN, static_cast<double>(max_index)));
						acq_samplestamp_map.insert(std::pair<int, double>(PRN, initial_sample)); // should be 0 (first sample upon which acq starts is always 0 in this case)
					}
				else
					{
						std::cout << " . ";
					}

				std::cout.flush();

			}

			uint32_t max_index = 0;
			uint32_t total_fft_scaling_factor;
			//uint32_t fw_fft_scaling_factor;
			float max_magnitude = 0.0;
			uint64_t initial_sample = 0;
			float second_magnitude = 0;
			float peak_to_power = 0;
			float test_statistics;
			uint32_t doppler_index = 0;

			if (show_results_table == 1)
			{
				for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
				{
					std::cout << std::endl << "############################################ Results for satellite " << PRN << std::endl;
					int doppler_num = 0;
					for (int doppler_shift = -acq_doppler_max;doppler_shift <= acq_doppler_max;doppler_shift = doppler_shift + acq_doppler_step)
					{
						max_magnitude = result_table[PRN][doppler_num][0];
						second_magnitude = result_table[PRN][doppler_num][1];
						total_fft_scaling_factor = result_table[PRN][doppler_num][2];
						//fw_fft_scaling_factor = result_table[PRN][doppler_num][3];
						doppler_num = doppler_num + 1;

						std::cout << "==================== Doppler shift " << doppler_shift << std::endl;
						std::cout << "Max magnitude = " << max_magnitude << std::endl;
						std::cout << "Second magnitude = " << second_magnitude << std::endl;
						std::cout << "FFT total scaling factor = " << total_fft_scaling_factor << std::endl;
						//peak_to_power = max_magnitude/power_sum;
						//power_sum = (power_sum - max_magnitude) / (fft_size - 1);
						test_statistics = (max_magnitude / second_magnitude);
						std::cout << " test_statistics = " << test_statistics << std::endl;
					}
					int dummy_val;
					std::cout << "Enter a value to continue";
					std::cin >> dummy_val;
				}
			}

//		}
    std::cout << "]" << std::endl;
    std::cout << "-------------------------------------------\n";

    for (auto& x : doppler_measurements_map)
        {
            std::cout << "DETECTED SATELLITE " << System_and_Signal << " PRN: " << x.first << " with Doppler: " << x.second << " [Hz], code phase: " << code_delay_measurements_map.at(x.first) << " [samples] at signal SampleStamp " << acq_samplestamp_map.at(x.first) << "\n";
        }


    for (unsigned k=0;k<MAX_PRN_IDX;k++)
    {
    	printf("index_debug %d = %d\n", k+1, (int) index_debug[k]);
    	printf("samplestamp_debug %d = %d\n", k+1, (int) samplestamp_debug[k]);
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

	//printf("AAAA000\n");

	struct DMA_handler_args args;

	int interpolation_factor;
	if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
	{
		interpolation_factor = 4;	// downsampling filter in L1/E1
	}
	else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
	{
		interpolation_factor = 4;  // downsampling filter in L1/E1
	}


//	// step 0 determine the sampling frequency
//	if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
//	{
//		baseband_sampling_freq = baseband_sampling_freq/4;	// downsampling filter in L1/E1
//	}
//	else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
//	{
//		baseband_sampling_freq = baseband_sampling_freq/4;  // downsampling filter in L1/E1
//	}



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

    //printf("BBB\n");

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

    //printf("CCC\n");

    // DEBUG

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

    //printf("#################################### CONFIGURE \n");
    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols);

    //printf("DDD\n");

    //******************************************************************************************
    //***** Obtain the initial signal sinchronization parameters (emulating an acquisition) ****
    //******************************************************************************************
    int test_satellite_PRN = 0;
    double true_acq_doppler_hz = 0.0;
    double true_acq_delay_samples = 0.0;
    uint64_t acq_samplestamp_samples = 0;

    //printf("#################################### NEXT\n");
    tracking_true_obs_reader true_obs_data;
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
	if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
	{
		code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));
	}
	else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
	{
		code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)));
	}
	else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
	{
		code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / Galileo_E5a_CODE_CHIP_RATE_HZ * static_cast<double>(Galileo_E5a_CODE_LENGTH_CHIPS)));

	}
	else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
	{
		code_length = static_cast<unsigned int>(std::round(static_cast<double>(baseband_sampling_freq) / (GPS_L5i_CODE_RATE_HZ / static_cast<double>(GPS_L5i_CODE_LENGTH_CHIPS))));
	}

	float nbits = ceilf(log2f((float)code_length));
	unsigned int fft_size = pow(2, nbits);


	printf("####################################\n");
    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
        {
            std::vector<double> pull_in_results_v;
            for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
                {
                    for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                        {
                    		// DEBUG TEST THE RESULTS OF THE SW
                    		//acq_samplestamp_samples = 108856983;
                    		//true_acq_doppler_hz = 3250;
                    		//true_acq_delay_samples = 836;
                			//acq_samplestamp_samples = 0;
                			//true_acq_doppler_hz = -3000;
                			//true_acq_delay_samples = 748;

//							acq_samplestamp_samples = 636461056;
//							true_acq_doppler_hz = -3125;
//							true_acq_delay_samples = 1077;

//							acq_samplestamp_samples = 104898560;
//							true_acq_doppler_hz = -2500;
//							true_acq_delay_samples = 5353;

							//acq_samplestamp_samples = 79470544;
							//true_acq_doppler_hz = -4000;
							//true_acq_delay_samples = 1292;

//                    		if ((implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0) or (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")))
//                    		{
//                    			acq_samplestamp_samples = 0;
//
//                    			true_acq_delay_samples = true_acq_delay_samples - interpolation_factor*(DOWNSAMPLING_FILTER_DELAY - 1);
								printf("acq_samplestamp_samples = %d\n", (int)acq_samplestamp_samples);
								printf("true_acq_delay_samples = %d\n", (int)true_acq_delay_samples);
//                    		}
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
                            //printf("loop part b2\n");

                            if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0)
                            {
                            	std::shared_ptr<GpsL1CaPcpsAcquisitionFpga> acquisition_Fpga;
                            	acquisition_Fpga = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                            	args.freq_band = 0;
                            }
                            else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga") == 0)
                            {
                            	std::shared_ptr<GalileoE1PcpsAmbiguousAcquisitionFpga> acquisition_Fpga;
                            	acquisition_Fpga = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                            	args.freq_band = 0;
                            }
                            else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking_Fpga") == 0)
                            {
                            	std::shared_ptr<GalileoE5aPcpsAcquisitionFpga> acquisition_Fpga;
                            	acquisition_Fpga = std::make_shared<GalileoE5aPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                            	args.freq_band = 1;
                            }
                            else if (implementation.compare("GPS_L5_DLL_PLL_Tracking_Fpga") == 0)
                            {
                            	std::shared_ptr<GpsL5iPcpsAcquisitionFpga> acquisition_Fpga;
                            	acquisition_Fpga = std::make_shared<GpsL5iPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);
                            	args.freq_band = 1;
                            }
                            else
                            {
                                std::cout << "The test can not run with the selected tracking implementation\n ";
                                throw(std::exception());
                            }

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

//        			        if ((implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_Fpga") == 0) or (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")))
//        			        {
//        						//----------------------------------------------------------------------------------
//        						// send the previous samples to set the downsampling filter in a good condition
//        						send_samples_start = 0;
//                                if (skip_samples_already_used == 1 && doppler_control_in_sw == 1)
//                                {
//                                	args.skip_used_samples = (gnss_synchro.PRN - 1)*fft_size;
//                                }
//                                else
//                                {
//                                	args.skip_used_samples = 0;
//                                }
//        						args.nsamples_tx = DOWNSAMPLING_FILTER_DELAY; //50000;		// max size of the FFT
//        						if (pthread_create(&thread_DMA, NULL, handler_DMA, (void *)&args) < 0)
//        						{
//        							printf("ERROR cannot create DMA Process\n");
//        						}
//        						pthread_mutex_lock(&mutex);
//        						send_samples_start = 1;
//        						pthread_mutex_unlock(&mutex);
//        						pthread_join(thread_DMA, NULL);
//        						send_samples_start = 0;
//        						//printf("finished sending samples init filter status\n");
//        						//-----------------------------------------------------------------------------------
//                                if (skip_samples_already_used == 1 && doppler_control_in_sw == 1)
//                                {
//                                	args.skip_used_samples = (gnss_synchro.PRN - 1)*fft_size + DOWNSAMPLING_FILTER_DELAY;
//                                }
//                                else
//                                {
//                                	args.skip_used_samples = DOWNSAMPLING_FILTER_DELAY;
//                                }
//
//        			        }
//        			        else
//        			        {
                                if (skip_samples_already_used == 1 && doppler_control_in_sw == 1)
                                {
                                	args.skip_used_samples = (gnss_synchro.PRN - 1)*fft_size;
                                }
                                else
                                {
                                	args.skip_used_samples = 0;
                                }
 //       			        }


                            //********************************************************************
                            //***** STEP 5: Perform the signal tracking and read the results *****
                            //********************************************************************

                            //std::string file = FLAGS_signal_file;

                            //args.file = file;
                            args.nsamples_tx = NSAMPLES_TRACKING;		// number of samples to transfer


//                            if (skip_samples_already_used == 1 && doppler_control_in_sw == 1)
//                            {
//                            	args.skip_used_samples = (gnss_synchro.PRN - 1)*fft_size;
//                            }
//                            else
//                            {
//                            	args.skip_used_samples = 0;
//                            }
                            //args.skip_used_samples = 0;

                            //printf("||||||||1 args freq_band = %d\n", args.freq_band);
                            if (pthread_create(&thread_DMA, NULL, handler_DMA_tracking, (void *)&args) < 0)
                        	{
                        		printf("ERROR cannot create DMA Process\n");
                        	}

                            std::cout << "--- START TRACKING WITH PULL-IN ERROR: " << acq_doppler_error_hz_values.at(current_acq_doppler_error_idx) << " [Hz] and " << acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) << " [Chips] ---" << std::endl;

                            tracking->start_tracking();

                            pthread_mutex_lock(&mutex);
                            send_samples_start = 1;
                            pthread_mutex_unlock(&mutex);

                            top_block->start();

                        	// wait for the child DMA process to finish
                        	pthread_join(thread_DMA, NULL);

                        	top_block->stop();

                        	printf("going to send more samples\n");
                        	// send more samples to unblock the tracking process in case it was waiting for samples
                            args.file = file;
                            if (skip_samples_already_used == 1 && doppler_control_in_sw == 1)
                            {
                            	// skip the samples that have already been used
                            	args.skip_used_samples = (gnss_synchro.PRN - 1)*fft_size + args.nsamples_tx;
                            }
                            else
                            {
                            	args.skip_used_samples = 0;
                            }
                            args.nsamples_tx = NSAMPLES_FINAL;
                            //printf("||||||||1 args freq_band = %d\n", args.freq_band);
                            if (pthread_create(&thread_DMA, NULL, handler_DMA, (void *)&args) < 0)
                        	{
                        		printf("ERROR cannot create DMA Process\n");
                        	}
                        	pthread_join(thread_DMA, NULL);
                        	pthread_mutex_lock(&mutex);
                        	send_samples_start = 0;
                        	pthread_mutex_unlock(&mutex);

                            pull_in_results_v.push_back(msg_rx->rx_message != 3);  //save last asynchronous tracking message in order to detect a loss of lock

                            //********************************
                            //***** STEP 7: Plot results *****
                            //********************************
                            if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                {
                                    //load the measured values
                                    tracking_dump_reader trk_dump;
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
                                                    boost::filesystem::path p(gnuplot_executable);
                                                    boost::filesystem::path dir = p.parent_path();
                                                    std::string gnuplot_path = dir.native();
                                                    Gnuplot::set_GNUPlotPath(gnuplot_path);
                                                    unsigned int decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

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

                                                            g1.set_grid();
                                                            g1.set_xlabel("Time [s]");
                                                            g1.set_ylabel("Correlators' output");
                                                            //g1.cmd("set key box opaque");
                                                            g1.plot_xy(trk_timestamp_s, prompt, "Prompt", decimate);
                                                            g1.plot_xy(trk_timestamp_s, early, "Early", decimate);
                                                            g1.plot_xy(trk_timestamp_s, late, "Late", decimate);
                                                            if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
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
                                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
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
                                                                    g3.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
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
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g3.set_legend();
                                                            g3.savetops("CN0_output");

                                                            g3.showonscreen();  // window output

                                                            Gnuplot g4("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g4.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
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
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

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
                }          //end acquisition Doppler errors loop

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
				g4.cmd("set palette defined ( 0 \"black\", 1 \"green\" )");
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

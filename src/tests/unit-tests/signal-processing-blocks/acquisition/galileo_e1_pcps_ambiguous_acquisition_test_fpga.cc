/*!
 * \file gps_l1_ca_pcps_acquisition_test_fpga.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsAcquisitionFpga class based on some input parameters.
 * \author Marc Majoral, 2017. mmajoral(at)cttc.cat
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "concurrent_queue.h"
#include "fpga_switch.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
//#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "in_memory_configuration.h"
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <chrono>
#include <cstdlib>
#include <cmath>      // for abs, pow, floor
#include <fcntl.h>  // for O_WRONLY
#include <unistd.h>
#include <utility>
#include <pthread.h>   // for pthread stuff
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif

struct DMA_handler_args_galileo_e1_pcps_ambiguous_acq_test
{
    std::string file;
    int32_t nsamples_tx;
    int32_t skip_used_samples;
    unsigned int freq_band;  // 0 for GPS L1/ Galileo E1, 1 for GPS L5/Galileo E5
};

struct acquisition_handler_args_galileo_e1_pcps_ambiguous_acq_test
{
	std::shared_ptr<AcquisitionInterface> acquisition;
};

class GalileoE1PcpsAmbiguousAcquisitionTestFpga : public ::testing::Test
{
public:
	bool acquire_signal();
    std::string implementation = "GPS_L1_CA_DLL_PLL_Tracking_Fpga";
    std::vector<Gnss_Synchro> gnss_synchro_vec;

    static const int32_t TEST_ACQ_SKIP_SAMPLES = 1024;
    static const int BASEBAND_SAMPLING_FREQ = 4000000;

protected:

    GalileoE1PcpsAmbiguousAcquisitionTestFpga();
    ~GalileoE1PcpsAmbiguousAcquisitionTestFpga() = default;

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    unsigned int doppler_max;
    unsigned int doppler_step;
    unsigned int nsamples_to_transfer;

};



GalileoE1PcpsAmbiguousAcquisitionTestFpga::GalileoE1PcpsAmbiguousAcquisitionTestFpga()
{
    factory = std::make_shared<GNSSBlockFactory>();
    config = std::make_shared<InMemoryConfiguration>();
    item_size = sizeof(gr_complex);
    gnss_synchro = Gnss_Synchro();
    doppler_max = 5000;
    doppler_step = 100;
}

void* handler_DMA_galileo_e1_pcps_ambiguous_acq_test(void* arguments)
{
	//const float MAX_SAMPLE_VALUE = 0.097781330347061;
	const float MAX_SAMPLE_VALUE = 0.096257761120796;
	const int DMA_BITS_PER_SAMPLE = 8;
	const float DMA_SCALING_FACTOR = (pow(2, DMA_BITS_PER_SAMPLE - 1) - 1) / MAX_SAMPLE_VALUE;
	const int MAX_INPUT_SAMPLES_TOTAL = 16384;

	auto* args = (struct DMA_handler_args_galileo_e1_pcps_ambiguous_acq_test*)arguments;

	std::string Filename = args->file;  // input filename
	int32_t skip_used_samples = args->skip_used_samples;
	int32_t nsamples_tx = args->nsamples_tx;

    std::vector<float> input_samples(MAX_INPUT_SAMPLES_TOTAL * 2);
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
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception opening file " << Filename << std::endl;
            return nullptr;
        }

    //**************************************************************************
    // Open DMA device
    //**************************************************************************
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 0)
        {
            std::cout << "Cannot open loop device" << std::endl;
            return nullptr;
        }

    //**************************************************************************
    // Open input file
    //**************************************************************************

    uint32_t skip_samples = 0; //static_cast<uint32_t>(FLAGS_skip_samples);

	if (skip_samples + skip_used_samples > 0)
	{
		try
			{
				infile.ignore((skip_samples + skip_used_samples) * 2);
			}
		catch (const std::ifstream::failure &e)
			{
				std::cerr << "Exception reading file " << Filename << std::endl;
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
                            infile.read(reinterpret_cast<char *>(input_samples.data()), nsamples_block_size * 2 * sizeof(float));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << Filename << std::endl;
                        }

                    for (int index0 = 0; index0 < (nsamples_block_size * 2); index0 += 2)
                        {

							if (args->freq_band == 0)
								{
									// channel 1 (queue 1) -> E5/L5
									input_samples_dma[dma_index] = 0;
									input_samples_dma[dma_index + 1] = 0;
									// channel 0 (queue 0) -> E1/L1
									input_samples_dma[dma_index + 2] = static_cast<int8_t>(input_samples[index0]*DMA_SCALING_FACTOR);
									input_samples_dma[dma_index + 3] = static_cast<int8_t>(input_samples[index0 + 1]*DMA_SCALING_FACTOR);
								}
							else
								{
									// channel 1 (queue 1) -> E5/L5
									input_samples_dma[dma_index] = static_cast<int8_t>(input_samples[index0]*DMA_SCALING_FACTOR);
									input_samples_dma[dma_index + 1] = static_cast<int8_t>(input_samples[index0 + 1]*DMA_SCALING_FACTOR);
									// channel 0 (queue 0) -> E1/L1
									input_samples_dma[dma_index + 2] = 0;
									input_samples_dma[dma_index + 3] = 0;
								}

                            dma_index += 4;

                        }

			if (write(tx_fd, input_samples_dma.data(), nsamples_block_size * 2 * 2) != nsamples_block_size * 2 * 2)
				{
					std::cerr << "Error: DMA could not send all the required samples " << std::endl;
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
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception closing files " << Filename << std::endl;
        }

    try
        {
    		close(tx_fd);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception closing loop device " << std::endl;
        }

	return nullptr;
}



void* handler_acquisition_galileo_e1_pcps_ambiguous_acq_test(void* arguments)
{
	// the acquisition is a blocking function so we have to
	// create a thread
	auto* args = (struct acquisition_handler_args_galileo_e1_pcps_ambiguous_acq_test*)arguments;
	args->acquisition->reset();
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
class ChannelFsm_galileo_e1_pcps_ambiguous_acq_test: public ChannelFsm
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



bool GalileoE1PcpsAmbiguousAcquisitionTestFpga::acquire_signal()
{

    pthread_t thread_DMA, thread_acquisition;

    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    int SV_ID = 1;  // initial sv id

    // fsm
    std::shared_ptr<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test>();
    bool acquisition_successful;

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;

    std::shared_ptr<AcquisitionInterface> acquisition;

//    std::string System_and_Signal;
    std::string signal;
    struct DMA_handler_args_galileo_e1_pcps_ambiguous_acq_test args;
    struct acquisition_handler_args_galileo_e1_pcps_ambiguous_acq_test args_acq;

    std::string file = "data/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    args.file = file; // DMA file configuration

    // instantiate the FPGA switch and set the
    // switch position to DMA.
    std::shared_ptr<Fpga_Switch> switch_fpga;
    switch_fpga = std::make_shared<Fpga_Switch>("/dev/uio1");
    switch_fpga->set_switch_position(0);     // set switch position to DMA

    // create the correspondign acquisition block according to the desired tracking signal
    tmp_gnss_synchro.System = 'E';
	signal = "1B";
	const char* str = signal.c_str();                                  // get a C style null terminated string
	std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 2);  // copy string into synchro char array: 2 char + null
	tmp_gnss_synchro.PRN = SV_ID;
//	System_and_Signal = "GPS L1 CA";
	const std::string& role = "Acquisition";
	acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

	args.freq_band = 1;	// frequency band on which the DMA has to transfer the samples

    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_channel_fsm(channel_fsm_);
    acquisition->set_channel(1);
    acquisition->set_doppler_max(doppler_max);
    acquisition->set_doppler_step(doppler_step);
    acquisition->set_doppler_center(0);
    acquisition->set_threshold(0.001);

    nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(BASEBAND_SAMPLING_FREQ) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));

	channel_fsm_->Event_clear_test_result();

	acquisition->stop_acquisition();  // reset the whole system including the sample counters
	acquisition->init();
	acquisition->set_local_code();

	args.skip_used_samples = 0;

	// Configure the DMA to send the required samples to perform an acquisition
	args.nsamples_tx = nsamples_to_transfer;

	// run the acquisition. The acquisition must run in a separate thread because it is a blocking function
	args_acq.acquisition = acquisition;

	if (pthread_create(&thread_acquisition, nullptr, handler_acquisition_galileo_e1_pcps_ambiguous_acq_test, reinterpret_cast<void*>(&args_acq)) < 0)
		{
			std::cout << "ERROR cannot create acquisition Process" << std::endl;
		}

	// wait to give time for the acquisition thread to set up the acquisition HW accelerator in the FPGA
	usleep(1000000);

	// create DMA child process
	if (pthread_create(&thread_DMA, nullptr, handler_DMA_galileo_e1_pcps_ambiguous_acq_test, reinterpret_cast<void*>(&args)) < 0)
		{
			std::cout << "ERROR cannot create DMA Process" << std::endl;
		}

	// wait until the acquisition is finished
	pthread_join(thread_acquisition, nullptr);

	// wait for the child DMA process to finish
	pthread_join(thread_DMA, nullptr);

	acquisition_successful = channel_fsm_->Event_check_test_result();

	if (acquisition_successful)
		{
			gnss_synchro_vec.push_back(tmp_gnss_synchro);
		}

    if (!gnss_synchro_vec.empty())
        {
            return true;
        }
    else
        {
            return false;
        }
}


void GalileoE1PcpsAmbiguousAcquisitionTestFpga::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    std::string signal = "1B";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition_Fpga");
    config->set_property("Acquisition.threshold", "0.00001");
    config->set_property("Acquisition.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition.repeat_satellite", "false");

    // the test file is sampled @ 4MSPs only ,so we have to use the FPGA queue corresponding
    // to the L5/E5a frequency band in order to avoid the L1/E1 factor :4 downsampling filter
    config->set_property("Acquisition.downsampling_factor", "1");
    config->set_property("Acquisition.select_queue_Fpga", "1");
    config->set_property("Acquisition.total_block_exp", "14");
}

TEST_F(GalileoE1PcpsAmbiguousAcquisitionTestFpga, ValidationOfResults)
{
    struct DMA_handler_args_galileo_e1_pcps_ambiguous_acq_test args;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    double expected_delay_samples = 2920;  // 18250;
    double expected_doppler_hz = -632;

    init();

    start = std::chrono::system_clock::now();

    ASSERT_EQ(acquire_signal(), true);

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;

    uint32_t n = 0; // there is only one channel
    std::cout << "Acquired " << nsamples_to_transfer << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;

	double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro_vec.at(n).Acq_delay_samples);
    auto delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro_vec.at(n).Acq_doppler_hz);

    // the acquisition grid is not available when using the FPGA

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";

}


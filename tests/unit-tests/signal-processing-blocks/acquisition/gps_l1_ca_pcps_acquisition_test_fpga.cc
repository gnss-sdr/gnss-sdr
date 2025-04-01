/*!
 * \file gps_l1_ca_pcps_acquisition_test_fpga.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsAcquisitionFpga class based on some input parameters.
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.cat
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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
#include "concurrent_queue.h"
#include "fpga_switch.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "in_memory_configuration.h"
#include "test_flags.h"
#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>    // for abs, pow, floor
#include <fcntl.h>  // for O_WRONLY
#include <pthread.h>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif


struct DMA_handler_args_gps_l1_acq_test
{
    std::string file;
    int32_t nsamples_tx;
    int32_t skip_used_samples;
    float scaling_factor;
};

struct acquisition_handler_args_gps_l1_acq_test
{
    std::shared_ptr<AcquisitionInterface> acquisition;
};

class GpsL1CaPcpsAcquisitionTestFpga : public ::testing::Test
{
public:
    bool acquire_signal();
    std::string implementation = "GPS_L1_CA_DLL_PLL_Tracking_FPGA";
    std::vector<Gnss_Synchro> gnss_synchro_vec;

    const int32_t TEST_ACQ_SKIP_SAMPLES = 1024;
    const int BASEBAND_SAMPLING_FREQ = 4000000;
    const float MAX_SAMPLE_VALUE = 0.096257761120796;
    const int DMA_BITS_PER_SAMPLE = 8;
    const float DMA_SIGNAL_SCALING_FACTOR = (pow(2, DMA_BITS_PER_SAMPLE - 1) - 1) / MAX_SAMPLE_VALUE;

protected:
    GpsL1CaPcpsAcquisitionTestFpga();
    ~GpsL1CaPcpsAcquisitionTestFpga() = default;

    void init();

    std::shared_ptr<InMemoryConfiguration> config;

    unsigned int doppler_max;
    unsigned int doppler_step;
    unsigned int nsamples_to_transfer;
};


GpsL1CaPcpsAcquisitionTestFpga::GpsL1CaPcpsAcquisitionTestFpga()
{
    config = std::make_shared<InMemoryConfiguration>();

    doppler_max = 5000;
    doppler_step = 100;
    nsamples_to_transfer = 0;
}


void* handler_DMA_gps_l1_acq_test(void* arguments)
{
    const int MAX_INPUT_SAMPLES_TOTAL = 16384;

    auto* args = (struct DMA_handler_args_gps_l1_acq_test*)arguments;

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
    uint32_t skip_samples = 0;  // static_cast<uint32_t>(FLAGS_skip_samples);

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
                    infile.read(reinterpret_cast<char*>(input_samples.data()), nsamples_block_size * 2 * sizeof(float));
                }
            catch (const std::ifstream::failure& e)
                {
                    std::cerr << "Exception reading file " << Filename << '\n';
                }

            for (int index0 = 0; index0 < (nsamples_block_size * 2); index0 += 2)
                {
                    // channel 1 (queue 1) -> E5/L5
                    input_samples_dma[dma_index] = static_cast<int8_t>(input_samples[index0] * args->scaling_factor);
                    input_samples_dma[dma_index + 1] = static_cast<int8_t>(input_samples[index0 + 1] * args->scaling_factor);
                    // channel 0 (queue 0) -> E1/L1
                    input_samples_dma[dma_index + 2] = 0;
                    input_samples_dma[dma_index + 3] = 0;

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


void* handler_acquisition_gps_l1_acq_test(void* arguments)
{
    // the acquisition is a blocking function so we have to
    // create a thread
    auto* args = (struct acquisition_handler_args_gps_l1_acq_test*)arguments;
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
class ChannelFsm_gps_l1_acq_test : public ChannelFsm
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
    bool acquisition_successful{};
};


bool GpsL1CaPcpsAcquisitionTestFpga::acquire_signal()
{
    pthread_t thread_DMA, thread_acquisition;

    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    int SV_ID = 1;  // initial sv id

    // fsm
    std::shared_ptr<ChannelFsm_gps_l1_acq_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_gps_l1_acq_test>();
    bool acquisition_successful;

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string signal;
    struct DMA_handler_args_gps_l1_acq_test args;
    struct acquisition_handler_args_gps_l1_acq_test args_acq;

    // set the scaling factor
    args.scaling_factor = DMA_SIGNAL_SCALING_FACTOR;

    std::string file = "data/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    args.file = std::move(file);  // DMA file configuration

    // instantiate the FPGA switch and set the
    // switch position to DMA.
    std::shared_ptr<Fpga_Switch> switch_fpga;
    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(0);  // set switch position to DMA

    // create the correspondign acquisition block according to the desired tracking signal
    tmp_gnss_synchro.System = 'G';
    signal = "1C";
    const char* str = signal.c_str();                                  // get a C style null terminated string
    std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
    tmp_gnss_synchro.PRN = SV_ID;
    acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition", 0, 0);

    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_channel_fsm(channel_fsm_);
    acquisition->set_channel(1);
    acquisition->set_doppler_max(doppler_max);
    acquisition->set_doppler_step(doppler_step);
    acquisition->set_doppler_center(0);
    acquisition->set_threshold(0.001);

    nsamples_to_transfer = static_cast<unsigned int>(std::round(static_cast<double>(BASEBAND_SAMPLING_FREQ) / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    channel_fsm_->Event_clear_test_result();

    acquisition->stop_acquisition();  // reset the whole system including the sample counters
    acquisition->init();
    acquisition->set_local_code();

    args.skip_used_samples = 0;

    // Configure the DMA to send the required samples to perform an acquisition
    args.nsamples_tx = nsamples_to_transfer;

    // run the acquisition. The acquisition must run in a separate thread because it is a blocking function
    args_acq.acquisition = std::move(acquisition);

    if (pthread_create(&thread_acquisition, nullptr, handler_acquisition_gps_l1_acq_test, reinterpret_cast<void*>(&args_acq)) < 0)
        {
            std::cout << "ERROR cannot create acquisition Process\n";
        }

    // wait to give time for the acquisition thread to set up the acquisition HW accelerator in the FPGA
    usleep(1000000);

    // create DMA child process
    if (pthread_create(&thread_DMA, nullptr, handler_DMA_gps_l1_acq_test, reinterpret_cast<void*>(&args)) < 0)
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


void GpsL1CaPcpsAcquisitionTestFpga::init()
{
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition_FPGA");
    config->set_property("Acquisition.threshold", "0.00001");
    config->set_property("Acquisition.doppler_max", std::to_string(doppler_max));
    config->set_property("Acquisition.doppler_step", std::to_string(doppler_step));
    config->set_property("Acquisition.repeat_satellite", "false");

    // the test file is sampled @ 4MSPs only ,so we have to use the FPGA queue corresponding
    // to the L5/E5a frequency band in order to avoid the L1/E1 factor :4 downsampling filter
    config->set_property("Acquisition.downsampling_factor", "1");
    config->set_property("Acquisition.select_queue_Fpga", "1");
    config->set_property("Acquisition.total_block_exp", "12");
}


TEST_F(GpsL1CaPcpsAcquisitionTestFpga, ValidationOfResults)
{
    struct DMA_handler_args_gps_l1_acq_test args;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    double expected_delay_samples = 524;
    double expected_doppler_hz = 1680;

    init();

    start = std::chrono::system_clock::now();

    ASSERT_EQ(acquire_signal(), true);

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;

    uint32_t n = 0;  // there is only one channel
    std::cout << "Acquired " << nsamples_to_transfer << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro_vec.at(n).Acq_delay_samples);
    auto delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro_vec.at(n).Acq_doppler_hz);

    // the acquisition grid is not available when using the FPGA

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}

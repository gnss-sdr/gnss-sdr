/*!
 * \file galileo_e1_pcps_acquisition_test_fpga.cc
 * \brief  This class implements an acquisition test for the
 *         Galileo FPGA acquisition
 * \authors <ul>
 *          <li> Marc Majoral, 2019-2026. mmajoral(at)cttc.cat
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "Galileo_E1.h"                     // Galileo E1 signal parameters
#include "acquisition_interface.h"          // AcquisitionInterface
#include "channel_fsm.h"                    // ChannelFsm
#include "fpga_dma-proxy.h"                 // Fpga_DMA
#include "fpga_switch.h"                    // Fpga_Switch
#include "gnss_synchro.h"                   // Gnss_Synchro
#include "in_memory_configuration.h"        // InMemoryConfiguration
#include "pcps_acquisition_adapter_fpga.h"  // PcpsAcquisitionAdapterFpga
#include "uio_fpga.h"                       // find_uio_dev_file_name
#include <gtest/gtest.h>                    // GoogleTest
#include <chrono>                           // clocks, durations, milliseconds
#include <cmath>                            // std::abs
#include <cstddef>                          // std::size_t
#include <cstdint>                          // fixed-width integer types
#include <cstring>                          // std::memcpy
#include <fcntl.h>                          // open, O_RDWR, O_SYNC
#include <fstream>                          // std::ifstream
#include <iostream>                         // std::cout, std::endl
#include <memory>                           // std::shared_ptr, std::make_shared
#include <string>                           // std::string, std::to_string
#include <sys/mman.h>                       // mmap, munmap
#include <thread>                           // std::thread, std::this_thread
#include <unistd.h>                         // close, usleep
#include <vector>                           // std::vector


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

class ChannelFsm_galileo_e1_pcps_ambiguous_acq_test;

class GalileoE1PcpsAmbiguousAcquisitionTestFpga : public ::testing::Test
{
protected:
    // acquisition configuration
    static constexpr int BASEBAND_SAMPLING_RATE_SPS = 12500000;
    static constexpr int DOPPLER_MAX_HZ = 5000;
    static constexpr int DOPPLER_STEP_HZ = 125;
    static constexpr float ACQUISITION_THRESHOLD = 2.25f;
    static constexpr int TOTAL_BLK_EXP = 12;
    static constexpr char IMPLEMENTATION[] = "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA";

    // acquisition expected results
    static constexpr double EXPECTED_DELAY_SAMPLES = 42964;
    static constexpr double EXPECTED_DOPPLER_HZ = 1625;

    // Signal parameters
    static constexpr int SV_ID = 1;
    static constexpr int COHERENT_INTEGRATION_TIME_ms = 4;
    static constexpr char SYSTEM = 'E';
    static constexpr char SIGNAL[] = "1B";
    static constexpr unsigned int NSAMPLES =
        static_cast<unsigned int>(
            (static_cast<std::uint64_t>(BASEBAND_SAMPLING_RATE_SPS) *
                static_cast<std::uint64_t>(COHERENT_INTEGRATION_TIME_ms)) /
            1000U);

    // File to DMA control
    static constexpr int COMPLEX_SAMPLE_SIZE_BYTES = sizeof(int8_t) * 2;  // interleaved byte
    static constexpr int SAMPLE_BLOCK_SIZE_BYTES = 16384;
    static constexpr char SIGNAL_FILE_PATH[] = "./signal_samples/Galileo_E1_ID_1_Fs_12.5Msps_10ms.dat";

    // FPGA switch
    static constexpr int POST_PROCESSING_MODE = 0;  // Select post-processing mode (read a signal from a recorded file)

    // FPGA Dynamic bit selection
    static constexpr int DYN_BIT_SEL_DEV_NUM = 0;  // device 0 is connected to the L1/E1 frequency-band path.
    static constexpr size_t FPGA_PAGE_SIZE_BYTES = 0x1000;
    static constexpr uint32_t DYN_BIT_SEL_SHIFT_OUT_BITS = 0;                   // No bit shift; select the least significant bits.
    static constexpr char DYN_BIT_SEL_DEVICE_NAME[] = "dynamic_bits_selector";  // device name
    static constexpr int SOBITS_REG_ADDR = 0;                                   // Shift out bits register address

    GalileoE1PcpsAmbiguousAcquisitionTestFpga();
    ~GalileoE1PcpsAmbiguousAcquisitionTestFpga() = default;

    void init();

    void create_and_open_DMA(std::shared_ptr<Fpga_DMA> &dma_fpga);
    void create_switch(std::shared_ptr<Fpga_Switch> &fpga_switch);
    void open_and_map_dynamic_bit_selector(int &dyn_bit_sel_dev_descr, volatile unsigned *&d_map_base_dyn_bit_sel);
    void create_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition);

    void configure_switch(std::shared_ptr<Fpga_Switch> &fpga_switch);
    void configure_dynamic_bit_selector(volatile unsigned *&d_map_base_dyn_bit_sel);
    void configure_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition, Gnss_Synchro &tmp_gnss_synchro, std::shared_ptr<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test> &channel_fsm_);

    void run_DMA_process(std::shared_ptr<Fpga_DMA> &dma_fpga);
    void run_acquisition_process(std::shared_ptr<AcquisitionInterface> acquisition);

    void close_DMA(std::shared_ptr<Fpga_DMA> &dma_fpga);
    void close_switch(std::shared_ptr<Fpga_Switch> &fpga_switch);
    void unmap_and_close_dynamic_bit_selector(int &dyn_bit_sel_dev_descr, volatile unsigned *&d_map_base_dyn_bit_sel);
    void release_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition);

    std::shared_ptr<InMemoryConfiguration> config;
};

#if __cplusplus < 201703L
// Storage for constexpr members that are odr-used before C++17.
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::BASEBAND_SAMPLING_RATE_SPS;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::DOPPLER_MAX_HZ;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::DOPPLER_STEP_HZ;
constexpr float GalileoE1PcpsAmbiguousAcquisitionTestFpga::ACQUISITION_THRESHOLD;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::TOTAL_BLK_EXP;
constexpr char GalileoE1PcpsAmbiguousAcquisitionTestFpga::IMPLEMENTATION[];
constexpr double GalileoE1PcpsAmbiguousAcquisitionTestFpga::EXPECTED_DELAY_SAMPLES;
constexpr double GalileoE1PcpsAmbiguousAcquisitionTestFpga::EXPECTED_DOPPLER_HZ;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::SV_ID;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::COHERENT_INTEGRATION_TIME_ms;
constexpr char GalileoE1PcpsAmbiguousAcquisitionTestFpga::SYSTEM;
constexpr char GalileoE1PcpsAmbiguousAcquisitionTestFpga::SIGNAL[];
constexpr unsigned int GalileoE1PcpsAmbiguousAcquisitionTestFpga::NSAMPLES;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::COMPLEX_SAMPLE_SIZE_BYTES;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::SAMPLE_BLOCK_SIZE_BYTES;
constexpr char GalileoE1PcpsAmbiguousAcquisitionTestFpga::SIGNAL_FILE_PATH[];
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::POST_PROCESSING_MODE;
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::DYN_BIT_SEL_DEV_NUM;
constexpr size_t GalileoE1PcpsAmbiguousAcquisitionTestFpga::FPGA_PAGE_SIZE_BYTES;
constexpr uint32_t GalileoE1PcpsAmbiguousAcquisitionTestFpga::DYN_BIT_SEL_SHIFT_OUT_BITS;
constexpr char GalileoE1PcpsAmbiguousAcquisitionTestFpga::DYN_BIT_SEL_DEVICE_NAME[];
constexpr int GalileoE1PcpsAmbiguousAcquisitionTestFpga::SOBITS_REG_ADDR;
#endif

// When using the FPGA the acquisition class calls the states
// of the channel finite state machine directly. This is done
// in order to reduce the latency of the receiver when going
// from acquisition to tracking. In order to execute the
// acquisition in the unit tests we need to create a derived
// class of the channel finite state machine.
class ChannelFsm_galileo_e1_pcps_ambiguous_acq_test : public ChannelFsm
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

GalileoE1PcpsAmbiguousAcquisitionTestFpga::GalileoE1PcpsAmbiguousAcquisitionTestFpga()
{
    config = std::make_shared<InMemoryConfiguration>();
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::run_acquisition_process(std::shared_ptr<AcquisitionInterface> acquisition)
{
    if (!acquisition)
        {
            FAIL() << "Null acquisition";
        }
    acquisition->reset();
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::init()
{
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(BASEBAND_SAMPLING_RATE_SPS));
    config->set_property("Acquisition.threshold", std::to_string(ACQUISITION_THRESHOLD));
    config->set_property("Acquisition.doppler_max", std::to_string(DOPPLER_MAX_HZ));
    config->set_property("Acquisition.doppler_step", std::to_string(DOPPLER_STEP_HZ));
    config->set_property("Acquisition.total_block_exp", std::to_string(TOTAL_BLK_EXP));
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::create_and_open_DMA(std::shared_ptr<Fpga_DMA> &dma_fpga)
{
    dma_fpga = std::make_shared<Fpga_DMA>();
    if (dma_fpga->DMA_open())
        {
            FAIL() << "Cannot open loop device\n";
        }
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::run_DMA_process(std::shared_ptr<Fpga_DMA> &dma_fpga)
{
    std::ifstream infile;
    infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    // open the file
    try
        {
            infile.open(std::string{SIGNAL_FILE_PATH}, std::ios::binary);
        }
    catch (const std::ifstream::failure &e)
        {
            FAIL() << "Exception opening file " << SIGNAL_FILE_PATH;
        }

    // rx signal vector
    std::vector<int8_t> input_samples(SAMPLE_BLOCK_SIZE_BYTES);  // complex samples

    // pointer to DMA buffer
    int8_t *dma_buffer;

    // Open DMA device
    if (dma_fpga->DMA_open())
        {
            FAIL() << "Cannot open loop device\n";
        }
    dma_buffer = dma_fpga->get_buffer_address();

    uint64_t nbytes_remaining = NSAMPLES * COMPLEX_SAMPLE_SIZE_BYTES;
    uint32_t read_buffer_size = SAMPLE_BLOCK_SIZE_BYTES;

    // run the DMA
    bool run_DMA = true;
    while (run_DMA)
        {
            // if (nbytes_remaining < read_buffer_size)
            if (nbytes_remaining < SAMPLE_BLOCK_SIZE_BYTES)
                {
                    read_buffer_size = nbytes_remaining;
                }
            nbytes_remaining = nbytes_remaining - read_buffer_size;

            // read file
            try
                {
                    infile.read(reinterpret_cast<char *>(input_samples.data()), read_buffer_size);
                }
            catch (const std::ifstream::failure &e)
                {
                    FAIL() << "Exception reading file " << SIGNAL_FILE_PATH;
                }

            const std::streamsize bytes_read = infile.gcount();

            if (bytes_read != static_cast<std::streamsize>(read_buffer_size))
                {
                    FAIL() << "Error reading " << SIGNAL_FILE_PATH
                           << ": requested " << read_buffer_size
                           << " bytes, but read only " << bytes_read << " bytes";
                }

            uint32_t dma_index = 0;

            for (int index0 = 0; index0 < (bytes_read); index0 = index0 + 2)
                {
                    // no signal on the L5/E5a frequency band
                    dma_buffer[dma_index] = 0;
                    dma_buffer[dma_index + 1] = 0;
                    // L1/E1 frequency band
                    dma_buffer[dma_index + 2] = input_samples[index0];
                    dma_buffer[dma_index + 1 + 2] = input_samples[index0 + 1];
                    dma_index += 4;
                }

            if (bytes_read > 0)
                {
                    if (dma_fpga->DMA_write(bytes_read * 2))
                        {
                            FAIL() << "Error: DMA could not send all the required samples";
                        }
                    // Throttle the DMA
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

            if (nbytes_remaining == 0)
                {
                    // the input file is completely processed. Stop the receiver.
                    run_DMA = false;
                }
        }

    if (dma_fpga->DMA_close())
        {
            FAIL() << "Error closing loop device ";
        }

    try
        {
            infile.close();
        }
    catch (const std::ifstream::failure &e)
        {
            FAIL() << "Exception closing file " << SIGNAL_FILE_PATH;
        }
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::close_DMA(std::shared_ptr<Fpga_DMA> &dma_fpga)
{
    dma_fpga.reset();
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::create_switch(std::shared_ptr<Fpga_Switch> &fpga_switch)
{
    fpga_switch = std::make_shared<Fpga_Switch>();
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::configure_switch(std::shared_ptr<Fpga_Switch> &fpga_switch)
{
    fpga_switch->set_switch_position(POST_PROCESSING_MODE);  // set switch position to post-processing mode
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::close_switch(std::shared_ptr<Fpga_Switch> &fpga_switch)
{
    fpga_switch.reset();
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::open_and_map_dynamic_bit_selector(int &dyn_bit_sel_dev_descr, volatile unsigned *&d_map_base_dyn_bit_sel)
{
    // find the uio device file corresponding to the dynamic bit selector 0 module.
    std::string device_name;
    if (find_uio_dev_file_name(device_name, std::string{DYN_BIT_SEL_DEVICE_NAME}, DYN_BIT_SEL_DEV_NUM) < 0)
        {
            FAIL() << "Cannot find the FPGA uio device file corresponding to device name " << DYN_BIT_SEL_DEVICE_NAME;
        }

    // dynamic bits selection corresponding to frequency band 1
    if ((dyn_bit_sel_dev_descr = open(device_name.c_str(), O_RDWR | O_SYNC)) == -1)
        {
            FAIL() << "Cannot open deviceio" << device_name;
        }
    if (dyn_bit_sel_dev_descr == -1)
        {
            FAIL() << "Cannot open device " << device_name;
        }

    d_map_base_dyn_bit_sel = reinterpret_cast<volatile unsigned *>(mmap(nullptr, FPGA_PAGE_SIZE_BYTES,
        PROT_READ | PROT_WRITE, MAP_SHARED, dyn_bit_sel_dev_descr, 0));

    if (d_map_base_dyn_bit_sel == reinterpret_cast<void *>(-1))
        {
            FAIL() << "Cannot map the FPGA dynamic bit selection module";
        }
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::configure_dynamic_bit_selector(volatile unsigned *&d_map_base_dyn_bit_sel)
{
    // set the dynamic bit selection to shift the received signal by DYN_BIT_SEL_SHIFT_OUT_BITS bits
    d_map_base_dyn_bit_sel[SOBITS_REG_ADDR] = DYN_BIT_SEL_SHIFT_OUT_BITS;
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::unmap_and_close_dynamic_bit_selector(int &dyn_bit_sel_dev_descr, volatile unsigned *&d_map_base_dyn_bit_sel)
{
    // close device
    auto *aux = const_cast<unsigned *>(d_map_base_dyn_bit_sel);
    if (munmap(static_cast<void *>(aux), FPGA_PAGE_SIZE_BYTES) == -1)
        {
            FAIL() << "Failed to unmap memory uio\n";
        }
    close(dyn_bit_sel_dev_descr);
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::create_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition)
{
    acquisition = std::make_shared<PcpsAcquisitionAdapterFpga>(config.get(), "Acquisition", std::string{IMPLEMENTATION}, 0, 0, GAL_1B);
}

void GalileoE1PcpsAmbiguousAcquisitionTestFpga::configure_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition, Gnss_Synchro &tmp_gnss_synchro, std::shared_ptr<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test> &channel_fsm_)
{
    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_channel_fsm(channel_fsm_);
    acquisition->set_channel(1);
    acquisition->set_doppler_center(0);
    channel_fsm_->Event_clear_test_result();
    acquisition->set_local_code();
}
void GalileoE1PcpsAmbiguousAcquisitionTestFpga::release_acquisition(std::shared_ptr<AcquisitionInterface> &acquisition)
{
    acquisition.reset();
}

TEST_F(GalileoE1PcpsAmbiguousAcquisitionTestFpga, Instantiate)
{
    init();

    // instantiate the DMA
    std::shared_ptr<Fpga_DMA> dma_fpga;
    create_and_open_DMA(dma_fpga);

    // instantiate the switch
    std::shared_ptr<Fpga_Switch> switch_fpga;
    create_switch(switch_fpga);

    // instantiate the dynamic bit selection
    int dyn_bit_sel_dev_descr;
    volatile unsigned *d_map_base_dyn_bit_sel;
    open_and_map_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // instantiate the acquisition IP
    std::shared_ptr<AcquisitionInterface> acquisition;
    create_acquisition(acquisition);

    // close the DMA
    close_DMA(dma_fpga);

    // close the switch
    close_switch(switch_fpga);

    // close the dynamic bit selection
    unmap_and_close_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // close the acquisition IP
    release_acquisition(acquisition);
}

TEST_F(GalileoE1PcpsAmbiguousAcquisitionTestFpga, ConnectAndRun)
{
    init();

    // instantiate the DMA
    std::shared_ptr<Fpga_DMA> dma_fpga;
    create_and_open_DMA(dma_fpga);

    // instantiate the switch
    std::shared_ptr<Fpga_Switch> switch_fpga;
    create_switch(switch_fpga);

    // instantiate the dynamic bit selection
    int dyn_bit_sel_dev_descr;
    volatile unsigned *d_map_base_dyn_bit_sel;
    open_and_map_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // instantiate the acquisition IP
    std::shared_ptr<AcquisitionInterface> acquisition;
    create_acquisition(acquisition);

    // configure the switch
    configure_switch(switch_fpga);

    // configure the dynamic bit selection
    configure_dynamic_bit_selector(d_map_base_dyn_bit_sel);

    // FSM
    std::shared_ptr<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test>();
    channel_fsm_->Event_clear_test_result();

    // Gnss_Synchro
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    tmp_gnss_synchro.System = SYSTEM;
    std::memcpy(static_cast<void *>(tmp_gnss_synchro.Signal), std::string{SIGNAL}.c_str(), 2);  // copy string into synchro char array: 2 char + null
    tmp_gnss_synchro.PRN = SV_ID;

    configure_acquisition(acquisition, tmp_gnss_synchro, channel_fsm_);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();

    std::thread thread_acquisition = std::thread([&] { run_acquisition_process(acquisition); });

    // wait to give time for the acquisition thread to set up the acquisition HW accelerator in the FPGA
    usleep(1000000);

    // create DMA child process
    std::thread thread_file_to_dma = std::thread([&] { run_DMA_process(dma_fpga); });

    // wait for the acquisition process to finish
    if (thread_acquisition.joinable())
        {
            thread_acquisition.join();
        }

    // wait for the DMA process to finish
    if (thread_file_to_dma.joinable())
        {
            thread_file_to_dma.join();
        }

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;

    // close the DMA
    close_DMA(dma_fpga);

    // close the switch
    close_switch(switch_fpga);

    // close the dynamic bit selection
    unmap_and_close_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // close the acquisition IP
    release_acquisition(acquisition);

    std::cout << "Processed " << NSAMPLES << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";
}

TEST_F(GalileoE1PcpsAmbiguousAcquisitionTestFpga, ValidationOfResults)
{
    init();

    // instantiate the DMA
    std::shared_ptr<Fpga_DMA> dma_fpga;
    create_and_open_DMA(dma_fpga);

    // instantiate the switch
    std::shared_ptr<Fpga_Switch> switch_fpga;
    create_switch(switch_fpga);

    // instantiate the dynamic bit selection
    int dyn_bit_sel_dev_descr;
    volatile unsigned *d_map_base_dyn_bit_sel;
    open_and_map_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // instantiate the acquisition IP
    std::shared_ptr<AcquisitionInterface> acquisition;
    create_acquisition(acquisition);

    // configure the switch
    configure_switch(switch_fpga);

    // configure the dynamic bit selection
    configure_dynamic_bit_selector(d_map_base_dyn_bit_sel);

    // FSM
    std::shared_ptr<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test> channel_fsm_;
    channel_fsm_ = std::make_shared<ChannelFsm_galileo_e1_pcps_ambiguous_acq_test>();
    channel_fsm_->Event_clear_test_result();

    // Gnss_Synchro
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    tmp_gnss_synchro.System = SYSTEM;
    std::memcpy(static_cast<void *>(tmp_gnss_synchro.Signal), std::string{SIGNAL}.c_str(), 2);  // copy string into synchro char array: 2 char + null
    tmp_gnss_synchro.PRN = SV_ID;

    configure_acquisition(acquisition, tmp_gnss_synchro, channel_fsm_);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();

    std::thread thread_acquisition = std::thread([&] { run_acquisition_process(acquisition); });

    // wait to give time for the acquisition thread to set up the acquisition HW accelerator in the FPGA
    usleep(1000000);

    // create DMA child process
    std::thread thread_file_to_dma = std::thread([&] { run_DMA_process(dma_fpga); });

    // wait for the acquisition process to finish
    if (thread_acquisition.joinable())
        {
            thread_acquisition.join();
        }

    // wait for the DMA process to finish
    if (thread_file_to_dma.joinable())
        {
            thread_file_to_dma.join();
        }

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;

    bool acquisition_successful = channel_fsm_->Event_check_test_result();

    // close the DMA
    close_DMA(dma_fpga);

    // close the switch
    close_switch(switch_fpga);

    // close the dynamic bit selection
    unmap_and_close_dynamic_bit_selector(dyn_bit_sel_dev_descr, d_map_base_dyn_bit_sel);

    // close the acquisition IP
    release_acquisition(acquisition);

    std::cout << "Processed " << NSAMPLES << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds\n";

    ASSERT_EQ(true, acquisition_successful) << "Acquisition failure. Expected result: true=ACQ SUCCESS.";

    std::cout << "Delay: " << tmp_gnss_synchro.Acq_delay_samples << '\n';
    std::cout << "Doppler: " << tmp_gnss_synchro.Acq_doppler_hz << '\n';

    double delay_error_samples = std::abs(EXPECTED_DELAY_SAMPLES - tmp_gnss_synchro.Acq_delay_samples);
    auto delay_error_chips = static_cast<float>(delay_error_samples * (GALILEO_E1_CODE_CHIP_RATE_CPS / BASEBAND_SAMPLING_RATE_SPS));
    double doppler_error_hz = std::abs(EXPECTED_DOPPLER_HZ - tmp_gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 166) << "Doppler error exceeds the expected value: 166 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.175) << "Delay error exceeds the expected value: 0.175 chips";
}

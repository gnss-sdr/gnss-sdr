/*!
 * \file dma_signal_source_fpga.cc
 * \brief signal source for a DMA connected directly to FPGA accelerators.
 * This source implements only the DMA control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 * \author Marc Majoral, mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "dma_signal_source_fpga.h"
#include "command_event.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include <algorithm>  // for std::min
#include <chrono>     // for std::chrono
#include <fcntl.h>    // for open, O_WRONLY
#include <fstream>    // for std::ifstream
#include <iomanip>    // for std::setprecision
#include <iostream>   // for std::cout
#include <vector>     // fr std::vector

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/check.h>
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

DMASignalSourceFPGA::DMASignalSourceFPGA(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "DMA_Signal_Source_FPGA"s),
      queue_(queue),
      filename0_(configuration->property(role + ".filename", empty_string)),
      sample_rate_(configuration->property(role + ".sampling_frequency", default_bandwidth)),
      samples_to_skip_(0),
      samples_(configuration->property(role + ".samples", static_cast<int64_t>(0))),
      num_input_files_(1),
      dma_buff_offset_pos_(0),
      in_stream_(in_stream),
      out_stream_(out_stream),
      item_size_(sizeof(int8_t)),
      enable_DMA_(false),
      enable_dynamic_bit_selection_(configuration->property(role + ".enable_dynamic_bit_selection", true)),
      repeat_(configuration->property(role + ".repeat", false))
{
    const double seconds_to_skip = configuration->property(role + ".seconds_to_skip", 0.0);
    const size_t header_size = configuration->property(role + ".header_size", 0);

    const bool enable_rx1_band((configuration->property("Channels_1C.count", 0) > 0) ||
                               (configuration->property("Channels_1B.count", 0) > 0));
    const bool enable_rx2_band((configuration->property("Channels_L2.count", 0) > 0) ||
                               (configuration->property("Channels_L5.count", 0) > 0) ||
                               (configuration->property("Channels_5X.count", 0) > 0));

#if USE_GLOG_AND_GFLAGS
    // override value with commandline flag, if present
    if (FLAGS_signal_source != "-")
        {
            filename0_ = FLAGS_signal_source;
        }
    if (FLAGS_s != "-")
        {
            filename0_ = FLAGS_s;
        }
#else
    if (absl::GetFlag(FLAGS_signal_source) != "-")
        {
            filename0_ = absl::GetFlag(FLAGS_signal_source);
        }
    if (absl::GetFlag(FLAGS_s) != "-")
        {
            filename0_ = absl::GetFlag(FLAGS_s);
        }
#endif
    if (filename0_.empty())
        {
            num_input_files_ = 2;
            filename0_ = configuration->property(role + ".filename0", empty_string);
            filename1_ = configuration->property(role + ".filename1", empty_string);
        }
    // if only one input file is specified in the configuration file then:
    // if there is at least one channel assigned to frequency band 1 then the DMA transfers the samples to the L1 frequency band channels
    // otherwise the DMA transfers the samples to the L2/L5 frequency band channels
    // if more than one input file are specified then the DMA transfer the samples to both the L1 and the L2/L5 frequency channels.
    if (filename1_.empty())
        {
            if (enable_rx1_band)
                {
                    dma_buff_offset_pos_ = 2;
                }
        }
    else
        {
            dma_buff_offset_pos_ = 2;
        }

    if (seconds_to_skip > 0)
        {
            samples_to_skip_ = static_cast<uint64_t>(seconds_to_skip * sample_rate_) * 2;
        }
    if (header_size > 0)
        {
            samples_to_skip_ += header_size;
        }

    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(switch_to_DMA);

    enable_DMA_ = true;

    if (samples_ == 0)  // read all file
        {
            std::ifstream file(filename0_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                }
            else
                {
                    std::cerr << "SignalSource: Unable to open the samples file " << filename0_.c_str() << '\n';
                    return;
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename0_ << ", which contains " << static_cast<double>(size) << " [bytes]\n";
            std::cout.precision(ss);

            if (size > 0)
                {
                    const uint64_t bytes_to_skip = samples_to_skip_ * item_size_;
                    const uint64_t bytes_to_process = static_cast<uint64_t>(size) - bytes_to_skip;
                    samples_ = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size_) - ceil(0.002 * static_cast<double>(sample_rate_)));  // process all the samples available in the file excluding at least the last 1 ms
                }

            if (!filename1_.empty())
                {
                    std::ifstream file(filename1_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
                    std::ifstream::pos_type size;

                    if (file.is_open())
                        {
                            size = file.tellg();
                            DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                        }
                    else
                        {
                            std::cerr << "SignalSource: Unable to open the samples file " << filename1_.c_str() << '\n';
                            return;
                        }
                    std::streamsize ss = std::cout.precision();
                    std::cout << std::setprecision(16);
                    std::cout << "Processing file " << filename1_ << ", which contains " << static_cast<double>(size) << " [bytes]\n";
                    std::cout.precision(ss);

                    int64_t samples_rx2 = 0;
                    if (size > 0)
                        {
                            const uint64_t bytes_to_skip = samples_to_skip_ * item_size_;
                            const uint64_t bytes_to_process = static_cast<uint64_t>(size) - bytes_to_skip;
                            samples_rx2 = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size_) - ceil(0.002 * static_cast<double>(sample_rate_)));  // process all the samples available in the file excluding at least the last 1 ms
                        }
                    samples_ = std::min(samples_, samples_rx2);
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = (static_cast<double>(samples_) * (1 / static_cast<double>(sample_rate_))) / 2.0;

    DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

    if (filename1_.empty())
        {
            DLOG(INFO) << "File source filename " << filename0_;
        }
    else
        {
            DLOG(INFO) << "File source filename rx1 " << filename0_;
            DLOG(INFO) << "File source filename rx2 " << filename1_;
        }
    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sample_rate_;
    DLOG(INFO) << "Item type " << std::string("ibyte");
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;
    //        }

    // dynamic bits selection
    if (enable_dynamic_bit_selection_)
        {
            dynamic_bit_selection_fpga = std::make_shared<Fpga_dynamic_bit_selection>(enable_rx1_band, enable_rx2_band);
            thread_dynamic_bit_selection = std::thread([&] { run_dynamic_bit_selection_process(); });
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


DMASignalSourceFPGA::~DMASignalSourceFPGA()
{
    std::unique_lock<std::mutex> lock_DMA(dma_mutex);
    enable_DMA_ = false;  // disable the DMA
    lock_DMA.unlock();
    if (thread_file_to_dma.joinable())
        {
            thread_file_to_dma.join();
        }

    std::unique_lock<std::mutex> lock_dyn_bit_sel(dynamic_bit_selection_mutex);
    bool bit_selection_enabled = enable_dynamic_bit_selection_;
    lock_dyn_bit_sel.unlock();

    if (bit_selection_enabled == true)
        {
            std::unique_lock<std::mutex> lock(dynamic_bit_selection_mutex);
            enable_dynamic_bit_selection_ = false;
            lock.unlock();

            if (thread_dynamic_bit_selection.joinable())
                {
                    thread_dynamic_bit_selection.join();
                }
        }
}


void DMASignalSourceFPGA::start()
{
    thread_file_to_dma = std::thread([&] { run_DMA_process(filename0_, filename1_, samples_to_skip_, item_size_, samples_, repeat_, dma_buff_offset_pos_, queue_); });
}


void DMASignalSourceFPGA::run_DMA_process(const std::string &filename0_, const std::string &filename1_, uint64_t &samples_to_skip, size_t &item_size, int64_t &samples, bool &repeat, uint32_t &dma_buff_offset_pos, Concurrent_Queue<pmt::pmt_t> *queue)
{
    std::ifstream infile1;
    infile1.exceptions(std::ifstream::failbit | std::ifstream::badbit);


    // FPGA DMA control
    dma_fpga = std::make_shared<Fpga_DMA>();

    // open the files
    try
        {
            infile1.open(filename0_, std::ios::binary);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception opening file " << filename0_ << '\n';
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    std::ifstream infile2;
    if (!filename1_.empty())
        {
            infile2.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            try
                {
                    infile2.open(filename1_, std::ios::binary);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception opening file " << filename1_ << '\n';
                    // stop the receiver
                    queue->push(pmt::make_any(command_event_make(200, 0)));
                    return;
                }
        }

    // skip the initial samples if needed
    uint64_t bytes_to_skeep = samples_to_skip * item_size;
    try
        {
            infile1.ignore(bytes_to_skeep);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception skipping initial samples file " << filename0_ << '\n';
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    if (!filename1_.empty())
        {
            try
                {
                    infile2.ignore(bytes_to_skeep);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception skipping initial samples file " << filename1_ << '\n';
                    // stop the receiver
                    queue->push(pmt::make_any(command_event_make(200, 0)));
                    return;
                }
        }

    // rx signal vectors
    std::vector<int8_t> input_samples(sample_block_size * 2);  // complex samples
    // pointer to DMA buffer
    int8_t *dma_buffer;
    int nread_elements = 0;  // num bytes read from the file corresponding to frequency band 1
    bool run_DMA = true;

    // Open DMA device
    if (dma_fpga->DMA_open())
        {
            std::cerr << "Cannot open loop device\n";
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }
    dma_buffer = dma_fpga->get_buffer_address();

    // if only one frequency band is used then clear the samples corresponding to the unused frequency band
    uint32_t dma_index = 0;
    if (num_input_files_ == 1)
        {
            // if only one file is enabled then clear the samples corresponding to the frequency band that is not used.
            for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                {
                    dma_buffer[dma_index + (2 - dma_buff_offset_pos)] = 0;
                    dma_buffer[dma_index + 1 + (2 - dma_buff_offset_pos)] = 0;
                    dma_index += 4;
                }
        }

    uint64_t nbytes_remaining = samples * item_size;
    uint32_t read_buffer_size = sample_block_size * 2;  // complex samples

    // run the DMA
    while (run_DMA)
        {
            dma_index = 0;
            if (nbytes_remaining < read_buffer_size)
                {
                    read_buffer_size = nbytes_remaining;
                }
            nbytes_remaining = nbytes_remaining - read_buffer_size;

            // read filename 0
            try
                {
                    infile1.read(reinterpret_cast<char *>(input_samples.data()), read_buffer_size);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception reading file " << filename0_ << '\n';
                    break;
                }
            if (infile1)
                {
                    nread_elements = read_buffer_size;
                }
            else
                {
                    // FLAG AS ERROR !! IT SHOULD NEVER HAPPEN
                    nread_elements = infile1.gcount();
                }

            for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                {
                    // dma_buff_offset_pos is 1 for the L1 band and 0 for the other bands
                    dma_buffer[dma_index + dma_buff_offset_pos] = input_samples[index0];
                    dma_buffer[dma_index + 1 + dma_buff_offset_pos] = input_samples[index0 + 1];
                    dma_index += 4;
                }

            // read filename 1 (if enabled)
            if (num_input_files_ > 1)
                {
                    dma_index = 0;
                    try
                        {
                            infile2.read(reinterpret_cast<char *>(input_samples.data()), read_buffer_size);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << filename1_ << '\n';
                            break;
                        }
                    if (infile2)
                        {
                            nread_elements = read_buffer_size;
                        }
                    else
                        {
                            // FLAG AS ERROR !! IT SHOULD NEVER HAPPEN
                            nread_elements = infile2.gcount();
                        }

                    for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                        {
                            // filename2 is never the L1 band
                            dma_buffer[dma_index] = input_samples[index0];
                            dma_buffer[dma_index + 1] = input_samples[index0 + 1];
                            dma_index += 4;
                        }
                }

            if (nread_elements > 0)
                {
                    if (dma_fpga->DMA_write(nread_elements * 2))
                        {
                            std::cerr << "Error: DMA could not send all the required samples\n";
                            break;
                        }
                    // Throttle the DMA
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

            if (nbytes_remaining == 0)
                {
                    if (repeat)
                        {
                            // read the file again
                            nbytes_remaining = samples * item_size;
                            read_buffer_size = sample_block_size * 2;
                            try
                                {
                                    infile1.seekg(0);
                                }
                            catch (const std::ifstream::failure &e)
                                {
                                    std::cerr << "Exception resetting the position of the next byte to be extracted to zero " << filename0_ << '\n';
                                    break;
                                }

                            // skip the initial samples if needed
                            uint64_t bytes_to_skeep = samples_to_skip * item_size;
                            try
                                {
                                    infile1.ignore(bytes_to_skeep);
                                }
                            catch (const std::ifstream::failure &e)
                                {
                                    std::cerr << "Exception skipping initial samples file " << filename0_ << '\n';
                                    break;
                                }

                            if (!filename1_.empty())
                                {
                                    try
                                        {
                                            infile2.seekg(0);
                                        }
                                    catch (const std::ifstream::failure &e)
                                        {
                                            std::cerr << "Exception setting the position of the next byte to be extracted to zero " << filename1_ << '\n';
                                            break;
                                        }

                                    try
                                        {
                                            infile2.ignore(bytes_to_skeep);
                                        }
                                    catch (const std::ifstream::failure &e)
                                        {
                                            std::cerr << "Exception skipping initial samples file " << filename1_ << '\n';
                                            break;
                                        }
                                }
                        }
                    else
                        {
                            // the input file is completely processed. Stop the receiver.
                            run_DMA = false;
                        }
                }
            std::unique_lock<std::mutex> lock_DMA(dma_mutex);
            if (enable_DMA_ == false)
                {
                    run_DMA = false;
                }
            lock_DMA.unlock();
        }

    if (dma_fpga->DMA_close())
        {
            std::cerr << "Error closing loop device " << '\n';
        }
    try
        {
            infile1.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception closing file " << filename0_ << '\n';
        }

    if (num_input_files_ > 1)
        {
            try
                {
                    infile2.close();
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception closing file " << filename1_ << '\n';
                }
        }

    // Stop the receiver
    queue->push(pmt::make_any(command_event_make(200, 0)));
}


void DMASignalSourceFPGA::run_dynamic_bit_selection_process()
{
    bool dynamic_bit_selection_active = true;

    while (dynamic_bit_selection_active)
        {
            // setting the bit selection to the top bits
            dynamic_bit_selection_fpga->bit_selection();
            std::this_thread::sleep_for(std::chrono::milliseconds(Gain_control_period_ms));
            std::unique_lock<std::mutex> lock_dyn_bit_sel(dynamic_bit_selection_mutex);
            if (enable_dynamic_bit_selection_ == false)
                {
                    dynamic_bit_selection_active = false;
                }
            lock_dyn_bit_sel.unlock();
        }
}


void DMASignalSourceFPGA::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void DMASignalSourceFPGA::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr DMASignalSourceFPGA::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr DMASignalSourceFPGA::get_right_block()
{
    return {};
}

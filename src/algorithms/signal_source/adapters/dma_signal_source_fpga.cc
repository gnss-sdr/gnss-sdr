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
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
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
#include <cmath>      // for std::isfinite
#include <fcntl.h>    // for open, O_WRONLY
#include <fstream>    // for std::ifstream
#include <iomanip>    // for std::setprecision
#include <iostream>   // for std::cout
#include <limits>     // for std::numeric_limits
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
      filename0_(configuration->property(role + ".filename", EMPTY_STRING)),
      sample_rate_(configuration->property(role + ".sampling_frequency", DEFAULT_BANDWIDTH)),
      bytes_to_skip_(0),
      samples_(configuration->property(role + ".samples", static_cast<int64_t>(0))),
      num_input_files_(1),
      dma_buff_offset_pos_(0),
      in_stream_(in_stream),
      out_stream_(out_stream),
      item_size_(sizeof(int8_t)),
      enable_DMA_(false),
      rx1_enable_(configuration->property(role + ".rx1_enable", true)),
      rx2_enable_(configuration->property(role + ".rx2_enable", true)),
      enable_dynamic_bit_selection_(configuration->property(role + ".enable_dynamic_bit_selection", true)),
      repeat_(configuration->property(role + ".repeat", false))
{
    const double seconds_to_skip = configuration->property(role + ".seconds_to_skip", 0.0);
    const size_t header_size = configuration->property(role + ".header_size", 0);

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
            filename0_ = configuration->property(role + ".filename0", EMPTY_STRING);
        }

    filename1_ = configuration->property(role + ".filename1", EMPTY_STRING);

    if ((!configuration->is_present(role + ".rx1_enable")) && (!configuration->is_present(role + ".rx2_enable")))
        {
            // If neither RX enable flag is specified, enable each RX with a nonempty input filename.
            rx1_enable_ = !filename0_.empty();
            rx2_enable_ = !filename1_.empty();
        }

    // configuration file check
    const bool only_filename0_provided = !filename0_.empty() && filename1_.empty();
    const bool both_filenames_provided = !filename0_.empty() && !filename1_.empty();
    const bool one_freq_band_enabled = rx1_enable_ != rx2_enable_;
    const bool both_freq_bands_enabled = rx1_enable_ && rx2_enable_;

    if (!((only_filename0_provided && one_freq_band_enabled) ||
            (both_filenames_provided && both_freq_bands_enabled)))
        {
            LOG(FATAL) << "Configuration error: one input file requires exactly one enabled "
                          "frequency band; two input files require both frequency bands enabled.";
        }

    num_input_files_ = filename1_.empty() ? 1U : 2U;

    // Set the DMA buffer offset.
    if (rx1_enable_)
        {
            dma_buff_offset_pos_ = IQ_COMPONENTS_PER_SAMPLE;
        }


    CHECK(sample_rate_ > 0) << "Sampling frequency must be positive.";
    CHECK(std::isfinite(seconds_to_skip) && seconds_to_skip >= 0)
        << "Seconds to skip must be finite and nonnegative.";
    CHECK(samples_ >= 0) << "Sample count must be nonnegative.";

    const uint64_t bytes_per_sample = IQ_COMPONENTS_PER_SAMPLE * item_size_;
    const long double samples_to_skip =
        static_cast<long double>(seconds_to_skip) * sample_rate_;
    // Leave room for the header and keep ignore()'s byte count representable.
    CHECK(static_cast<long double>(header_size) <
          static_cast<long double>(std::numeric_limits<std::streamsize>::max()))
        << "Header size is too large.";
    CHECK(samples_to_skip <
          (static_cast<long double>(std::numeric_limits<std::streamsize>::max()) -
              header_size) /
              bytes_per_sample)
        << "Requested skip is too large.";
    bytes_to_skip_ = static_cast<uint64_t>(samples_to_skip) * bytes_per_sample +
                     header_size;

    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(POST_PROCESSING_MODE);

    enable_DMA_ = true;

    // Validate both files before starting the DMA thread.
    uint64_t available = get_available_items(filename0_);
    if (num_input_files_ == 2)
        {
            available = std::min(available, get_available_items(filename1_));
        }

    if (samples_ == 0)
        {
            // Preserve the existing tail margin: about 1 ms of interleaved I/Q.
            const uint64_t tail_items = sample_rate_ / 500 +
                                        (sample_rate_ % 500 != 0 ? 1 : 0);
            CHECK(available > tail_items)
                << "File does not contain enough samples to process.";
            uint64_t items_to_process = available - tail_items;
            items_to_process -= items_to_process % IQ_COMPONENTS_PER_SAMPLE;
            CHECK(items_to_process <=
                  static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
                << "Input sample count is too large.";
            samples_ = static_cast<int64_t>(items_to_process);
        }
    else
        {
            CHECK(static_cast<uint64_t>(samples_) <= available)
                << "Requested sample count exceeds the available input data.";
        }

    CHECK(samples_ % IQ_COMPONENTS_PER_SAMPLE == 0) << "Sample count must contain complete I/Q pairs.";
    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = (static_cast<double>(samples_) * (1 / static_cast<double>(sample_rate_))) / static_cast<double>(IQ_COMPONENTS_PER_SAMPLE);

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

    // dynamic bits selection
    if (enable_dynamic_bit_selection_)
        {
            dynamic_bit_selection_fpga = std::make_shared<Fpga_dynamic_bit_selection>(rx1_enable_, rx2_enable_);
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

uint64_t DMASignalSourceFPGA::get_available_items(
    const std::string &filename) const
{
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    CHECK(file.is_open()) << "Cannot open input file: " << filename;

    const auto position = file.tellg();
    CHECK(position != std::ifstream::pos_type(-1))
        << "Cannot determine input file size: " << filename;

    const uint64_t file_size = static_cast<uint64_t>(position);
    CHECK(bytes_to_skip_ <= file_size)
        << "Requested skip of " << bytes_to_skip_
        << " bytes exceeds file size of " << file_size
        << " bytes: " << filename;

    std::cout << "Processing file " << filename
              << ", which contains " << file_size << " [bytes]\n";

    return (file_size - bytes_to_skip_) / item_size_;
}

void DMASignalSourceFPGA::start()
{
    thread_file_to_dma = std::thread([&] { run_DMA_process(filename0_, filename1_, bytes_to_skip_, item_size_, samples_, repeat_, dma_buff_offset_pos_, queue_); });
}


void DMASignalSourceFPGA::run_DMA_process(const std::string &filename0_, const std::string &filename1_, uint64_t &bytes_to_skip, size_t &item_size, int64_t &samples, bool &repeat, uint32_t &dma_buff_offset_pos, Concurrent_Queue<pmt::pmt_t> *queue)
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
    try
        {
            infile1.ignore(bytes_to_skip);
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
                    infile2.ignore(bytes_to_skip);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception skipping initial samples file " << filename1_ << '\n';
                    // stop the receiver
                    queue->push(pmt::make_any(command_event_make(200, 0)));
                    return;
                }
        }

    int8_t *dma_buffer;
    uint32_t dma_buffer_size;
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
    dma_buffer_size = dma_fpga->get_buffer_size();
    uint32_t sample_block_size = dma_buffer_size / IQ_COMPONENTS_PER_DMA_FRAME;

    std::vector<int8_t> input_samples(sample_block_size * IQ_COMPONENTS_PER_SAMPLE);
    uint32_t dma_index = 0;

    // Clear every unused-band I/Q pair in the reusable DMA buffer.
    if (num_input_files_ == 1)
        {
            for (uint32_t sample = 0; sample < sample_block_size; ++sample)
                {
                    const uint32_t unused_pos =
                        sample * IQ_COMPONENTS_PER_DMA_FRAME + (IQ_COMPONENTS_PER_SAMPLE - dma_buff_offset_pos);
                    dma_buffer[unused_pos] = 0;
                    dma_buffer[unused_pos + 1] = 0;
                }
        }

    uint64_t nbytes_remaining = samples * item_size;
    uint32_t read_buffer_size = sample_block_size * IQ_COMPONENTS_PER_SAMPLE;  // complex samples

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

            for (int index0 = 0; index0 < (nread_elements); index0 += IQ_COMPONENTS_PER_SAMPLE)
                {
                    dma_buffer[dma_index + dma_buff_offset_pos] = input_samples[index0];
                    dma_buffer[dma_index + 1 + dma_buff_offset_pos] = input_samples[index0 + 1];
                    dma_index += IQ_COMPONENTS_PER_DMA_FRAME;
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

                    for (int index0 = 0; index0 < (nread_elements); index0 += IQ_COMPONENTS_PER_SAMPLE)
                        {
                            dma_buffer[dma_index] = input_samples[index0];
                            dma_buffer[dma_index + 1] = input_samples[index0 + 1];
                            dma_index += IQ_COMPONENTS_PER_DMA_FRAME;
                        }
                }

            if (nread_elements > 0)
                {
                    if (dma_fpga->DMA_write((nread_elements / IQ_COMPONENTS_PER_SAMPLE) * IQ_COMPONENTS_PER_DMA_FRAME))
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
                            read_buffer_size = sample_block_size * IQ_COMPONENTS_PER_SAMPLE;
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
                            try
                                {
                                    infile1.ignore(bytes_to_skip);
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
                                            infile2.ignore(bytes_to_skip);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_CONTROL_PERIOD_ms));
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

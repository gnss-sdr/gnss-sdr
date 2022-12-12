/*!
 * \file spir_gss6450_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Antonio Ramos, 2017 antonio.ramos(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is not part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "spir_gss6450_file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <utility>

using namespace std::string_literals;

SpirGSS6450FileSignalSource::SpirGSS6450FileSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    uint32_t in_streams,
    uint32_t out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Spir_GSS6450_File_Signal_Source"s),
      item_type_("int"),
      samples_(configuration->property(role + ".samples", static_cast<uint64_t>(0))),
      sampling_frequency_(configuration->property(role + ".sampling_frequency", static_cast<int64_t>(0))),
      item_size_(sizeof(int32_t)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      adc_bits_(configuration->property(role + ".adc_bits", 4)),
      n_channels_(configuration->property(role + ".total_channels", 1)),
      sel_ch_(configuration->property(role + ".sel_ch", 1)),
      repeat_(configuration->property(role + ".repeat", false)),
      dump_(configuration->property(role + ".dump", false)),
      enable_throttle_control_(configuration->property(role + ".enable_throttle_control", false)),
      endian_swap_(configuration->property(role + ".endian", false))
{
    const std::string default_filename("../data/my_capture.dat");
    const std::string default_dump_filename("../data/my_capture_dump.dat");
    filename_ = configuration->property(role + ".filename", default_filename);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    const int64_t bytes_seek = configuration->property(role + ".bytes_to_skip", static_cast<int64_t>(65536));
    const double sample_size_byte = static_cast<double>(adc_bits_) / 4.0;

    if (sel_ch_ > n_channels_)
        {
            LOG(WARNING) << "Invalid RF channel selection";
        }
    if (n_channels_ > 1)
        {
            for (int32_t i = 0; i < n_channels_; i++)
                {
                    null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                    unpack_spir_vec_.push_back(make_unpack_spir_gss6450_samples(adc_bits_));
                    if (endian_swap_)
                        {
                            endian_vec_.push_back(gr::blocks::endian_swap::make(item_size_));
                        }
                }
        }
    try
        {
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);
            file_source_->seek(bytes_seek / item_size_, SEEK_SET);


            deint_ = gr::blocks::deinterleave::make(item_size_);
        }
    catch (const std::exception& e)
        {
            std::cerr
                << "The receiver was configured to work with a file signal source "
                << '\n'
                << "but the specified file is unreachable by GNSS-SDR."
                << '\n'
                << "Please modify your configuration file"
                << '\n'
                << "and point SignalSource.filename to a valid raw data file. Then:"
                << '\n'
                << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf"
                << '\n'
                << "Examples of configuration files available at:"
                << '\n'
                << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/"
                << '\n';

            LOG(WARNING) << "file_signal_source: Unable to open the samples file "
                         << filename_.c_str() << ", exiting the program.";
            throw(e);
        }
    DLOG(INFO) << "file_source(" << file_source_->unique_id() << ")";

    if (samples_ == 0)  // read all file
        {
            /*!
             * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
             * A possible solution is to compute the file length in samples using file size, excluding the last 2 milliseconds, and enable always the
             * valve block
             */
            std::ifstream file(filename_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    LOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << '\n';
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << size << " [bytes]\n";
            std::cout.precision(ss);

            if (size > 0)
                {
                    samples_ = static_cast<uint64_t>(floor(static_cast<double>(static_cast<int64_t>(size) - static_cast<int64_t>(bytes_seek)) / (sample_size_byte * static_cast<double>(n_channels_))));
                    samples_ = samples_ - static_cast<uint64_t>(ceil(0.002 * sampling_frequency_));  // process all the samples available in the file excluding the last 2 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = static_cast<double>(samples_) / sampling_frequency_;
    LOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

    for (int32_t i = 0; i < n_channels_; i++)
        {
            valve_vec_.emplace_back(gnss_sdr_make_valve(sizeof(gr_complex), samples_, queue));
            if (dump_)
                {
                    std::string tmp_str = dump_filename_ + "_ch" + std::to_string(i);
                    sink_vec_.push_back(gr::blocks::file_sink::make(sizeof(gr_complex), tmp_str.c_str()));
                }
            if (enable_throttle_control_)
                {
                    throttle_vec_.push_back(gr::blocks::throttle::make(sizeof(gr_complex), sampling_frequency_));
                }
        }

    DLOG(INFO) << "File source filename " << filename_;
    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sampling_frequency_;
    DLOG(INFO) << "Item type " << item_type_;
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;
    DLOG(INFO) << "Dump " << dump_;
    DLOG(INFO) << "Dump filename " << dump_filename_;
    if (in_streams_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void SpirGSS6450FileSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            top_block->connect(file_source_, 0, deint_, 0);

            if (endian_swap_)
                {
                    top_block->connect(deint_, sel_ch_ - 1, endian_vec_.at(sel_ch_ - 1), 0);
                    top_block->connect(endian_vec_.at(sel_ch_ - 1), 0, unpack_spir_vec_.at(sel_ch_ - 1), 0);
                }
            else
                {
                    top_block->connect(deint_, sel_ch_ - 1, unpack_spir_vec_.at(sel_ch_ - 1), 0);
                }

            if (n_channels_ > 1)
                {
                    for (int32_t i = 0; i < n_channels_; i++)
                        {
                            if (i != (sel_ch_ - 1))
                                {
                                    if (endian_swap_)
                                        {
                                            top_block->connect(deint_, i, endian_vec_.at(i), 0);
                                            top_block->connect(endian_vec_.at(i), 0, unpack_spir_vec_.at(i), 0);
                                        }
                                    else
                                        {
                                            top_block->connect(deint_, i, unpack_spir_vec_.at(i), 0);
                                        }
                                }
                        }
                }
            for (int32_t i = 0; i < n_channels_; i++)
                {
                    if (enable_throttle_control_)
                        {
                            top_block->connect(unpack_spir_vec_.at(i), 0, throttle_vec_.at(i), 0);
                            top_block->connect(throttle_vec_.at(i), 0, valve_vec_.at(i), 0);
                        }
                    else
                        {
                            top_block->connect(unpack_spir_vec_.at(i), 0, valve_vec_.at(i), 0);
                        }
                    if (dump_)
                        {
                            top_block->connect(valve_vec_.at(i), 0, sink_vec_.at(i), 0);
                        }

                    top_block->connect(valve_vec_.at(i), 0, null_sinks_.at(i), 0);
                }
        }
    else
        {
            LOG(WARNING) << "0 samples to read";
        }
}


void SpirGSS6450FileSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            top_block->disconnect(file_source_, 0, deint_, 0);
            if (endian_swap_)
                {
                    top_block->disconnect(deint_, sel_ch_ - 1, endian_vec_.at(sel_ch_ - 1), 0);
                    top_block->disconnect(endian_vec_.at(sel_ch_ - 1), 0, unpack_spir_vec_.at(sel_ch_ - 1), 0);
                }
            else
                {
                    top_block->disconnect(deint_, sel_ch_ - 1, unpack_spir_vec_.at(sel_ch_ - 1), 0);
                }
            if (n_channels_ > 1)
                {
                    for (int32_t i = 0; i < n_channels_; i++)
                        {
                            if (i != (sel_ch_ - 1))
                                {
                                    if (endian_swap_)
                                        {
                                            top_block->disconnect(deint_, i, endian_vec_.at(i), 0);
                                            top_block->disconnect(endian_vec_.at(i), 0, unpack_spir_vec_.at(i), 0);
                                        }
                                    else
                                        {
                                            top_block->disconnect(deint_, i, unpack_spir_vec_.at(i), 0);
                                        }
                                }
                        }
                }

            for (int32_t i = 0; i < (n_channels_); i++)
                {
                    if (enable_throttle_control_)
                        {
                            top_block->disconnect(unpack_spir_vec_.at(i), 0, throttle_vec_.at(i), 0);
                            top_block->disconnect(throttle_vec_.at(i), 0, valve_vec_.at(i), 0);
                        }
                    else
                        {
                            top_block->disconnect(unpack_spir_vec_.at(i), 0, valve_vec_.at(i), 0);
                        }
                    if (dump_)
                        {
                            top_block->disconnect(valve_vec_.at(i), 0, sink_vec_.at(i), 0);
                        }

                    top_block->disconnect(valve_vec_.at(i), 0, null_sinks_.at(i), 0);
                }
        }
    else
        {
            LOG(WARNING) << "Nothing to disconnect";
        }
}


gr::basic_block_sptr SpirGSS6450FileSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}

gr::basic_block_sptr SpirGSS6450FileSignalSource::get_right_block(int RF_channel)
{
    return valve_vec_.at(RF_channel);
}

gr::basic_block_sptr SpirGSS6450FileSignalSource::get_right_block()
{
    return valve_vec_.at(0);
}

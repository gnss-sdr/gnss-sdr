/*!
 * \file spir_gss6450_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Antonio Ramos, 2017 antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is not part of GNSS-SDR.
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

#include "spir_gss6450_file_signal_source.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>


using google::LogMessage;


SpirGSS6450FileSignalSource::SpirGSS6450FileSignalSource(ConfigurationInterface* configuration,
    std::string role, uint32_t in_streams, uint32_t out_streams, gr::msg_queue::sptr queue) : role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(queue)
{
    std::string default_filename = "../data/my_capture.dat";
    std::string default_dump_filename = "../data/my_capture_dump.dat";
    item_type_ = "int";

    samples_ = configuration->property(role + ".samples", 0);
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", 0.0);
    filename_ = configuration->property(role + ".filename", default_filename);
    repeat_ = configuration->property(role + ".repeat", false);
    dump_ = configuration->property(role + ".dump", false);
    endian_swap_ = configuration->property(role + ".endian", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);
    adc_bits_ = configuration->property(role + ".adc_bits", 4);
    n_channels_ = configuration->property(role + ".total_channels", 1);
    sel_ch_ = configuration->property(role + ".sel_ch", 1);
    item_size_ = sizeof(int);
    int64_t bytes_seek = configuration->property(role + ".bytes_to_skip", 65536);
    double sample_size_byte = static_cast<double>(adc_bits_) / 4.0;

    if (sel_ch_ > n_channels_)
        {
            LOG(WARNING) << "Invalid RF channel selection";
        }
    if (n_channels_ > 1)
        {
            for (uint32_t i = 0; i < (n_channels_ - 1); i++)
                {
                    null_sinks_.push_back(gr::blocks::null_sink::make(item_size_));
                }
            DLOG(INFO) << "NUMBER OF NULL SINKS = " << null_sinks_.size();
        }
    try
        {
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);
            file_source_->seek(bytes_seek / item_size_, SEEK_SET);
            unpack_spir_ = make_unpack_spir_gss6450_samples(adc_bits_);
            deint_ = gr::blocks::deinterleave::make(item_size_);
        }
    catch (const std::exception& e)
        {
            std::cerr
                << "The receiver was configured to work with a file signal source "
                << std::endl
                << "but the specified file is unreachable by GNSS-SDR."
                << std::endl
                << "Please modify your configuration file"
                << std::endl
                << "and point SignalSource.filename to a valid raw data file. Then:"
                << std::endl
                << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf"
                << std::endl
                << "Examples of configuration files available at:"
                << std::endl
                << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/"
                << std::endl;

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
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << std::endl;
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << size << " [bytes]" << std::endl;
            std::cout.precision(ss);

            if (size > 0)
                {
                    samples_ = static_cast<uint64_t>(floor(static_cast<double>(static_cast<int64_t>(size) - static_cast<int64_t>(bytes_seek)) / (sample_size_byte * static_cast<double>(n_channels_))));
                    samples_ = samples_ - static_cast<uint64_t>(ceil(0.002 * sampling_frequency_));  //process all the samples available in the file excluding the last 2 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = static_cast<double>(samples_) / sampling_frequency_;
    LOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]" << std::endl;

    valve_ = gnss_sdr_make_valve(sizeof(gr_complex), samples_, queue_);
    DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

    if (dump_)
        {
            sink_ = gr::blocks::file_sink::make(sizeof(gr_complex), dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";
        }
    if (enable_throttle_control_)
        {
            throttle_ = gr::blocks::throttle::make(sizeof(gr_complex), sampling_frequency_);
        }
    if (endian_swap_)
        {
            endian_ = gr::blocks::endian_swap::make(item_size_);
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


SpirGSS6450FileSignalSource::~SpirGSS6450FileSignalSource()
{
}


void SpirGSS6450FileSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            top_block->connect(file_source_, 0, deint_, 0);
            if (endian_swap_)
                {
                    top_block->connect(deint_, sel_ch_ - 1, endian_, 0);
                    top_block->connect(endian_, 0, unpack_spir_, 0);
                }
            else
                {
                    top_block->connect(deint_, sel_ch_ - 1, unpack_spir_, 0);
                }
            if (n_channels_ > 1)
                {
                    uint32_t aux = 0;
                    for (uint32_t i = 0; i < n_channels_; i++)
                        {
                            if (i != (sel_ch_ - 1))
                                {
                                    top_block->connect(deint_, i, null_sinks_.at(aux), 0);
                                    aux++;
                                }
                        }
                }
            if (enable_throttle_control_)
                {
                    top_block->connect(unpack_spir_, 0, throttle_, 0);
                    top_block->connect(throttle_, 0, valve_, 0);
                }
            else
                {
                    top_block->connect(unpack_spir_, 0, valve_, 0);
                }
            if (dump_)
                {
                    top_block->connect(valve_, 0, sink_, 0);
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
                    top_block->disconnect(deint_, sel_ch_ - 1, endian_, 0);
                    top_block->disconnect(endian_, 0, unpack_spir_, 0);
                }
            else
                {
                    top_block->disconnect(deint_, sel_ch_ - 1, unpack_spir_, 0);
                }
            if (n_channels_ > 1)
                {
                    uint32_t aux = 0;
                    for (uint32_t i = 0; i < n_channels_; i++)
                        {
                            if (i != (sel_ch_ - 1))
                                {
                                    top_block->disconnect(deint_, i, null_sinks_.at(aux), 0);
                                    aux++;
                                }
                        }
                }
            if (enable_throttle_control_)
                {
                    top_block->disconnect(unpack_spir_, 0, throttle_, 0);
                    top_block->disconnect(throttle_, 0, valve_, 0);
                }
            else
                {
                    top_block->disconnect(unpack_spir_, 0, valve_, 0);
                }
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, sink_, 0);
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


gr::basic_block_sptr SpirGSS6450FileSignalSource::get_right_block()
{
    if (samples_ > 0)
        {
            return valve_;
        }
    else
        {
            if (enable_throttle_control_)
                {
                    return throttle_;
                }
            else
                {
                    return unpack_spir_;
                }
        }
}

/*!
 * \file multichannel_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from files at
 * different frequency band and adapts it to a SignalSourceInterface
 * \author Javier Arribas, 2019 jarribas(at)cttc.es
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

#include "multichannel_file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>  // for std::cerr
#include <utility>


MultichannelFileSignalSource::MultichannelFileSignalSource(ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue) : role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(std::move(queue))
{
    std::string default_filename = "./example_capture.dat";
    std::string default_item_type = "short";
    std::string default_dump_filename = "./my_capture.dat";

    double default_seconds_to_skip = 0.0;
    size_t header_size = 0;
    samples_ = configuration->property(role + ".samples", 0);
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", 0);
    n_channels_ = configuration->property(role + ".total_channels", 1);

    for (unsigned int n = 0; n < n_channels_; n++)
        {
            filename_vec_.push_back(configuration->property(role + ".filename" + std::to_string(n), default_filename));
        }

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    repeat_ = configuration->property(role + ".repeat", false);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);

    double seconds_to_skip = configuration->property(role + ".seconds_to_skip", default_seconds_to_skip);
    header_size = configuration->property(role + ".header_size", 0);
    int64_t samples_to_skip = 0;

    bool is_complex = false;

    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
        }
    else if (item_type_ == "float")
        {
            item_size_ = sizeof(float);
        }
    else if (item_type_ == "short")
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "ishort")
        {
            item_size_ = sizeof(int16_t);
            is_complex = true;
        }
    else if (item_type_ == "byte")
        {
            item_size_ = sizeof(int8_t);
        }
    else if (item_type_ == "ibyte")
        {
            item_size_ = sizeof(int8_t);
            is_complex = true;
        }
    else
        {
            LOG(WARNING) << item_type_
                         << " unrecognized item type. Using gr_complex.";
            item_size_ = sizeof(gr_complex);
        }
    try
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    file_source_vec_.push_back(gr::blocks::file_source::make(item_size_, filename_vec_.at(n).c_str(), repeat_));

                    if (seconds_to_skip > 0)
                        {
                            samples_to_skip = static_cast<int64_t>(seconds_to_skip * sampling_frequency_);

                            if (is_complex)
                                {
                                    samples_to_skip *= 2;
                                }
                        }
                    if (header_size > 0)
                        {
                            samples_to_skip += header_size;
                        }

                    if (samples_to_skip > 0)
                        {
                            LOG(INFO) << "Skipping " << samples_to_skip << " samples of the input file #" << n;
                            if (not file_source_vec_.back()->seek(samples_to_skip, SEEK_SET))
                                {
                                    LOG(INFO) << "Error skipping bytes!";
                                }
                        }
                }
        }
    catch (const std::exception& e)
        {
            if (filename_vec_.at(0) == default_filename)
                {
                    std::cerr
                        << "The configuration file has not been found."
                        << std::endl
                        << "Please create a configuration file based on the examples at the 'conf/' folder "
                        << std::endl
                        << "and then generate your own GNSS Software Defined Receiver by doing:"
                        << std::endl
                        << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf"
                        << std::endl;
                }
            else
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
                }

            LOG(INFO) << "file_signal_source: Unable to open the samples file "
                      << filename_vec_.at(0).c_str() << ", exiting the program.";
            throw(e);
        }

    //todo from here.... add mux demux also
    if (samples_ == 0)  // read all file
        {
            /*!
             * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
             * A possible solution is to compute the file length in samples using file size, excluding the last 100 milliseconds, and enable always the
             * valve block
             */
            std::ifstream file(filename_vec_.at(0).c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size()));
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_vec_.at(0).c_str() << std::endl;
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_vec_.at(0).c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_vec_.at(0) << ", which contains " << static_cast<double>(size) << " [bytes]" << std::endl;
            std::cout.precision(ss);

            if (size > 0)
                {
                    int64_t bytes_to_skip = samples_to_skip * item_size_;
                    int64_t bytes_to_process = static_cast<int64_t>(size) - bytes_to_skip;
                    samples_ = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size()) - ceil(0.002 * static_cast<double>(sampling_frequency_)));  // process all the samples available in the file excluding at least the last 1 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s;
    signal_duration_s = static_cast<double>(samples_) * (1 / static_cast<double>(sampling_frequency_));

    if (is_complex)
        {
            signal_duration_s /= 2.0;
        }

    DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]" << std::endl;

    valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
    DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

    if (enable_throttle_control_)
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    throttle_vec_.push_back(gr::blocks::throttle::make(item_size_, sampling_frequency_));
                }
        }

    for (unsigned int n = 0; n < n_channels_; n++)
        {
            LOG(INFO) << "Multichanne File source filename #" << n << filename_vec_.at(n);
        }

    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sampling_frequency_;
    DLOG(INFO) << "Item type " << item_type_;
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;

    if (in_streams_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void MultichannelFileSignalSource::connect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_ == true)
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    top_block->connect(file_source_vec_.at(n), 0, throttle_vec_.at(n), 0);
                    DLOG(INFO) << "connected file_source #" << n << " to throttle";
                    top_block->connect(throttle_vec_.at(n), 0, valve_, n);
                    DLOG(INFO) << "connected throttle #" << n << " to valve_";
                }
        }
    else
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    top_block->connect(file_source_vec_.at(n), 0, valve_, n);
                    DLOG(INFO) << "connected file_source #" << n << " to valve_";
                }
        }
}


void MultichannelFileSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_ == true)
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    top_block->disconnect(file_source_vec_.at(n), 0, throttle_vec_.at(n), 0);
                    DLOG(INFO) << "disconnected file_source #" << n << " to throttle";
                    top_block->disconnect(throttle_vec_.at(n), 0, valve_, n);
                    DLOG(INFO) << "disconnected throttle #" << n << " to valve_";
                }
        }
    else
        {
            for (unsigned int n = 0; n < n_channels_; n++)
                {
                    top_block->disconnect(file_source_vec_.at(n), 0, valve_, n);
                    DLOG(INFO) << "disconnected file_source #" << n << " to valve_";
                }
        }
}


gr::basic_block_sptr MultichannelFileSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr MultichannelFileSignalSource::get_right_block()
{
    return valve_;
}

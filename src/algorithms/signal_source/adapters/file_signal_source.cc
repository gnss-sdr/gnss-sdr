/*!
 * \file file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011 jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <iostream>  // for std::cerr
#include <fstream>
#include <iomanip>
#include <exception>



using google::LogMessage;


FileSignalSource::FileSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(queue)
{
    std::string default_filename = "./example_capture.dat";
    std::string default_item_type = "short";
    std::string default_dump_filename = "./my_capture.dat";

    double default_seconds_to_skip = 0.0;
    size_t header_size = 0;
    samples_ = configuration->property(role + ".samples", 0);
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", 0);
    filename_ = configuration->property(role + ".filename", default_filename);

    // override value with commandline flag, if present
    if (FLAGS_signal_source.compare("-") != 0) filename_= FLAGS_signal_source;
    if (FLAGS_s.compare("-") != 0) filename_= FLAGS_s;

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    repeat_ = configuration->property(role + ".repeat", false);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);

    double seconds_to_skip = configuration->property(role + ".seconds_to_skip", default_seconds_to_skip );
    header_size = configuration->property( role + ".header_size", 0 );
    long samples_to_skip = 0;

    bool is_complex = false;

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
        }
    else if (item_type_.compare("float") == 0)
        {
            item_size_ = sizeof(float);
        }
    else if (item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_.compare("ishort") == 0)
        {
            item_size_ = sizeof(int16_t);
            is_complex = true;
        }
    else if (item_type_.compare("byte") == 0)
        {
            item_size_ = sizeof(int8_t);
        }
    else if (item_type_.compare("ibyte") == 0)
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
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);

            if( seconds_to_skip > 0 )
                {
                    samples_to_skip = static_cast< long >( seconds_to_skip * sampling_frequency_ );

                    if( is_complex )
                        {
                            samples_to_skip *= 2;
                        }
                }
            if( header_size > 0 )
                {
                    samples_to_skip += header_size;
                }

            if( samples_to_skip > 0 )
                {
                    LOG(INFO) << "Skipping " << samples_to_skip << " samples of the input file";
                    if( not file_source_->seek( samples_to_skip, SEEK_SET ) )
                        {
                            LOG(INFO) << "Error skipping bytes!";
                        }
                }
    }
    catch (const std::exception &e)
    {
            if (filename_.compare(default_filename) == 0)
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
                    <<  "Please modify your configuration file"
                    << std::endl
                    <<  "and point SignalSource.filename to a valid raw data file. Then:"
                    << std::endl
                    << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf"
                    << std::endl
                    << "Examples of configuration files available at:"
                    << std::endl
                    << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/"
                    << std::endl;
                }

            LOG(INFO) << "file_signal_source: Unable to open the samples file "
                      << filename_.c_str() << ", exiting the program.";
            throw(e);
    }

    DLOG(INFO) << "file_source(" << file_source_->unique_id() << ")";

    if (samples_ == 0) // read all file
        {
            /*!
             * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
             * A possible solution is to compute the file length in samples using file size, excluding the last 100 milliseconds, and enable always the
             * valve block
             */
            std::ifstream file (filename_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size()));
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << std::endl;
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << static_cast<double>(size) << " [bytes]" << std::endl;
            std::cout.precision (ss);

            if (size > 0)
                {
                    long bytes_to_skip = samples_to_skip*item_size_;
                    long bytes_to_process = static_cast<long>(size) - bytes_to_skip;
                    samples_ = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size()) - ceil(0.002 * static_cast<double>(sampling_frequency_))); //process all the samples available in the file excluding at least the last 1 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s;
    signal_duration_s = static_cast<double>(samples_) * ( 1 / static_cast<double>(sampling_frequency_));

    if( is_complex )
        {
            signal_duration_s /= 2.0;
        }

    DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]" << std::endl;

    valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
    DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

    if (dump_)
        {
            sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";
        }

    if (enable_throttle_control_)
        {
            throttle_ = gr::blocks::throttle::make(item_size_, sampling_frequency_);
        }

    DLOG(INFO) << "File source filename " << filename_;
    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sampling_frequency_;
    DLOG(INFO) << "Item type " << item_type_;
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;
    DLOG(INFO) << "Dump " << dump_;
    DLOG(INFO) << "Dump filename " << dump_filename_;
}


FileSignalSource::~FileSignalSource()
{}


void FileSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            if (enable_throttle_control_ == true)
                {
                    top_block->connect(file_source_, 0, throttle_, 0);
                    DLOG(INFO) << "connected file source to throttle";
                    top_block->connect(throttle_, 0, valve_, 0);
                    DLOG(INFO) << "connected throttle to valve";
                    if (dump_)
                        {
                            top_block->connect(valve_, 0, sink_, 0);
                            DLOG(INFO) << "connected valve to file sink";
                        }
                }
            else
                {
                    top_block->connect(file_source_, 0, valve_, 0);
                    DLOG(INFO) << "connected file source to valve";
                    if (dump_)
                        {
                            top_block->connect(valve_, 0, sink_, 0);
                            DLOG(INFO) << "connected valve to file sink";
                        }
                }
        }
    else
        {
            if (enable_throttle_control_ == true)
                {
                    top_block->connect(file_source_, 0, throttle_, 0);
                    DLOG(INFO) << "connected file source to throttle";
                    if (dump_)
                        {
                            top_block->connect(file_source_, 0, sink_, 0);
                            DLOG(INFO) << "connected file source to sink";
                        }
                }
            else
                {
                    if (dump_)
                        {
                            top_block->connect(file_source_, 0, sink_, 0);
                            DLOG(INFO) << "connected file source to sink";
                        }
                }
        }
}


void FileSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            if (enable_throttle_control_ == true)
                {
                    top_block->disconnect(file_source_, 0, throttle_, 0);
                    DLOG(INFO) << "disconnected file source to throttle";
                    top_block->disconnect(throttle_, 0, valve_, 0);
                    DLOG(INFO) << "disconnected throttle to valve";
                    if (dump_)
                        {
                            top_block->disconnect(valve_, 0, sink_, 0);
                            DLOG(INFO) << "disconnected valve to file sink";
                        }
                }
            else
                {
                    top_block->disconnect(file_source_, 0, valve_, 0);
                    DLOG(INFO) << "disconnected file source to valve";
                    if (dump_)
                        {
                            top_block->disconnect(valve_, 0, sink_, 0);
                            DLOG(INFO) << "disconnected valve to file sink";
                        }
                }
        }
    else
        {
            if (enable_throttle_control_ == true)
                {
                    top_block->disconnect(file_source_, 0, throttle_, 0);
                    DLOG(INFO) << "disconnected file source to throttle";
                    if (dump_)
                        {
                            top_block->disconnect(file_source_, 0, sink_, 0);
                            DLOG(INFO) << "disconnected file source to sink";
                        }
                }
            else
                {
                    if (dump_)
                        {
                            top_block->disconnect(file_source_, 0, sink_, 0);
                            DLOG(INFO) << "disconnected file source to sink";
                        }
                }
        }
}


gr::basic_block_sptr FileSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr FileSignalSource::get_right_block()
{
    if (samples_ > 0)
        {
            return valve_;
        }
    else
        {
            if (enable_throttle_control_ == true)
                {
                    return throttle_;
                }
            else
                {
                    return file_source_;
                }
        }
}

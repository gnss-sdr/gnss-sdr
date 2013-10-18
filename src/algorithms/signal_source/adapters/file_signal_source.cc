/*!
 * \file file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011 jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#include "gnss_sdr_valve.h"
#include "configuration_interface.h"
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <exception>
#include <gflags/gflags.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
//#include <gnuradio/io_signature.h>

using google::LogMessage;

DEFINE_string(signal_source, "-",
		"If defined, path to the file containing the signal samples (overrides the configuration file)");


FileSignalSource::FileSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
		                role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(queue)
{
    std::string default_filename = "../data/my_capture.dat";
    std::string default_item_type = "short";
    std::string default_dump_filename = "../data/my_capture_dump.dat";

    samples_ = configuration->property(role + ".samples", 0);
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", 0);
    filename_ = configuration->property(role + ".filename", default_filename);

    // override value with commandline flag, if present
    if (FLAGS_signal_source.compare("-") != 0) filename_= FLAGS_signal_source;

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    repeat_ = configuration->property(role + ".repeat", false);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);

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
            item_size_ = sizeof(short int);
        }
    else
        {
            LOG_AT_LEVEL(WARNING) << item_type_
                    << " unrecognized item type. Using gr_complex.";
            item_size_ = sizeof(gr_complex);
        }
    try
    {
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);

    }
    catch (const std::exception &e)
    {
            std::cerr
            << "The receiver was configured to work with a file signal source "
            << std::endl
            << "but the specified file is unreachable by GNSS-SDR."
            << std::endl
            << "Please modify the configuration at "
            << "conf/gnss-sdr.conf (the default configuration file)"
            << std::endl
            << "and point SignalSource.filename to a valid file,"
            << std::endl
            << "or specify your own receiver and source with the flag"
            << std::endl
            <<"gnss-sdr --config_file=my_GNSS_SDR_configuration.conf"
            << std::endl;
            LOG_AT_LEVEL(INFO) << "file_signal_source: Unable to open the samples file "
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
                    DLOG(INFO) << "Total samples in the file= " << floor((double)size / (double)item_size());
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << std::endl;
                    LOG_AT_LEVEL(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << (double)size << " [bytes]" << std::endl;

            if (size > 0)
                {
                    samples_ = floor((double)size / (double)item_size() - ceil(0.002 * (double)sampling_frequency_)); //process all the samples available in the file excluding the last 2 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s;
    signal_duration_s = (double)samples_ * ( 1 /(double)sampling_frequency_);
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
    LOG_AT_LEVEL(WARNING) << "Left block of a signal source should not be retrieved";
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

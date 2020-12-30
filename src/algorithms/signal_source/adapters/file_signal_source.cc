/*!
 * \file file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011 jarribas(at)cttc.es
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

#include "file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>  // for std::cerr
#include <utility>


FileSignalSource::FileSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    const std::string default_filename("./example_capture.dat");
    const std::string default_item_type("short");
    const std::string default_dump_filename("./my_capture.dat");

    const double default_seconds_to_skip = 0.0;

    samples_ = configuration->property(role + ".samples", static_cast<uint64_t>(0));
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", static_cast<int64_t>(0));
    filename_ = configuration->property(role + ".filename", default_filename);

    // override value with commandline flag, if present
    if (FLAGS_signal_source != "-")
        {
            filename_ = FLAGS_signal_source;
        }
    if (FLAGS_s != "-")
        {
            filename_ = FLAGS_s;
        }

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    repeat_ = configuration->property(role + ".repeat", false);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);

    const double seconds_to_skip = configuration->property(role + ".seconds_to_skip", default_seconds_to_skip);
    const size_t header_size = configuration->property(role + ".header_size", 0);
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
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);

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
                    LOG(INFO) << "Skipping " << samples_to_skip << " samples of the input file";
                    if (not file_source_->seek(samples_to_skip, SEEK_SET))
                        {
                            LOG(INFO) << "Error skipping bytes!";
                        }
                }
        }
    catch (const std::exception& e)
        {
            if (filename_ == default_filename)
                {
                    std::cerr
                        << "The configuration file has not been found.\n"
                        << "Please create a configuration file based on the examples at the 'conf/' folder\n"
                        << "and then generate your own GNSS Software Defined Receiver by doing:\n"
                        << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf\n";
                }
            else
                {
                    std::cerr
                        << "The receiver was configured to work with a file signal source\n"
                        << "but the specified file is unreachable by GNSS-SDR.\n"
                        << "Please modify your configuration file\n"
                        << "and point SignalSource.filename to a valid raw data file. Then:\n"
                        << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf\n"
                        << "Examples of configuration files available at:\n"
                        << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/\n";
                }

            LOG(INFO) << "file_signal_source: Unable to open the samples file "
                      << filename_.c_str() << ", exiting the program.";
            throw(e);
        }

    DLOG(INFO) << "file_source(" << file_source_->unique_id() << ")";

    if (samples_ == 0)  // read all file
        {
            /*!
             * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
             * A possible solution is to compute the file length in samples using file size, excluding the last 100 milliseconds, and enable always the
             * valve block
             */
            std::ifstream file(filename_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << '\n';
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << static_cast<double>(size) << " [bytes]\n";
            std::cout.precision(ss);

            if (size > 0)
                {
                    const int64_t bytes_to_skip = samples_to_skip * item_size_;
                    const int64_t bytes_to_process = static_cast<int64_t>(size) - bytes_to_skip;
                    samples_ = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size_) - ceil(0.002 * static_cast<double>(sampling_frequency_)));  // process all the samples available in the file excluding at least the last 1 ms
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = static_cast<double>(samples_) * (1 / static_cast<double>(sampling_frequency_));

    if (is_complex)
        {
            signal_duration_s /= 2.0;
        }

    DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

    valve_ = gnss_sdr_make_valve(item_size_, samples_, queue);
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
    if (in_streams_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


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
    if (enable_throttle_control_ == true)
        {
            return throttle_;
        }
    return file_source_;
}

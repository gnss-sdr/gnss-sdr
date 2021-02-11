/*!
 * \file file_source_base.cc
 * \brief Implementation of the base class for file-oriented signal_source GNSS blocks
 * \author Jim Melton, 2021. jim.melton(at)sncorp.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "file_source_base.h"

#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <fstream>
#include <cmath>		// ceil, floor


using namespace std::string_literals;

void FileSourceBase::connect(gr::top_block_sptr top_block)
{
  init();

    auto source = gr::basic_block_sptr();
    auto output = gr::basic_block_sptr();

    // THROTTLE
    if (enable_throttle_control_)
        {
            // if we are throttling...
            throttle_ = gr::blocks::throttle::make(item_size_, sampling_frequency_);

            top_block->connect(file_source_, 0, throttle_, 0);
            DLOG(INFO) << "connected file source to throttle";

            source = throttle_;
        }
    else
        {
            // no throttle; let 'er rip
            source = file_source_;
        }

    // VALVE
    if (samples_ > 0)
        {
            // if a number of samples is specified, honor it by creating a valve
            // In practice, this is always true
            valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

            top_block->connect(source, 0, valve_, 0);
            DLOG(INFO) << "connected source to valve";

            output = valve_;
        }
    else
        {
	  // TODO: dumping a file source is unlikely, but should this be the raw file source or the
	  // throttle if there is one?  I'm leaning towards "output=source"
            output = file_source_;
        }

    // DUMP
    if (dump_)
        {
            sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";

            top_block->connect(output, 0, sink_, 0);
            DLOG(INFO) << "connected output to file sink";
        }
}


void FileSourceBase::disconnect(gr::top_block_sptr top_block)
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


gr::basic_block_sptr FileSourceBase::get_left_block()
{
  // TODO: is this right? Shouldn't the left block be a nullptr?
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr FileSourceBase::get_right_block()
{
    if (valve_) return valve_;
    if (throttle_) return throttle_;
    return file_source_;
}


std::string FileSourceBase::filename() const
{
    return filename_;
}

std::string FileSourceBase::item_type() const
{
    return item_type_;
}

size_t FileSourceBase::item_size()
{
    return item_size_;
}
size_t FileSourceBase::item_size() const
{
    return item_size_;
}

bool FileSourceBase::repeat() const
{
    return repeat_;
}

int64_t FileSourceBase::sampling_frequency() const
{
    return sampling_frequency_;
}

uint64_t FileSourceBase::samples() const
{
    return samples_;
}


void FileSourceBase::init()
{
    auto item_tuple = itemTypeToSize();
    item_size_ = std::get<0>(item_tuple);
    is_complex_ = std::get<1>(item_tuple);

    try
        {
            // TODO: why are we manually seeking, instead of passing the samples_to_skip to the file_source factory?
            auto samples_to_skip = samplesToSkip();

            file_source_ = gr::blocks::file_source::make(item_size(), filename().data(), repeat());

            if (samples_to_skip > 0)
                {
                    LOG(INFO) << "Skipping " << samples_to_skip << " samples of the input file";
                    if (not file_source_->seek(samples_to_skip, SEEK_SET))
                        {
                            LOG(ERROR) << "Error skipping bytes!";
                        }
                }
        }
    catch (const std::exception& e)
        {
            std::cerr
                << "The receiver was configured to work with a file-based signal source\n"
                << "but the specified file is unreachable by GNSS-SDR.\n"
                << "[" << filename() << "]\n"
                << "\n"
                << "Please modify your configuration file\n"
                << "and point SignalSource.filename to a valid raw data file. Then:\n"
                << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf\n"
                << "Examples of configuration files available at:\n"
                << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/\n"
		<< std::endl;

            LOG(ERROR) << "file_signal_source: Unable to open the samples file "
                       << filename() << ", exiting the program.";
            throw;
        }

    DLOG(INFO) << implementation() << "(" << file_source_->unique_id() << ")";

    // At this point, we know that the file exists
    samples_ = computeSamplesInFile();
    auto signal_duration_s = 1.0 * samples_ / sampling_frequency_;

    if (is_complex_)
        {
            signal_duration_s /= 2.0;
        }

    DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";


    DLOG(INFO) << "File source filename " << filename_;
    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sampling_frequency_;
    DLOG(INFO) << "Item type " << item_type_;
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;
    DLOG(INFO) << "Dump " << dump_;
    DLOG(INFO) << "Dump filename " << dump_filename_;
}


FileSourceBase::FileSourceBase(ConfigurationInterface const* configuration, std::string role, std::string impl,
        Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, impl)
    , filename_(configuration->property(role + ".filename"s, "../data/example_capture.dat"s))
    , file_source_()

    , item_type_(configuration->property(role + ".item_type"s, "short"s))
    , item_size_(0)		// invalid
    , is_complex_(false)

    , header_size_(configuration->property(role + ".header_size"s, 0UL))
    , seconds_to_skip_(configuration->property(role + ".seconds_to_skip", 0.0))
    , repeat_(configuration->property(role + ".repeat"s, false))

    , samples_(configuration->property(role + ".samples"s, 0UL))
    , sampling_frequency_(configuration->property(role + ".sampling_frequency"s, 0UL))
    , valve_()
    , queue_(queue)
      
    , enable_throttle_control_(configuration->property(role + ".enable_throttle_control"s, false))
    , throttle_()
      
    , dump_(configuration->property(role + ".dump"s, false))
    , dump_filename_(configuration->property(role + ".dump_filename"s, "../data/my_capture.dat"s))
    , sink_()
{
    // override value with commandline flag, if present
    if (FLAGS_signal_source != "-")
        {
            filename_ = FLAGS_signal_source;
        }
    if (FLAGS_s != "-")
        {
            filename_ = FLAGS_s;
        }

}

std::tuple<size_t, bool> FileSourceBase::itemTypeToSize() const
{
    auto is_complex = false;
    auto item_size = size_t(0);

    if (item_type_ == "gr_complex")
        {
            item_size = sizeof(gr_complex);
        }
    else if (item_type_ == "float")
        {
            item_size = sizeof(float);
        }
    else if (item_type_ == "short")
        {
            item_size = sizeof(int16_t);
        }
    else if (item_type_ == "ishort")
        {
            item_size = sizeof(int16_t);
            is_complex = true;
        }
    else if (item_type_ == "byte")
        {
            item_size = sizeof(int8_t);
        }
    else if (item_type_ == "ibyte")
        {
            item_size = sizeof(int8_t);
            is_complex = true;
        }
    else
        {
            LOG(WARNING) << item_type_
                         << " unrecognized item type. Using gr_complex.";
            item_size = sizeof(gr_complex);
        }

    return std::make_tuple(item_size, is_complex);
}

size_t FileSourceBase::samplesToSkip() const
{
    auto samples_to_skip = size_t(0);

    if (seconds_to_skip_ > 0)
        {
            samples_to_skip = static_cast<size_t>(seconds_to_skip_ * sampling_frequency_);

            if (is_complex_)
                {
                    samples_to_skip *= 2;
                }
        }

    if (header_size_ > 0)
        {
            samples_to_skip += header_size_;
        }

    return samples_to_skip;
}

size_t FileSourceBase::computeSamplesInFile() const
{
    auto n_samples = size_t(samples());


    // this could throw, but the existence of the file has been proven before we get here.
    auto size = fs::file_size(filename());
    n_samples = std::floor(1.0 * size / item_size());

    auto to_skip = samplesToSkip();

    /*!
     * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
     * A possible solution is to compute the file length in samples using file size, excluding at least
     * the last 2 milliseconds, and enable always the valve block
     */
    auto tail = static_cast<size_t>(std::ceil(0.002 * sampling_frequency()));

    DLOG(INFO) << "Total samples in the file= " << n_samples;
    std::cout << "Processing file " << filename() << ", which contains " << n_samples << " samples (" << size << " bytes)\n";

    if (n_samples > (to_skip + tail))
        {
            // process all the samples available in the file excluding up to the last 2 ms
            n_samples -= to_skip + tail;
        }
    else
        {
	    // this will terminate the program
            LOG(FATAL) << "Skipping " << to_skip << " samples from the front and truncating 2ms (" << tail << " samples)\n"
		       << "is greater than the number of samples in the file (" << n_samples << ")";
        }

    return n_samples;
}

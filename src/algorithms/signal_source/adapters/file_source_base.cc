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
#include "Beidou_B1I.h"
#include "Beidou_B3I.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "configuration_interface.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <algorithm>  // for std::max
#include <cmath>      // for ceil, floor
#include <iostream>   // for std::cout, std:cerr
#include <utility>    // for std::move

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

FileSourceBase::FileSourceBase(ConfigurationInterface const* configuration, std::string const& role, std::string impl,
    Concurrent_Queue<pmt::pmt_t>* queue,
    std::string default_item_type)
    : SignalSourceBase(configuration, role, std::move(impl)),
      queue_(queue),
      role_(role),
      filename_(configuration->property(role_ + ".filename"s, "./example_capture.dat"s)),
      dump_filename_(configuration->property(role_ + ".dump_filename"s, "./my_capture.dat"s)),
      item_type_(configuration->property(role_ + ".item_type"s, std::move(default_item_type))),
      item_size_(0),
      header_size_(configuration->property(role_ + ".header_size"s, uint64_t(0))),
      samples_(configuration->property(role_ + ".samples"s, uint64_t(0))),
      sampling_frequency_(configuration->property(role_ + ".sampling_frequency"s, int64_t(0))),
      minimum_tail_s_(0.1),
      seconds_to_skip_(configuration->property(role_ + ".seconds_to_skip"s, 0.0)),
      is_complex_(false),
      repeat_(configuration->property(role_ + ".repeat"s, false)),
      enable_throttle_control_(configuration->property(role_ + ".enable_throttle_control"s, false)),
      dump_(configuration->property(role_ + ".dump"s, false)),
      // Configuration for attaching extra data to the sample stream
      attach_extra_data_(configuration->property(role_ + ".extra_data.enabled"s, false)),
      ed_path_(configuration->property(role_ + ".extra_data.filename"s, "../data/extra_data.dat"s)),
      ed_offset_in_file_(configuration->property(role_ + ".extra_data.file_offset"s, uint64_t{0UL})),
      ed_item_size_(configuration->property(role_ + ".extra_data.item_size"s, uint64_t{1UL})),
      ed_repeat_(configuration->property(role_ + ".extra_data.repeat"s, false)),
      ed_offset_in_samples_(configuration->property(role_ + ".extra_data.sample_offset"s, uint64_t{0UL})),
      ed_sample_period_(configuration->property(role_ + ".extra_data.sample_period"s, uint64_t{1UL}))
{
    minimum_tail_s_ = std::max(configuration->property("Acquisition_1C.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_2S.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_L5.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_1B.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_5X.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_7X.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_E6.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_B1.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_B3.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_1G.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Acquisition_2G.coherent_integration_time_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);

    minimum_tail_s_ = std::max(configuration->property("Tracking_1C.extend_correlation_symbols", 0.0) * GPS_L1_CA_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_2S.extend_correlation_symbols", 0.0) * GPS_L2_M_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_L5.extend_correlation_symbols", 0.0) * GPS_L5I_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_1B.extend_correlation_symbols", 0.0) * GALILEO_E1_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_5X.extend_correlation_symbols", 0.0) * GALILEO_E5A_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_7X.extend_correlation_symbols", 0.0) * GALILEO_E5B_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_E6.extend_correlation_symbols", 0.0) * GALILEO_E6_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_B1.extend_correlation_symbols", 0.0) * BEIDOU_B1I_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_B3.extend_correlation_symbols", 0.0) * BEIDOU_B3I_CODE_PERIOD_S * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_1G.extend_correlation_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);
    minimum_tail_s_ = std::max(configuration->property("Tracking_2G.extend_correlation_ms", 0.0) * 0.001 * 2.0, minimum_tail_s_);

    if (repeat())
        {
            minimum_tail_s_ = 0.0;
            if (seconds_to_skip_ != 0.0)
                {
                    seconds_to_skip_ = 0.0;
                    std::cout << "Warning: since " << role_ << ".repeat is set to true, "
                              << role_ << ".seconds_to_skip parameter will be ignored.\n";
                }
        }

// override value with commandline flag, if present
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_signal_source != "-")
        {
            filename_ = FLAGS_signal_source;
        }
    if (FLAGS_s != "-")
        {
            filename_ = FLAGS_s;
        }
#else
    if (absl::GetFlag(FLAGS_signal_source) != "-")
        {
            filename_ = absl::GetFlag(FLAGS_signal_source);
        }
    if (absl::GetFlag(FLAGS_s) != "-")
        {
            filename_ = absl::GetFlag(FLAGS_s);
        }
#endif
    if (sampling_frequency_ == 0)
        {
            std::cerr << "Warning: parameter " << role_ << ".sampling_frequency is not set, this could lead to wrong results.\n"
                      << "  Please set the " << role_ << ".sampling_frequency parameter in your configuration file.\n"
                      << "  If not set, " << role_ << ".sampling_frequency=" << configuration->property("GNSS-SDR.internal_fs_sps"s, int64_t(0))
                      << " will be assumed.\n";
            sampling_frequency_ = configuration->property("GNSS-SDR.internal_fs_sps"s, int64_t(0));
        }
}


void FileSourceBase::init()
{
    create_file_source();

    // At this point, we know that the file exists
    samples_ = computeSamplesInFile();
    auto signal_duration_s = 1.0 * samples_ / sampling_frequency_;

    if (is_complex())
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

    create_throttle();
    create_valve();
    create_sink();
    create_extra_data_source();
}


void FileSourceBase::connect(gr::top_block_sptr top_block)
{
    init();
    pre_connect_hook(top_block);

    auto input = gr::basic_block_sptr();
    auto output = gr::basic_block_sptr();

    // THROTTLE
    if (throttle())
        {
            // if we are throttling...
            top_block->connect(source(), 0, throttle(), 0);
            DLOG(INFO) << "connected file source to throttle";

            input = throttle();
        }
    else
        {
            // no throttle; let 'er rip
            input = source();
        }

    // VALVE
    if (valve())
        {
            top_block->connect(std::move(input), 0, valve(), 0);
            DLOG(INFO) << "connected source to valve";

            output = valve();
        }
    else
        {
            // TODO: dumping a file source is unlikely, but should this be the raw file source or the
            // throttle if there is one?  I'm leaning towards "output=input"
            output = source();  // output = input;
        }

    // DUMP
    if (sink())
        {
            top_block->connect(std::move(output), 0, sink(), 0);
            DLOG(INFO) << "connected output to file sink";
        }

    // EXTRA DATA
    if (extra_data_source())
        {
            top_block->connect(std::move(output), 0, extra_data_source(), 0);
            DLOG(INFO) << "connected output to extra data source, which now becomes the new output";
            output = extra_data_source();
        }

    post_connect_hook(std::move(top_block));
}


void FileSourceBase::disconnect(gr::top_block_sptr top_block)
{
    auto input = gr::basic_block_sptr();
    auto output = gr::basic_block_sptr();

    pre_disconnect_hook(top_block);

    // THROTTLE
    if (throttle())
        {
            // if we are throttling...
            top_block->disconnect(source(), 0, throttle(), 0);
            DLOG(INFO) << "disconnected file source from throttle";

            input = throttle();
        }
    else
        {
            // no throttle; let 'er rip
            input = source();
        }

    // VALVE
    if (valve())
        {
            top_block->disconnect(std::move(input), 0, valve(), 0);
            DLOG(INFO) << "disconnected source to valve";

            output = valve();
        }
    else
        {
            // TODO: dumping a file source is unlikely, but should this be the raw file source or the
            // throttle if there is one?  I'm leaning towards "output=input"
            output = source();  // output = input;
        }

    // DUMP
    if (sink())
        {
            top_block->disconnect(std::move(output), 0, sink(), 0);
            DLOG(INFO) << "disconnected output to file sink";
        }

    // EXTRA DATA
    if (extra_data_source())
        {
            // TODO - FIXME: This is NOT okay, `output` is the extra data source, not the valve/source/throttle/whatever is left of this
            top_block->disconnect(std::move(output), 0, extra_data_source(), 0);
            DLOG(INFO) << "disconnected output to extra data source";
            output = extra_data_source();
        }

    post_disconnect_hook(std::move(top_block));
}


gr::basic_block_sptr FileSourceBase::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr FileSourceBase::get_right_block()
{
    // clang-tidy wants braces around the if-conditions. clang-format wants to break the braces into
    // multiple line blocks. It's much more readable this way
    // clang-format off
    if (extra_data_source_) { return extra_data_source_; }
    if (valve_) { return valve_; }
    if (throttle_) { return throttle_; }
    return source();
    // clang-format on
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


std::tuple<size_t, bool> FileSourceBase::itemTypeToSize()
{
    auto is_interleaved = false;
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
            is_interleaved = true;
        }
    else if (item_type_ == "byte")
        {
            item_size = sizeof(int8_t);
        }
    else if (item_type_ == "ibyte")
        {
            item_size = sizeof(int8_t);
            is_interleaved = true;
        }
    else
        {
            LOG(WARNING) << item_type_
                         << " unrecognized item type. Using gr_complex.";
            item_size = sizeof(gr_complex);
        }

    return std::make_tuple(item_size, is_interleaved);
}


// Default case is one decoded packet per one read sample
double FileSourceBase::packetsPerSample() const { return 1.0; }


size_t FileSourceBase::samplesToSkip() const
{
    auto samples_to_skip = size_t(0);

    if (seconds_to_skip_ > 0)
        {
            // sampling_frequency is in terms of actual samples (output packets). If this source is
            // compressed, there may be multiple packets per file (read) sample. First compute the
            // actual number of samples to skip (function of time and sample rate)
            samples_to_skip = static_cast<size_t>(seconds_to_skip_ * sampling_frequency_);

            // convert from sample to input items, scaling this value to input item space
            // (rounding up)
            samples_to_skip = std::ceil(samples_to_skip / packetsPerSample());

            // complex inputs require two input items for one sample (arguably, packetsPerSample could be scaled by 0.5)
            if (is_complex())
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
    auto n_samples = samples();

    // this could throw, but the existence of the file has been proven before we get here.
    const auto size = fs::file_size(filename());

    const auto to_skip = samplesToSkip();

    /*!
     * BUG workaround: The GNU Radio 3.7 file source does not stop the receiver after reaching the End of File.
     * A possible solution is to compute the file length in samples using file size, excluding at least
     * the last 2 milliseconds, and enable always the valve block
     */
    const auto tail = static_cast<size_t>(std::ceil(minimum_tail_s_ * sampling_frequency()));

    if (tail > size)
        {
            std::cout << "Warning: file " << filename() << " has " << size << " samples (it is too short).\n";
            return 1;
        }
    if (to_skip + tail > size)
        {
            std::cout << "Warning: " << role_ << ".seconds_to_skip is larger than file duration.\n";
            return 1;
        }

    // if configured with 0 samples (read the whole file), figure out how many samples are in the file, and go from there
    if (n_samples == 0)
        {
            // if there is some kind of compression/encoding, figure out the uncompressed number of samples
            n_samples = std::floor(packetsPerSample() * size / item_size());

            DLOG(INFO) << "Total samples in the file= " << n_samples;
            std::cout << "Processing file " << filename() << ", which contains " << n_samples << " samples (" << size << " bytes)\n";

            if (n_samples > (to_skip + tail))
                {
                    // process all the samples available in the file excluding the hearder and the tail
                    n_samples -= to_skip + tail;
                }
            else
                {
                    std::cout << "Warning: Skipping " << to_skip << " samples from the front and truncating " << tail << " samples\n"
                              << "is greater than the number of samples in the file (" << size << ")\n";
                    return 1;
                }
        }
    else
        {
            if (n_samples > size - to_skip - tail)
                {
                    std::cout << "Warning: file " << filename() << " has " << size - to_skip
                              << " samples, but " << role_ << ".samples has been set to " << n_samples << ".\n"
                              << " Setting " << role_ << ".samples to " << size - to_skip - tail
                              << " (" << to_skip << " samples skipped at header and " << tail << " samples skipped at the tail).\n";
                    n_samples = size - to_skip - tail;
                }
            std::cout << "Processing " << n_samples << " samples from file " << filename() << '\n';
        }

    return n_samples;
}


size_t FileSourceBase::source_item_size() const
{
    // delegate the size of the source to the source() object, so sub-classes have less work to do
    DLOG(INFO) << "source_item_size is " << source()->output_signature()->sizeof_stream_item(0);
    return source()->output_signature()->sizeof_stream_item(0);
}


bool FileSourceBase::is_complex() const { return is_complex_; }


// Simple accessors
gnss_shared_ptr<gr::block> FileSourceBase::source() const { return file_source(); }
gnss_shared_ptr<gr::block> FileSourceBase::file_source() const { return file_source_; }
gnss_shared_ptr<gr::block> FileSourceBase::valve() const { return valve_; }
gnss_shared_ptr<gr::block> FileSourceBase::throttle() const { return throttle_; }
gnss_shared_ptr<gr::block> FileSourceBase::sink() const { return sink_; }
ExtraDataSource::sptr FileSourceBase::extra_data_source() const { return extra_data_source_; }


gr::blocks::file_source::sptr FileSourceBase::create_file_source()
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
                    if (!file_source_->seek(samples_to_skip, SEEK_SET))
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
                << "and point " << role_ << ".filename to a valid raw data file. Then:\n"
                << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf\n"
                << "Examples of configuration files available at:\n"
                << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/\n"
                << std::endl;

            LOG(ERROR) << "file_signal_source: Unable to open the samples file "
                       << filename() << ", exiting the program.";
            throw;
        }

    DLOG(INFO) << implementation() << "(" << file_source_->unique_id() << ")";

    // enable subclass hooks
    create_file_source_hook();

    return file_source_;
}


gr::blocks::throttle::sptr FileSourceBase::create_throttle()
{
    if (enable_throttle_control_)
        {
            // if we are throttling...
            throttle_ = gr::blocks::throttle::make(source_item_size(), sampling_frequency());
            DLOG(INFO) << "throttle(" << throttle_->unique_id() << ")";

            // enable subclass hooks
            create_throttle_hook();
        }
    return throttle_;
}


gnss_shared_ptr<gr::block> FileSourceBase::create_valve()
{
    if (samples() > 0)
        {
            // if a number of samples is specified, honor it by creating a valve
            // in practice, this is always true
            valve_ = gnss_sdr_make_valve(source_item_size(), samples(), queue_);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

            // enable subclass hooks
            create_valve_hook();
        }
    return valve_;
}


gr::blocks::file_sink::sptr FileSourceBase::create_sink()
{
    if (dump_)
        {
            sink_ = gr::blocks::file_sink::make(source_item_size(), dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";

            // enable subclass hooks
            create_sink_hook();
        }
    return sink_;
}

ExtraDataSource::sptr FileSourceBase::create_extra_data_source()
{
    if (attach_extra_data_)
        {
            extra_data_source_ = gnss_make_shared<ExtraDataSource>(
                ed_path_,
                ed_offset_in_file_,
                ed_item_size_,
                ed_repeat_,
                ed_offset_in_samples_,
                ed_sample_period_,
                gr::io_signature::make(1, 1, item_size_));
            DLOG(INFO) << "extra_data_source(" << extra_data_source_->unique_id() << ")";
        }
    return extra_data_source_;
}


// Subclass hooks to augment created objects, as required
void FileSourceBase::create_file_source_hook() {}
void FileSourceBase::create_throttle_hook() {}
void FileSourceBase::create_valve_hook() {}
void FileSourceBase::create_sink_hook() {}


// Subclass hooks for connection/disconnection
void FileSourceBase::pre_connect_hook(gr::top_block_sptr top_block [[maybe_unused]]) {}      // NOLINT(performance-unnecessary-value-param)
void FileSourceBase::post_connect_hook(gr::top_block_sptr top_block [[maybe_unused]]) {}     // NOLINT(performance-unnecessary-value-param)
void FileSourceBase::pre_disconnect_hook(gr::top_block_sptr top_block [[maybe_unused]]) {}   // NOLINT(performance-unnecessary-value-param)
void FileSourceBase::post_disconnect_hook(gr::top_block_sptr top_block [[maybe_unused]]) {}  // NOLINT(performance-unnecessary-value-param)

/*!
 * \file spir_gss6450_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Antonio Ramos, 2017 antonio.ramos(at)cttc.es
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
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

#include "spir_gss6450_file_signal_source.h"
#include "configuration_interface.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/check.h>
#include <absl/log/log.h>
#endif

SpirGSS6450FileSignalSource::SpirGSS6450FileSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    uint32_t in_streams,
    uint32_t out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, std::string("Spir_GSS6450_File_Signal_Source")),
      item_type_("int"),
      samples_(configuration->property(role + ".samples", static_cast<uint64_t>(0))),
      bytes_to_skip_(configuration->property(role + ".bytes_to_skip", static_cast<int64_t>(-1))),
      sampling_frequency_(configuration->property(role + ".sampling_frequency", static_cast<int64_t>(0))),
      item_size_(sizeof(int32_t)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      adc_bits_(configuration->property(role + ".adc_bits", 0)),
      n_channels_(configuration->property(role + ".total_channels",
          configuration->property(role + ".RF_channels", 0))),
      sel_ch_(configuration->property(role + ".sel_ch", 1)),
      rf_channels_(0),
      gss6425_compatible_(false),
      repeat_(configuration->property(role + ".repeat", false)),
      dump_(configuration->property(role + ".dump", false)),
      enable_throttle_control_(configuration->property(role + ".enable_throttle_control", false)),
      endian_swap_(configuration->property(role + ".endian", false))
{
    const std::string default_filename("./my_capture.dat");
    const std::string default_dump_filename("./my_capture_dump.dat");
    filename_ = configuration->property(role + ".filename", default_filename);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    const auto detected_layout = detect_gss6450_layout(filename_);
    gss6425_compatible_ = configuration->property(role + ".gss6425_compatible",
        detected_layout.gss6425_compatible);
    if (bytes_to_skip_ < 0)
        {
            bytes_to_skip_ = detected_layout.data_offset;
            if (bytes_to_skip_ < 0)
                {
                    bytes_to_skip_ = 65536;
                    LOG(WARNING) << "Unable to auto-detect the GSS6450 IQ data offset. "
                                 << "Using legacy bytes_to_skip=" << bytes_to_skip_
                                 << ". Set " << role << ".bytes_to_skip explicitly if needed.";
                }
        }
    if (adc_bits_ == 0 && is_valid_gss6450_bits(detected_layout.adc_bits))
        {
            adc_bits_ = detected_layout.adc_bits;
        }
    if (adc_bits_ == 0)
        {
            adc_bits_ = 4;
            LOG(WARNING) << "Unable to auto-detect GSS6450 ADC bits. Using adc_bits=4.";
        }
    CHECK(is_valid_gss6450_bits(adc_bits_)) << "GSS6450 ADC bits must be one of 2, 4, 8 or 16.";
    if (gss6425_compatible_ && adc_bits_ != 2)
        {
            LOG(WARNING) << "GSS6425 compatibility decoding only applies to 2-bit data. "
                         << "Using GSS6450 two's-complement decoding for adc_bits=" << adc_bits_ << ".";
            gss6425_compatible_ = false;
        }

    if (n_channels_ == 0 && detected_layout.channels > 0)
        {
            n_channels_ = detected_layout.channels;
        }
    if (n_channels_ == 0)
        {
            n_channels_ = 1;
            LOG(WARNING) << "Unable to auto-detect the number of GSS6450 channels in the file. Using total_channels=1.";
        }
    CHECK(n_channels_ > 0 && n_channels_ <= 4) << "GSS6450 total_channels must be between 1 and 4.";

    if (detected_layout.channels > 0 && detected_layout.channels != n_channels_)
        {
            LOG(WARNING) << "Detected " << detected_layout.channels << " GSS6450 channel(s) in "
                         << filename_ << " but configuration uses " << n_channels_ << ".";
        }
    if (is_valid_gss6450_bits(detected_layout.adc_bits) && detected_layout.adc_bits != adc_bits_)
        {
            LOG(WARNING) << "Detected " << detected_layout.adc_bits << " GSS6450 ADC bits in "
                         << filename_ << " but configuration uses " << adc_bits_ << ".";
        }

    if (sel_ch_ < 1 || sel_ch_ > n_channels_)
        {
            LOG(WARNING) << "Invalid RF channel selection";
            sel_ch_ = 1;
        }

    rf_channels_ = configuration->property(role + ".RF_channels", static_cast<int32_t>(n_channels_));
    if (rf_channels_ == 0)
        {
            rf_channels_ = n_channels_;
        }
    if (rf_channels_ > static_cast<size_t>(n_channels_))
        {
            LOG(WARNING) << "SignalSource.RF_channels=" << rf_channels_
                         << " exceeds decoded GSS6450 channels=" << n_channels_
                         << ". Exposing " << n_channels_ << " RF channel(s).";
            rf_channels_ = static_cast<size_t>(n_channels_);
        }

    const double sample_size_byte = static_cast<double>(adc_bits_) / 4.0;

    for (int32_t i = 0; i < n_channels_; i++)
        {
            null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
            unpack_spir_vec_.push_back(make_unpack_spir_gss6450_samples(adc_bits_, gss6425_compatible_));
            if (endian_swap_)
                {
                    endian_vec_.push_back(gr::blocks::endian_swap::make(item_size_));
                }
        }
    try
        {
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);
            if ((bytes_to_skip_ % static_cast<int64_t>(item_size_)) != 0)
                {
                    LOG(WARNING) << "GSS6450 bytes_to_skip=" << bytes_to_skip_
                                 << " is not aligned to a 32-bit word. Seeking to the previous word boundary.";
                }
            file_source_->seek(bytes_to_skip_ / static_cast<int64_t>(item_size_), SEEK_SET);
            if (n_channels_ > 1)
                {
                    stream_to_streams_ = gr::blocks::stream_to_streams::make(item_size_, n_channels_);
                }
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
            std::ifstream::pos_type size = 0;

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
                    const auto bytes_available = static_cast<int64_t>(size) - bytes_to_skip_;
                    if (bytes_available > 0)
                        {
                            const auto samples_available = static_cast<uint64_t>(floor(static_cast<double>(bytes_available) / (sample_size_byte * static_cast<double>(n_channels_))));
                            const auto samples_to_trim = static_cast<uint64_t>(ceil(0.002 * static_cast<double>(sampling_frequency_)));
                            samples_ = samples_available > samples_to_trim ? samples_available - samples_to_trim : samples_available;
                        }
                }
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s = 0.0;
    if (sampling_frequency_ > 0)
        {
            signal_duration_s = static_cast<double>(samples_) / static_cast<double>(sampling_frequency_);
        }
    LOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

    for (int32_t i = 0; i < n_channels_; i++)
        {
            valve_vec_.emplace_back(gnss_sdr_make_valve(sizeof(gr_complex), samples_, queue, true, 1));
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

    LOG(INFO) << "File source filename " << filename_;
    LOG(INFO) << "Samples " << samples_;
    LOG(INFO) << "Sampling frequency " << sampling_frequency_;
    LOG(INFO) << "Item type " << item_type_;
    LOG(INFO) << "Item size " << item_size_;
    LOG(INFO) << "ADC bits " << adc_bits_;
    LOG(INFO) << "Total channels " << n_channels_;
    LOG(INFO) << "Bytes to skip " << bytes_to_skip_;
    LOG(INFO) << "GSS6425 compatibility " << gss6425_compatible_;
    LOG(INFO) << "Repeat " << repeat_;
    LOG(INFO) << "Dump " << dump_;
    LOG(INFO) << "Dump filename " << dump_filename_;
    if (in_streams_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams_ == 0)
        {
            LOG(ERROR) << "A signal source must have at least one output stream";
        }
}


uint32_t SpirGSS6450FileSignalSource::read_le_word(const std::vector<uint8_t>& bytes, size_t offset)
{
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}


bool SpirGSS6450FileSignalSource::matches_sync_channel(uint32_t word, uint8_t channel_pattern)
{
    const auto pattern = static_cast<uint32_t>(channel_pattern);
    return (word & 0xFFFFFF00U) == ((pattern << 24) | (pattern << 16) | (pattern << 8));
}


bool SpirGSS6450FileSignalSource::is_ascii_digit(char value)
{
    return value >= '0' && value <= '9';
}


bool SpirGSS6450FileSignalSource::is_ascii_space(char value)
{
    return value == ' ' || value == '\t' || value == '\n' || value == '\r' || value == '\f' || value == '\v';
}


char SpirGSS6450FileSignalSource::to_lower_ascii(char value)
{
    if (value >= 'A' && value <= 'Z')
        {
            return static_cast<char>(value - 'A' + 'a');
        }
    return value;
}


size_t SpirGSS6450FileSignalSource::find_case_insensitive(const std::string& text, const std::string& pattern, size_t start_pos)
{
    if (pattern.empty())
        {
            return start_pos <= text.size() ? start_pos : std::string::npos;
        }

    if (start_pos >= text.size() || pattern.size() > text.size() - start_pos)
        {
            return std::string::npos;
        }

    for (size_t pos = start_pos; pos <= text.size() - pattern.size(); ++pos)
        {
            bool found = true;
            for (size_t i = 0; i < pattern.size(); ++i)
                {
                    if (to_lower_ascii(text[pos + i]) != to_lower_ascii(pattern[i]))
                        {
                            found = false;
                            break;
                        }
                }
            if (found)
                {
                    return pos;
                }
        }

    return std::string::npos;
}


uint32_t SpirGSS6450FileSignalSource::bits_from_header(const std::string& header, const std::string& filename)
{
    std::vector<std::pair<int, uint32_t>> recorded_signals;
    const std::string signal_recorded_tag = "<Signal Recorded>";
    const std::string bits_tag = "Bits";
    size_t search_pos = 0;

    while (search_pos < header.size())
        {
            const size_t signal_pos = find_case_insensitive(header, signal_recorded_tag, search_pos);
            if (signal_pos == std::string::npos)
                {
                    break;
                }

            const size_t line_end = header.find_first_of("\r\n", signal_pos);
            const size_t line_limit = line_end == std::string::npos ? header.size() : line_end;

            size_t signal_number_pos = signal_pos + signal_recorded_tag.size();
            while (signal_number_pos < line_limit && is_ascii_space(header[signal_number_pos]))
                {
                    ++signal_number_pos;
                }

            size_t signal_number_end = signal_number_pos;
            while (signal_number_end < line_limit && is_ascii_digit(header[signal_number_end]))
                {
                    ++signal_number_end;
                }

            const size_t bits_pos = find_case_insensitive(header, bits_tag, signal_number_end);
            if (signal_number_end > signal_number_pos && bits_pos != std::string::npos && bits_pos < line_limit)
                {
                    size_t bits_number_pos = bits_pos + bits_tag.size();
                    while (bits_number_pos < line_limit && is_ascii_space(header[bits_number_pos]))
                        {
                            ++bits_number_pos;
                        }

                    size_t bits_number_end = bits_number_pos;
                    while (bits_number_end < line_limit && is_ascii_digit(header[bits_number_end]))
                        {
                            ++bits_number_end;
                        }

                    if (bits_number_end > bits_number_pos)
                        {
                            recorded_signals.emplace_back(std::stoi(header.substr(signal_number_pos, signal_number_end - signal_number_pos)),
                                static_cast<uint32_t>(std::stoul(header.substr(bits_number_pos, bits_number_end - bits_number_pos))));
                        }
                }

            search_pos = line_end == std::string::npos ? header.size() : line_end + 1;
        }

    if (recorded_signals.empty())
        {
            return 0;
        }

    const bool is_fpga_b = filename.find(".B.") != std::string::npos || filename.find(".b.") != std::string::npos;
    const bool is_fpga_a = filename.find(".A.") != std::string::npos || filename.find(".a.") != std::string::npos;

    std::vector<uint32_t> candidates;
    for (const auto& recorded_signal : recorded_signals)
        {
            if ((is_fpga_a && recorded_signal.first <= 2) ||
                (is_fpga_b && recorded_signal.first >= 3) ||
                (!is_fpga_a && !is_fpga_b))
                {
                    candidates.push_back(recorded_signal.second);
                }
        }

    if (candidates.empty())
        {
            for (const auto& recorded_signal : recorded_signals)
                {
                    candidates.push_back(recorded_signal.second);
                }
        }

    const uint32_t selected_bits = candidates.front();
    if (std::all_of(candidates.begin(), candidates.end(),
            [selected_bits](uint32_t candidate) { return candidate == selected_bits; }))
        {
            return selected_bits;
        }

    return 0;
}


bool SpirGSS6450FileSignalSource::is_gss6425_compatibility_header(const std::string& header)
{
    const std::string header_tag = "<Header>";
    const std::string gss6425_tag = "GSS6425";
    const size_t header_pos = find_case_insensitive(header, header_tag);
    if (header_pos == std::string::npos)
        {
            return false;
        }

    const size_t line_end = header.find_first_of("\r\n", header_pos);
    const size_t line_limit = line_end == std::string::npos ? header.size() : line_end;
    size_t value_pos = header_pos + header_tag.size();
    while (value_pos < line_limit && is_ascii_space(header[value_pos]))
        {
            ++value_pos;
        }

    const size_t gss6425_pos = find_case_insensitive(header, gss6425_tag, value_pos);
    return gss6425_pos == value_pos;
}


SpirGSS6450FileSignalSource::Gss6450FileLayout SpirGSS6450FileSignalSource::detect_gss6450_layout(const std::string& filename)
{
    constexpr size_t MAX_HEADER_SCAN_BYTES = 1024 * 1024;
    constexpr int64_t SYNC_BYTES_PER_CHANNEL = 4096;
    constexpr size_t REFERENCE_WORDS_BEFORE_SYNC = 2;
    const std::string end_marker = "<End of Header>";

    Gss6450FileLayout layout;
    std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open())
        {
            return layout;
        }

    std::vector<uint8_t> bytes(MAX_HEADER_SCAN_BYTES);
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    bytes.resize(static_cast<size_t>(file.gcount()));

    const std::string text(bytes.begin(), bytes.end());
    const auto header_end = text.find(end_marker);
    if (header_end != std::string::npos)
        {
            const auto header_size = header_end + end_marker.size();
            const std::string header = text.substr(0, header_size);
            layout.gss6425_compatible = is_gss6425_compatibility_header(header);
            layout.adc_bits = bits_from_header(header, filename);
            if (layout.gss6425_compatible && layout.adc_bits == 0)
                {
                    layout.adc_bits = 2;
                }

            const size_t search_start = (header_size + 3U) & ~size_t{3U};
            size_t search_offset = search_start;
            while (search_offset + sizeof(uint32_t) <= bytes.size())
                {
                    const uint32_t word = read_le_word(bytes, search_offset);
                    if (matches_sync_channel(word, 0x11))
                        {
                            layout.channels = 1;
                            if (search_offset + (2U * sizeof(uint32_t)) <= bytes.size() &&
                                matches_sync_channel(read_le_word(bytes, search_offset + sizeof(uint32_t)), 0x22))
                                {
                                    layout.channels = 2;
                                    if (search_offset + (3U * sizeof(uint32_t)) <= bytes.size() &&
                                        matches_sync_channel(read_le_word(bytes, search_offset + (2U * sizeof(uint32_t))), 0x33))
                                        {
                                            layout.channels = 3;
                                        }
                                }

                            size_t reference_offset = search_offset;
                            if (!layout.gss6425_compatible &&
                                search_offset >= search_start + (REFERENCE_WORDS_BEFORE_SYNC * sizeof(uint32_t)))
                                {
                                    reference_offset -= REFERENCE_WORDS_BEFORE_SYNC * sizeof(uint32_t);
                                }
                            layout.data_offset = static_cast<int64_t>(reference_offset) +
                                                 (SYNC_BYTES_PER_CHANNEL * layout.channels);
                            return layout;
                        }
                    search_offset += sizeof(uint32_t);
                }
        }

    return layout;
}


bool SpirGSS6450FileSignalSource::is_valid_gss6450_bits(uint32_t bits)
{
    return bits == 2 || bits == 4 || bits == 8 || bits == 16;
}


void SpirGSS6450FileSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ > 0)
        {
            if (n_channels_ > 1)
                {
                    top_block->connect(file_source_, 0, stream_to_streams_, 0);
                }

            for (int32_t i = 0; i < n_channels_; i++)
                {
                    if (endian_swap_)
                        {
                            if (n_channels_ > 1)
                                {
                                    top_block->connect(stream_to_streams_, i, endian_vec_.at(i), 0);
                                }
                            else
                                {
                                    top_block->connect(file_source_, 0, endian_vec_.at(i), 0);
                                }
                            top_block->connect(endian_vec_.at(i), 0, unpack_spir_vec_.at(i), 0);
                        }
                    else
                        {
                            if (n_channels_ > 1)
                                {
                                    top_block->connect(stream_to_streams_, i, unpack_spir_vec_.at(i), 0);
                                }
                            else
                                {
                                    top_block->connect(file_source_, 0, unpack_spir_vec_.at(i), 0);
                                }
                        }

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
            if (n_channels_ > 1)
                {
                    top_block->disconnect(file_source_, 0, stream_to_streams_, 0);
                }

            for (int32_t i = 0; i < (n_channels_); i++)
                {
                    if (endian_swap_)
                        {
                            if (n_channels_ > 1)
                                {
                                    top_block->disconnect(stream_to_streams_, i, endian_vec_.at(i), 0);
                                }
                            else
                                {
                                    top_block->disconnect(file_source_, 0, endian_vec_.at(i), 0);
                                }
                            top_block->disconnect(endian_vec_.at(i), 0, unpack_spir_vec_.at(i), 0);
                        }
                    else
                        {
                            if (n_channels_ > 1)
                                {
                                    top_block->disconnect(stream_to_streams_, i, unpack_spir_vec_.at(i), 0);
                                }
                            else
                                {
                                    top_block->disconnect(file_source_, 0, unpack_spir_vec_.at(i), 0);
                                }
                        }

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
    if (RF_channel < 0 || RF_channel >= static_cast<int>(valve_vec_.size()))
        {
            LOG(WARNING) << "'RF_channel' out of bounds while trying to get GSS6450 signal source right block.";
            return valve_vec_.at(0);
        }
    return valve_vec_.at(RF_channel);
}


gr::basic_block_sptr SpirGSS6450FileSignalSource::get_right_block()
{
    if (rf_channels_ == 1 && sel_ch_ > 1)
        {
            return valve_vec_.at(sel_ch_ - 1);
        }
    return valve_vec_.at(0);
}


size_t SpirGSS6450FileSignalSource::getRfChannels() const
{
    return rf_channels_;
}

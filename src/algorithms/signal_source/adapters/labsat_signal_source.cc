/*!
 * \file labsat_signal_source.cc
 * \brief LabSat version 2, 3, and 3 Wideband format reader
 * \author Javier Arribas, jarribas(at)cttc.es
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

#include "labsat_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include "labsat23_source.h"
#include <glog/logging.h>
#include <iostream>
#include <sstream>

using namespace std::string_literals;

LabsatSignalSource::LabsatSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Labsat_Signal_Source"s),
      in_stream_(in_stream),
      out_stream_(out_stream),
      enable_throttle_control_(configuration->property(role + ".enable_throttle_control", false)),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./labsat_output.dat");
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    const int64_t sampling_frequency_deprecated = configuration->property(role + ".sampling_frequency", static_cast<int64_t>(16368000));
    const int64_t throttle_frequency_sps = configuration->property(role + ".throttle_frequency_sps", static_cast<int64_t>(sampling_frequency_deprecated));

    std::string channels_to_read = configuration->property(role + ".selected_channel", default_item_type);
    std::stringstream ss(channels_to_read);
    int found;
    while (ss.good())
        {
            std::string substr;
            getline(ss, substr, ',');
            if (std::stringstream(substr) >> found)
                {
                    if (found >= 1 && found <= 3)
                        {
                            channels_selector_vec_.push_back(found);
                        }
                }
        }
    if (channels_selector_vec_.empty())
        {
            channels_selector_vec_.push_back(1);
        }

    const std::string default_filename("./example_capture.LS3");
    filename_ = configuration->property(role + ".filename", default_filename);

    const bool digital_io_enabled = configuration->property(role + ".digital_io_enabled", false);

    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            labsat23_source_ = labsat23_make_source_sptr(filename_.c_str(), channels_selector_vec_, queue, digital_io_enabled);
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "labsat23_source_(" << labsat23_source_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for LabSat source";
            item_size_ = sizeof(int16_t);
        }
    if (dump_)
        {
            std::vector<std::string> dump_filename;
            file_sink_.reserve(channels_selector_vec_.size());
            for (int i : channels_selector_vec_)
                {
                    if (channels_selector_vec_.size() == 1)
                        {
                            dump_filename.push_back(dump_filename_);
                        }
                    else
                        {
                            std::string aux(dump_filename_.substr(0, dump_filename_.length() - 4));
                            std::string extension(dump_filename_.substr(dump_filename_.length() - 3, dump_filename_.length()));
                            if (i == 1)
                                {
                                    aux += "_chA."s;
                                }
                            if (i == 2)
                                {
                                    aux += "_chB."s;
                                }
                            if (i == 3)
                                {
                                    aux += "_chC."s;
                                }
                            dump_filename.push_back(aux + extension);
                        }
                    std::cout << "Dumping output into file " << dump_filename.back() << '\n';
                    file_sink_.push_back(gr::blocks::file_sink::make(item_size_, dump_filename.back().c_str()));
                    DLOG(INFO) << "file_sink(" << file_sink_.back()->unique_id() << ")";
                }
        }

    if (enable_throttle_control_)
        {
            for (auto it = channels_selector_vec_.begin(); it != channels_selector_vec_.end(); ++it)
                {
                    throttle_.push_back(gr::blocks::throttle::make(item_size_, throttle_frequency_sps));
                }
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 3)
        {
            LOG(ERROR) << "This implementation supports up to 3 output streams";
        }
}


size_t LabsatSignalSource::getRfChannels() const
{
    return channels_selector_vec_.size();
}


void LabsatSignalSource::connect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_ == true)
        {
            int rf_chan = 0;
            for (const auto& th : throttle_)
                {
                    top_block->connect(labsat23_source_, rf_chan, th, 0);
                    DLOG(INFO) << "connected labsat23_source_ to throttle";
                    if (dump_)
                        {
                            top_block->connect(labsat23_source_, rf_chan, file_sink_[rf_chan], 0);
                            DLOG(INFO) << "connected labsat23_source_to sink";
                        }
                    rf_chan++;
                }
        }
    else
        {
            int rf_chan = 0;
            for (auto it = channels_selector_vec_.begin(); it != channels_selector_vec_.end(); ++it)
                {
                    if (dump_)
                        {
                            top_block->connect(labsat23_source_, 0, file_sink_[rf_chan], 0);
                            DLOG(INFO) << "connected labsat23_source_ to sink";
                        }
                    else
                        {
                            DLOG(INFO) << "nothing to connect internally";
                        }
                    rf_chan++;
                }
        }
}


void LabsatSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_ == true)
        {
            int rf_chan = 0;
            for (const auto& th : throttle_)
                {
                    top_block->disconnect(labsat23_source_, rf_chan, th, 0);
                    DLOG(INFO) << "disconnected labsat23_source_ to throttle";
                    if (dump_)
                        {
                            top_block->disconnect(labsat23_source_, rf_chan, file_sink_[rf_chan], 0);
                            DLOG(INFO) << "disconnected labsat23_source_ to sink";
                        }
                    rf_chan++;
                }
        }
    else
        {
            int rf_chan = 0;
            for (auto it = channels_selector_vec_.begin(); it != channels_selector_vec_.end(); ++it)
                {
                    if (dump_)
                        {
                            top_block->disconnect(labsat23_source_, rf_chan, file_sink_[rf_chan], 0);
                            DLOG(INFO) << "disconnected labsat23_source_ to sink";
                        }
                    rf_chan++;
                }
        }
}


gr::basic_block_sptr LabsatSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr LabsatSignalSource::get_right_block()
{
    if (enable_throttle_control_ == true)
        {
            return throttle_[0];
        }
    return labsat23_source_;
}


gr::basic_block_sptr LabsatSignalSource::get_right_block(int i)
{
    if (enable_throttle_control_ == true)
        {
            return throttle_[i];
        }
    return labsat23_source_;
}

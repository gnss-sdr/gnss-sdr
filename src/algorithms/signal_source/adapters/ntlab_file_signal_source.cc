/*!
 * \file ntlab_file_signal_source.cc
 * \brief Interface of a class that reads signal samples from a file. Each
 * sample is two bits from multiple channels.
 *
 * \author Pedro Pereira, 2025 pereirapedrocp (at) gmail.com
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

#include "ntlab_file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

NTLabFileSignalSource::NTLabFileSignalSource(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : FileSourceBase(configuration, role, "NTLab_File_Signal_Source"s, queue, "byte"s),
      sample_type_(configuration->property(role + ".sample_type", "real"s))
{
    int default_n_channlels_ = 4;
    n_channels_ = configuration->property(role + ".RF_channels", default_n_channlels_);
    if ((n_channels_ != 1) && (n_channels_ != 2) && (n_channels_ != 4))
        {
            n_channels_ = 4;
            LOG(ERROR) << "Number of channels must be 1, 2 or 4 (got " << n_channels_ << "). Using 4.";
        }

    if (in_streams > 0)
        {
            LOG(ERROR) << "A signal source does not have input streams";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}

std::tuple<size_t, bool> NTLabFileSignalSource::itemTypeToSize()
{
    auto is_complex_t = false;
    auto item_size = sizeof(char);  // default

    if (item_type() == "byte")
        {
            item_size = sizeof(char);
        }
    else
        {
            LOG(WARNING) << "Configuration error: Unsupported item type. Using byte.";
        }

    if (sample_type_ != "real")
        {
            is_complex_t = false;
        }
    else
        {
            LOG(WARNING) << "Configuration error: Unsupported sample type. Using real.";
        }

    return std::make_tuple(item_size, is_complex_t);
}

double NTLabFileSignalSource::packetsPerSample() const
{
    return 4 / n_channels_;  // sampling instants in one byte depend on channel count
}

gnss_shared_ptr<gr::block> NTLabFileSignalSource::source() const
{
    return unpack_samples_;
}

void NTLabFileSignalSource::create_file_source_hook()
{
    unpack_samples_ = make_unpack_ntlab_2bit_samples(item_size(), n_channels_);
    DLOG(INFO) << "unpack_byte_2bit_samples(" << unpack_samples_->unique_id() << ")";
}

void NTLabFileSignalSource::pre_connect_hook(gr::top_block_sptr top_block)
{
    top_block->connect(file_source(), 0, unpack_samples_, 0);
    DLOG(INFO) << "connected file source to samples unpacker";

    for (int n = 1; n < n_channels_; n++)
        {
            top_block->connect(unpack_samples_, n, valve(), n);
            DLOG(INFO) << "connected samples unpacker to valve port " << n;
        }
}

void NTLabFileSignalSource::pre_disconnect_hook(gr::top_block_sptr top_block)
{
    top_block->disconnect(file_source(), 0, unpack_samples_, 0);
    DLOG(INFO) << "disconnected file source of samples unpacker";

    for (int n = 1; n < n_channels_; n++)
        {
            top_block->disconnect(unpack_samples_, n, valve(), n);
            DLOG(INFO) << "disconnected samples unpacker of valve port " << n;
        }
}

gr::basic_block_sptr NTLabFileSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}

gr::basic_block_sptr NTLabFileSignalSource::get_right_block()
{
    return valve();
}

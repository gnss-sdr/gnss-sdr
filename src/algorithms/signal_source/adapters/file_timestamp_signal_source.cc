/*!
 * \file file_timestamp_signal_source.h
 * \brief This class reads samples stored in a file and generate stream tags with its timestamp information stored in separated file
 * \author Javier Arribas, jarribas(at)cttc.es
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
#include "file_timestamp_signal_source.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_timestamp.h"
#include <glog/logging.h>
#include <string>

using namespace std::string_literals;

FileTimestampSignalSource::FileTimestampSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : FileSourceBase(configuration, role, "File_Timestamp_Signal_Source"s, queue, "byte"s),
      timestamp_file_(configuration->property(role + ".timestamp_filename"s, "../data/example_capture_timestamp.dat"s)),
      timestamp_clock_offset_ms_(configuration->property(role + ".timestamp_clock_offset_ms"s, 0.0))
{
    if (in_streams > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }

    // override value with commandline flag, if present
    if (FLAGS_timestamp_source != "-")
        {
            timestamp_file_ = FLAGS_timestamp_source;
        }
}


gnss_shared_ptr<gr::block> FileTimestampSignalSource::source() const { return timestamp_block_; }


void FileTimestampSignalSource::create_file_source_hook()
{
    timestamp_block_ = gnss_sdr_make_Timestamp(
        std::get<0>(itemTypeToSize()),
        timestamp_file_,
        timestamp_clock_offset_ms_);
    DLOG(INFO) << "timestamp_block_(" << timestamp_block_->unique_id() << ")";
}

void FileTimestampSignalSource::pre_connect_hook(gr::top_block_sptr top_block)
{
    top_block->connect(file_source(), 0, timestamp_block_, 0);
    DLOG(INFO) << "connected file_source to timestamp_block_";
}

void FileTimestampSignalSource::pre_disconnect_hook(gr::top_block_sptr top_block)
{
    top_block->disconnect(file_source(), 0, timestamp_block_, 0);
    DLOG(INFO) << "disconnected file_source from timestamp_block_";
}

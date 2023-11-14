/*!
 * \file nsr_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a NSR 2 bits sampler front-end file
 * and adapts it to a SignalSourceInterface. More information about the front-end here
 * http://www.ifen.com/products/sx-scientific-gnss-solutions/nsr-software-receiver.html
 * \author Javier Arribas, 2013 jarribas(at)cttc.es
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

#include "nsr_file_signal_source.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>

using namespace std::string_literals;

NsrFileSignalSource::NsrFileSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : FileSourceBase(configuration, role, "Nsr_File_Signal_Source"s, queue, "byte"s)
{
    if (in_streams > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}

std::tuple<size_t, bool> NsrFileSignalSource::itemTypeToSize()
{
    auto is_complex = false;
    auto item_size = sizeof(char);  // default

    if (item_type() == "byte")
        {
            item_size = sizeof(char);
        }
    else
        {
            LOG(WARNING) << item_type() << " unrecognized item type. Using byte.";
        }

    return std::make_tuple(item_size, is_complex);
}

// 1 byte -> 4 samples
double NsrFileSignalSource::packetsPerSample() const { return 4.0; }
gnss_shared_ptr<gr::block> NsrFileSignalSource::source() const { return unpack_byte_; }


void NsrFileSignalSource::create_file_source_hook()
{
    unpack_byte_ = make_unpack_byte_2bit_samples();
    DLOG(INFO) << "unpack_byte_2bit_samples(" << unpack_byte_->unique_id() << ")";
}

void NsrFileSignalSource::pre_connect_hook(gr::top_block_sptr top_block)
{
    top_block->connect(file_source(), 0, unpack_byte_, 0);
    DLOG(INFO) << "connected file_source to unpacker";
}

void NsrFileSignalSource::pre_disconnect_hook(gr::top_block_sptr top_block)
{
    top_block->disconnect(file_source(), 0, unpack_byte_, 0);
    DLOG(INFO) << "disconnected file_source from unpacker";
}

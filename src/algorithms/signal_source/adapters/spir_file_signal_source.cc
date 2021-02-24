/*!
 * \file spir_file_signal_source.cc
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Fran Fabra, 2014 fabra(at)ice.csic.es
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

#include "spir_file_signal_source.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>


using namespace std::string_literals;

SpirFileSignalSource::SpirFileSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : FileSourceBase(configuration, role, "Spir_File_Signal_Source"s, queue, "int"s)
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

std::tuple<size_t, bool> SpirFileSignalSource::itemTypeToSize()
{
    auto is_complex = false;
    auto item_size = size_t(0);

    if (item_type() == "int")
        {
            item_size = sizeof(int);
        }
    else
        {
            LOG(WARNING) << item_type() << " unsupported item type. Using int.";
            item_size = sizeof(int);
        }

    return std::make_tuple(item_size, is_complex);
}

// This class feeds the file data through a decoder to produce samples; for all intents this is the "source"
gnss_shared_ptr<gr::block> SpirFileSignalSource::source() const { return unpack_intspir_; }


void SpirFileSignalSource::create_file_source_hook()
{
    // connect the file to the decoder
    unpack_intspir_ = make_unpack_intspir_1bit_samples();
    DLOG(INFO) << "unpack_intspir_1bit_samples(" << unpack_intspir_->unique_id() << ")";
}

void SpirFileSignalSource::pre_connect_hook(gr::top_block_sptr top_block)
{
    top_block->connect(file_source(), 0, unpack_intspir_, 0);
    DLOG(INFO) << "connected file_source to unpacker";
}

void SpirFileSignalSource::post_disconnect_hook(gr::top_block_sptr top_block)
{
    top_block->disconnect(file_source(), 0, unpack_intspir_, 0);
    DLOG(INFO) << "disconnected file_source from unpacker";
}

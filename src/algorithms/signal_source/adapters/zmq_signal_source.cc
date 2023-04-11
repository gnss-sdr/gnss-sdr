/*!
 * \file zmq_signal_source.cc
 * \brief Signal source which reads from ZeroMQ.
 * \author Jim Melton, 2022. jim.melton(at)sncorp.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "zmq_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>

using namespace std::string_literals;

ZmqSignalSource::ZmqSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int /* in_stream [[maybe_unused]] */,
    unsigned int /* out_stream [[maybe_unused]] */,
    Concurrent_Queue<pmt::pmt_t>* /* queue [[maybe_unused]] */)
    : SignalSourceBase(configuration, role, "ZMQ_Signal_Source"s),
      d_item_size(decode_item_type(configuration->property(role + ".item_type", "gr_complex"s), nullptr, true)),
      d_dump_filename(configuration->property(role + ".dump_filename", "data/zmq_dump.dat"s)),
      d_dump(configuration->property(role + ".dump", false))
{
    auto vlen = configuration->property(role + ".vlen"s, 1);
    auto pass_tags = configuration->property(role + ".pass_tags"s, false);
    auto timeout_ms = configuration->property(role + ".timeout_ms"s, 100);
    auto hwm = configuration->property(role + ".hwm"s, -1);

    auto property = role + ".endpoint"s;
    auto endpoint = configuration->property(property, ""s);

    if (!endpoint.empty())
        {
            LOG(INFO) << "Connecting to ZMQ pub at " << endpoint;
            // work around gnuradio interface const-deficiency
            d_source_block = gr::zeromq::sub_source::make(d_item_size, vlen, const_cast<char*>(endpoint.data()), timeout_ms, pass_tags, hwm);

            // work around another bug. GNU Radio passes tags through the ZMQ block
            // unconditionally if pass_tags is true, but that flag controls protocol more
            // than the intent of the block. Since we have the vector-to-stream block,
            // unconditionally create it, and have it squelch the tags. GNSS-SDR should
            // not fail if unexpected tags are received

            // vector-to-stream should be coherent even if vlen == 1
            d_vec_block = gr::blocks::vector_to_stream::make(item_size(), vlen);
            d_vec_block->set_tag_propagation_policy(gr::block::TPP_DONT);  // GNSS-SDR doesn't do well with tags/
        }
    else
        {
            std::cerr << "For ZMQ_Signal_Source " << property << " must be defined" << std::endl;
            throw std::invalid_argument(property + ": undefined");
        }
}


auto ZmqSignalSource::item_size() -> size_t { return d_item_size; }


auto ZmqSignalSource::connect(gr::top_block_sptr top_block) -> void
{
    if (d_dump)
        {
            d_dump_sink = gr::blocks::file_sink::make(item_size(), d_dump_filename.data());
            top_block->connect(get_right_block(), 0, d_dump_sink, 0);
        }

    top_block->connect(d_source_block, 0, d_vec_block, 0);
}


auto ZmqSignalSource::disconnect(gr::top_block_sptr top_block) -> void
{
    if (d_dump)
        {
            top_block->disconnect(d_dump_sink);
        }

    // this might be redundant
    top_block->disconnect(d_vec_block);
    top_block->disconnect(d_source_block);
}


auto ZmqSignalSource::get_right_block() -> gr::basic_block_sptr
{
    return d_vec_block;
}

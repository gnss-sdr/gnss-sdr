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

using namespace std::string_literals;

ZmqSignalSource::ZmqSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int /* in_stream [[maybe_unused]] */,
    unsigned int /* out_stream [[maybe_unused]] */,
    Concurrent_Queue<pmt::pmt_t>* /* queue [[maybe_unused]] */)
    : SignalSourceBase(configuration, role, "ZMQ_Signal_Source"s)  //
      ,
      d_item_size(decode_item_type(configuration->property(role + ".item_type"s, "gr_complex"s), nullptr, true))
{
    auto vlen = configuration->property(role + ".vlen"s, 1);
    auto pass_tags = configuration->property(role + ".pass_tags"s, false);
    auto timeout_ms = configuration->property(role + ".timeout_ms"s, 100);
    auto hwm = configuration->property(role + ".hwm"s, -1);

    // Each .endpointN must be specified
    for (auto n = 0u; n < getRfChannels(); ++n)
        {
            auto property = role + ".endpoint"s + std::to_string(n);
            auto endpoint = configuration->property(property, ""s);
            if (not endpoint.empty())
                {
                    LOG(INFO) << "Connecting to ZMQ pub at " << endpoint;
                    d_source_blocks.push_back(gr::zeromq::sub_source::make(d_item_size, vlen, endpoint.data(), timeout_ms, pass_tags, hwm));
                }
            else
                {
                    std::cerr
                        << "For ZMQ_Signal_Source " << role << ", the .endpointN property must be defined\n"
                        << "for all values of N from 0 to " << getRfChannels() - 1 << std::endl;

                    throw std::invalid_argument(property + ": undefined");
                }
        }
}


auto ZmqSignalSource::item_size() -> size_t { return d_item_size; }

auto ZmqSignalSource::connect(gr::top_block_sptr /* top_block [[maybe_unused]] */) -> void
{
    // for now, nothing to connect
}

auto ZmqSignalSource::disconnect(gr::top_block_sptr /* top_block [[maybe_unused]] */) -> void
{
    // for now, nothing to disconnect
}

auto ZmqSignalSource::get_right_block() -> gr::basic_block_sptr
{
    return d_source_blocks.front();
}

auto ZmqSignalSource::get_right_block(int RF_channel) -> gr::basic_block_sptr
{
    return d_source_blocks.at(RF_channel);  // throws std::out_of_range
}

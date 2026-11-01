/*!
 * \file galileo_tow_map.cc
 * \brief GNU Radio block that stores TOW for Galileo channels
 * \author Carles Fernandez-Prades, 2022. cfernandez(at)cttc.es
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


#include "galileo_tow_map.h"
#include <iterator>  // for std::prev
#include <memory>    // for std::shared_ptr
#include <typeinfo>  // for typeid

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

galileo_tow_map_sptr galileo_tow_map_make()
{
    return galileo_tow_map_sptr(new galileo_tow_map());
}


galileo_tow_map::galileo_tow_map() : gr::block("galileo_tow_map", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    // register Gal E6 HAS input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("TOW_from_TLM"));
    // register nav message monitor out
    this->message_port_register_out(pmt::mp("TOW_to_TLM"));
    // handler for input port
    this->set_msg_handler(pmt::mp("TOW_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_galileo_tow_map(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&galileo_tow_map::msg_handler_galileo_tow_map, this, boost::placeholders::_1));
#else
        boost::bind(&galileo_tow_map::msg_handler_galileo_tow_map, this, _1));
#endif
#endif

    for (uint32_t prn = 0; prn < 37; prn++)
        {
            d_galileo_tow[prn] = GalileoTowMapEntry{};
        }
}


void galileo_tow_map::msg_handler_galileo_tow_map(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<GalileoTowMapMessage>).hash_code())
                {
                    const auto received_tow_map = wht::any_cast<std::shared_ptr<GalileoTowMapMessage>>(pmt::any_ref(msg));
                    const uint32_t received_prn = received_tow_map->first;
                    const GalileoTowMapEntry received_entry = received_tow_map->second;

                    d_galileo_tow.erase(received_prn);
                    if (received_entry.week != GALILEO_TOW_MAP_INVALID_WEEK && received_entry.tow_ms < GALILEO_TOW_MAP_WEEK_MS)  // received TOW is in ms
                        {
                            d_galileo_tow[received_prn] = received_entry;
                        }
                    else
                        {
                            d_galileo_tow[received_prn] = GalileoTowMapEntry{};
                        }
                    const std::shared_ptr<GalileoTowMap> tmp_obj = std::make_shared<GalileoTowMap>(d_galileo_tow);
                    this->message_port_pub(pmt::mp("TOW_to_TLM"), pmt::make_any(tmp_obj));
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "galileo_tow_map Bad any_cast: " << e.what();
        }
}


namespace
{
bool is_valid_tow_entry(const GalileoTowMapEntry& entry)
{
    return entry.week != GALILEO_TOW_MAP_INVALID_WEEK &&
           entry.tow_ms < GALILEO_TOW_MAP_WEEK_MS &&
           entry.sample_counter != GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER;
}
}  // namespace


GalileoTowHistory& GalileoTowHistory::instance()
{
    static GalileoTowHistory history;
    return history;
}


GalileoTowHistory::GalileoTowHistory() : d_retention(std::numeric_limits<uint64_t>::max())
{
}


void GalileoTowHistory::reset(uint64_t retention)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_records.clear();
    d_retention = retention;
}


void GalileoTowHistory::record(uint32_t prn, int32_t channel, uint64_t stamp, const GalileoTowMapEntry& entry)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    auto& timeline = d_records[prn][channel];
    const auto inserted = timeline.emplace(stamp, entry);
    if (!inserted.second && !is_valid_tow_entry(entry))
        {
            // A withdrawal wins over an entry with the same stamp
            inserted.first->second = entry;
        }
    const uint64_t newest = timeline.rbegin()->first;
    if (newest > d_retention)
        {
            // Keep the record in force at the oldest sample that can still be asked for
            const auto after_oldest = timeline.upper_bound(newest - d_retention);
            if (after_oldest != timeline.begin())
                {
                    timeline.erase(timeline.begin(), std::prev(after_oldest));
                }
        }
}


GalileoTowMapEntry GalileoTowHistory::lookup(uint32_t prn, uint64_t sample) const
{
    std::lock_guard<std::mutex> lock(d_mutex);
    GalileoTowMapEntry newest{};
    const auto satellite = d_records.find(prn);
    if (satellite == d_records.end())
        {
            return newest;
        }
    uint64_t newest_stamp = 0;
    for (const auto& channel : satellite->second)
        {
            const auto& timeline = channel.second;
            auto in_force = timeline.upper_bound(sample);
            if (in_force == timeline.begin())
                {
                    continue;
                }
            --in_force;
            if (is_valid_tow_entry(in_force->second) && (!is_valid_tow_entry(newest) || in_force->first > newest_stamp))
                {
                    newest = in_force->second;
                    newest_stamp = in_force->first;
                }
        }
    return newest;
}

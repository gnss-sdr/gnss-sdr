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
#include <glog/logging.h>  // for LOG
#include <limits>          // for std::numeric_limits
#include <memory>          // for std::shared
#include <typeinfo>        // for typeid

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
            d_galileo_tow[prn] = std::pair<uint32_t, uint64_t>(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint64_t>::max());
        }
}


void galileo_tow_map::msg_handler_galileo_tow_map(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<std::pair<uint32_t, std::pair<uint32_t, uint64_t>>>).hash_code())
                {
                    const auto received_tow_map = wht::any_cast<std::shared_ptr<std::pair<uint32_t, std::pair<uint32_t, uint64_t>>>>(pmt::any_ref(msg));
                    const uint32_t received_prn = received_tow_map->first;
                    const uint32_t received_tow = received_tow_map->second.first;
                    const uint64_t received_sample_counter = received_tow_map->second.second;

                    d_galileo_tow.erase(received_prn);
                    if (received_tow < 604800000)  // received TOW is in ms
                        {
                            d_galileo_tow[received_prn] = std::pair<uint32_t, uint64_t>(received_tow, received_sample_counter);
                        }
                    else
                        {
                            d_galileo_tow[received_prn] = std::pair<uint32_t, uint64_t>(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint64_t>::max());
                        }
                    const std::shared_ptr<std::map<uint32_t, std::pair<uint32_t, uint64_t>>> tmp_obj = std::make_shared<std::map<uint32_t, std::pair<uint32_t, uint64_t>>>(d_galileo_tow);
                    this->message_port_pub(pmt::mp("TOW_to_TLM"), pmt::make_any(tmp_obj));
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "galileo_tow_map Bad any_cast: " << e.what();
        }
}

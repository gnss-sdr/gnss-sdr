/*!
 * \file channel_status_msg_receiver.cc
 * \brief GNU Radio block that receives asynchronous channel messages from
 * acquisition and tracking blocks
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
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


#include "channel_status_msg_receiver.h"
#include <boost/any.hpp>
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <cstddef>
#include <cstdint>
#include <typeinfo>
#include <utility>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif


channel_status_msg_receiver_sptr channel_status_msg_receiver_make()
{
    return channel_status_msg_receiver_sptr(new channel_status_msg_receiver());
}


channel_status_msg_receiver::channel_status_msg_receiver() : gr::block("channel_status_msg_receiver", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("status"));
    this->set_msg_handler(pmt::mp("status"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_status(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&channel_status_msg_receiver::msg_handler_channel_status, this, boost::placeholders::_1));
#else
        boost::bind(&channel_status_msg_receiver::msg_handler_channel_status, this, _1));
#endif
#endif
    d_pvt_status.RX_time = -1;  // to indicate that the PVT is not available
}


void channel_status_msg_receiver::msg_handler_channel_status(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_channel_status function called by the scheduler
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            // ****************** Gnss_Synchro received ************************
            if (msg_type_hash_code == typeid(std::shared_ptr<Gnss_Synchro>).hash_code())
                {
                    const auto gnss_synchro_obj = boost::any_cast<std::shared_ptr<Gnss_Synchro>>(pmt::any_ref(msg));
                    if (gnss_synchro_obj->Flag_valid_pseudorange == true)
                        {
                            d_channel_status_map[gnss_synchro_obj->Channel_ID] = gnss_synchro_obj;
                        }
                    else
                        {
                            d_channel_status_map.erase(gnss_synchro_obj->Channel_ID);
                        }

                    // std::cout << "-------- \n" << '\n';
                    // for (std::map<int, std::shared_ptr<Gnss_Synchro>>::iterator it = d_channel_status_map.begin(); it != d_channel_status_map.end(); ++it)
                    //     {
                    //         std::cout << " Channel: " << it->first << " => Doppler: " << it->second->Carrier_Doppler_hz << "[Hz] \n";
                    //     }
                    // std::cout << "-------- \n" << '\n';
                }
            else if (msg_type_hash_code == typeid(std::shared_ptr<Monitor_Pvt>).hash_code())
                {
                    // ***************** Monitor_Pvt received ******************
                    const auto monitor_pvt_obj = boost::any_cast<std::shared_ptr<Monitor_Pvt>>(pmt::any_ref(msg));
                    d_pvt_status = *monitor_pvt_obj.get();

                    // std::cout << "-------- \n" << '\n';
                    // std::cout << "PVT TOW: " << d_pvt_status->TOW_at_current_symbol_ms << '\n';
                    // std::cout << "-------- \n" << '\n';
                }
            else
                {
                    LOG(WARNING) << "channel_status_msg_receiver unknown object type!";
                }
        }
    catch (const boost::bad_any_cast& e)
        {
            LOG(WARNING) << "channel_status_msg_receiver Bad any_cast: " << e.what();
        }
}


std::map<int, std::shared_ptr<Gnss_Synchro>> channel_status_msg_receiver::get_current_status_map()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_channel_status function called by the scheduler
    return d_channel_status_map;
}


Monitor_Pvt channel_status_msg_receiver::get_current_status_pvt()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_channel_status function called by the scheduler
    return d_pvt_status;
}

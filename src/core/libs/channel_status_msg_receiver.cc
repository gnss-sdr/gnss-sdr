/*!
 * \file channel_status_msg_receiver.cc
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "channel_status_msg_receiver.h"
#include <boost/any.hpp>
#include <boost/bind.hpp>
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <utility>


channel_status_msg_receiver_sptr channel_status_msg_receiver_make()
{
    return channel_status_msg_receiver_sptr(new channel_status_msg_receiver());
}


channel_status_msg_receiver::channel_status_msg_receiver() : gr::block("channel_status_msg_receiver", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("status"));
    this->set_msg_handler(pmt::mp("status"), boost::bind(&channel_status_msg_receiver::msg_handler_events, this, _1));
    d_pvt_status.RX_time = -1;  // to indicate that the PVT is not available
}


void channel_status_msg_receiver::msg_handler_events(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_events function called by the scheduler
    try
        {
            // ************* Gnss_Synchro received *****************
            if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gnss_Synchro>))
                {
                    std::shared_ptr<Gnss_Synchro> gnss_synchro_obj;
                    gnss_synchro_obj = boost::any_cast<std::shared_ptr<Gnss_Synchro>>(pmt::any_ref(msg));
                    if (gnss_synchro_obj->Flag_valid_pseudorange == true)
                        {
                            d_channel_status_map[gnss_synchro_obj->Channel_ID] = gnss_synchro_obj;
                        }
                    else
                        {
                            d_channel_status_map.erase(gnss_synchro_obj->Channel_ID);
                        }

                    // std::cout << "-------- " << std::endl << std::endl;
                    // for (std::map<int, std::shared_ptr<Gnss_Synchro>>::iterator it = d_channel_status_map.begin(); it != d_channel_status_map.end(); ++it)
                    //     {
                    //         std::cout << " Channel: " << it->first << " => Doppler: " << it->second->Carrier_Doppler_hz << "[Hz] " << std::endl;
                    //     }
                    // std::cout << "-------- " << std::endl << std::endl;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Monitor_Pvt>))
                {
                    // ************* Monitor_Pvt received *****************
                    std::shared_ptr<Monitor_Pvt> monitor_pvt_obj;
                    monitor_pvt_obj = boost::any_cast<std::shared_ptr<Monitor_Pvt>>(pmt::any_ref(msg));
                    d_pvt_status = *monitor_pvt_obj.get();

                    // std::cout << "-------- " << std::endl << std::endl;
                    // std::cout << "PVT TOW: " << d_pvt_status->TOW_at_current_symbol_ms << std::endl;
                    // std::cout << "-------- " << std::endl << std::endl;
                }
            else
                {
                    LOG(WARNING) << "channel_status_msg_receiver unknown object type!";
                }
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "channel_status_msg_receiver Bad any cast!";
        }
}


std::map<int, std::shared_ptr<Gnss_Synchro>> channel_status_msg_receiver::get_current_status_map()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_events function called by the scheduler
    return d_channel_status_map;
}


Monitor_Pvt channel_status_msg_receiver::get_current_status_pvt()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_events function called by the scheduler
    return d_pvt_status;
}

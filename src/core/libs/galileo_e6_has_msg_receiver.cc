/*!
 * \file galileo_e6_has_msg_receiver.cc
 * \brief GNU Radio block that receives asynchronous Galileo E6 HAS message
 * sections from Galileo E6 telemetry blocks
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
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


#include "galileo_e6_has_msg_receiver.h"
#include "galileo_has_data.h"  // for Galileo_HAS_data
#include "galileo_has_page.h"  // for Galileo_HAS_page
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


galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make()
{
    return galileo_e6_has_msg_receiver_sptr(new galileo_e6_has_msg_receiver());
}


galileo_e6_has_msg_receiver::galileo_e6_has_msg_receiver() : gr::block("galileo_e6_has_msg_receiver", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    // register Gal E6 HAS input message port
    this->message_port_register_in(pmt::mp("E6_HAS_from_TLM"));
    this->set_msg_handler(pmt::mp("E6_HAS_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_galileo_e6_has(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has, this, boost::placeholders::_1));
#else
        boost::bind(&galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has, this, _1));
#endif
#endif

    // register Gal E6 processed HAS out
    this->message_port_register_out(pmt::mp("E6_HAS_to_PVT"));
}


void galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_galileo_e6_has function called by the scheduler
    // 1. receive the PMT message and reconstruct the object...
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();

            if (msg_type_hash_code == typeid(std::shared_ptr<Galileo_HAS_page>).hash_code())
                {
                    const auto HAS_data_obj_obj = boost::any_cast<std::shared_ptr<Galileo_HAS_page>>(pmt::any_ref(msg));
                    // std::cout << HAS_data_obj_obj->has_message_string << '\n';
                    // store / do things with the data
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

    // 2. Trigger the HAS processing function if required
    // TODO

    // 3. Send the resulting decoded HAS data (if available) to PVT

    // TODO: fill message object and send to PVT when ready
    std::shared_ptr<Galileo_HAS_data> dummy{};
    this->message_port_pub(pmt::mp("E6_HAS_to_PVT"), pmt::make_any(dummy));
}

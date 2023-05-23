/*!
 * \file osnma_msg_receiver.cc
 * \brief GNU Radio block that processes Galileo OSNMA data received from
 * Galileo E1B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2023. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "osnma_msg_receiver.h"
#include "display.h"                // for colors in terminal
#include "gnss_sdr_make_unique.h"   // for std::make_unique in C++11
#include <glog/logging.h>           // for DLOG
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <bitset>
#include <cstdint>
#include <iostream>
#include <typeinfo>  // for typeid

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

osnma_msg_receiver_sptr osnma_msg_receiver_make()
{
    return osnma_msg_receiver_sptr(new osnma_msg_receiver());
}


osnma_msg_receiver::osnma_msg_receiver() : gr::block("osnma_msg_receiver",
                                               gr::io_signature::make(0, 0, 0),
                                               gr::io_signature::make(0, 0, 0))
{
    // register OSNMA input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("OSNMA_from_TLM"));
    // register OSNMA output message port to PVT block
    this->message_port_register_out(pmt::mp("OSNMA_to_PVT"));

    this->set_msg_handler(pmt::mp("OSNMA_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_osnma(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, boost::placeholders::_1));
#else
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, _1));
#endif
#endif
}


void osnma_msg_receiver::msg_handler_osnma(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_osnma function called by the scheduler
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(OSNMA_msg).hash_code())
                {
                    const auto nma_msg = wht::any_cast<OSNMA_msg>(pmt::any_ref(msg));
                    process_osnma_message(nma_msg);
                }
            else
                {
                    LOG(WARNING) << "osnma_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "osnma_msg_receiver Bad any_cast: " << e.what();
        }

    //  Send the resulting decoded NMA data (if available) to PVT
    if (d_new_data == true)
        {
            auto osnma_data_ptr = std::make_shared<OSNMA_data>(d_osnma_data);
            this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(osnma_data_ptr));
            d_new_data = false;
            DLOG(INFO) << "NMA info sent to the PVT block through the OSNMA_to_PVT async message port";
        }
}


void osnma_msg_receiver::process_osnma_message(const OSNMA_msg& osnma_msg)
{
    auto hkroot_msg = osnma_msg.hkroot;
    read_nma_header(hkroot_msg[0]);
    read_dsm_header(hkroot_msg[1]);
}


void osnma_msg_receiver::read_nma_header(uint8_t nma_header)
{
    d_osnma_data.d_nma_header.nmas = (nma_header & 0b11000000) << 6;
    d_osnma_data.d_nma_header.cid = (nma_header & 0b00110000) << 4;
    d_osnma_data.d_nma_header.cpks = (nma_header & 0b00001110) << 1;
    d_osnma_data.d_nma_header.reserved = ((nma_header & 0b00000001) ? true : false);
}


void osnma_msg_receiver::read_dsm_header(uint8_t dsm_header)
{
    d_osnma_data.d_dsm_header.dsm_id = (dsm_header & 0b11110000) << 4;
    d_osnma_data.d_dsm_header.dsm_block_id = dsm_header & 0b00001111;
}
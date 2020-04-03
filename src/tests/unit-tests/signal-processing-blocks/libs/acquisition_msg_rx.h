/*!
 * \file acquisition_msg_rx.h
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQUISITION_MSG_RX_H
#define GNSS_SDR_ACQUISITION_MSG_RX_H

#include <gnuradio/block.h>
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>

#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

// ######## GNURADIO ACQUISITION BLOCK MESSAGE RECEVER #########
class Acquisition_msg_rx;

#if GNURADIO_USES_STD_POINTERS
using Acquisition_msg_rx_sptr = std::shared_ptr<Acquisition_msg_rx>;
#else
using Acquisition_msg_rx_sptr = boost::shared_ptr<Acquisition_msg_rx>;
#endif

Acquisition_msg_rx_sptr Acquisition_msg_rx_make();


class Acquisition_msg_rx : public gr::block
{
private:
    friend Acquisition_msg_rx_sptr Acquisition_msg_rx_make();
    void msg_handler_events(const pmt::pmt_t& msg);
    Acquisition_msg_rx();

public:
    int rx_message;
    gr::top_block_sptr top_block;
    ~Acquisition_msg_rx();  //!< Default destructor
};


#endif  // GNSS_SDR_ACQUISITION_MSG_RX_H

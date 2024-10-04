/*!
 * \file acquisition_msg_rx.h
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQUISITION_MSG_RX_H
#define GNSS_SDR_ACQUISITION_MSG_RX_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>


// ######## GNURADIO ACQUISITION BLOCK MESSAGE RECEVER #########
class Acquisition_msg_rx;

using Acquisition_msg_rx_sptr = gnss_shared_ptr<Acquisition_msg_rx>;

Acquisition_msg_rx_sptr Acquisition_msg_rx_make();


class Acquisition_msg_rx : public gr::block
{
private:
    friend Acquisition_msg_rx_sptr Acquisition_msg_rx_make();
    void msg_handler_channel_events(const pmt::pmt_t& msg);
    Acquisition_msg_rx();

public:
    int rx_message;
    gr::top_block_sptr top_block;
    ~Acquisition_msg_rx();  //!< Default destructor
};


#endif  // GNSS_SDR_ACQUISITION_MSG_RX_H

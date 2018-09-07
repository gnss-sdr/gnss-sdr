/*!
 * \file acquisition_msg_rx.h
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_ACQUISITION_MSG_RX_H
#define GNSS_SDR_ACQUISITION_MSG_RX_H

#include <gnuradio/top_block.h>
#include <gnuradio/block.h>
#include <pmt/pmt.h>

// ######## GNURADIO ACQUISITION BLOCK MESSAGE RECEVER #########
class Acquisition_msg_rx;

typedef boost::shared_ptr<Acquisition_msg_rx> Acquisition_msg_rx_sptr;

Acquisition_msg_rx_sptr Acquisition_msg_rx_make();


class Acquisition_msg_rx : public gr::block
{
private:
    friend Acquisition_msg_rx_sptr Acquisition_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    Acquisition_msg_rx();

public:
    int rx_message;
    gr::top_block_sptr top_block;
    ~Acquisition_msg_rx();  //!< Default destructor
};


#endif

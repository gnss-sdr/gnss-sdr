/*!
 * \file galileo_e6_has_msg_receiver.h
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

#ifndef GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H
#define GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H

#include "gnss_block_interface.h"
#include "reed_solomon.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <map>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


class galileo_e6_has_msg_receiver;

using galileo_e6_has_msg_receiver_sptr = gnss_shared_ptr<galileo_e6_has_msg_receiver>;

galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make();

/*!
 * \brief GNU Radio block that receives asynchronous galileo e6 has messages from tlm blocks
 */
class galileo_e6_has_msg_receiver : public gr::block
{
public:
    ~galileo_e6_has_msg_receiver() = default;  //!< Default destructor

private:
    friend galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make();
    galileo_e6_has_msg_receiver();
    void msg_handler_galileo_e6_has(const pmt::pmt_t& msg);
    ReedSolomon rs = ReedSolomon();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H

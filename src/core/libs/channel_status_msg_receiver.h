/*!
 * \file channel_status_msg_receiver.h
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

#ifndef GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H
#define GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H

#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "monitor_pvt.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <map>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


class channel_status_msg_receiver;

using channel_status_msg_receiver_sptr = gnss_shared_ptr<channel_status_msg_receiver>;

channel_status_msg_receiver_sptr channel_status_msg_receiver_make();

/*!
 * \brief GNU Radio block that receives asynchronous channel messages from tlm blocks
 */
class channel_status_msg_receiver : public gr::block
{
public:
    ~channel_status_msg_receiver() = default;  //!< Default destructor

    /*!
     * \brief return the current status map of all channels with valid telemetry
     */
    std::map<int, std::shared_ptr<Gnss_Synchro>> get_current_status_map();

    /*!
     * \brief return the current status map of all locked channels, including
     * those without a valid time reference yet (no valid pseudorange). Meant
     * for the secondary-frequency acquisition assistance, which only needs the
     * satellite and its Doppler.
     */
    std::map<int, std::shared_ptr<Gnss_Synchro>> get_current_tracking_map();

    /*!
     * \brief return the latest successful PVT and optionally elapsed sample time
     */
    Monitor_Pvt get_current_status_pvt(double* receiver_time_s = nullptr);

    /*!
     * \brief forget the status of a channel that has been stopped, so that a
     * stale entry is not reported until the channel delivers new observables
     */
    void clear_channel_status(int channel_id);

private:
    friend channel_status_msg_receiver_sptr channel_status_msg_receiver_make();
    channel_status_msg_receiver();
    void msg_handler_channel_status(const pmt::pmt_t& msg);
    Monitor_Pvt d_pvt_status{};
    double d_receiver_time_s{0.0};  // elapsed sample time, including epochs without a fix
    std::map<int, std::shared_ptr<Gnss_Synchro>> d_channel_status_map;
    std::map<int, std::shared_ptr<Gnss_Synchro>> d_channel_tracking_map;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H

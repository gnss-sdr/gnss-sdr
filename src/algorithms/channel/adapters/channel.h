/*!
 * \file channel.h
 * \brief Interface of a GNSS channel.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * It holds blocks for acquisition, tracking,
 * navigation data extraction and pseudorange calculation.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CHANNEL_H_
#define GNSS_SDR_CHANNEL_H_

#include <memory>
#include <string>
#include <gnuradio/msg_queue.h>
#include <gnuradio/block.h>
#include "channel_interface.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "channel_msg_receiver_cc.h"

class ConfigurationInterface;
class AcquisitionInterface;
class TrackingInterface;
class TelemetryDecoderInterface;

/*!
 * \brief This class represents a GNSS channel. It wraps an AcquisitionInterface,
 * a Tracking Interface and a TelemetryDecoderInterface, and handles
 * their interaction through a Finite State Machine
 *
 */
class Channel: public ChannelInterface
{

public:
    //! Constructor
    Channel(ConfigurationInterface *configuration, unsigned int channel,
            std::shared_ptr<GNSSBlockInterface> pass_through, std::shared_ptr<AcquisitionInterface> acq,
            std::shared_ptr<TrackingInterface> trk, std::shared_ptr<TelemetryDecoderInterface> nav,
            std::string role, std::string implementation,
            boost::shared_ptr<gr::msg_queue> queue);
    //! Virtual destructor
    virtual ~Channel();
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();
    std::string role(){ return role_; }

    //! Returns "Channel"
    std::string implementation(){ return implementation_; }
    size_t item_size(){ return 0; }
    Gnss_Signal get_signal() const { return gnss_signal_; }
    std::shared_ptr<AcquisitionInterface> acquisition(){ return acq_; }
    std::shared_ptr<TrackingInterface> tracking(){ return trk_; }
    std::shared_ptr<TelemetryDecoderInterface> telemetry(){ return nav_; }
    void start_acquisition();                   //!< Start the State Machine
    void set_signal(const Gnss_Signal& gnss_signal_);  //!< Sets the channel GNSS signal

    void msg_handler_events(pmt::pmt_t msg);


private:
    channel_msg_receiver_cc_sptr channel_msg_rx;
    std::shared_ptr<GNSSBlockInterface> pass_through_;
    std::shared_ptr<AcquisitionInterface> acq_;
    std::shared_ptr<TrackingInterface> trk_;
    std::shared_ptr<TelemetryDecoderInterface> nav_;
    std::string role_;
    std::string implementation_;
    bool flag_enable_fpga;
    unsigned int channel_;
    Gnss_Synchro gnss_synchro_;
    Gnss_Signal gnss_signal_;
    bool connected_;
    bool repeat_;
    ChannelFsm channel_fsm_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_CHANNEL_H_*/

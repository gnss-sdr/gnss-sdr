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

#include <string>
#include <gnuradio/msg_queue.h>
#include "channel_interface.h"
#include "channel_fsm.h"
#include "control_message_factory.h"
#include "concurrent_queue.h"
#include "gnss_signal.h"
#include "gnss_synchro.h"


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
            GNSSBlockInterface *pass_through, AcquisitionInterface *acq,
            TrackingInterface *trk, TelemetryDecoderInterface *nav,
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
    AcquisitionInterface* acquisition(){ return acq_; }
    TrackingInterface* tracking(){ return trk_; }
    TelemetryDecoderInterface* telemetry(){ return nav_; }
    void start_acquisition();                   //!< Start the State Machine
    void set_signal(const Gnss_Signal& gnss_signal_);  //!< Sets the channel GNSS signal
    void start();                               //!< Start the thread
    void standby();
    /*!
     * \brief Set stop_ to true and blocks the calling thread until
     * the thread of the constructor has completed
     */
    void stop();

private:
    GNSSBlockInterface *pass_through_;
    AcquisitionInterface *acq_;
    TrackingInterface *trk_;
    TelemetryDecoderInterface *nav_;
    std::string role_;
    std::string implementation_;
    unsigned int channel_;
    Gnss_Synchro gnss_synchro_;
    Gnss_Signal gnss_signal_;
    bool connected_;
    bool stop_;
    int message_;
    bool repeat_;
    ChannelFsm channel_fsm_;
    boost::shared_ptr<gr::msg_queue> queue_;
    concurrent_queue<int> channel_internal_queue_;
    boost::thread ch_thread_;
    void run();
    void process_channel_messages();
};

#endif /*GNSS_SDR_CHANNEL_H_*/

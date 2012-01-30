/*!
 * \file channel.cc
 * \brief Implementation of a GPS_L1_CA_Channel with a Finite State Machine
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include "channel.h"
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"
#include "configuration_interface.h"
#include "gnss_flowgraph.h"
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/gr_io_signature.h>
#include <gnuradio/gr_message.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

// Constructor
Channel::Channel(ConfigurationInterface *configuration, unsigned int channel,
        GNSSBlockInterface *pass_through, AcquisitionInterface *acq,
        TrackingInterface *trk, TelemetryDecoderInterface *nav,
        std::string role, std::string implementation, gr_msg_queue_sptr queue) :
        pass_through_(pass_through), acq_(acq), trk_(trk), nav_(nav),
        role_(role), implementation_(implementation), channel_(channel),
        queue_(queue)

{
    stop_ = false;
    acq_->set_channel(channel_);
    trk_->set_channel(channel_);
    nav_->set_channel(channel_);

    gnss_synchro_.Channel_ID=channel_;

    acq_->set_gnss_synchro(&gnss_synchro_);
    trk_->set_gnss_synchro(&gnss_synchro_);

    acq_->set_threshold(configuration->property("Acquisition"
            + boost::lexical_cast<std::string>(channel_) + ".threshold", 0.0));
    acq_->set_doppler_max(configuration->property("Acquisition"
            + boost::lexical_cast<std::string>(channel_) + ".doppler_max",
            10000));
    acq_->set_doppler_step(configuration->property("Acquisition"
            + boost::lexical_cast<std::string>(channel_) + ".doppler_step",
            250));

    repeat_ = configuration->property("Acquisition" + boost::lexical_cast<
            std::string>(channel_) + ".repeat_satellite", false);

    DLOG(INFO) << "Channel " << channel_ << " satellite repeat = " << repeat_
            << std::endl;

    acq_->set_channel_queue(&channel_internal_queue_);
    trk_->set_channel_queue(&channel_internal_queue_);

    channel_fsm_.set_acquisition(acq_);
    channel_fsm_.set_tracking(trk_);
    channel_fsm_.set_channel(channel_);
    channel_fsm_.set_queue(queue_);

    connected_ = false;
    message_ = 0;
    gnss_signal_ = Gnss_Signal();
}


// Destructor
Channel::~Channel()
{
    delete acq_;
    delete trk_;
    delete nav_;
    delete pass_through_;
}



void Channel::connect(gr_top_block_sptr top_block)
{

    if (connected_)
        {
            LOG_AT_LEVEL(WARNING) << "channel already connected internally";
            return;
        }

    pass_through_->connect(top_block);
    acq_->connect(top_block);
    trk_->connect(top_block);
    nav_->connect(top_block);

    top_block->connect(pass_through_->get_right_block(), 0,
            acq_->get_left_block(), 0);
    DLOG(INFO) << "pass_through_ -> acquisition";

    top_block->connect(pass_through_->get_right_block(), 0,
            trk_->get_left_block(), 0);
    DLOG(INFO) << "pass_through_ -> tracking";
    top_block->connect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);
    DLOG(INFO) << "tracking -> telemetry_decoder";

    connected_ = true;
}


void Channel::disconnect(gr_top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG_AT_LEVEL(WARNING) << "Channel already disconnected internally";
            return;
        }

    top_block->disconnect(acq_->get_right_block(), 0, trk_->get_left_block(), 0);
    top_block->disconnect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);

    acq_->disconnect(top_block);
    trk_->disconnect(top_block);
    nav_->disconnect(top_block);

    connected_ = false;
}



gr_basic_block_sptr Channel::get_left_block()
{
    return pass_through_->get_left_block();
}



gr_basic_block_sptr Channel::get_right_block()
{
    return nav_->get_right_block();
}



void Channel::set_signal(Gnss_Signal gnss_signal)
{
	gnss_signal_=gnss_signal;
	gnss_signal_.get_signal().copy(gnss_synchro_.Signal,2,0);
	gnss_synchro_.PRN=gnss_signal_.get_satellite().get_PRN();
	gnss_synchro_.System=gnss_signal_.get_satellite().get_system_short().c_str()[0];
	acq_->init();
    nav_->set_satellite(gnss_signal_.get_satellite());
}



void Channel::start_acquisition()
{
    channel_fsm_.Event_gps_start_acquisition();
}



void Channel::start()
{
    ch_thread_ = boost::thread(&Channel::run, this);
}



void Channel::run()
{
    start_acquisition();
    while (!stop_)
        {
            channel_internal_queue_.wait_and_pop(message_);
            process_channel_messages();
        }

}

/*
 * Set stop_ to true and blocks the calling thread until
 * the thread of the constructor has completed
 */
void Channel::stop()
{
    stop_ = true;
    channel_internal_queue_.push(0); //message to stop channel

    /* When the boost::thread object that represents a thread of execution
     * is destroyed the thread becomes detached. Once a thread is detached,
     * it will continue executing until the invocation of the function or
     * callable object supplied on construction has completed,
     * or the program is terminated. In order to wait for a thread of
     * execution to finish, the join() or timed_join() member functions of
     * the boost::thread object must be used. join() will block the calling
     * thread until the thread represented by the boost::thread object
     * has completed.
     *
     */
    ch_thread_.join();
}

void Channel::process_channel_messages()
{
    switch (message_)
    {
    case 0:

        LOG_AT_LEVEL(INFO) << "Stop channel " << channel_;
        break;

    case 1:

        LOG_AT_LEVEL(INFO) << "Channel " << channel_
        << " ACQ SUCCESS satellite " << gnss_synchro_.System << " "<< gnss_synchro_.PRN;
        channel_fsm_.Event_gps_valid_acquisition();
        break;

    case 2:

        LOG_AT_LEVEL(INFO) << "Channel " << channel_
        << " ACQ FAILED satellite " << gnss_synchro_.System << " "<< gnss_synchro_.PRN;
        if (repeat_ == true)
            {
                channel_fsm_.Event_gps_failed_acquisition_repeat();
            }
        else
            {
                channel_fsm_.Event_gps_failed_acquisition_no_repeat();
            }
        break;

    case 3:
        LOG_AT_LEVEL(INFO) << "Channel " << channel_
        << " TRACKING FAILED satellite " << gnss_synchro_.System << " "<< gnss_synchro_.PRN
        << ", reacquisition.";
        channel_fsm_.Event_gps_failed_tracking();
        break;

    default:
        LOG_AT_LEVEL(WARNING) << "Default case, invalid message.";
        break;
    }
}


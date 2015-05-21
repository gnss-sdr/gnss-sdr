/*!
 * \file channel.cc
 * \brief Implementation of a GPS_L1_CA_Channel with a Finite State Machine
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#include "channel.h"
#include <cstring>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/message.h>
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"
#include "configuration_interface.h"
#include "gnss_flowgraph.h"




using google::LogMessage;

// Constructor
Channel::Channel(ConfigurationInterface *configuration, unsigned int channel,
        GNSSBlockInterface *pass_through, AcquisitionInterface *acq,
        TrackingInterface *trk, TelemetryDecoderInterface *nav,
        std::string role, std::string implementation, boost::shared_ptr<gr::msg_queue> queue) :
                pass_through_(pass_through), acq_(acq), trk_(trk), nav_(nav),
                role_(role), implementation_(implementation), channel_(channel),
                queue_(queue)
{
    stop_ = false;
    acq_->set_channel(channel_);
    trk_->set_channel(channel_);
    nav_->set_channel(channel_);

    gnss_synchro_.Channel_ID = channel_;
    acq_->set_gnss_synchro(&gnss_synchro_);
    trk_->set_gnss_synchro(&gnss_synchro_);

    // IMPORTANT: Do not change the order between set_doppler_max, set_doppler_step and set_threshold
    unsigned int doppler_max = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".doppler_max", 0);
    if(doppler_max == 0) doppler_max = configuration->property("Acquisition_" + implementation_+ ".doppler_max", 0);
    DLOG(INFO) << "Channel "<< channel_ << " Doppler_max = " << doppler_max;

    acq_->set_doppler_max(doppler_max);

    unsigned int doppler_step = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".doppler_step" ,0);
    if(doppler_step == 0) doppler_step = configuration->property("Acquisition_" + implementation_+".doppler_step", 500);
    DLOG(INFO) << "Channel "<< channel_ << " Doppler_step = " << doppler_step;

    acq_->set_doppler_step(doppler_step);

    float threshold = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".threshold", 0.0);
    if(threshold == 0.0) threshold = configuration->property("Acquisition_" + implementation_ + ".threshold", 0.0);

    acq_->set_threshold(threshold);

    acq_->init();

    repeat_ = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".repeat_satellite", false);
    DLOG(INFO) << "Channel " << channel_ << " satellite repeat = " << repeat_;

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



void Channel::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "channel already connected internally";
            return;
        }
    pass_through_->connect(top_block);
    acq_->connect(top_block);
    trk_->connect(top_block);
    nav_->connect(top_block);

    top_block->connect(pass_through_->get_right_block(), 0, acq_->get_left_block(), 0);
    DLOG(INFO) << "pass_through_ -> acquisition";
    top_block->connect(pass_through_->get_right_block(), 0, trk_->get_left_block(), 0);
    DLOG(INFO) << "pass_through_ -> tracking";
    top_block->connect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);
    DLOG(INFO) << "tracking -> telemetry_decoder";
    connected_ = true;
}



void Channel::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Channel already disconnected internally";
            return;
        }
    top_block->disconnect(pass_through_->get_right_block(), 0, acq_->get_left_block(), 0);
    top_block->disconnect(pass_through_->get_right_block(), 0, trk_->get_left_block(), 0);
    top_block->disconnect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);
    pass_through_->disconnect(top_block);
    acq_->disconnect(top_block);
    trk_->disconnect(top_block);
    nav_->disconnect(top_block);
    connected_ = false;
}



gr::basic_block_sptr Channel::get_left_block()
{
    return pass_through_->get_left_block();
}



gr::basic_block_sptr Channel::get_right_block()
{
    return nav_->get_right_block();
}



void Channel::set_signal(const Gnss_Signal& gnss_signal)
{
    gnss_signal_ = gnss_signal;
    std::string str_aux = gnss_signal_.get_signal_str();
    const char * str = str_aux.c_str(); // get a C style null terminated string
    std::memcpy((void*)gnss_synchro_.Signal, str, 3); // copy string into synchro char array: 2 char + null
    gnss_synchro_.Signal[2] = 0; // make sure that string length is only two characters
    gnss_synchro_.PRN = gnss_signal_.get_satellite().get_PRN();
    gnss_synchro_.System = gnss_signal_.get_satellite().get_system_short().c_str()[0];
    acq_->set_local_code();
    nav_->set_satellite(gnss_signal_.get_satellite());
}



void Channel::start_acquisition()
{
    channel_fsm_.Event_start_acquisition();
}



void Channel::start()
{
    ch_thread_ = boost::thread(&Channel::run, this);
}



void Channel::run()
{
    while (!stop_)
        {
            channel_internal_queue_.wait_and_pop(message_);
            process_channel_messages();
        }
}



void Channel::standby()
{
    channel_fsm_.Event_failed_tracking_standby();
}



/*
 * Set stop_ to true and blocks the calling thread until
 * the thread of the constructor has completed
 */
void Channel::stop()
{
    channel_internal_queue_.push(0); //message to stop channel
    stop_ = true;
    /* When the boost::thread object that represents a thread of execution
     * is destroyed the thread becomes detached. Once a thread is detached,
     * it will continue executing until the invocation of the function or
     * callable object supplied on construction has completed,
     * or the program is terminated. In order to wait for a thread of
     * execution to finish, the join() or timed_join() member functions of
     * the boost::thread object must be used. join() will block the calling
     * thread until the thread represented by the boost::thread object
     * has completed.
     * NOTE: timed_join() is deprecated and only present up to Boost 1.56
     *       try_join_until() was introduced in Boost 1.50
     */
#ifdef OLD_BOOST
    ch_thread_.timed_join(boost::posix_time::seconds(1));
#endif
#ifndef OLD_BOOST
    ch_thread_.try_join_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(50));
#endif
}



void Channel::process_channel_messages()
{
    switch (message_)
    {
    case 0:
        DLOG(INFO) << "Stop channel " << channel_;
        break;
    case 1:
        DLOG(INFO) << "Channel " << channel_ << " ACQ SUCCESS satellite " <<
            gnss_synchro_.System << " " << gnss_synchro_.PRN;
        channel_fsm_.Event_valid_acquisition();
        break;
    case 2:
        DLOG(INFO) << "Channel " << channel_
            << " ACQ FAILED satellite " << gnss_synchro_.System << " " << gnss_synchro_.PRN;
        if (repeat_ == true)
            {
                channel_fsm_.Event_failed_acquisition_repeat();
            }
        else
            {
                channel_fsm_.Event_failed_acquisition_no_repeat();
            }
        break;
    default:
        LOG(WARNING) << "Default case, invalid message.";
        break;
    }
}


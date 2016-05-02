/*!
 * \file channel_fsm.cc
 * \brief Implementation of a State Machine for channel using boost::statechart
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#include "channel_fsm.h"
#include <memory>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <glog/logging.h>
#include "control_message_factory.h"


struct Ev_channel_start_acquisition: sc::event<Ev_channel_start_acquisition>
{};

struct Ev_channel_valid_acquisition: sc::event<Ev_channel_valid_acquisition>
{};

struct Ev_channel_failed_acquisition_repeat: sc::event<Ev_channel_failed_acquisition_repeat>
{};

struct Ev_channel_failed_acquisition_no_repeat: sc::event<Ev_channel_failed_acquisition_no_repeat>
{};

struct Ev_channel_failed_tracking_standby: sc::event<Ev_channel_failed_tracking_standby>
{};

//struct Ev_channel_failed_tracking_reacq: sc::event<Ev_channel_failed_tracking_reacq>
//{};

struct channel_idle_fsm_S0: public sc::state<channel_idle_fsm_S0, ChannelFsm>
{
public:
    // sc::transition(event, next state)
    typedef sc::transition<Ev_channel_start_acquisition, channel_acquiring_fsm_S1> reactions;
    channel_idle_fsm_S0(my_context ctx) : my_base(ctx)
    {
        //std::cout << "Enter Channel_Idle_S0 " << std::endl;
    }
};


struct channel_acquiring_fsm_S1: public sc::state<channel_acquiring_fsm_S1, ChannelFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_channel_failed_acquisition_no_repeat, channel_waiting_fsm_S3>,
                      sc::transition<Ev_channel_failed_acquisition_repeat, channel_acquiring_fsm_S1>,
                      sc::transition<Ev_channel_valid_acquisition, channel_tracking_fsm_S2> > reactions;

    channel_acquiring_fsm_S1(my_context ctx) : my_base(ctx)
    {
        //std::cout << "Enter Channel_Acq_S1 " << std::endl;
        context<ChannelFsm> ().start_acquisition();
    }
    ~channel_acquiring_fsm_S1()
    {
        //std::cout << "Exit Channel_Acq_S1 " << std::endl;
    }
};


struct channel_tracking_fsm_S2: public sc::state<channel_tracking_fsm_S2, ChannelFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_channel_failed_tracking_standby, channel_idle_fsm_S0>,
                      sc::transition<Ev_channel_start_acquisition, channel_acquiring_fsm_S1>> reactions;

    channel_tracking_fsm_S2(my_context ctx) : my_base(ctx)
    {
       //std::cout << "Enter Channel_tracking_S2 " << std::endl;
        context<ChannelFsm> ().start_tracking();
    }

    ~channel_tracking_fsm_S2()
    {
        //std::cout << "Exit Channel_tracking_S2 " << std::endl;
        context<ChannelFsm> ().notify_stop_tracking();
    }

};


struct channel_waiting_fsm_S3: public sc::state<channel_waiting_fsm_S3, ChannelFsm>
{
public:
    typedef sc::transition<Ev_channel_start_acquisition,
            channel_acquiring_fsm_S1> reactions;

    channel_waiting_fsm_S3(my_context ctx) :
        my_base(ctx)
    {
        //std::cout << "Enter Channel_waiting_S3 " << std::endl;
        context<ChannelFsm> ().request_satellite();
    }
   // ~channel_waiting_fsm_S3(){}
};



ChannelFsm::ChannelFsm()
{
    acq_ = nullptr;
    trk_ = nullptr;
    channel_ = 0;
    initiate(); //start the FSM
}



ChannelFsm::ChannelFsm(std::shared_ptr<AcquisitionInterface> acquisition) :
            acq_(acquisition)
{
    trk_ = nullptr;
    channel_ = 0;
    initiate(); //start the FSM
}



void ChannelFsm::Event_start_acquisition()
{
    this->process_event(Ev_channel_start_acquisition());
    //std::cout<<"Ev_channel_start_acquisition launched"<<std::endl;
}


void ChannelFsm::Event_valid_acquisition()
{
    this->process_event(Ev_channel_valid_acquisition());
}


void ChannelFsm::Event_failed_acquisition_repeat()
{
    this->process_event(Ev_channel_failed_acquisition_repeat());
}

void ChannelFsm::Event_failed_acquisition_no_repeat()
{
    this->process_event(Ev_channel_failed_acquisition_no_repeat());
}


// Something is wrong here, we are using a memory after it ts freed
void ChannelFsm::Event_failed_tracking_standby()
{
    this->process_event(Ev_channel_failed_tracking_standby());
}

//void ChannelFsm::Event_failed_tracking_reacq() {
//    this->process_event(Ev_channel_failed_tracking_reacq());
//}

void ChannelFsm::set_acquisition(std::shared_ptr<AcquisitionInterface> acquisition)
{
    acq_ = acquisition;
}

void ChannelFsm::set_tracking(std::shared_ptr<TrackingInterface> tracking)
{
    trk_ = tracking;
}

void ChannelFsm::set_queue(boost::shared_ptr<gr::msg_queue> queue)
{
    queue_ = queue;
}

void ChannelFsm::set_channel(unsigned int channel)
{
    channel_ = channel;
}

void ChannelFsm::start_acquisition()
{
    acq_->reset();
}

void ChannelFsm::start_tracking()
{
    trk_->start_tracking();
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (queue_ != gr::msg_queue::make())
        {
            queue_->handle(cmf->GetQueueMessage(channel_, 1));
        }
}

void ChannelFsm::request_satellite()
{
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (queue_ != gr::msg_queue::make())
        {
            queue_->handle(cmf->GetQueueMessage(channel_, 0));
        }
}

void ChannelFsm::notify_stop_tracking()
{
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (queue_ != gr::msg_queue::make())
        {
            queue_->handle(cmf->GetQueueMessage(channel_, 2));
        }
}

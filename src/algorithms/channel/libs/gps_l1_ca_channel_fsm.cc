/*!
 * \file gps_l1_ca_channel_fsm.cc
 * \brief Implementation of a State Machine for channel using boost::statechart
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#include "gps_l1_ca_channel_fsm.h"
#include "control_message_factory.h"
#include "channel.h"
#include <glog/log_severity.h>
#include <glog/logging.h>

struct Ev_gps_channel_start_acquisition: sc::event<Ev_gps_channel_start_acquisition>
{};

struct Ev_gps_channel_valid_acquisition: sc::event<Ev_gps_channel_valid_acquisition>
{};

struct Ev_gps_channel_failed_acquisition_repeat: sc::event<Ev_gps_channel_failed_acquisition_repeat>
{};

struct Ev_gps_channel_failed_acquisition_no_repeat: sc::event<Ev_gps_channel_failed_acquisition_no_repeat>
{};

struct Ev_gps_channel_failed_tracking: sc::event<Ev_gps_channel_failed_tracking>
{};

struct gps_channel_idle_fsm_S0: public sc::state<gps_channel_idle_fsm_S0,GpsL1CaChannelFsm>
{
public:
    // sc::transition(event, next state)
    typedef sc::transition<Ev_gps_channel_start_acquisition, gps_channel_acquiring_fsm_S1> reactions;
    gps_channel_idle_fsm_S0(my_context ctx) :
        my_base(ctx)
    {
        //std::cout << "Enter Channel_Idle_S0 " << std::endl;
    }
};


struct gps_channel_acquiring_fsm_S1: public sc::state<gps_channel_acquiring_fsm_S1, GpsL1CaChannelFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_gps_channel_failed_acquisition_no_repeat, gps_channel_waiting_fsm_S3>,
            sc::transition<Ev_gps_channel_failed_acquisition_repeat, gps_channel_acquiring_fsm_S1>,
            sc::transition<Ev_gps_channel_valid_acquisition, gps_channel_tracking_fsm_S2> >reactions;

    gps_channel_acquiring_fsm_S1(my_context ctx) : my_base(ctx)
    {
        //std::cout << "Enter Channel_Acq_S1 " << std::endl;
        context<GpsL1CaChannelFsm> ().start_acquisition();
    }
};



struct gps_channel_tracking_fsm_S2: public sc::state<gps_channel_tracking_fsm_S2, GpsL1CaChannelFsm>
{
public:
    typedef sc::transition<Ev_gps_channel_failed_tracking, gps_channel_acquiring_fsm_S1> reactions;

    gps_channel_tracking_fsm_S2(my_context ctx) : my_base(ctx)
    {
        //std::cout << "Enter Channel_tracking_S2 " << std::endl;
        context<GpsL1CaChannelFsm> ().start_tracking();
    }
};



struct gps_channel_waiting_fsm_S3: public sc::state<gps_channel_waiting_fsm_S3, GpsL1CaChannelFsm>
{
public:
    typedef sc::transition<Ev_gps_channel_start_acquisition, gps_channel_acquiring_fsm_S1> reactions;

    gps_channel_waiting_fsm_S3(my_context ctx) : my_base(ctx)
    {
        //std::cout << "Enter Channel_waiting_S3 " << std::endl;
        context<GpsL1CaChannelFsm> ().request_satellite();
    }
};



GpsL1CaChannelFsm::GpsL1CaChannelFsm()
{
    initiate(); //start the FSM
}




GpsL1CaChannelFsm::GpsL1CaChannelFsm(AcquisitionInterface *acquisition) : acq_(acquisition)
{
    initiate(); //start the FSM
}




void GpsL1CaChannelFsm::Event_gps_start_acquisition()
{
    this->process_event(Ev_gps_channel_start_acquisition());
}



void GpsL1CaChannelFsm::Event_gps_valid_acquisition()
{
    this->process_event(Ev_gps_channel_valid_acquisition());
}

void GpsL1CaChannelFsm::Event_gps_failed_acquisition_repeat()
{
    this->process_event(Ev_gps_channel_failed_acquisition_repeat());
}



void GpsL1CaChannelFsm::Event_gps_failed_acquisition_no_repeat()
{
    this->process_event(Ev_gps_channel_failed_acquisition_no_repeat());
}



void GpsL1CaChannelFsm::Event_gps_failed_tracking()
{
    this->process_event(Ev_gps_channel_failed_tracking());
}


void GpsL1CaChannelFsm::set_acquisition(AcquisitionInterface *acquisition)
{
    acq_ = acquisition;
}



void GpsL1CaChannelFsm::set_tracking(TrackingInterface *tracking)
{
    trk_ = tracking;
}



void GpsL1CaChannelFsm::set_queue(gr_msg_queue_sptr queue)
{
    queue_ = queue;
}



void GpsL1CaChannelFsm::set_channel(unsigned int channel)
{
    channel_ = channel;
}



void GpsL1CaChannelFsm::start_acquisition()
{
    acq_->reset();
}



void GpsL1CaChannelFsm::start_tracking()
{
    LOG_AT_LEVEL(INFO) << "Channel " << channel_
            << " passing prn code phase " << acq_->prn_code_phase();
    LOG_AT_LEVEL(INFO) << "Channel " << channel_
            << " passing doppler freq shift " << acq_->doppler_freq_shift();
    LOG_AT_LEVEL(INFO) << "Channel " << channel_
            << " passing acquisition sample stamp "
            << acq_->get_sample_stamp();
    trk_->set_prn_code_phase(acq_->prn_code_phase());
    trk_->set_doppler_freq_shift(acq_->doppler_freq_shift());
    trk_->set_acq_sample_stamp(acq_->get_sample_stamp());
    trk_->start_tracking();
}



void GpsL1CaChannelFsm::request_satellite()
{
    ControlMessageFactory* cmf = new ControlMessageFactory();
    if (queue_ != gr_msg_queue_sptr())
        {
            queue_->handle(cmf->GetQueueMessage(channel_, 0));
        }
    delete cmf;
}


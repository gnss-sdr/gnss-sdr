/*!
 * \file gps_l1_ca_channel_fsm.h
 * \brief Interface of the State Machine for channel using boost::statechart
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_CA_CHANNEL_FSM_H
#define GNSS_SDR_GPS_L1_CA_CHANNEL_FSM_H

#include <cstring>
#include <iostream>
#include <queue>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/msg_queue.h>
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"




namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct gps_channel_idle_fsm_S0;
struct gps_channel_acquiring_fsm_S1;
struct gps_channel_tracking_fsm_S2;
struct gps_channel_waiting_fsm_S3;

/*!
 * \brief This class implements a State Machine for channel using boost::statechart
 */
class GpsL1CaChannelFsm: public sc::state_machine<GpsL1CaChannelFsm,gps_channel_idle_fsm_S0>
{

public:

    GpsL1CaChannelFsm();
    GpsL1CaChannelFsm(AcquisitionInterface *acquisition);

    void set_acquisition(AcquisitionInterface *acquisition);
    void set_tracking(TrackingInterface *tracking);
    void set_queue(boost::shared_ptr<gr::msg_queue> queue);
    void set_channel(unsigned int channel);
    void start_acquisition();
    void start_tracking();
    void request_satellite();

    //FSM EVENTS
    void Event_gps_start_acquisition();
    void Event_gps_valid_acquisition();
    void Event_gps_failed_acquisition_repeat();
    void Event_gps_failed_acquisition_no_repeat();
    //void Event_gps_failed_tracking_reacq();
    void Event_gps_failed_tracking_standby();

private:
    AcquisitionInterface *acq_;
    TrackingInterface *trk_;
    TelemetryDecoderInterface *nav_;
    boost::shared_ptr<gr::msg_queue> queue_;
    unsigned int channel_;
};

#endif /*GNSS_SDR_GPS_L1_CA_CHANNEL_FSM_H*/

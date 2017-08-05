/*!
 * \file channel_fsm.h
 * \brief Interface of the State Machine for channel using boost::statechart
 * \author Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
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

#ifndef GNSS_SDR_CHANNEL_FSM_H
#define GNSS_SDR_CHANNEL_FSM_H


#include <boost/statechart/state_machine.hpp>
#include <gnuradio/msg_queue.h>
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"


namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct channel_idle_fsm_S0;
struct channel_acquiring_fsm_S1;
struct channel_tracking_fsm_S2;
struct channel_waiting_fsm_S3;

/*!
 * \brief This class implements a State Machine for channel using boost::statechart
 */
class ChannelFsm : public sc::state_machine<ChannelFsm, channel_idle_fsm_S0> {
public:
    ChannelFsm();

    ChannelFsm(std::shared_ptr<AcquisitionInterface> acquisition);

    void set_acquisition(std::shared_ptr<AcquisitionInterface> acquisition);

    void set_tracking(std::shared_ptr<TrackingInterface> tracking);

    void set_queue(boost::shared_ptr<gr::msg_queue> queue);

    void set_channel(unsigned int channel);

    void start_acquisition();

    void start_tracking();

    void request_satellite();

    void notify_stop_tracking();

    //FSM EVENTS
    void Event_start_acquisition();

    void Event_valid_acquisition();

    void Event_failed_acquisition_repeat();

    void Event_failed_acquisition_no_repeat();

    //void Event_gps_failed_tracking_reacq();
    void Event_failed_tracking_standby();

private:
    std::shared_ptr<AcquisitionInterface> acq_;
    std::shared_ptr<TrackingInterface> trk_;
    boost::shared_ptr<gr::msg_queue> queue_;
    unsigned int channel_;
};

#endif /*GNSS_SDR_CHANNEL_FSM_H*/

/*!
 * \file channel_fsm.h
 * \brief Interface of the State Machine for channel
 * \authors Javier Arribas, 2019. javiarribas@gmail.com
 *          Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *          Luis Esteve,   2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CHANNEL_FSM_H
#define GNSS_SDR_CHANNEL_FSM_H

#include "acquisition_interface.h"
#include "concurrent_queue.h"
#include "telemetry_decoder_interface.h"
#include "tracking_interface.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <mutex>


/*!
 * \brief This class implements a State Machine for channel
 */
class ChannelFsm
{
public:
    ChannelFsm();
    ChannelFsm(std::shared_ptr<AcquisitionInterface> acquisition);

    void set_acquisition(std::shared_ptr<AcquisitionInterface> acquisition);
    void set_tracking(std::shared_ptr<TrackingInterface> tracking);
    void set_telemetry(std::shared_ptr<TelemetryDecoderInterface> telemetry);
    void set_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);
    void set_channel(uint32_t channel);
    void start_acquisition();
    // FSM EVENTS
    bool Event_start_acquisition();
    bool Event_start_acquisition_fpga();
    bool Event_valid_acquisition();
    bool Event_stop_channel();
    bool Event_failed_acquisition_repeat();
    bool Event_failed_acquisition_no_repeat();
    bool Event_failed_tracking_standby();

private:
    void start_tracking();
    void stop_acquisition();
    void stop_tracking();
    void request_satellite();
    void notify_stop_tracking();

    std::shared_ptr<AcquisitionInterface> acq_;
    std::shared_ptr<TrackingInterface> trk_;
    std::shared_ptr<TelemetryDecoderInterface> nav_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
    uint32_t channel_;
    uint32_t d_state;
    std::mutex mx;
};

#endif  // GNSS_SDR_CHANNEL_FSM_H

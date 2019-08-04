/*!
 * \file channel_fsm.cc
 * \brief Implementation of a State Machine for channel
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

#include "channel_fsm.h"
#include "channel_event.h"
#include <glog/logging.h>
#include <utility>

ChannelFsm::ChannelFsm()
{
    acq_ = nullptr;
    trk_ = nullptr;
    channel_ = 0U;
    d_state = 0U;
}


ChannelFsm::ChannelFsm(std::shared_ptr<AcquisitionInterface> acquisition) : acq_(std::move(acquisition))
{
    trk_ = nullptr;
    channel_ = 0U;
    d_state = 0U;
}


bool ChannelFsm::Event_stop_channel()
{
    std::lock_guard<std::mutex> lk(mx);
    DLOG(INFO) << "CH = " << channel_ << ". Ev stop channel";
    switch (d_state)
        {
        case 0:  // already in stanby
            break;
        case 1:  // acquisition
            d_state = 0;
            stop_acquisition();
            break;
        case 2:  // tracking
            d_state = 0;
            stop_tracking();
            break;
        default:
            break;
        }
    return true;
}


bool ChannelFsm::Event_start_acquisition_fpga()
{
    std::lock_guard<std::mutex> lk(mx);
    if ((d_state == 1) || (d_state == 2))
        {
            return false;
        }
    d_state = 1;
    DLOG(INFO) << "CH = " << channel_ << ". Ev start acquisition FPGA";
    return true;
}


bool ChannelFsm::Event_start_acquisition()
{
    std::lock_guard<std::mutex> lk(mx);
    if ((d_state == 1) || (d_state == 2))
        {
            return false;
        }
    d_state = 1;
    start_acquisition();
    DLOG(INFO) << "CH = " << channel_ << ". Ev start acquisition";
    return true;
}


bool ChannelFsm::Event_valid_acquisition()
{
    std::lock_guard<std::mutex> lk(mx);
    if (d_state != 1)
        {
            return false;
        }
    d_state = 2;
    start_tracking();
    DLOG(INFO) << "CH = " << channel_ << ". Ev valid acquisition";
    return true;
}


bool ChannelFsm::Event_failed_acquisition_repeat()
{
    std::lock_guard<std::mutex> lk(mx);
    if (d_state != 1)
        {
            return false;
        }
    d_state = 1;
    start_acquisition();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed acquisition repeat";
    return true;
}


bool ChannelFsm::Event_failed_acquisition_no_repeat()
{
    std::lock_guard<std::mutex> lk(mx);
    if (d_state != 1)
        {
            return false;
        }
    d_state = 3;
    request_satellite();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed acquisition no repeat";
    return true;
}


bool ChannelFsm::Event_failed_tracking_standby()
{
    std::lock_guard<std::mutex> lk(mx);
    if (d_state != 2)
        {
            return false;
        }
    d_state = 0U;
    notify_stop_tracking();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed tracking standby";
    return true;
}


void ChannelFsm::set_acquisition(std::shared_ptr<AcquisitionInterface> acquisition)
{
    std::lock_guard<std::mutex> lk(mx);
    acq_ = std::move(acquisition);
}


void ChannelFsm::set_tracking(std::shared_ptr<TrackingInterface> tracking)
{
    std::lock_guard<std::mutex> lk(mx);
    trk_ = std::move(tracking);
}


void ChannelFsm::set_telemetry(std::shared_ptr<TelemetryDecoderInterface> telemetry)
{
    std::lock_guard<std::mutex> lk(mx);
    nav_ = std::move(telemetry);
}


void ChannelFsm::set_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue)
{
    std::lock_guard<std::mutex> lk(mx);
    queue_ = std::move(queue);
}


void ChannelFsm::set_channel(uint32_t channel)
{
    std::lock_guard<std::mutex> lk(mx);
    channel_ = channel;
}


void ChannelFsm::stop_acquisition()
{
    acq_->stop_acquisition();
}


void ChannelFsm::stop_tracking()
{
    trk_->stop_tracking();
}


void ChannelFsm::start_acquisition()
{
    acq_->reset();
    nav_->reset();
}


void ChannelFsm::start_tracking()
{
    trk_->start_tracking();
    queue_->push(pmt::make_any(channel_event_make(channel_, 1)));
}


void ChannelFsm::request_satellite()
{
    queue_->push(pmt::make_any(channel_event_make(channel_, 0)));
}


void ChannelFsm::notify_stop_tracking()
{
    queue_->push(pmt::make_any(channel_event_make(channel_, 2)));
}

/*!
 * \file channel_fsm.cc
 * \brief Implementation of a State Machine for channel
 * \authors Javier Arribas, 2019. javiarribas@gmail.com
 *          Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *          Luis Esteve,   2011. luis(at)epsilon-formacion.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "channel_fsm.h"
#include "channel_event.h"
#include <glog/logging.h>
#include <utility>

ChannelFsm::ChannelFsm()
    : queue_(nullptr),
      channel_(0U),
      state_(0U)
{
    acq_ = nullptr;
    trk_ = nullptr;
}


ChannelFsm::ChannelFsm(std::shared_ptr<AcquisitionInterface> acquisition)
    : acq_(std::move(acquisition)),
      queue_(nullptr),
      channel_(0U),
      state_(0U)
{
    trk_ = nullptr;
}


bool ChannelFsm::Event_stop_channel()
{
    std::lock_guard<std::mutex> lk(mx_);
    DLOG(INFO) << "CH = " << channel_ << ". Ev stop channel";
    switch (state_)
        {
        case 0:  // already in stanby
            break;
        case 1:  // acquisition
            state_ = 0;
            stop_acquisition();
            break;
        case 2:  // tracking
            state_ = 0;
            stop_tracking();
            break;
        default:
            break;
        }
    return true;
}


bool ChannelFsm::Event_start_acquisition_fpga()
{
    std::lock_guard<std::mutex> lk(mx_);
    if ((state_ == 1) || (state_ == 2))
        {
            return false;
        }
    state_ = 1;
    DLOG(INFO) << "CH = " << channel_ << ". Ev start acquisition FPGA";
    return true;
}


bool ChannelFsm::Event_start_acquisition()
{
    std::lock_guard<std::mutex> lk(mx_);
    if ((state_ == 1) || (state_ == 2))
        {
            return false;
        }
    state_ = 1;
    start_acquisition();
    DLOG(INFO) << "CH = " << channel_ << ". Ev start acquisition";
    return true;
}


bool ChannelFsm::Event_valid_acquisition()
{
    std::lock_guard<std::mutex> lk(mx_);
    if (state_ != 1)
        {
            return false;
        }
    state_ = 2;
    start_tracking();
    DLOG(INFO) << "CH = " << channel_ << ". Ev valid acquisition";
    return true;
}


bool ChannelFsm::Event_failed_acquisition_repeat()
{
    std::lock_guard<std::mutex> lk(mx_);
    if (state_ != 1)
        {
            return false;
        }
    state_ = 1;
    start_acquisition();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed acquisition repeat";
    return true;
}


bool ChannelFsm::Event_failed_acquisition_no_repeat()
{
    std::lock_guard<std::mutex> lk(mx_);
    if (state_ != 1)
        {
            return false;
        }
    state_ = 3;
    request_satellite();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed acquisition no repeat";
    return true;
}


bool ChannelFsm::Event_failed_tracking_standby()
{
    std::lock_guard<std::mutex> lk(mx_);
    if (state_ != 2)
        {
            return false;
        }
    state_ = 0U;
    notify_stop_tracking();
    DLOG(INFO) << "CH = " << channel_ << ". Ev failed tracking standby";
    return true;
}


void ChannelFsm::set_acquisition(std::shared_ptr<AcquisitionInterface> acquisition)
{
    std::lock_guard<std::mutex> lk(mx_);
    acq_ = std::move(acquisition);
}


void ChannelFsm::set_tracking(std::shared_ptr<TrackingInterface> tracking)
{
    std::lock_guard<std::mutex> lk(mx_);
    trk_ = std::move(tracking);
}


void ChannelFsm::set_telemetry(std::shared_ptr<TelemetryDecoderInterface> telemetry)
{
    std::lock_guard<std::mutex> lk(mx_);
    nav_ = std::move(telemetry);
}


void ChannelFsm::set_queue(Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::lock_guard<std::mutex> lk(mx_);
    queue_ = queue;
}


void ChannelFsm::set_channel(uint32_t channel)
{
    std::lock_guard<std::mutex> lk(mx_);
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

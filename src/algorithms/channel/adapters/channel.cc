/*!
 * \file channel.cc
 * \brief Implementation of a GNSS_Channel with a Finite State Machine
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "channel.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <cstdint>

using google::LogMessage;

// Constructor
Channel::Channel(ConfigurationInterface* configuration, uint32_t channel,
    std::shared_ptr<GNSSBlockInterface> pass_through, std::shared_ptr<AcquisitionInterface> acq,
    std::shared_ptr<TrackingInterface> trk, std::shared_ptr<TelemetryDecoderInterface> nav,
    std::string role, std::string implementation, gr::msg_queue::sptr queue)
{
    pass_through_ = pass_through;
    acq_ = acq;
    trk_ = trk;
    nav_ = nav;
    role_ = role;
    implementation_ = implementation;
    channel_ = channel;
    queue_ = queue;
    channel_fsm_ = std::make_shared<ChannelFsm>();

    flag_enable_fpga = configuration->property("Channel.enable_FPGA", false);
    acq_->set_channel(channel_);
    trk_->set_channel(channel_);
    nav_->set_channel(channel_);

    gnss_synchro_.Channel_ID = channel_;
    acq_->set_gnss_synchro(&gnss_synchro_);
    trk_->set_gnss_synchro(&gnss_synchro_);

    // Provide a warning to the user about the change of parameter name
    if (channel_ == 0)
        {
            int64_t deprecation_warning = configuration->property("GNSS-SDR.internal_fs_hz", 0);
            if (deprecation_warning != 0)
                {
                    std::cout << "WARNING: The global parameter name GNSS-SDR.internal_fs_hz has been DEPRECATED." << std::endl;
                    std::cout << "WARNING: Please replace it by GNSS-SDR.internal_fs_sps in your configuration file." << std::endl;
                }
        }

    // IMPORTANT: Do not change the order between set_doppler_step and set_threshold

    uint32_t doppler_step = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".doppler_step", 0);
    if (doppler_step == 0) doppler_step = configuration->property("Acquisition_" + implementation_ + ".doppler_step", 500);
    if (FLAGS_doppler_step != 0) doppler_step = static_cast<uint32_t>(FLAGS_doppler_step);
    DLOG(INFO) << "Channel " << channel_ << " Doppler_step = " << doppler_step;

    acq_->set_doppler_step(doppler_step);

    float threshold = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".threshold", 0.0);
    if (threshold == 0.0) threshold = configuration->property("Acquisition_" + implementation_ + ".threshold", 0.0);

    acq_->set_threshold(threshold);

    acq_->init();

    repeat_ = configuration->property("Acquisition_" + implementation_ + boost::lexical_cast<std::string>(channel_) + ".repeat_satellite", false);
    DLOG(INFO) << "Channel " << channel_ << " satellite repeat = " << repeat_;

    channel_fsm_->set_acquisition(acq_);
    channel_fsm_->set_tracking(trk_);
    channel_fsm_->set_channel(channel_);
    channel_fsm_->set_queue(queue_);

    connected_ = false;

    gnss_signal_ = Gnss_Signal(implementation_);

    channel_msg_rx = channel_msg_receiver_make_cc(channel_fsm_, repeat_);
}


// Destructor
Channel::~Channel() {}
void Channel::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "channel already connected internally";
            return;
        }
    if (flag_enable_fpga == false)
        {
            pass_through_->connect(top_block);
        }
    acq_->connect(top_block);
    trk_->connect(top_block);
    nav_->connect(top_block);

    //Synchronous ports
    if (flag_enable_fpga == false)
        {
            top_block->connect(pass_through_->get_right_block(), 0, acq_->get_left_block(), 0);
            DLOG(INFO) << "pass_through_ -> acquisition";
            top_block->connect(pass_through_->get_right_block(), 0, trk_->get_left_block(), 0);
            DLOG(INFO) << "pass_through_ -> tracking";
        }
    top_block->connect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);
    DLOG(INFO) << "tracking -> telemetry_decoder";

    // Message ports

    top_block->msg_connect(acq_->get_right_block(), pmt::mp("events"), channel_msg_rx, pmt::mp("events"));
    top_block->msg_connect(trk_->get_right_block(), pmt::mp("events"), channel_msg_rx, pmt::mp("events"));

    connected_ = true;
}


void Channel::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Channel already disconnected internally";
            return;
        }

    if (flag_enable_fpga == false)
        {
            top_block->disconnect(pass_through_->get_right_block(), 0, acq_->get_left_block(), 0);
            top_block->disconnect(pass_through_->get_right_block(), 0, trk_->get_left_block(), 0);
        }
    top_block->disconnect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);

    if (flag_enable_fpga == false)
        {
            pass_through_->disconnect(top_block);
        }
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
    std::lock_guard<std::mutex> lk(mx);
    gnss_signal_ = gnss_signal;
    std::string str_aux = gnss_signal_.get_signal_str();
    const char* str = str_aux.c_str();                              // get a C style null terminated string
    std::memcpy(static_cast<void*>(gnss_synchro_.Signal), str, 3);  // copy string into synchro char array: 2 char + null
    gnss_synchro_.Signal[2] = 0;                                    // make sure that string length is only two characters
    gnss_synchro_.PRN = gnss_signal_.get_satellite().get_PRN();
    gnss_synchro_.System = gnss_signal_.get_satellite().get_system_short().c_str()[0];
    acq_->set_local_code();
    nav_->set_satellite(gnss_signal_.get_satellite());
}


void Channel::start_acquisition()
{
    std::lock_guard<std::mutex> lk(mx);
    bool result = false;
    result = channel_fsm_->Event_start_acquisition();
    if (!result)
        {
            LOG(WARNING) << "Invalid channel event";
            return;
        }
    DLOG(INFO) << "Channel start_acquisition()";
}

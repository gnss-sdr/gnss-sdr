/*!
 * \file channel.cc
 * \brief Implementation of a GNSS_Channel with a Finite State Machine
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#include "channel.h"
#include "acquisition_interface.h"
#include "channel_fsm.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "telemetry_decoder_interface.h"
#include "tracking_interface.h"
#include <glog/logging.h>
#include <stdexcept>  // for std::invalid_argument
#include <utility>    // for std::move


Channel::Channel(const ConfigurationInterface* configuration,
    uint32_t channel,
    std::shared_ptr<AcquisitionInterface> acq,
    std::shared_ptr<TrackingInterface> trk,
    std::shared_ptr<TelemetryDecoderInterface> nav,
    const std::string& role,
    const std::string& signal_str,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : acq_(std::move(acq)),
      trk_(std::move(trk)),
      nav_(std::move(nav)),
      role_(role),
      channel_(channel),
      glonass_extend_correlation_ms_(configuration->property("Tracking_1G.extend_correlation_ms", 0) + configuration->property("Tracking_2G.extend_correlation_ms", 0)),
      connected_(false),
      repeat_(configuration->property("Acquisition_" + signal_str + ".repeat_satellite", false)),
      flag_enable_fpga_(configuration->property("GNSS-SDR.enable_FPGA", false))
{
    channel_fsm_ = std::make_shared<ChannelFsm>();

    acq_->set_channel(channel_);
    acq_->set_channel_fsm(channel_fsm_);
    trk_->set_channel(channel_);
    nav_->set_channel(channel_);

    gnss_synchro_ = Gnss_Synchro();
    gnss_synchro_.Channel_ID = channel_;
    acq_->set_gnss_synchro(&gnss_synchro_);
    trk_->set_gnss_synchro(&gnss_synchro_);

    repeat_ = configuration->property("Acquisition_" + signal_str + std::to_string(channel_) + ".repeat_satellite", repeat_);

    // Provide a warning to the user about the change of parameter name
    if (channel_ == 0)
        {
            const int64_t deprecation_warning = configuration->property("GNSS-SDR.internal_fs_hz", 0);
            if (deprecation_warning != 0)
                {
                    std::cout << "WARNING: The global parameter name GNSS-SDR.internal_fs_hz has been DEPRECATED.\n";
                    std::cout << "WARNING: Please replace it by GNSS-SDR.internal_fs_sps in your configuration file.\n";
                }
        }

    // IMPORTANT: Do not change the order between set_doppler_step and set_threshold

    uint32_t doppler_step = configuration->property("Acquisition_" + signal_str + std::to_string(channel_) + ".doppler_step", 0);
    if (doppler_step == 0)
        {
            doppler_step = configuration->property("Acquisition_" + signal_str + ".doppler_step", 500);
        }
    if (FLAGS_doppler_step != 0)
        {
            doppler_step = static_cast<uint32_t>(FLAGS_doppler_step);
        }
    DLOG(INFO) << "Channel " << channel_ << " Doppler_step = " << doppler_step;

    acq_->set_doppler_step(doppler_step);

    float threshold = configuration->property("Acquisition_" + signal_str + std::to_string(channel_) + ".threshold", static_cast<float>(0.0));
    if (threshold == 0.0)
        {
            threshold = configuration->property("Acquisition_" + signal_str + ".threshold", static_cast<float>(0.0));
        }

    acq_->set_threshold(threshold);

    acq_->init();

    channel_fsm_->set_acquisition(acq_);
    channel_fsm_->set_tracking(trk_);
    channel_fsm_->set_telemetry(nav_);
    channel_fsm_->set_channel(channel_);
    channel_fsm_->set_queue(queue);

    gnss_signal_ = Gnss_Signal(signal_str);

    channel_msg_rx_ = channel_msg_receiver_make_cc(channel_fsm_, repeat_);
}


void Channel::connect(gr::top_block_sptr top_block)
{
    if (!flag_enable_fpga_)
        {
            acq_->connect(top_block);
        }
    trk_->connect(top_block);

    if (trk_->item_size() == 0)
        {
            std::string msg = trk_->role() + ".item_type is not defined for implementation " + trk_->implementation() + '\n';
            throw std::invalid_argument(msg);
        }

    nav_->connect(top_block);

    // Synchronous ports
    top_block->connect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);

    // Message ports
    top_block->msg_connect(nav_->get_left_block(), pmt::mp("telemetry_to_trk"), trk_->get_right_block(), pmt::mp("telemetry_to_trk"));
    if (glonass_dll_pll_c_aid_tracking_check())
        {
            top_block->msg_connect(nav_->get_left_block(), pmt::mp("preamble_timestamp_samples"), trk_->get_right_block(), pmt::mp("preamble_timestamp_samples"));
        }
    DLOG(INFO) << "tracking -> telemetry_decoder";

    // Message ports
    if (!flag_enable_fpga_)
        {
            top_block->msg_connect(acq_->get_right_block(), pmt::mp("events"), channel_msg_rx_, pmt::mp("events"));
        }
    top_block->msg_connect(trk_->get_right_block(), pmt::mp("events"), channel_msg_rx_, pmt::mp("events"));

    connected_ = true;
}


void Channel::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Channel already disconnected internally";
            return;
        }

    top_block->disconnect(trk_->get_right_block(), 0, nav_->get_left_block(), 0);
    if (!flag_enable_fpga_)
        {
            acq_->disconnect(top_block);
        }
    trk_->disconnect(top_block);
    nav_->disconnect(top_block);

    top_block->msg_disconnect(nav_->get_left_block(), pmt::mp("telemetry_to_trk"), trk_->get_right_block(), pmt::mp("telemetry_to_trk"));
    if (glonass_dll_pll_c_aid_tracking_check())
        {
            top_block->msg_disconnect(nav_->get_left_block(), pmt::mp("preamble_timestamp_samples"), trk_->get_right_block(), pmt::mp("preamble_timestamp_samples"));
        }
    if (!flag_enable_fpga_)
        {
            top_block->msg_disconnect(acq_->get_right_block(), pmt::mp("events"), channel_msg_rx_, pmt::mp("events"));
        }
    top_block->msg_disconnect(trk_->get_right_block(), pmt::mp("events"), channel_msg_rx_, pmt::mp("events"));
    connected_ = false;
}


gr::basic_block_sptr Channel::get_left_block()
{
    LOG(ERROR) << "Deprecated call to get_left_block() in channel interface";
    return nullptr;
}


gr::basic_block_sptr Channel::get_left_block_trk()
{
    return trk_->get_left_block();
}


gr::basic_block_sptr Channel::get_right_block_trk()
{
    return trk_->get_right_block();
}


gr::basic_block_sptr Channel::get_left_block_acq()
{
    return acq_->get_left_block();
}


gr::basic_block_sptr Channel::get_right_block_acq()
{
    return acq_->get_right_block();
}


gr::basic_block_sptr Channel::get_right_block()
{
    return nav_->get_right_block();
}


void Channel::set_signal(const Gnss_Signal& gnss_signal)
{
    std::lock_guard<std::mutex> lk(mx_);
    gnss_signal_ = gnss_signal;
    const std::string str_aux = gnss_signal_.get_signal_str();
    gnss_synchro_.Signal[0] = str_aux[0];
    gnss_synchro_.Signal[1] = str_aux[1];
    gnss_synchro_.Signal[2] = '\0';  // make sure that string length is only two characters
    gnss_synchro_.PRN = gnss_signal_.get_satellite().get_PRN();
    gnss_synchro_.System = gnss_signal_.get_satellite().get_system_short().c_str()[0];
    acq_->set_local_code();
    if (flag_enable_fpga_)
        {
            trk_->set_gnss_synchro(&gnss_synchro_);
        }
    nav_->set_satellite(gnss_signal_.get_satellite());
}


void Channel::stop_channel()
{
    std::lock_guard<std::mutex> lk(mx_);
    const bool result = channel_fsm_->Event_stop_channel();
    if (!result)
        {
            LOG(WARNING) << "Invalid channel event";
            return;
        }
    DLOG(INFO) << "Channel stop_channel()";
}


void Channel::assist_acquisition_doppler(double Carrier_Doppler_hz)
{
    acq_->set_doppler_center(static_cast<int>(Carrier_Doppler_hz));
}


void Channel::start_acquisition()
{
    std::lock_guard<std::mutex> lk(mx_);
    bool result = false;
    if (!flag_enable_fpga_)
        {
            result = channel_fsm_->Event_start_acquisition();
        }
    else
        {
            result = channel_fsm_->Event_start_acquisition_fpga();
            channel_fsm_->start_acquisition();
        }
    if (!result)
        {
            LOG(WARNING) << "Invalid channel event";
            return;
        }
    DLOG(INFO) << "Channel start_acquisition()";
}

bool Channel::glonass_dll_pll_c_aid_tracking_check() const
{
    if (glonass_extend_correlation_ms_)
        {
            const pmt::pmt_t nav_ports_out = nav_->get_left_block()->message_ports_out();
            const pmt::pmt_t trk_ports_in = trk_->get_right_block()->message_ports_in();
            const pmt::pmt_t symbol = pmt::mp("preamble_timestamp_samples");
            for (unsigned k = 0; k < pmt::length(nav_ports_out); k++)
                {
                    if (pmt::vector_ref(nav_ports_out, k) == symbol)
                        {
                            for (unsigned j = 0; j < pmt::length(trk_ports_in); j++)
                                {
                                    if (pmt::vector_ref(trk_ports_in, j) == symbol)
                                        {
                                            return true;
                                        }
                                }
                            return false;
                        }
                }
        }
    return false;
}

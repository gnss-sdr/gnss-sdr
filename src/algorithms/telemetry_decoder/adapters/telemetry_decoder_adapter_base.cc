/*!
 * \file telemetry_decoder_adapter_base.cc
 * \brief Common functionality for telemetry decoder adapters
 * \author Carles Fernandez-Prades, 2025 cfernandez@cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "telemetry_decoder_adapter_base.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


TelemetryDecoderAdapterBase::TelemetryDecoderAdapterBase(
    const std::string& role,
    const std::string& implementation,
    unsigned int in_streams,
    unsigned int out_streams,
    telemetry_impl_interface_sptr decoder) : telemetry_decoder_(std::move(decoder)),
                                             role_(role),
                                             implementation_(implementation)
{
    DLOG(INFO) << "role " << role_;
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";

    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream ("
                       << in_streams << " provided)";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream ("
                       << out_streams << " provided)";
        }
}


void TelemetryDecoderAdapterBase::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        }
    DLOG(INFO) << "nothing to connect internally";
}


void TelemetryDecoderAdapterBase::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        }
}


gr::basic_block_sptr TelemetryDecoderAdapterBase::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr TelemetryDecoderAdapterBase::get_right_block()
{
    return telemetry_decoder_;
}


void TelemetryDecoderAdapterBase::set_satellite(const Gnss_Satellite& satellite)
{
    if (telemetry_decoder_)
        {
            const auto sat = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());  // Why is this required
            telemetry_decoder_->set_satellite(sat);
            DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << sat;
        }
}


std::string TelemetryDecoderAdapterBase::role()
{
    return role_;
}

std::string TelemetryDecoderAdapterBase::implementation()
{
    return implementation_;
}

void TelemetryDecoderAdapterBase::set_channel(int channel)
{
    if (telemetry_decoder_)
        {
            telemetry_decoder_->set_channel(channel);
        }
}


void TelemetryDecoderAdapterBase::reset()
{
    if (telemetry_decoder_)
        {
            telemetry_decoder_->reset();
        }
}


size_t TelemetryDecoderAdapterBase::item_size()
{
    return sizeof(Gnss_Synchro);
}

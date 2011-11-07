/*!
 * \file gps_l1_ca_dll_pll_tracking.cc
 * \brief code DLL + carrier PLL
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in [1]
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach, Birkha user, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#include "gps_l1_ca_dll_pll_tracking.h"

#include "configuration_interface.h"

#include <gnuradio/gr_io_signature.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

GpsL1CaDllPllTracking::GpsL1CaDllPllTracking(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        gr_msg_queue_sptr queue) :
    role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(
            queue)
{

    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./tracking.dat";

    DLOG(INFO) << "role " << role;
    DLOG(INFO) << "vector length " << vector_length_;

    item_type_ = configuration->property(role + ".item_type",
            default_item_type);
    vector_length_ = configuration->property(role + ".vector_length", 2048);
    fs_in_ = configuration->property(role + ".fs_in", 2048000);
    if_ = configuration->property(role + ".if", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!


    if (item_type_.compare("gr_complex") == 0)
    {
        item_size_ = sizeof(gr_complex);
        tracking_ = gps_l1_ca_dll_pll_make_tracking_cc(satellite_, if_,
                fs_in_, vector_length_, queue_, dump_);
    }
    else
    {
        LOG_AT_LEVEL(WARNING) << item_type_ << " unknown tracking item type.";
    }

    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}

GpsL1CaDllPllTracking::~GpsL1CaDllPllTracking()
{
}

void GpsL1CaDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

void GpsL1CaDllPllTracking::set_satellite(unsigned int satellite)
{
    satellite_ = satellite;
    tracking_->set_satellite(satellite);
    DLOG(INFO) << "satellite set to " << satellite_;
}

void GpsL1CaDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

void GpsL1CaDllPllTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;

    tracking_->set_channel_queue(channel_internal_queue_);

}
void GpsL1CaDllPllTracking::set_prn_code_phase(signed int phase_samples)
{
    return tracking_->set_acq_code_phase((float)phase_samples);
}

void GpsL1CaDllPllTracking::set_doppler_freq_shift(float doppler_freq_hz)
{
    return tracking_->set_acq_doppler(doppler_freq_hz);
}

void GpsL1CaDllPllTracking::set_acq_sample_stamp(
        unsigned long int sample_stamp)
{
    return tracking_->set_acq_sample_stamp(sample_stamp);
}
void GpsL1CaDllPllTracking::connect(gr_top_block_sptr top_block)
{
    //nothing to connect, now the tracking uses gr_sync_decimator
}

void GpsL1CaDllPllTracking::disconnect(gr_top_block_sptr top_block)
{
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr_basic_block_sptr GpsL1CaDllPllTracking::get_left_block()
{
    return tracking_;
}

gr_basic_block_sptr GpsL1CaDllPllTracking::get_right_block()
{
    return tracking_;
}


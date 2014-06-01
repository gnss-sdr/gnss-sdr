/*!
 * \file galileo_e5a_dll_fll_pll_tracking_cc.h
 * \brief Adapts code DLL + carrier PLL aided with FLL
 *  tracking block to TrackingInterface for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
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
#include "galileo_e5a_dll_fll_pll_tracking.h"
#include <glog/logging.h>
#include "Galileo_E5a.h"
#include "configuration_interface.h"

using google::LogMessage;

GalileoE5aDllFllPllTracking::GalileoE5aDllFllPllTracking(
        ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams, unsigned int
        out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
        role_(role),
        in_streams_(in_streams),
        out_streams_(out_streams),
        queue_(queue)
{
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    int fs_in;
    int vector_length;
    int f_if;
    bool dump;
    std::string dump_filename;
    std::string item_type;
    std::string default_item_type = "gr_complex";
    float pll_bw_hz;
    float fll_bw_hz;
    float dll_bw_hz;
    float early_late_space_chips;
    int order;
    item_type = configuration->property(role + ".item_type",default_item_type);
    //vector_length = configuration->property(role + ".vector_length", 2048);
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    order = configuration->property(role + ".order", 2);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    fll_bw_hz = configuration->property(role + ".fll_bw_hz", 100.0);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    vector_length = round(fs_in / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = galileo_e5a_dll_fll_pll_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    queue_,
                    dump,
                    dump_filename,
                    order,
                    fll_bw_hz,
                    pll_bw_hz,
                    dll_bw_hz,
                    early_late_space_chips);
        }
    else
        {
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }

    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GalileoE5aDllFllPllTracking::~GalileoE5aDllFllPllTracking()
{}


void GalileoE5aDllFllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

void GalileoE5aDllFllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

void GalileoE5aDllFllPllTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;
    tracking_->set_channel_queue(channel_internal_queue_);
}

void GalileoE5aDllFllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    return tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GalileoE5aDllFllPllTracking::connect(gr::top_block_sptr top_block)
{
    //nothing to connect, now the tracking uses gr_sync_decimator
}

void GalileoE5aDllFllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GalileoE5aDllFllPllTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GalileoE5aDllFllPllTracking::get_right_block()
{
    return tracking_;
}

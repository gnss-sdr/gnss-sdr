/*!
 * \file gps_l5i_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5i to a TrackingInterface
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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


#include "gps_l5i_dll_pll_tracking.h"
#include <glog/logging.h>
#include "GPS_L5.h"
#include "configuration_interface.h"


using google::LogMessage;

GpsL5iDllPllTracking::GpsL5iDllPllTracking(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
                role_(role), in_streams_(in_streams), out_streams_(out_streams)
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
    float dll_bw_hz;
    float early_late_space_chips;
    item_type = configuration->property(role + ".item_type", default_item_type);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    vector_length = std::round(static_cast<double>(fs_in) / (static_cast<double>(GPS_L5i_CODE_RATE_HZ) / static_cast<double>(GPS_L5i_CODE_LENGTH_CHIPS)));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = gps_l5i_dll_pll_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    dump,
                    dump_filename,
                    pll_bw_hz,
                    dll_bw_hz,
                    early_late_space_chips);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GpsL5iDllPllTracking::~GpsL5iDllPllTracking()
{}


void GpsL5iDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GpsL5iDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void GpsL5iDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GpsL5iDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    //nothing to connect, now the tracking uses gr_sync_decimator
}

void GpsL5iDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GpsL5iDllPllTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GpsL5iDllPllTracking::get_right_block()
{
    return tracking_;
}


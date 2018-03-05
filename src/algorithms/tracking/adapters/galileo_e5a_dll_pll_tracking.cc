/*!
 * \file galileo_e5a_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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

#include "galileo_e5a_dll_pll_tracking.h"
#include "configuration_interface.h"
#include "Galileo_E5a.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>

using google::LogMessage;

GalileoE5aDllPllTracking::GalileoE5aDllPllTracking(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
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
    float pll_bw_narrow_hz;
    float dll_bw_narrow_hz;
    int ti_ms;
    float early_late_space_chips;
    item_type = configuration->property(role + ".item_type", default_item_type);
    //vector_length = configuration->property(role + ".vector_length", 2048);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 12000000);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 20.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 20.0);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 5.0);
    dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 2.0);
    ti_ms = configuration->property(role + ".ti_ms", 3);

    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
        default_dump_filename);  //unused!
    vector_length = std::round(fs_in / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = galileo_e5a_dll_pll_make_tracking_cc(
                f_if,
                fs_in,
                vector_length,
                dump,
                dump_filename,
                pll_bw_hz,
                dll_bw_hz,
                pll_bw_narrow_hz,
                dll_bw_narrow_hz,
                ti_ms,
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


GalileoE5aDllPllTracking::~GalileoE5aDllPllTracking()
{
}


void GalileoE5aDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GalileoE5aDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void GalileoE5aDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GalileoE5aDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}

void GalileoE5aDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GalileoE5aDllPllTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GalileoE5aDllPllTracking::get_right_block()
{
    return tracking_;
}

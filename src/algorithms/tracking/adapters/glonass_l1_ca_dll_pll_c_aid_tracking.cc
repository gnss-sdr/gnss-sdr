/*!
 * \file glonass_l1_ca_dll_pll_c_aid_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Glonass L1 C/A to a TrackingInterface
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "glonass_l1_ca_dll_pll_c_aid_tracking.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "GLONASS_L1_L2_CA.h"
#include <glog/logging.h>


using google::LogMessage;

GlonassL1CaDllPllCAidTracking::GlonassL1CaDllPllCAidTracking(
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
    std::string default_item_type = "gr_complex";
    float pll_bw_hz;
    float pll_bw_narrow_hz;
    float dll_bw_hz;
    float dll_bw_narrow_hz;
    float early_late_space_chips;
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    //vector_length = configuration->property(role + ".vector_length", 2048);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 20.0);
    dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 2.0);
    int extend_correlation_ms;
    extend_correlation_ms = configuration->property(role + ".extend_correlation_ms", 1);

    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
        default_dump_filename);  //unused!
    vector_length = std::round(fs_in / (GLONASS_L1_CA_CODE_RATE_HZ / GLONASS_L1_CA_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_cc = glonass_l1_ca_dll_pll_c_aid_make_tracking_cc(
                f_if,
                fs_in,
                vector_length,
                dump,
                dump_filename,
                pll_bw_hz,
                dll_bw_hz,
                pll_bw_narrow_hz,
                dll_bw_narrow_hz,
                extend_correlation_ms,
                early_late_space_chips);
            DLOG(INFO) << "tracking(" << tracking_cc->unique_id() << ")";
        }
    else if (item_type_.compare("cshort") == 0)
        {
            item_size_ = sizeof(lv_16sc_t);
            tracking_sc = glonass_l1_ca_dll_pll_c_aid_make_tracking_sc(
                f_if,
                fs_in,
                vector_length,
                dump,
                dump_filename,
                pll_bw_hz,
                dll_bw_hz,
                pll_bw_narrow_hz,
                dll_bw_narrow_hz,
                extend_correlation_ms,
                early_late_space_chips);
            DLOG(INFO) << "tracking(" << tracking_sc->unique_id() << ")";
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type_ << " unknown tracking item type.";
        }
    channel_ = 0;
}


GlonassL1CaDllPllCAidTracking::~GlonassL1CaDllPllCAidTracking()
{
}


void GlonassL1CaDllPllCAidTracking::start_tracking()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            tracking_cc->start_tracking();
        }
    else if (item_type_.compare("cshort") == 0)
        {
            tracking_sc->start_tracking();
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown tracking item type";
        }
}


/*
 * Set tracking channel unique ID
 */
void GlonassL1CaDllPllCAidTracking::set_channel(unsigned int channel)
{
    channel_ = channel;

    if (item_type_.compare("gr_complex") == 0)
        {
            tracking_cc->set_channel(channel);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            tracking_sc->set_channel(channel);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown tracking item type";
        }
}


void GlonassL1CaDllPllCAidTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            tracking_cc->set_gnss_synchro(p_gnss_synchro);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            tracking_sc->set_gnss_synchro(p_gnss_synchro);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown tracking item type";
        }
}


void GlonassL1CaDllPllCAidTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}


void GlonassL1CaDllPllCAidTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GlonassL1CaDllPllCAidTracking::get_left_block()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            return tracking_cc;
        }
    else if (item_type_.compare("cshort") == 0)
        {
            return tracking_sc;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown tracking item type";
            return nullptr;
        }
}


gr::basic_block_sptr GlonassL1CaDllPllCAidTracking::get_right_block()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            return tracking_cc;
        }
    else if (item_type_.compare("cshort") == 0)
        {
            return tracking_sc;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown tracking item type";
            return nullptr;
        }
}

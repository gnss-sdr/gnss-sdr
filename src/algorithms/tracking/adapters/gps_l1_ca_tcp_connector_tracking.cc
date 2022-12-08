/*!
 * \file gps_l1_ca_tcp_connector_tracking.cc
 * \brief Implementation of an adapter of a TCP connector block based on code DLL + carrier PLL
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *            Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#include "gps_l1_ca_tcp_connector_tracking.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <utility>


GpsL1CaTcpConnectorTracking::GpsL1CaTcpConnectorTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      item_size_(sizeof(gr_complex)),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    // ################# CONFIGURATION PARAMETERS ########################
    const std::string default_item_type("gr_complex");
    std::string item_type = configuration->property(role_ + ".item_type", default_item_type);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    bool dump = configuration->property(role_ + ".dump", false);
    float early_late_space_chips = configuration->property(role_ + ".early_late_space_chips", static_cast<float>(0.5));
    size_t port_ch0 = configuration->property(role_ + ".port_ch0", 2060);
    const std::string default_dump_filename("./track_ch");
    std::string dump_filename = configuration->property(role_ + ".dump_filename", default_dump_filename);
    const auto vector_length = static_cast<int>(std::round(fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    // ################# MAKE TRACKING GNURadio object ###################
    DLOG(INFO) << "role " << role_;
    if (item_type == "gr_complex")
        {
            tracking_sptr_ = gps_l1_ca_tcp_connector_make_tracking_cc(
                fs_in,
                vector_length,
                dump,
                dump_filename,
                early_late_space_chips,
                port_ch0);
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            item_size_ = 0;
            tracking_sptr_ = nullptr;
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void GpsL1CaTcpConnectorTracking::stop_tracking()
{
}


void GpsL1CaTcpConnectorTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GpsL1CaTcpConnectorTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void GpsL1CaTcpConnectorTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL1CaTcpConnectorTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL1CaTcpConnectorTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL1CaTcpConnectorTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr GpsL1CaTcpConnectorTracking::get_right_block()
{
    return tracking_sptr_;
}

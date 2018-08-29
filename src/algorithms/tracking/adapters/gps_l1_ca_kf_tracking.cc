/*!
 * \file gps_l1_ca_kf_tracking.cc
 * \brief Implementation of an adapter of a DLL + Kalman carrier
 * tracking loop block for GPS L1 C/A signals
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Jordi Vila-Valls 2018. jvila(at)cttc.es
 * \author Carles Fernandez-Prades 2018. cfernandez(at)cttc.es
 *
 * Reference:
 * J. Vila-Valls, P. Closas, M. Navarro and C. Fernández-Prades,
 * "Are PLLs Dead? A Tutorial on Kalman Filter-based Techniques for Digital
 * Carrier Synchronization", IEEE Aerospace and Electronic Systems Magazine,
 * Vol. 32, No. 7, pp. 28–45, July 2017. DOI: 10.1109/MAES.2017.150260
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l1_ca_kf_tracking.h"
#include "gnss_sdr_flags.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include <glog/logging.h>


using google::LogMessage;

GpsL1CaKfTracking::GpsL1CaKfTracking(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    int order;
    int fs_in;
    int vector_length;
    int f_if;
    bool dump;
    std::string dump_filename;
    std::string item_type;
    std::string default_item_type = "gr_complex";
    float dll_bw_hz;
    float early_late_space_chips;
    bool bce_run;
    unsigned int bce_ptrans;
    unsigned int bce_strans;
    int bce_nu;
    int bce_kappa;

    item_type = configuration->property(role + ".item_type", default_item_type);
    order = configuration->property(role + ".order", 2);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    vector_length = std::round(fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    bce_run = configuration->property(role + ".bce_run", false);
    bce_ptrans = configuration->property(role + ".p_transient", 0);
    bce_strans = configuration->property(role + ".s_transient", 0);
    bce_nu = configuration->property(role + ".bce_nu", 0);
    bce_kappa = configuration->property(role + ".bce_kappa", 0);

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = gps_l1_ca_kf_make_tracking_cc(
                order,
                f_if,
                fs_in,
                vector_length,
                dump,
                dump_filename,
                dll_bw_hz,
                early_late_space_chips,
                bce_run,
                bce_ptrans,
                bce_strans,
                bce_nu,
                bce_kappa);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GpsL1CaKfTracking::~GpsL1CaKfTracking()
{
}


void GpsL1CaKfTracking::start_tracking()
{
    tracking_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL1CaKfTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void GpsL1CaKfTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL1CaKfTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL1CaKfTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL1CaKfTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr GpsL1CaKfTracking::get_right_block()
{
    return tracking_;
}

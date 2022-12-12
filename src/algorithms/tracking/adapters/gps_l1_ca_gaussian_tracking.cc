/*!
 * \file gps_l1_ca_gaussian_tracking.cc
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


#include "gps_l1_ca_gaussian_tracking.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>


GpsL1CaGaussianTracking::GpsL1CaGaussianTracking(
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
    int order = configuration->property(role_ + ".order", 2);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    bool dump = configuration->property(role_ + ".dump", false);
    float dll_bw_hz = configuration->property(role_ + ".dll_bw_hz", static_cast<float>(2.0));
    if (FLAGS_dll_bw_hz != 0.0)
        {
            dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
        }
    float early_late_space_chips = configuration->property(role_ + ".early_late_space_chips", static_cast<float>(0.5));
    const std::string default_dump_filename("./track_ch");
    std::string dump_filename = configuration->property(role_ + ".dump_filename", default_dump_filename);
    const auto vector_length = static_cast<int>(std::round(fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    bool bce_run = configuration->property(role_ + ".bce_run", false);
    unsigned int bce_ptrans = configuration->property(role_ + ".p_transient", 0);
    unsigned int bce_strans = configuration->property(role_ + ".s_transient", 0);
    int bce_nu = configuration->property(role_ + ".bce_nu", 0);
    int bce_kappa = configuration->property(role_ + ".bce_kappa", 0);

    // ################# MAKE TRACKING GNURadio object ###################
    DLOG(INFO) << "role " << role_;
    if (item_type == "gr_complex")
        {
            tracking_sptr_ = gps_l1_ca_gaussian_make_tracking_cc(
                order,
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
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            item_size_ = 0;
            tracking_sptr_ = nullptr;
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }

    if (in_streams_ == 0)
        {
            in_streams_ = 1;
            // Avoid compiler warning
        }
    if (out_streams_ == 0)
        {
            out_streams_ = 1;
            // Avoid compiler warning
        }
}


void GpsL1CaGaussianTracking::stop_tracking()
{
}


void GpsL1CaGaussianTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL1CaGaussianTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void GpsL1CaGaussianTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL1CaGaussianTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL1CaGaussianTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL1CaGaussianTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr GpsL1CaGaussianTracking::get_right_block()
{
    return tracking_sptr_;
}

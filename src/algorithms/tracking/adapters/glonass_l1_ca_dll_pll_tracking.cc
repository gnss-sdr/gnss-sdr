/*!
 * \file glonass_l1_ca_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for Glonass L1 C/A to a TrackingInterface
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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

#include "glonass_l1_ca_dll_pll_tracking.h"
#include "GLONASS_L1_L2_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GlonassL1CaDllPllTracking::GlonassL1CaDllPllTracking(
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
    float pll_bw_hz = configuration->property(role_ + ".pll_bw_hz", static_cast<float>(50.0));

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_pll_bw_hz != 0.0)
        {
            pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
        }
#else
    if (absl::GetFlag(FLAGS_pll_bw_hz) != 0.0)
        {
            pll_bw_hz = static_cast<float>(absl::GetFlag(FLAGS_pll_bw_hz));
        }
#endif
    float dll_bw_hz = configuration->property(role_ + ".dll_bw_hz", static_cast<float>(2.0));
#if USE_GLOG_AND_GFLAGS
    if (FLAGS_dll_bw_hz != 0.0)
        {
            dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
        }
#else
    if (absl::GetFlag(FLAGS_dll_bw_hz) != 0.0)
        {
            dll_bw_hz = static_cast<float>(absl::GetFlag(FLAGS_dll_bw_hz));
        }
#endif
    float early_late_space_chips = configuration->property(role_ + ".early_late_space_chips", static_cast<float>(0.5));
    const std::string default_dump_filename("./track_ch");
    std::string dump_filename = configuration->property(role_ + ".dump_filename", default_dump_filename);
    const auto vector_length = static_cast<int>(std::round(fs_in / (GLONASS_L1_CA_CODE_RATE_CPS / GLONASS_L1_CA_CODE_LENGTH_CHIPS)));

    // ################# MAKE TRACKING GNURadio object ###################
    DLOG(INFO) << "role " << role_;
    if (item_type == "gr_complex")
        {
            tracking_sptr_ = glonass_l1_ca_dll_pll_make_tracking_cc(
                fs_in,
                vector_length,
                dump,
                dump_filename,
                pll_bw_hz,
                dll_bw_hz,
                early_late_space_chips);
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


void GlonassL1CaDllPllTracking::stop_tracking()
{
}


void GlonassL1CaDllPllTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GlonassL1CaDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void GlonassL1CaDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GlonassL1CaDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GlonassL1CaDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GlonassL1CaDllPllTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr GlonassL1CaDllPllTracking::get_right_block()
{
    return tracking_sptr_;
}

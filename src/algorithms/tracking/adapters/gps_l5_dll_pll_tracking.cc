/*!
 * \file gps_l5_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l5_dll_pll_tracking.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

GpsL5DllPllTracking::GpsL5DllPllTracking(
    ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    trk_params.SetFromConfiguration(configuration, role);

    int vector_length = std::round(static_cast<double>(trk_params.fs_in) / (static_cast<double>(GPS_L5I_CODE_RATE_CPS) / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS)));
    trk_params.vector_length = vector_length;
    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!trk_params.track_pilot and trk_params.extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
        {
            trk_params.extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be lower than 11 when tracking the data component. Coherent integration has been set to 10 symbols (10 ms)" << TEXT_RESET << std::endl;
        }
    if ((trk_params.extend_correlation_symbols > 1) and (trk_params.pll_bw_narrow_hz > trk_params.pll_bw_hz or trk_params.dll_bw_narrow_hz > trk_params.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L5. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_params.system = 'G';
    std::array<char, 3> sig_{'L', '5', '\0'};
    std::memcpy(trk_params.signal, sig_.data(), 3);

    // ################# Make a GNU Radio Tracking block object ################
    if (trk_params.item_type == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = dll_pll_veml_make_tracking(trk_params);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void GpsL5DllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
}


void GpsL5DllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL5DllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void GpsL5DllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL5DllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL5DllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL5DllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr GpsL5DllPllTracking::get_right_block()
{
    return tracking_;
}

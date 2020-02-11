/*!
 * \file galileo_e5a_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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
#include "galileo_e5a_dll_pll_tracking.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

GalileoE5aDllPllTracking::GalileoE5aDllPllTracking(
    ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_param = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    trk_param.SetFromConfiguration(configuration, role);
    if (trk_param.smoother_length < 1)
        {
            trk_param.smoother_length = 1;
            std::cout << TEXT_RED << "WARNING: Gal. E5a. smoother_length must be bigger than 0. It has been set to 1" << TEXT_RESET << std::endl;
        }
    if (FLAGS_pll_bw_hz != 0.0) trk_param.pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    if (FLAGS_dll_bw_hz != 0.0) trk_param.dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    if (trk_param.extend_correlation_symbols < 1)
        {
            trk_param.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!trk_param.track_pilot and trk_param.extend_correlation_symbols > 1)
        {
            trk_param.extend_correlation_symbols = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be lower than 21 when tracking the data component. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    if ((trk_param.extend_correlation_symbols > 1) and (trk_param.pll_bw_narrow_hz > trk_param.pll_bw_hz or trk_param.dll_bw_narrow_hz > trk_param.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5a. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_param.very_early_late_space_chips = 0.0;
    trk_param.very_early_late_space_narrow_chips = 0.0;
    trk_param.system = 'E';
    std::array<char, 3> sig_{'5', 'X', '\0'};
    std::memcpy(trk_param.signal, sig_.data(), 3);
    int vector_length = std::round(trk_param.fs_in / (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS));
    trk_param.vector_length = vector_length;

    //################# MAKE TRACKING GNURadio object ###################
    if (trk_param.item_type != "gr_complex")
        {
            LOG(WARNING) << trk_param.item_type << " unknown tracking item type changing to gr_complex";
            trk_param.item_type = "gr_complex";
        }
    tracking_ = dll_pll_veml_make_tracking(trk_param);
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


void GalileoE5aDllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
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
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE5aDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE5aDllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr GalileoE5aDllPllTracking::get_right_block()
{
    return tracking_;
}

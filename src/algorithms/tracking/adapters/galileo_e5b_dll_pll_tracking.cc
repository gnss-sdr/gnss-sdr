/*!
 * \file galileo_e5b_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5b signals
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas@cttc.es
 *          <li> Luis Esteve, 2012. luis@epsilon-formacion.com
 *          <li> Marc Sales, 2014. marcsales92@gmail.com
 *          </ul>
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

#include "galileo_e5b_dll_pll_tracking.h"
#include "Galileo_E5b.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE5bDllPllTracking::GalileoE5bDllPllTracking(
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
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    trk_params.SetFromConfiguration(configuration, role_);

    const auto vector_length = static_cast<int>(std::round(trk_params.fs_in / (GALILEO_E5B_CODE_CHIP_RATE_CPS / GALILEO_E5B_CODE_LENGTH_CHIPS)));
    trk_params.vector_length = vector_length;
    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5b. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (!trk_params.track_pilot and trk_params.extend_correlation_symbols > GALILEO_E5B_I_SECONDARY_CODE_LENGTH)
        {
            trk_params.extend_correlation_symbols = GALILEO_E5B_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: Galileo E5b. extend_correlation_symbols must be lower than 5 when tracking the data component. Coherent integration has been set to 4 symbols (4 ms)" << TEXT_RESET << '\n';
        }
    if ((trk_params.extend_correlation_symbols > 1) and (trk_params.pll_bw_narrow_hz > trk_params.pll_bw_hz or trk_params.dll_bw_narrow_hz > trk_params.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5b. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    trk_params.system = 'E';
    const std::array<char, 3> sig{'7', 'X', '\0'};
    std::copy_n(sig.data(), 3, trk_params.signal);

    // ################# Make a GNU Radio Tracking block object ################
    DLOG(INFO) << "role " << role_;
    if (trk_params.item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(trk_params);
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            item_size_ = 0;
            tracking_sptr_ = nullptr;
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
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


void GalileoE5bDllPllTracking::stop_tracking()
{
    tracking_sptr_->stop_tracking();
}


void GalileoE5bDllPllTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GalileoE5bDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void GalileoE5bDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GalileoE5bDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE5bDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE5bDllPllTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr GalileoE5bDllPllTracking::get_right_block()
{
    return tracking_sptr_;
}
